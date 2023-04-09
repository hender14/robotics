use super::{
    sensor_data::{Landmark, SensorData},
    state::Twist,
    utils as ut,
};
use nalgebra as na;

/* Belief struct representing the robot's estimated state (mean) and covariance matrix. */
pub struct Belief {
    pub mean: na::Vector3<f32>,
    cov: na::Matrix3<f32>,
}

/* KFilterPose struct representing the Kalman Filter for robot pose estimation. */
pub struct KFilterPose<'a> {
    pub belief: Belief,
    system_cov: &'a ut::MotionNoise,
    distance_dev_rate: f32,
    direction_dev: f32,
}

impl<'a> KFilterPose<'a> {
    /* Creates a new KFilterPose object with initial mean state and covariance. */
    pub fn new(initial_state_mean: &na::Vector3<f32>, initial_cov: f32) -> Self {
        let system_cov = &ut::MotionNoise {
            nn: 0.19,
            no: 0.001,
            oo: 0.13,
            on: 0.2,
        };
        let belief = Belief {
            mean: *initial_state_mean,
            cov: initial_cov * na::Matrix3::identity(),
        };
        Self {
            belief,
            system_cov,
            distance_dev_rate: 0.14,
            direction_dev: 0.05,
        }
    }

    /* Prediction step of the Kalman Filter. */
    pub fn kf_predict(&mut self, velocity: Twist, time: f32) {
        let m = ut::motion_noise_cov(velocity.nu, velocity.omega, time, self.system_cov);
        let a = ut::state_trans_model(velocity.nu, velocity.omega, time, self.belief.mean[2]);
        let f = ut::state_trans_jac(velocity.nu, velocity.omega, time, self.belief.mean[2]);
        self.belief.cov = f * (self.belief.cov) * (f.transpose()) + a * (m) * (a.transpose());
        self.belief.mean = state_transition(velocity.nu, velocity.omega, time, &self.belief.mean);
    }

    /* Update step of the Kalman Filter. */
    pub fn kf_update(&mut self, sensor_data: &[SensorData], landmarks: &[Landmark]) {
        // Process each landmark.
        for landmark in landmarks {
            if sensor_data[landmark.id].result {
                /* Compute observation model and Jacobian. */
                let h = ut::observ_model_jac(&self.belief.mean, &landmark.pose);
                let estimated_z = ut::cartesian_to_polar(&self.belief.mean, &landmark.pose);
                let z = sensor_data[landmark.id].data.polor;
                let q = ut::observ_noise_cov(
                    estimated_z[0] * self.distance_dev_rate,
                    self.direction_dev,
                );
                /* Compute Kalman Gain. */
                let kalman_gain = self.belief.cov
                    * (h.transpose())
                    * (h * self.belief.cov * h.transpose() + q)
                        .try_inverse()
                        .unwrap();
                /* Update state mean and covariance. */
                self.belief.mean += kalman_gain * (z - estimated_z);
                self.belief.cov = (na::Matrix3::identity() - kalman_gain * h) * self.belief.cov;
            }
        }
    }
}

/* Calculate the state transition  */
pub fn state_transition(
    nu: f32,
    omega: f32,
    time: f32,
    pose: &na::Vector3<f32>,
) -> na::Vector3<f32> {
    let t0 = pose[2];
    if omega.abs() < 1e-10 {
        pose + na::Vector3::new(nu * t0.cos(), nu * t0.sin(), omega) * time
    } else {
        pose + na::Vector3::new(
            nu / omega * ((t0 + omega * time).sin() - t0.sin()),
            nu / omega * (-(t0 + omega * time).cos() + t0.cos()),
            omega * time,
        )
    }
}
