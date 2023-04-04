use super::{
    sensor_data::{Landmark, SensorData},
    utils as ut,
};
use nalgebra as na;

struct Belief {
    mean: na::Vector3<f32>,
    cov: na::Matrix3<f32>,
}

pub struct KFilterPose<'a> {
    belief: Belief,
    system_cov: &'a ut::Stds,
    distance_dev_rate: f32,
    direction_dev: f32,
}

impl<'a> KFilterPose<'a> {
    pub fn new(initial_state_mean: &na::Vector3<f32>, initial_cov: f32) -> Self {
        let system_cov = &ut::Stds {
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

    pub fn kf_predict(&mut self, nu: f32, omega: f32, time: f32) {
        let m = ut::mat_m(nu, omega, time, self.system_cov);
        let a = ut::mat_a(nu, omega, time, self.belief.mean[2]);
        let f = ut::mat_f(nu, omega, time, self.belief.mean[2]);
        self.belief.cov = f * (self.belief.cov) * (f.transpose()) + a * (m) * (a.transpose());
        self.belief.mean = state_transition(nu, omega, time, &self.belief.mean);
    }

    pub fn kf_update(
        &mut self,
        sensor_data: &Vec<SensorData>,
        landmarks: &[Landmark; 6],
    ) -> na::Vector3<f32> {
        /* Process by landmark */
        for landmark in landmarks {
            if sensor_data[landmark.id].result {
                /* calculate landmark */
                let h = ut::mat_h(&self.belief.mean, &landmark.pose);
                let estimated_z = ut::polar_trans(&self.belief.mean, &landmark.pose);
                let z = sensor_data[landmark.id].data.polor;
                let q = ut::mat_q(estimated_z[0] * self.distance_dev_rate, self.direction_dev);
                let kalman_gain = self.belief.cov
                    * (h.transpose())
                    * (h * self.belief.cov * h.transpose() + q)
                        .try_inverse()
                        .unwrap();
                self.belief.mean += kalman_gain * (z - estimated_z);
                self.belief.cov = (na::Matrix3::identity() - kalman_gain * h) * self.belief.cov;
            }
        }
        self.belief.mean
    }
}

pub fn state_transition(
    nu: f32,
    omega: f32,
    time: f32,
    pose: &na::Vector3<f32>,
) -> na::Vector3<f32> {
    let t0 = pose[2];
    if omega.abs() < 1e-10 {
        return pose + na::Vector3::new(nu * t0.cos(), nu * t0.sin(), omega) * time;
    } else {
        return pose
            + na::Vector3::new(
                nu / omega * ((t0 + omega * time).sin() - t0.sin()),
                nu / omega * (-(t0 + omega * time).cos() + t0.cos()),
                omega * time,
            );
    }
}
