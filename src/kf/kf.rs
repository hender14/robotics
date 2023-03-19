use super::sensor;
use nalgebra as na;

pub struct Stds {
    pub nn: f32,
    pub no: f32,
    pub on: f32,
    pub oo: f32,
}

pub fn mat_m(nu: f32, omega: f32, time: f32, stds: &Stds) -> na::Matrix2<f32> {
    na::Matrix2::new(
        stds.nn.powf(2.) * nu.abs() / time + stds.no.powf(2.) * omega.abs() / time,
        0.,
        0.,
        stds.on.powf(2.) * nu.abs() / time + stds.oo.powf(2.) * omega.abs() / time,
    )
}

fn mat_a(nu: f32, omega: f32, time: f32, theta: f32) -> na::Matrix3x2<f32> {
    let st = theta.sin();
    let ct = theta.cos();
    let stw = (theta + omega * time).sin();
    let ctw = (theta + omega * time).cos();
    na::Matrix3x2::new(
        (stw - st) / omega,
        (-nu / (omega.powf(2.0)) * (stw - st)) + (nu / omega * time * ctw),
        (-ctw + ct) / omega,
        (-nu / (omega.powf(2.0)) * (-ctw + ct)) + (nu / omega * time * stw),
        0.,
        time,
    )
}

fn mat_f(nu: f32, omega: f32, time: f32, theta: f32) -> na::Matrix3<f32> {
    let mut mat = na::Matrix3::identity();
    mat[(0, 2)] = (nu / omega) * ((theta + omega * time).cos() - theta.cos());
    mat[(1, 2)] = (nu / omega) * ((theta + omega * time).sin() - theta.sin());
    mat
}

fn mat_h(pose: &na::Vector3<f32>, landmark: &na::Vector3<f32>) -> na::Matrix2x3<f32> {
    let mx = landmark[0];
    let my = landmark[1];
    let mux = pose[0];
    let muy = pose[1];
    let q = (mux - mx).powf(2.0) + (muy - my).powf(2.0);
    na::Matrix2x3::new(
        (mux - mx) / q.sqrt(),
        (muy - my) / q.sqrt(),
        0.,
        (my - muy) / q,
        (mux - mx) / q,
        -1.0,
    )
}

fn mat_q(distance_dev: f32, direction_dev: f32) -> na::Matrix2<f32> {
    na::Matrix2::new(distance_dev.powf(2.0), 0., 0., direction_dev.powf(2.0))
}

pub struct KFilterPose<'a> {
    belief: Belief,
    system_cov: &'a Stds,
    distance_dev_rate: f32,
    direction_dev: f32,
}

struct Belief {
    mean: na::Vector3<f32>,
    cov: na::Matrix3<f32>,
}

impl<'a> KFilterPose<'a> {
    pub fn new(initial_state_mean: &na::Vector3<f32>, initial_cov: f32) -> Self {
        let system_cov = &Stds {
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
        let m = mat_m(nu, omega, time, self.system_cov);
        let a = mat_a(nu, omega, time, self.belief.mean[2]);
        let f = mat_f(nu, omega, time, self.belief.mean[2]);
        self.belief.cov = f * (self.belief.cov) * (f.transpose()) + a * (m) * (a.transpose());
        self.belief.mean = KFilterPose::state_transition(nu, omega, time, &self.belief.mean);
    }

    pub fn kf_update(
        &mut self,
        obj_dis: &na::Vector2<f32>,
        landmark: &na::Vector3<f32>,
    ) -> na::Vector3<f32> {
        let h = mat_h(&self.belief.mean, landmark);
        let estimated_z = sensor::observation_function(&self.belief.mean, landmark);
        let z = obj_dis;
        let q = mat_q(estimated_z[0] * self.distance_dev_rate, self.direction_dev);
        let kalman_gain = self.belief.cov
            * (h.transpose())
            * (h * self.belief.cov * h.transpose() + q)
                .try_inverse()
                .unwrap();
        self.belief.mean += kalman_gain * (z - estimated_z);
        self.belief.cov = (na::Matrix3::identity() - kalman_gain * h) * self.belief.cov;

        self.belief.mean
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
}
