// use rand::Rng;
use rand::prelude::{thread_rng, Distribution};
// use rand::distributions::Normal;
use rand_distr::Normal;
// use rand_distr::{Normal, Distribution};
use nalgebra as na;
use std::f32::consts::PI;

pub struct Sensor {
    distance_bias_rate_std: f32,
    direction_bias: f32,
    distance_noise_rate: f32,
    direction_noise: f32,
    distance_range: [f32; 2],
    direction_range: [f32; 2],
}

pub fn observation_function(
    pose: &na::Vector3<f32>,
    landmark: &na::Vector3<f32>,
) -> na::Matrix2x1<f32> {
    let diff = landmark - pose;
    let mut phi = diff[1].atan2(diff[0]) - pose[2];
    while phi >= PI {
        phi -= 2. * PI;
    }
    while phi < -PI {
        phi += 2. * PI;
    }
    na::Matrix2x1::new(diff[0].hypot(diff[1]), phi)
}

impl Sensor {
    pub fn new() -> Self {
        Self {
            distance_bias_rate_std: 0.1,
            direction_bias: 2. * PI * 2. / 360.,
            distance_noise_rate: 0.1,
            direction_noise: 2. * PI * 2. / 360.,
            distance_range: [0.5, 6.],
            direction_range: [-(PI / 3.), PI / 3.],
        }
    }

    pub fn observation_predict(
        &self,
        pose: &na::Vector3<f32>,
        landmark: &na::Vector3<f32>,
    ) -> na::Matrix2x1<f32> {
        let diff = landmark - pose;
        let mut phi = diff[1].atan2(diff[0]) - pose[2];
        while phi >= PI {
            phi -= 2. * PI;
        }
        while phi < -PI {
            phi += 2. * PI;
        }
        na::Matrix2x1::new(diff[0].hypot(diff[1]), phi)
    }

    pub fn psi_predict(&self, pose: &na::Vector3<f32>, landmark: &na::Vector3<f32>) -> f32 {
        let noise = PI / 90.;
        let mut rng = thread_rng();
        let diff = pose - landmark;
        let normal = Normal::new((diff[1]).atan2(diff[0]), noise).unwrap();
        let psi = normal.sample(&mut rng);
        psi
    }

    pub fn visible(&self, polarpos: &na::Matrix2x1<f32>) -> bool {
        return self.distance_range[0] <= polarpos[0]
            && polarpos[0] <= self.distance_range[1]
            && self.direction_range[0] <= polarpos[1]
            && polarpos[1] <= self.direction_range[1];
    }

    pub fn exter_dist(&self, mut obj_dis: na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        obj_dis = Sensor::bias(self, &obj_dis);
        obj_dis = Sensor::noise(self, &obj_dis);
        obj_dis
    }
    fn bias(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mut mat = na::Matrix2x1::zeros();
        mat[0] = obj_dis[0] + obj_dis[0] * self.distance_bias_rate_std;
        mat[1] = obj_dis[1] + self.direction_bias;
        mat
    }

    fn noise(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mut rng = thread_rng();
        let normal_ell = Normal::new(obj_dis[0], obj_dis[0] * self.distance_noise_rate).unwrap();
        let normal_phi = Normal::new(obj_dis[1], self.direction_noise).unwrap();
        let ell = normal_ell.sample(&mut rng);
        let phi = normal_phi.sample(&mut rng);
        na::Matrix2x1::new(ell, phi)
    }
}
