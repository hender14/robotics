use nalgebra as na;
use std::f32::consts::PI;

pub struct Sensor {
    distance_bias_rate_std: f32,
    direction_bias: f32,
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
    let mat = na::Matrix2x1::new(diff[0].hypot(diff[1]), phi);
    mat
}

impl Sensor {
    pub fn new() -> Self {
        Self {
            distance_bias_rate_std: 0.1,
            direction_bias: 2. * PI * 2. / 360.,
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
        let mat = na::Matrix2x1::new(diff[0].hypot(diff[1]), phi);
        mat
    }

    pub fn noise(&self, obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mat = Sensor::bias(&self, &obj_dis);
        mat
    }
    fn bias(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mut mat = na::Matrix2x1::zeros();
        mat[0] = obj_dis[0] + obj_dis[0] * self.distance_bias_rate_std;
        mat[1] = obj_dis[1] + self.direction_bias;
        mat
    }
}
