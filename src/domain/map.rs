use nalgebra as na;
use std::f32::consts::PI;

use super::sensor_data::LandmarkData;

pub struct MapEdge {
    x: na::Vector3<f32>,
    z: LandmarkData,
    m: na::Vector3<f32>,
    pub omega: na::Matrix3<f32>,
    pub xi: na::Vector3<f32>,
}

impl MapEdge {
    pub fn new() -> Self {
        Self {
            x: Default::default(),
            z: LandmarkData {
                polor: na::Matrix2x1::new(0., 0.),
                psi: 0.,
            },
            m: Default::default(),
            omega: Default::default(),
            xi: Default::default(),
        }
    }

    pub fn land_matrix(
        &mut self,
        t: usize,
        z: LandmarkData,
        head_t: usize,
        head_z: LandmarkData,
        xs_vec: &Vec<(f32, f32, f32)>,
    ) {
        let sensor_noise_rate = [0.14, 0.05, 0.05];
        self.x = na::Vector3::new(xs_vec[t].0, xs_vec[t].1, xs_vec[t].2);
        self.z = z;

        let array = na::Vector3::new(
            self.z.polor[0] * (self.x[2] + self.z.polor[1]).cos(),
            self.z.polor[0] * (self.x[2] + self.z.polor[1]).sin(),
            -xs_vec[head_t].2 + self.z.polor[1] - head_z.polor[1] - self.z.psi + head_z.psi,
        );
        self.m = self.x + array;

        while self.m[2] >= PI {
            self.m[2] -= 2. * PI;
        }
        while self.m[2] < -PI {
            self.m[2] += 2. * PI;
        }

        let q1_diag = na::Vector3::new(
            (self.z.polor[1] * sensor_noise_rate[0]).powi(2),
            sensor_noise_rate[1].powi(2),
            sensor_noise_rate[2].powi(2),
        );
        let q1 = na::Matrix3::from_diagonal(&q1_diag);

        let s1 = (self.x[2] + self.z.polor[1]).sin();
        let c1 = (self.x[2] + self.z.polor[1]).cos();
        let r = na::Matrix3::new(
            -c1,
            self.z.polor[0] * s1,
            0.,
            -s1,
            -self.z.polor[0] * c1,
            0.,
            0.,
            -1.,
            1.,
        );
        self.omega = (r * q1 * r.transpose()).try_inverse().unwrap();
        self.xi = self.omega * self.m;
    }
}
