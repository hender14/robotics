use super::{sensor_data::LandmarkData, utils};
use nalgebra as na;

/* Structure representing an edge between a sensor observation and a landmark in the SLAM graph. */
pub struct MapEdge {
    x: na::Vector3<f32>,
    z: LandmarkData,
    m: na::Vector3<f32>,
    pub omega: na::Matrix3<f32>,
    pub xi: na::Vector3<f32>,
}

/* Default implementation for MapEdge. */
impl Default for MapEdge {
    fn default() -> Self {
        Self::new()
    }
}

impl MapEdge {
    /* Constructor for MapEdge. */
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

    /* Calculates the landmark matrix, which is used to update the map edge based on sensor measurements. */
    pub fn land_matrix(
        &mut self,
        t: usize,
        z: LandmarkData,
        head_t: usize,
        head_z: LandmarkData,
        state_vec: &[(f32, f32, f32)],
    ) {
        /* Sensor noise rates */
        let sensor_noise_rate = [0.14, 0.05, 0.05];
        /* Update the current state vector and landmark data. */
        self.x = na::Vector3::new(state_vec[t].0, state_vec[t].1, state_vec[t].2);
        self.z = z;

        /* Calculate the position of the landmark in the global frame. */
        let landmarl_offset = na::Vector3::new(
            self.z.polor[0] * (self.x[2] + self.z.polor[1]).cos(),
            self.z.polor[0] * (self.x[2] + self.z.polor[1]).sin(),
            -state_vec[head_t].2 + self.z.polor[1] - head_z.polor[1] - self.z.psi + head_z.psi,
        );
        self.m = self.x + landmarl_offset;

        /* Normalize the angle of the landmark's position in the global frame. */
        self.m[2] = utils::normalize_angle(self.m[2]);

        /* Compute the observation covariance matrix. */
        let q1_diag = na::Vector3::new(
            (self.z.polor[1] * sensor_noise_rate[0]).powi(2),
            sensor_noise_rate[1].powi(2),
            sensor_noise_rate[2].powi(2),
        );
        let q1 = na::Matrix3::from_diagonal(&q1_diag);

        /* Calculate the Jacobian matrix. */
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
        /* Compute the inverse of the covariance matrix transformed by the Jacobian. */
        self.omega = (r * q1 * r.transpose()).try_inverse().unwrap();
        /* Calculate the information vector. */
        self.xi = self.omega * self.m;
    }
}
