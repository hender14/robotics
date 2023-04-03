use super::file;
use crate::domain::sensor_data::Landmark;
use nalgebra as na;
use std::f32::consts::PI;

/* input condition */
pub const INIT_NU: f32 = 0.2;
pub const INIT_OMEGA: f32 = 2. * PI * 10. / 360.;
pub const TIME: f32 = 1.;
pub const LOOP_NUM: i32 = 36;
pub const INIT_POSE: na::Vector3<f32> = na::Vector3::new(0., 0., 0.);
pub const INIT_COV: f32 = 1e-10;

/* init application */
pub fn init() -> [Landmark; 6] {
    file::directry_init();
    file::file_init();
    let landmarks = get_landmark();
    landmarks
}

/* get landmark position */
pub fn get_landmark() -> [Landmark; 6] {
    let pose: [Landmark; 6] = [
        Landmark {
            id: 0,
            pose: na::Vector3::new(-4., 2., 0.),
        },
        Landmark {
            id: 1,
            pose: na::Vector3::new(2., -3., 0.),
        },
        Landmark {
            id: 2,
            pose: na::Vector3::new(3., 3., 0.),
        },
        Landmark {
            id: 3,
            pose: na::Vector3::new(0., 4., 0.),
        },
        Landmark {
            id: 4,
            pose: na::Vector3::new(1., 1., 0.),
        },
        Landmark {
            id: 5,
            pose: na::Vector3::new(-3., -1., 0.),
        },
    ];
    pose
}
