use super::file;

use nalgebra as na;
use std::f32::consts::PI;

/* input condition */
pub const INIT_NU: f32 = 0.2;
pub const INIT_OMEGA: f32 = 2. * PI * 10. / 360.;
pub const TIME: f32 = 1.;
pub const LOOP_NUM: i32 = 36;
pub const INIT_POSE: na::Vector3<f32> = na::Vector3::new(0., 0., 0.);

pub const INIT_COV: f32 = 1e-10;

// pub const DISTANCE_BIAS_RATE_STD: 0.1,
// pub const direction_bias: 2. * PI * 2. / 360.,
// pub const distance_noise_rate: 0.1,
// pub const direction_noise: 2. * PI * 2. / 360.,
// pub const distance_range: [0.5, 6.],
// direction_range: [-(PI / 3.), PI / 3.],

pub fn init() {
    file::write_init();
    let (lpose, landsize) = dec_landmark();
}

/* config landmark */
pub fn dec_landmark() -> ([[f32; 3]; 6], usize) {
    let lpose: [[f32; 3]; 6] = [
        [-4., 2., 0.],
        [2., -3., 0.],
        [3., 3., 0.],
        [0., 4., 0.],
        [1., 1., 0.],
        [-3., -1., 0.],
    ];
    let landsize = lpose.len();

    (lpose, landsize)
}
