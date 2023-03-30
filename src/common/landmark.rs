use nalgebra as na;

/* get landmark position */
// pub fn get_land() -> Vec<(f32, f32)> {
//     return vec![
//         (-4., 2.),
//         (2., -3.),
//         (3., 3.),
//         (0., 4.),
//         (1., 1.),
//         (-3., -1.),
//     ];
// }

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
