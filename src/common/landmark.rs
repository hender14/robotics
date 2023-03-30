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
