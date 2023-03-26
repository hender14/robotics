use nalgebra as na;

/// データの取得（固定値）
pub fn get_land() -> Vec<(f32, f32)> {
    return vec![
        (-4., 2.),
        (2., -3.),
        (3., 3.),
        (0., 4.),
        (1., 1.),
        (-3., -1.),
    ];
}

/* config landmark */
pub fn dec_landmark() -> (na::Matrix6x3<f32>, usize) {
  let lpose1: na::RowVector3<f32> = na::RowVector3::new(-4., 2., 0.);
  let lpose2: na::RowVector3<f32> = na::RowVector3::new(2., -3., 0.);
  let lpose3: na::RowVector3<f32> = na::RowVector3::new(3., 3., 0.);
  let lpose4: na::RowVector3<f32> = na::RowVector3::new(0., 4., 0.);
  let lpose5: na::RowVector3<f32> = na::RowVector3::new(1., 1., 0.);
  let lpose6: na::RowVector3<f32> = na::RowVector3::new(-3., -1., 0.);
  let lpose: na::Matrix6x3<f32> =
      na::Matrix6x3::from_rows(&[lpose1, lpose2, lpose3, lpose4, lpose5, lpose6]);
  let landsize = lpose.nrows();
  // println!("size:{}", landsize);
  (lpose, landsize)
}