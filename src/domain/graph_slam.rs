use super::{constraint::Constraint, state::Twist};
use crate::domain::solver::{MotionEdge, SensorEdge};
use itertools::Itertools;
use nalgebra as na;

pub fn graph_slam(
    hat_xs: &Vec<(f32, f32, f32)>,
    velocity: &[Twist],
    landmark_list: &[Vec<(usize, Constraint)>; 6],
) -> na::Matrix<f32, na::Dyn, na::Const<1>, na::VecStorage<f32, na::Dyn, na::Const<1>>> {
    let xdim = hat_xs.len() * 3;
    let mut omega = na::DMatrix::<f32>::zeros(xdim, xdim);
    for i in 0..3 {
        omega[(i, i)] = 1000000.;
    }
    let mut xi = na::DVector::<f32>::zeros(xdim);

    for landmark in landmark_list {
        for comb in landmark.iter().combinations(2) {
            let mut sensordges = SensorEdge::new(
                comb[0].0,
                comb[1].0,
                comb[0].1.clone(),
                comb[1].1.clone(),
                hat_xs,
            );
            sensordges.precision_matrix();
            (omega, xi) = sensordges.add_edge(omega, xi);

            for i in 0..hat_xs.len() - 1 {
                let mut motionedges = MotionEdge::new(i, i + 1, hat_xs);
                motionedges.action_matrix(velocity, 1.);
                (omega, xi) = motionedges.add_edge(omega, xi);
            }
        }
    }
    (omega.try_inverse().unwrap()) * xi
}
