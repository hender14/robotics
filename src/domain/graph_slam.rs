use super::super::domain::solver::MotionEdge;
use super::super::domain::solver::SensorEdge;
use super::constraint::Constraint;
use itertools::Itertools;
use nalgebra as na;

pub fn graph_slam(
    hat_xs: &Vec<(f32, f32, f32)>,
    us: &Vec<(f32, f32)>,
    landmark_list: &[Vec<(usize, Constraint)>; 6],
) -> na::Matrix<f32, na::Dyn, na::Const<1>, na::VecStorage<f32, na::Dyn, na::Const<1>>> {
    let xdim = hat_xs.len() * 3;
    let mut omega = na::DMatrix::<f32>::zeros(xdim, xdim);
    for i in 0..3 {
        omega[(i, i)] = 1000000.;
    }
    let mut xi = na::DVector::<f32>::zeros(xdim);

    for index in 0..landmark_list.len() {
        for comb in landmark_list[index].iter().combinations(2) {
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
                motionedges.action_matrix(us.clone(), 1.);
                (omega, xi) = motionedges.add_edge(omega, xi);
            }
        }
    }

    let delta_xs = (omega.try_inverse().unwrap()) * xi;
    delta_xs
}
