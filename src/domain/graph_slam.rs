use super::super::domain::solver::MotionEdge;
use super::super::domain::solver::SensorEdge;
use itertools::Itertools;
use nalgebra as na;

pub fn graph_slam(
    hat_xs: &Vec<(f32, f32, f32)>,
    us: &Vec<(f32, f32)>,
    land_klist: &[Vec<(f32, (f32, f32, f32, f32))>; 6],
) -> (
    na::Matrix<f32, na::Dyn, na::Const<1>, na::VecStorage<f32, na::Dyn, na::Const<1>>>,
    [Vec<(f32, (f32, f32, f32, f32))>; 6],
) {
    let xdim = hat_xs.len() * 3;
    let mut omega = na::DMatrix::<f32>::zeros(xdim, xdim);
    for i in 0..3 {
        omega[(i, i)] = 1000000.;
    }
    let mut xi = na::DVector::<f32>::zeros(xdim);

    for index in 0..land_klist.len() {
        for comb in land_klist[index].iter().combinations(2) {
            let mut sensordges =
                SensorEdge::new(comb[0].0, comb[1].0, comb[0].1, comb[1].1, hat_xs.clone());
            sensordges.precision_matrix();
            (omega, xi) = sensordges.add_edge(omega, xi);

            for i in 0..hat_xs.len() - 1 {
                let mut motionedges = MotionEdge::new(i, i + 1, hat_xs.clone());
                motionedges.action_matrix(us.clone(), 1.);
                (omega, xi) = motionedges.add_edge(omega, xi);
            }
        }
    }
    // println!("omega:{}, xi:{}", omega, xi);

    let delta_xs = (omega.try_inverse().unwrap()) * xi;
    (delta_xs, land_klist.clone())
}
