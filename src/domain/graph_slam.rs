use super::{constraint::Constraint, state::Twist};
use crate::domain::solver::{MotionEdge, SensorEdge};
use itertools::Itertools;
use nalgebra as na;

/* run graphed slam */
pub fn graph_slam(
    estimate_states: &Vec<(f32, f32, f32)>,
    velocity: &[Twist],
    landmark_list: &[Vec<(usize, Constraint)>],
) -> na::Matrix<f32, na::Dyn, na::Const<1>, na::VecStorage<f32, na::Dyn, na::Const<1>>> {
    let xdim = estimate_states.len() * 3;

    /* Initialize the Omega, Xi matrix with high confidence for the first pose */
    let mut omega = na::DMatrix::<f32>::zeros(xdim, xdim);
    for i in 0..3 {
        omega[(i, i)] = 1000000.;
    }
    let mut xi = na::DVector::<f32>::zeros(xdim);

    for landmark in landmark_list {
        for comb in landmark.iter().combinations(2) {
            /* Create a SensorEdge from the pair of constraints */
            let mut sensor_dges = SensorEdge::new(
                comb[0].0,
                comb[1].0,
                comb[0].1.clone(),
                comb[1].1.clone(),
                estimate_states,
            );
            /* Compute the precision matrix for the SensorEdge */
            sensor_dges.calc_precision_matrix();
            /* Update the Omega matrix and Xi vector with the SensorEdge */
            (omega, xi) = sensor_dges.add_edge(omega, xi);

            for i in 0..estimate_states.len() - 1 {
                /* create MotionEdges */
                let mut motion_edges = MotionEdge::new(i, i + 1, estimate_states);
                /*  Compute the action matrix for the MotionEdge using the given velocity and time step */
                motion_edges.calc_action_matrix(velocity, 1.);
                /* Update the Omega matrix and Xi vector with the MotionEdge */
                (omega, xi) = motion_edges.add_edge(omega, xi);
            }
        }
    }
    /* Solve the linear system to obtain the final estimated states */
    (omega.try_inverse().unwrap()) * xi
}
