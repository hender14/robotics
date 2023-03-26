use robotics::common::landmark;
use robotics::debug::plot;
use robotics::kf::agent;
use robotics::kf::kf;
use robotics::slam::slam;

use plotters::prelude::*;
use nalgebra as na;
use std::f32::consts::PI;

fn main() {
    let root = BitMapBackend::new("plot.png", (640, 480)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .caption("Point Cloud", ("sans-serif", 30))
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-5.0..5.0, -5.0..5.0)
        .unwrap();

    chart.configure_mesh().draw().unwrap();
    /* input condition */
    let nu = 0.2;
    let omega = 2. * PI * 10. / 360.;
    let time = 1.;
    let lpose = landmark::dec_landmark();
    let loop_num = 12;

    /* initial */
    let mut pose = na::Vector3::zeros();

    /* create object */
    let mut agent = agent::Agent::new(&pose, lpose.1);

    /* main loop */
    for _i in 0..loop_num {
        /* update robot position */
        pose = kf::KFilterPose::state_transition(nu, omega, time, &pose);

        /* kalman filter */
        let kf_pose = agent.pose_estimate(nu, omega, &lpose.0, time, &pose);
        // println!("{} pose: {}", i, &kf_pose);
    }
    /* slam */
    slam::slam();

    /* plot */
    plot::plot_pose(&mut chart);
    plot::plot_land(&mut chart);
    plot::plot_edge(&mut chart);
}

