use robotics::common::file;
use robotics::common::landmark;
use robotics::debug::plot;
use robotics::kf::agent;
use robotics::kf::kf;
use robotics::slam::slam;

use nalgebra as na;
use std::f32::consts::PI;

fn main() {
    /* input condition */
    let (nu, omega) = (0.2, 2. * PI * 10. / 360.);
    let time = 1.;
    let (lpose, landsize) = landmark::dec_landmark();
    let loop_num = 36;
    let mut pose = na::Vector3::new(0., 0., 0.);
    file::write_init();

    /* create object */
    let mut agent = agent::Agent::new(&pose, landsize);

    /* main loop */
    for _i in 0..loop_num {
        /* update robot position */
        pose = kf::KFilterPose::state_transition(nu, omega, time, &pose);

        /* kalman filter */
        agent.pose_estimate(nu, omega, &lpose, time, &pose);

        file::pose_write(
            agent.time,
            agent.nuo,
            agent.omegao,
            &agent.pose,
            &agent.zres,
            &agent.zlist,
        );
    }
    /* slam */
    slam::slam();

    /* plot */
    plot::plot_control();
}
