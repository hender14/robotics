use nalgebra as na;
use robotics::kf::agent;
use robotics::kf::kf;
use std::f32::consts::PI;

fn main() {
    /* input condition */
    let nu = 0.2;
    let omega = 2. * PI * 10. / 360.;
    let time = 1.;
    let lpose = dec_landmark();
    let loop_num = 2;

    /* initial */
    let mut pose = na::Vector3::zeros();

    /* create object */
    let mut agent = agent::Agent::new(&pose, lpose.1);

    /* main loop */
    for i in 0..loop_num {
        /* update robot position */
        pose = kf::KFilterPose::state_transition(nu, omega, time, &pose);

        /* kalman filter */
        let kf_pose = agent.pose_estimate(nu, omega, &lpose.0, time, &pose);
        println!("{} pose: {}", i, &kf_pose);
    }
}

/* config landmark */
fn dec_landmark() -> (na::Matrix6x3<f32>, usize) {
    let lpose1: na::RowVector3<f32> = na::RowVector3::new(-4., 2., 0.);
    let lpose2: na::RowVector3<f32> = na::RowVector3::new(2., -3., 0.);
    let lpose3: na::RowVector3<f32> = na::RowVector3::new(3., 3., 0.);
    let lpose4: na::RowVector3<f32> = na::RowVector3::new(0., 4., 0.);
    let lpose5: na::RowVector3<f32> = na::RowVector3::new(1., 1., 0.);
    let lpose6: na::RowVector3<f32> = na::RowVector3::new(-3., -1., 0.);
    let lpose: na::Matrix6x3<f32> = na::Matrix6x3::from_rows(&[lpose1, lpose2, lpose3, lpose4, lpose5, lpose6]);
    let landsize = lpose.nrows();
    println!("size:{}", landsize);
    (lpose, landsize)
}
