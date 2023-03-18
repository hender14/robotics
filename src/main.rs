// mod kf;
// pub(crate) mod kf;
use nalgebra as na;
use robotics::kf::agent;
// use std::f32::consts::PI;

fn main() {
    // let system_cov = pose::Stds {
    //     nn: 0.19,
    //     no: 0.001,
    //     oo: 0.13,
    //     on: 0.2,
    // };
    let nu = 0.2;
    // let omega = 2.*PI * 10./360.;
    let time = 0.1;
    let input = na::Vector3::zeros();
    let lpose = na::Vector3::new(3., 3., 0.);

    let mut agent = agent::Agent::new();

    for i in 0..2 {
        println!("{}", i);
        let pose = agent.pose_estimate(&input, nu, &lpose, time );
        println!("{}", pose);
    }
}
