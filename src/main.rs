// mod kf;
// pub(crate) mod kf;
use robotics::kf::pose;
use std::f32::consts::PI;

fn main() {

    let system_cov = vec![0.19, 0.001, 0.13, 0.2];
    let nu = 2.;
    let omega = 2. * PI * 10. / 360.;
    let time = 1.;

    let result = pose::mat_m(nu, omega, time, &system_cov);
    println!("{}", result);  
    println!("Hello, world!");
}
