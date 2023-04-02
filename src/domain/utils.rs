use nalgebra as na;
use rand::prelude::{thread_rng, Distribution};
use rand_distr::Normal;
use std::f32::consts::PI;

pub struct Stds {
    pub nn: f32,
    pub no: f32,
    pub on: f32,
    pub oo: f32,
}

pub fn mat_m(nu: f32, omega: f32, time: f32, stds: &Stds) -> na::Matrix2<f32> {
    na::Matrix2::new(
        stds.nn.powf(2.) * nu.abs() / time + stds.no.powf(2.) * omega.abs() / time,
        0.,
        0.,
        stds.on.powf(2.) * nu.abs() / time + stds.oo.powf(2.) * omega.abs() / time,
    )
}

pub fn mat_a(nu: f32, omega: f32, time: f32, theta: f32) -> na::Matrix3x2<f32> {
    let st = theta.sin();
    let ct = theta.cos();
    let stw = (theta + omega * time).sin();
    let ctw = (theta + omega * time).cos();
    na::Matrix3x2::new(
        (stw - st) / omega,
        (-nu / (omega.powf(2.0)) * (stw - st)) + (nu / omega * time * ctw),
        (-ctw + ct) / omega,
        (-nu / (omega.powf(2.0)) * (-ctw + ct)) + (nu / omega * time * stw),
        0.,
        time,
    )
}

pub fn mat_f(nu: f32, omega: f32, time: f32, theta: f32) -> na::Matrix3<f32> {
    let mut mat = na::Matrix3::identity();
    mat[(0, 2)] = (nu / omega) * ((theta + omega * time).cos() - theta.cos());
    mat[(1, 2)] = (nu / omega) * ((theta + omega * time).sin() - theta.sin());
    mat
}

pub fn mat_h(pose: &na::Vector3<f32>, landmark: &na::Vector3<f32>) -> na::Matrix2x3<f32> {
    let mx = landmark[0];
    let my = landmark[1];
    let mux = pose[0];
    let muy = pose[1];
    let q = (mux - mx).powf(2.0) + (muy - my).powf(2.0);
    na::Matrix2x3::new(
        (mux - mx) / q.sqrt(),
        (muy - my) / q.sqrt(),
        0.,
        (my - muy) / q,
        (mux - mx) / q,
        -1.0,
    )
}

pub fn mat_q(distance_dev: f32, direction_dev: f32) -> na::Matrix2<f32> {
    na::Matrix2::new(distance_dev.powf(2.0), 0., 0., direction_dev.powf(2.0))
}

pub fn polar_trans(pose: &na::Vector3<f32>, landmark: &na::Vector3<f32>) -> na::Matrix2x1<f32> {
    let diff = landmark - pose;
    let mut phi = diff[1].atan2(diff[0]) - pose[2];
    while phi >= PI {
        phi -= 2. * PI;
    }
    while phi < -PI {
        phi += 2. * PI;
    }
    na::Matrix2x1::new(diff[0].hypot(diff[1]), phi)
}

pub fn psi_predict(pose: &na::Vector3<f32>, landmark: &na::Vector3<f32>) -> f32 {
    let noise = PI / 90.;
    let mut rng = thread_rng();
    let diff = pose - landmark;
    let normal = Normal::new((diff[1]).atan2(diff[0]), noise).unwrap();
    let psi = normal.sample(&mut rng);
    psi
}
