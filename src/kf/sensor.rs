use nalgebra as na;
use std::f32::consts::PI;

pub struct Sensor {
  // belief: Belief,
  distance_bias_rate_std: f32,
  direction_bias: f32,
  // pose_change: na::Vector3<f32>,
}

impl Sensor {
  pub fn new( ) -> Self {
  // pub fn new( initial_state_mean: &na::Vector3<f32>, initial_cov: f32) -> Self {
      // let system_cov = &Stds {nn:0.19, no:0.001, oo:0.13, on:0.2,};
      // let belief = Belief { mean: *initial_state_mean, cov: initial_cov * na::Matrix3::identity()};
      Self{ distance_bias_rate_std: 0.1, direction_bias: 2.*PI*2./360. }
      // Self{ belief, system_cov, distance_bias_rate_std: 0.14, direction_bias: 0.05, pose_change: na::Vector3::zeros() }
  }

  pub fn observation_predict(&self, pose: &na::Vector3<f32>, landmark :&na::Vector3<f32>) -> na::Matrix2x1<f32> {
    let diff = landmark - pose;
    let mut phi = diff[1].atan2(diff[0]) - pose[2];
    while phi >= PI { phi -= 2. * PI; }
    while phi < -PI { phi += 2. * PI; }
    let mat = na::Matrix2x1::new(diff[0].hypot(diff[1]), phi);
    mat
  }

  pub fn noize(&self, obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
    let mat = Sensor::bias(&self, &obj_dis);
    mat
  }
  fn bias(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
    let mut mat = na::Matrix2x1::zeros();
    mat[0] = obj_dis[0] + obj_dis[0] * self.distance_bias_rate_std;
    mat[1] = obj_dis[1] + self.direction_bias;
    mat
  }



}