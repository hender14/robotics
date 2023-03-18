use nalgebra as na;
// use crate::kf::sensor::Sensor;
use super::sensor;
use super::kf;
use std::f32::consts::PI;

pub struct Agent {
  nuo: f32,
  omegao: f32,
  // distance_dev_rate: f32,
  // direction_dev: f32,
  // pose_change: na::Vector3<f32>,
}

impl Agent {
  pub fn new( ) -> Self {
  // pub fn new( ) -> Self {}
  // pub fn new( initial_state_mean: &na::Vector3<f32>, initial_cov: f32) -> Self {
      // let system_cov = &Stds {nn:0.19, no:0.001, oo:0.13, on:0.2,};
      // let belief = Belief { mean: *initial_state_mean, cov: initial_cov * na::Matrix3::identity()};
      Self{ nuo: 0.2, omegao: 2.*PI * 10./360. }
      // Self{ belief, system_cov, distance_bias_rate_std: 0.14, direction_bias: 0.05, pose_change: na::Vector3::zeros() }
  }

  pub fn pose_estimate( & mut self, pose: &na::Vector3<f32>, nu: f32, lpose: &na::Vector3<f32>, time: f32 ) -> na::Vector3<f32> {
    // let pose = na::Vector3::new(x, y, theta);
    // let nu = (pose[1] - self.belief.mean[1]).hypot(pose[0] - self.belief.mean[0])/time;
    let omega = pose[2] / time;
    // フィルタリング分布取得
    let sensor = sensor::Sensor::new();
    let mut obj_dis = sensor.observation_predict(&pose, &lpose);
    obj_dis = sensor.noize(&obj_dis);

    /* kf */
    let init_cov: f32 = 0.01;
    let mut kf = kf::KFilterPose::new(&pose, init_cov);
    kf.kf_predict(self.nuo, self.omegao, time);
    let kf_state = kf.kf_update(&obj_dis, &lpose);

    self.nuo = nu;
    self.omegao = omega; //nu, omegaの更新タイミングは？

    // robot_noise(&kf_state, time);
    // robot_move(&kf_state, time);
    // robot_noise2(&kf_state, time); //sensor処理にて代替
    kf_state
  }
}