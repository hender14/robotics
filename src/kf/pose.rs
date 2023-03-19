use nalgebra as na;
use std::f32::consts::PI;

pub struct Stds {
    pub nn: f32,
    pub no: f32,
    pub on: f32,
    pub oo: f32,
}

pub fn mat_m(nu: f32, omega: f32, time: f32, stds: &Stds) -> na::Matrix2<f32> {
    let mat = na::Matrix2::new(
        stds.nn.powf(2.)*nu.abs()/time + stds.no.powf(2.)*omega.abs()/time,   0.,
        0.  , stds.on.powf(2.)*nu.abs()/time + stds.oo.powf(2.)*omega.abs()/time);
    mat
}

fn mat_a(nu: f32, omega: f32, time: f32, theta: f32) -> na::Matrix3x2<f32> {
    let st = theta.sin();
    let ct = theta.cos();
    let stw = (theta + omega * time).sin();
    let ctw = (theta + omega * time).cos();
    let mat = na::Matrix3x2::new(
        (-stw + st)/omega, (-nu/(omega.powf(2.0))*(stw - st))+(nu/omega*time*ctw), 
        (-ctw + ct)/omega, (-nu/(omega.powf(2.0))*(-ctw + ct))+(nu/omega*time*stw),
        0.               , time);
    mat
}

fn mat_f(nu: f32, omega: f32, time: f32, theta: f32) -> na::Matrix3<f32> {
    let mut mat = na::Matrix3::identity();
    mat[(0, 2)] = (nu/omega)*(theta + omega*time).cos() - (nu/omega)*theta.cos();
    mat[(1, 2)] = (nu/omega)*(theta + omega*time).sin() - (nu/omega)*theta.sin();
    mat
}

fn mat_h(pose: &na::Vector3<f32>, poseo: &na::Vector3<f32>) -> na::Matrix2x3<f32> {
    let mx = poseo[0];
    let my = poseo[1];
    let mux = pose[0];
    let muy = pose[1];
    let q = (mux - mx).powf(2.0) + (muy - my).powf(2.0);
    let mat = na::Matrix2x3::new(
        (mux - mx)/q.sqrt(), (muy - my)/q.sqrt(), 0.,
        (my - muy)/q       , (mux - mx)/q       , -1.0);
    mat
}

fn mat_q(distance_dev: f32, direction_dev: f32) -> na::Matrix2<f32> {
    let mat = na::Matrix2::new(
        distance_dev.powf(2.0), 0.,
        0., direction_dev.powf(2.0));
    mat
}

pub struct KalmanFilterPose <'a> {
    belief: Belief,
    system_cov: & 'a Stds,
    distance_dev_rate: f32,
    direction_dev: f32,
    pose_change: na::Vector3<f32>,
}

struct Belief {
    mean: na::Vector3<f32>,
    cov: na::Matrix3<f32>,
}

impl <'a> KalmanFilterPose <'a> {
    pub fn new( initial_state_mean: &na::Vector3<f32>, initial_cov: f32) -> Self {
        let system_cov = &Stds {nn:0.19, no:0.001, oo:0.13, on:0.2,};
        let belief = Belief { mean: *initial_state_mean, cov: initial_cov * na::Matrix3::identity()};
        Self{ belief, system_cov, distance_dev_rate: 0.14, direction_dev: 0.05, pose_change: na::Vector3::zeros() }
    }

    pub fn filter_predict(&self, nu: f32, omega: f32, time: f32) -> (na::Vector3<f32>, na::Matrix3<f32>){
        let m = mat_m(nu, omega, time, self.system_cov);
        let a = mat_a(nu, omega, time, self.belief.mean[2]);
        let f = mat_f(nu, omega, time, self.belief.mean[2]);
        let predicted_state_mean = f * (&self.belief.mean);
        let predicted_state_cov = f * (&self.belief.cov) * (&f.transpose()) + a * (&m) * (&a.transpose());
        (predicted_state_mean, predicted_state_cov)
    }

    pub fn filter_update(& mut self, predicted_state_mean: &na::Vector3<f32>, predicted_state_cov: &na::Matrix3<f32>, pose: &na::Vector3<f32>)-> (na::Vector3<f32>, na::Matrix3<f32>){
        let h = mat_h(&pose, &self.belief.mean);
        let estimated_z = self.observation_function(&self.belief.mean, &predicted_state_mean);
        // let z = na::Vector2::new(pose - self.belief.mean);
        let z = self.observation_function(&pose, &self.belief.mean);
        let q = mat_q(estimated_z[0]*self.distance_dev_rate, self.direction_dev);
        let kalman_gain = predicted_state_cov * (h.transpose()) * (h * predicted_state_cov * h.transpose() + q).try_inverse().unwrap();
        // let kalman_gain = predicted_state_cov * (&h.transpose()) * (&h * (&predicted_state_cov) * (&h.transpose()) + q).try_inverse().unwrap();
        let filtered_state_mean = predicted_state_mean + kalman_gain * (z - estimated_z);
        let filtered_state_cov = predicted_state_cov - ( kalman_gain * h * predicted_state_cov);
        self.pose_change = filtered_state_mean - self.belief.mean;
        self.belief.mean = filtered_state_mean;
        self.belief.cov = filtered_state_cov;
        (filtered_state_mean, filtered_state_cov)
    }

    pub fn observation_function(&self, poseo: &na::Vector3<f32>, pose_estimate: &na::Vector3<f32>) -> na::Matrix2x1<f32> {
        let diff = pose_estimate - poseo;
        let mut phi = diff[1].atan2(diff[0]) - poseo[2];
        while phi >= PI { phi -= 2. * PI; }
        while phi < -PI { phi += 2. * PI; }
        let mat = na::Matrix2x1::new(diff[0].hypot(diff[1]), phi);
        mat
    }

    // pub fn pose_estimate(& mut self, x: f32, y: f32, theta: f32, time: f32) -> na::Vector3<f32> {
    //     let pose = na::Vector3::new(x, y, theta);
    //     let nu = (pose[1] - self.belief.mean[1]).hypot(pose[0] - self.belief.mean[0])/time;
    //     let omega = theta / time;
    //     // フィルタリング分布取得
    //     let predicted_state = self.filter_predict(nu, omega, time);
    //     let filtered_state = self.filter_update(&predicted_state.0, &predicted_state.1, &pose);

    //     return filtered_state.0
    // }
}