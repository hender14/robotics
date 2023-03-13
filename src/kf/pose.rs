use nalgebra as na;
// use std::f64::consts::PI as π;

// use ndarray::prelude::*;
// use ndarray_linalg::solve::Inverse;

pub struct Stds {
    pub nn: f32,
    pub no: f32,
    pub on: f32,
    pub oo: f32,
}
struct Pose {
    x: f32,
    y: f32,
    theta: f32,
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
    let mut mat = na::Matrix3::new(1., 0., 0.,
                                                                       0., 1., 0.,
                                                                       0., 0., 1.);
    mat[(0, 2)] = (nu/omega)*(theta + omega*time).cos() - (nu/omega)*theta.cos();
    mat[(1, 2)] = (nu/omega)*(theta + omega*time).sin() - (nu/omega)*theta.sin();
    mat
}

fn mat_h(pose: &Pose, poseo: &Pose) -> na::Matrix2x3<f32> {
    let mx = poseo.x;
    let my = poseo.y;
    let mux = pose.x;
    let muy = pose.y;
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

// pub struct KalmanFilter_Pose{
//     belief: MultivariateNormal,
//     system_cov: Array2<f64>,
//     distance_dev_rate: f64,
//     direction_dev: f64,
// }

// impl KalmanFilter_Pose{
//     pub fn new(initial_state_mean: Array1<f64>, system_cov: Array2<f64>) -> Self{
//         let belief = MultivariateNormal::new(initial_state_mean, Array2::eye(3).into_dyn()).unwrap();
//         Self{belief, system_cov, distance_dev_rate: 0.14, direction_dev: 0.05}
//     }

//     pub fn filter_predict(&self, nu: f64, omega: f64, time: f64) -> (Array1<f64>, Array2<f64>){
//         let (M, A, F) = matM(nu, omega, time, self.system_cov);
//         let predicted_state_mean = F.dot(&self.belief.mean());
//         let predicted_state_cov = F.dot(&self.belief.covariance()).dot(&F.t()) + A.dot(&M).dot(&A.t());
//         (predicted_state_mean, predicted_state_cov)
//     }

//     pub fn filter_update(&self, predicted_state_mean: Array1<f64>, predicted_state_cov: Array2<f64>, pose: Array1<f64>){
//         let H = matH(pose.slice(s![..1]), self.belief.mean().slice(s![..1]));
//         let estimated_z = self.observation_function(self.belief.mean(), predicted_state_mean);
//         let z = (pose.slice(s![..1]) - self.belief.mean().slice(s![..1])).mapv(|x| x.hypot(0.));
//         let Q = matQ(estimated_z[0]*self.distance_dev_rate, self.direction_dev);
//         let kalman_gain = predicted_state_cov.dot(&H.t()).dot(&(H.dot(&predicted_state_cov).dot(&H.t()) + Q).inv().unwrap());
//         let filtered_state_mean = predicted_state_mean + kalman_gain.dot(&(z - estimated_z));
//         let filtered_state_cov = predicted_state_cov - kalman_gain.dot(&H).dot(&predicted_state_cov);
//         self.acc_change = filtered_state_mean - self.belief.mean();
//         self.belief.mean = filtered_state_mean;
//         self.belief.cov = filtered_state_cov;
//         (filtered_state_mean, filtered_state_cov)
//     }

//     pub fn observation_function(&self, poseo: &Vec<f64>, pose_estimate: &Vec<f64>) -> Vec<f64> {
//         let diff = pose_estimate - poseo[0:2];
//         let phi = math.atan2(diff[1], diff[0]) - poseo[2];
//         // while phi >= np.pi: phi -= 2*np.pi
//         // while phi < -np.pi: phi += 2*np.pi
//         return np.array( [np.hypot(*diff), phi ] ).T
    

//     pub fn pose_estimate(&self, x: f64, y: f64, theta: f64, time: f64) -> Vec<f64> {
//         let pose = (x, y, theta);
//         let nu = (np.hypot(pose[:1] - self.belief.mean[:1]))/time
//         let omega = theta / time
//         // フィルタリング分布取得
//         let predicted_state_mean, predicted_state_cov = self.filter_predict(nu, omega, time)
//         let filtered_state_mean, filtered_state_cov = self.filter_update(predicted_state_mean, predicted_state_cov, pose)

//         return filtered_state_mean
// }


