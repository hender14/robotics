// pub fn add(left: usize, right: usize) -> usize {
//     left + right
// }


// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {
//         let result = add(2, 2);
//         assert_eq!(result, 4);
//     }
// }

use ndarray::prelude::*;
use ndarray::Array;
use ndarray_linalg::solve::Inverse;
use ndarray_linalg::solve::Solve;
use ndarray_linalg::types::c64;
use ndarray_linalg::Lapack;

fn matM(stds: &[f64; 4]) -> Array<f64, Ix2> {
    let mut mat = Array::eye(2);
    mat[[0, 0]] = stds[0].powi(2) + stds[1].powi(2);
    mat[[1, 1]] = stds[2].powi(2) + stds[3].powi(2);
    mat
}

fn matF(change: &[f64; 2]) -> Array<f64, Ix2> {
    let mut mat = Array::eye(2);
    mat[[1, 0]] = change[0];
    mat[[1, 1]] = change[1];
    mat
}

fn matH(acc: &[f64; 2], acco: &[f64; 2]) -> Array<f64, Ix2> {
    let q = (acc[0] - acco[0]).powi(2) + (acc[1] - acco[1]).powi(2);
    let mut mat = Array::eye(2);
    mat[[0, 0]] = (acc[0] - acco[0]) / q.sqrt();
    mat[[1, 1]] = (acc[1] - acco[1]) / q.sqrt();
    mat
}

fn matQ(distance_dev: f64, direction_dev: f64) -> Array<f64, Ix2> {
    let mut mat = Array::eye(2);
    mat[[0, 0]] = distance_dev.powi(2);
    mat[[1, 1]] = direction_dev.powi(2);
    mat
}

struct KalmanFilter_Acc {
    belief: Array<f64, Ix2>,
    system_cov: [f64; 4],
    distance_dev_rate: f64,
    direction_dev: f64,
    acc_change: [f64; 2],
}

fn filter_predict(&mut self) -> (Array<f64, Ix2>, Array<f64, Ix2>) {
    let M = matM(&self.system_cov);
    let F = matF(&self.acc_change);
    let predicted_state_mean = F.dot(&self.belief);
    let predicted_state_cov = (F.dot(&self.belief).dot(&F.t()) + M).inv().unwrap();
    (predicted_state_mean, predicted_state_cov)
}

fn filter_update(&mut self, predicted_state_mean: f64, predicted_state_cov: f64, acc: f64) -> (f64, f64) {
    let H = matH(acc, self.belief.mean);
    let estimated_z = np.array(np.hypot([predicted_state_mean - self.belief.mean]) ).T;
    let z = np.array(np.hypot([acc - self.belief.mean]) );
    let Q = matQ(estimated_z[0]*self.distance_dev_rate, self.direction_dev);
    let kalman_gain = (predicted_state_cov * (H.T) * np.linalg.inv(H * predicted_state_cov * (H.T)+ Q) );
    let filtered_state_mean = (predicted_state_mean + kalman_gain * (z - estimated_z) );
    let filtered_state_cov = (predicted_state_cov - (kalman_gain * H * predicted_state_cov) );
    self.acc_change = filtered_state_mean - self.belief.mean;
    self.belief.mean = filtered_state_mean;
    self.belief.cov = filtered_state_cov;

    return (filtered_state_mean, filtered_state_cov);
}

fn acc_estimate(&mut self, acc: f64, x: f64, y: f64) -> f64 {
    // フィルタリング分布取得
    let predicted_state = self.filter_predict();
    let filtered_state = self.filter_update(predicted_state_mean, predicted_state_cov, acc);

    return filtered_state_mean;
}
