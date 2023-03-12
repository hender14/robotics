import numpy as np

class KalmanFilter(object):
  def __init__(self, system_matrix, observation_matrix,
                system_cov, observation_cov,
                initial_state_mean, initial_state_cov,):
      self.system_matrix = np.array(system_matrix) #システム行列
      self.observation_matrix = np.array(observation_matrix) #観測行列
      self.system_cov = np.array(system_cov) #システムノイズの共分散行列
      self.observation_cov = np.array(observation_cov) #観測ノイズの共分散行列
      self.initial_state_mean = np.array(initial_state_mean) #状態の初期値の平均ベクトル
      self.initial_state_cov = np.array(initial_state_cov) #状態の初期値の共分散行列

  def predict(self, n_timesteps, observations):
      (filtered_state_means,
      filtered_state_covs,
      predicted_state_means,
      predicted_state_covs) = self.filter(observations)

      _, n_dim_state = filtered_state_means.shape

      predicted_state_means = np.zeros((n_timesteps, n_dim_state))
      predicted_state_covs = np.zeros((n_timesteps, n_dim_state, n_dim_state))

      for t in range(n_timesteps):
          if t == 0:
              predicted_state_means[t], predicted_state_covs[t] = self.filter_predict(
                  filtered_state_means[-1],
                  filtered_state_covs[-1]
              )
          else:
              predicted_state_means[t], predicted_state_covs[t] = self.filter_predict(
                  predicted_state_means[t-1],
                  predicted_state_covs[t-1]
              )

      return (predicted_state_means, predicted_state_covs)

  def smooth_update(self, filtered_state_mean, filtered_state_cov,
                      predicted_state_mean, predicted_state_cov,
                      next_smoothed_state_mean, next_smoothed_state_cov):
      kalman_smoothing_gain = (
          filtered_state_cov
          @ self.system_matrix.T
          @ np.linalg.inv(predicted_state_cov)
      )
      smoothed_state_mean = (
          filtered_state_mean
          + kalman_smoothing_gain
          @ (next_smoothed_state_mean - predicted_state_mean)
      )
      smoothed_state_cov = (
          filtered_state_cov
          + kalman_smoothing_gain
          @ (next_smoothed_state_cov - predicted_state_cov)
          @ kalman_smoothing_gain.T
      )
      return (smoothed_state_mean, smoothed_state_cov)

  def smooth(self, observations):
      (filtered_state_means,
      filtered_state_covs,
      predicted_state_means,
      predicted_state_covs) = self.filter(observations)

      n_timesteps, n_dim_state = filtered_state_means.shape

      smoothed_state_means = np.zeros((n_timesteps, n_dim_state))
      smoothed_state_covs = np.zeros((n_timesteps, n_dim_state, n_dim_state))

      smoothed_state_means[-1] = filtered_state_means[-1]
      smoothed_state_covs[-1] = filtered_state_covs[-1]

      for t in reversed(range(n_timesteps - 1)):
          smoothed_state_means[t], smoothed_state_covs[t] = self.smooth_update(
              filtered_state_means[t],
              filtered_state_covs[t],
              predicted_state_means[t+1],
              predicted_state_covs[t+1],
              smoothed_state_means[t+1],
              smoothed_state_covs[t+1]
          )
      
      return (smoothed_state_means, smoothed_state_covs)