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

  def filter_predict(self, current_state_mean, current_state_cov):
      predicted_state_mean = self.system_matrix @ current_state_mean
      predicted_state_cov = (
          self.system_matrix
          @ current_state_cov
          @ self.system_matrix.T
          + self.system_cov
      )
      return (predicted_state_mean, predicted_state_cov)

  def filter_update(self, predicted_state_mean, predicted_state_cov, observation):
      kalman_gain = (
          predicted_state_cov 
          @ self.observation_matrix.T 
          @ np.linalg.inv(
              self.observation_matrix
              @ predicted_state_cov
              @ self.observation_matrix.T
              + self.observation_cov
          )
      )
      filtered_state_mean = (
          predicted_state_mean
          + kalman_gain
          @ (observation
            - self.observation_matrix
            @ predicted_state_mean)
      )
      filtered_state_cov = (
          predicted_state_cov
          - (kalman_gain
            @ self.observation_matrix
            @ predicted_state_cov)
      )

  def filter(self, observations):
      observations = np.array(observations)
      
      n_timesteps = len(observations)
      n_dim_state = len(self.initial_state_mean)
      
      predicted_state_means = np.zeros((n_timesteps, n_dim_state))
      predicted_state_covs = np.zeros((n_timesteps, n_dim_state, n_dim_state))
      filtered_state_means = np.zeros((n_timesteps, n_dim_state))
      filtered_state_covs = np.zeros((n_timesteps, n_dim_state, n_dim_state))
      
      for t in range(n_timesteps):
          if t == 0:
              predicted_state_means[t] = self.initial_state_mean
              predicted_state_covs[t] = self.initial_state_cov
          else:
              predicted_state_means[t], predicted_state_covs[t] = self.filter_predict(
                  filtered_state_means[t-1],
                  filtered_state_covs[t-1]
              )
          filtered_state_means[t], filtered_state_covs[t] = self.filter_update(
              predicted_state_means[t],
              predicted_state_covs[t],
              observations[t]
          )
      
      return (
          filtered_state_means,
          filtered_state_covs,
          predicted_state_means,
          predicted_state_covs
      )