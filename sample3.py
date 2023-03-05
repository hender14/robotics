from sample1 import *
from sample2 import *

np.random.seed(1) # 乱数固定

n_timesteps = 120

system_matrix = [[1]]
observation_matrix = [[1]]
system_cov = [[1]]
observation_cov = [[10]]
initial_state_mean = [0]
initial_state_cov = [[1e7]]


states = np.zeros(n_timesteps)
observations = np.zeros(n_timesteps)
system_noise = np.random.multivariate_normal(
    np.zeros(1),
    system_cov,
    n_timesteps
)
observation_noise = np.random.multivariate_normal(
    np.zeros(1),
    observation_cov,
    n_timesteps
)

states[0] = initial_state_mean + system_noise[0]
observations[0] = states[0] + observation_noise[0]
for t in range(1, n_timesteps):
    states[t] = states[t-1] + system_noise[t]
    observations[t] = states[t] + observation_noise[t]

# 分析するデータ(時刻0から99まで)と予測と比較する用のデータ(時刻100以降)に分けておく
states_train = states[:100]
states_test = states[100:]
observations_train = observations[:100]
observations_test = observations[100:]

model = KalmanFilter(
    system_matrix,
    observation_matrix,
    system_cov,
    observation_cov,
    initial_state_mean,
    initial_state_cov
)

# フィルタリング分布取得
filtered_state_means, filtered_state_covs, _, _ = model.filter(observations_train)

# 予測分布取得
predicted_state_means, predicted_state_covs = model.predict(len(states_test), observations_train)

# 平滑化分布取得
smoothed_state_means, smoothed_state_covs = model.smooth(observations_train)