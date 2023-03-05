import numpy as np
from scipy.stats import multivariate_normal


def matM(stds):
    return np.diag([stds["nn"]**2 + stds["no"]**2, stds["on"]**2 + stds["oo"]**2])

def matF(change):
    F = np.diag([1.0, 1.0])
    F[1, 0] += change[0]
    F[1, 1] += change[1]
    return F

def matH(acc, acco): ###kf4funcs
    q = (acc[0] - acco[0])**2 + (acc[1] - acco[1])**2
    return np.array([(acc[0] - acco[0])/np.sqrt(q), (acc[1] - acco[1])/np.sqrt(q)])

def matQ(distance_dev, direction_dev):
    return np.diag(np.array([distance_dev**2, direction_dev**2]))

class KalmanFilter_Acc():
    def __init__(self, initial_state_mean, system_cov={"nn":0.19, "no":0.001, "on":0.13, "oo":0.2},):
        self.belief = multivariate_normal(mean=initial_state_mean, cov=np.diag([1e-10, 1e-10, 1e-10])) 
        self.system_cov = np.array(system_cov) #システムノイズの共分散行列
        self.distance_dev_rate=0.14
        self.direction_dev=0.05
        # self.initial_state_mean = np.array(initial_state_mean) #状態の初期値の平均ベクトル
        # self.initial_state_cov = np.array(initial_state_cov) #状態の初期値の共分散行列
        self.acc_change = np.zeros(initial_state_mean.size)

    def filter_predict(self):
        M = matM(self.system_cov)
        F = matF(self.acc_change)
        predicted_state_mean = F @ self.belief.mean
        predicted_state_cov = (F @ self.belief.cov @ (F.T) + M)

        return (predicted_state_mean, predicted_state_cov)

    def filter_update(self, predicted_state_mean, predicted_state_cov, acc):
        H = matH(acc, self.belief.mean)
        estimated_z = np.array(np.hypot([predicted_state_mean - self.belief.mean]) ).T
        z = np.array(np.hypot([acc - self.belief.mean]) )
        Q = matQ(estimated_z[0]*self.distance_dev_rate, self.direction_dev)
        kalman_gain = (predicted_state_cov @ (H.T) @ np.linalg.inv(H @ predicted_state_cov @ (H.T)+ Q) )
        filtered_state_mean = (predicted_state_mean + kalman_gain @ (z - estimated_z) )
        filtered_state_cov = (predicted_state_cov - (kalman_gain @ H @ predicted_state_cov) )
        self.acc_change = filtered_state_mean - self.belief.mean
        self.belief.mean = filtered_state_mean
        self.belief.cov = filtered_state_cov

        return (filtered_state_mean, filtered_state_cov)

    def acc_estimate(self, acc, x, y):
        # フィルタリング分布取得
        predicted_state_mean, predicted_state_cov = self.filter_predict()
        filtered_state_mean, filtered_state_cov = self.filter_update(predicted_state_mean, predicted_state_cov, acc)

        return filtered_state_mean