import numpy as np
from scipy.stats import multivariate_normal
import math


def matM(nu, omega, time, stds):
    return np.diag([stds["nn"]**2*abs(nu)/time + stds["no"]**2*abs(omega)/time, 
                    stds["on"]**2*abs(nu)/time + stds["oo"]**2*abs(omega)/time])
    
def matA(nu, omega, time, theta):
    st, ct = math.sin(theta), math.cos(theta)
    stw, ctw = math.sin(theta + omega*time), math.cos(theta + omega*time)
    return np.array([[(stw - st)/omega,    -nu/(omega**2)*(stw - st) + nu/omega*time*ctw],
                    [(-ctw + ct)/omega, -nu/(omega**2)*(-ctw + ct) + nu/omega*time*stw],
                    [0,                                time]] )

def matF(nu, omega, time, theta):
    F = np.diag([1.0, 1.0, 1.0])
    F[0, 2] = nu / omega * (math.cos(theta + omega * time) - math.cos(theta))
    F[1, 2] = nu / omega * (math.sin(theta + omega * time) - math.sin(theta))
    return F

def matH(pose, poseo): ###kf4funcs
    mx, my = poseo
    mux, muy = pose
    q = (mux - mx)**2 + (muy - my)**2
    return np.array([[(mux - mx)/np.sqrt(q), (muy - my)/np.sqrt(q), 0.0],  [(my - muy)/q, (mux - mx)/q, -1.0]])

def matQ(distance_dev, direction_dev):
    return np.diag(np.array([distance_dev**2, direction_dev**2]))


class KalmanFilter_Pose():
    def __init__(self, initial_state_mean, acc, system_cov={"nn":0.19, "no":0.001, "on":0.13, "oo":0.2},):
        self.belief = multivariate_normal(mean=initial_state_mean, cov=np.diag([1e-10, 1e-10, 1e-10])) 
        self.system_cov = np.array(system_cov) #システムノイズの共分散行列
        self.distance_dev_rate=0.14
        self.direction_dev=0.05
        # self.initial_state_mean = np.array(initial_state_mean) #状態の初期値の平均ベクトル
        # self.initial_state_cov = np.array(initial_state_cov) #状態の初期値の共分散行列
        self.acc_change = np.zeros(acc.size)

        
    def filter_predict(self, nu, omega, time):
        M = matM(nu, omega, time, self.system_cov)
        A = matA(nu, omega, time, self.belief.mean[2])
        F = matF(nu, omega, time, self.belief.mean[2])
        predicted_state_mean = F @ self.belief.mean
        predicted_state_cov = (F @ self.belief.cov @ (F.T) + A @ M @ (A.T))

        return (predicted_state_mean, predicted_state_cov)

    def filter_update(self, predicted_state_mean, predicted_state_cov, pose):
        H = matH(pose[:1], self.belief.mean[:1])
        estimated_z = self.observation_function(self.belief.mean, predicted_state_mean)
        z = np.array(np.hypot([pose[:1] - self.belief.mean[:1]]) )
        Q = matQ(estimated_z[0]*self.distance_dev_rate, self.direction_dev)
        kalman_gain = (predicted_state_cov @ (H.T) @ np.linalg.inv(H @ predicted_state_cov @ (H.T)+ Q) )
        filtered_state_mean = (predicted_state_mean + kalman_gain @ (z - estimated_z) )
        filtered_state_cov = (predicted_state_cov - (kalman_gain @ H @ predicted_state_cov) )
        self.acc_change = filtered_state_mean - self.belief.mean
        self.belief.mean = filtered_state_mean
        self.belief.cov = filtered_state_cov

        return (filtered_state_mean, filtered_state_cov)
        
    def observation_function(poseo, pose_estimate):
        diff = pose_estimate - poseo[0:2]
        phi = math.atan2(diff[1], diff[0]) - poseo[2]
        # while phi >= np.pi: phi -= 2*np.pi
        # while phi < -np.pi: phi += 2*np.pi
        return np.array( [np.hypot(*diff), phi ] ).T
    

    def pose_estimate(self, x, y, theta, time):
        pose = (x, y, theta)
        nu = (np.hypot(pose[:1] - self.belief.mean[:1]))/time
        omega = theta / time
        # フィルタリング分布取得
        predicted_state_mean, predicted_state_cov = self.filter_predict(nu, omega, time)
        filtered_state_mean, filtered_state_cov = self.filter_update(predicted_state_mean, predicted_state_cov, pose)

        return filtered_state_mean