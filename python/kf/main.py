from kf.acc import KalmanFilter_Acc
from kf.pose import KalmanFilter_Pose
import numpy as np

if __name__ == '__main__': 

    # create object
    model_acc = KalmanFilter_Acc((np.zeros(2)))
    model_pose = KalmanFilter_Pose((np.zeros(3)), (np.zeros(3)))

    for i in range ():
        # sensing result receive
        acc = imu_sensing()
        x, y, theta, camera_time = camera_sensing()

        # localization, not forgetting reseting
        # (1)acc estimate
        acc = model_acc.acc_estimate(acc, x, y)
        # (2)angle estimate
        angle = angle_estimate(theta, acc)
        # (3)pose estimate
        pose = model_pose.pose_estimate(x, y, angle, camera_time)

        # planning(goal, checkpoint, object)
        planning()

        # drawing, debugging
        drawing()