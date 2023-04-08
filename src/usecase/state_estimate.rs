use crate::domain::{
    kalman_filter as kf,
    sensor_data::{Landmark, SensorData},
    state::{Robot, Twist},
};
use crate::infrastructure::{config, file, sensor_data::Sensor};
use nalgebra as na;

pub fn state_estimate(
    nu: f32,
    omega: f32,
    delta: f32, /* control period */
    pose: na::Vector3<f32>,
    landmarks: &[Landmark; 6],
) -> (na::Vector3<f32>, [SensorData; 6]) {
    /* 初期値はインフラ層で定義すべき */
    let init_cov = config::INIT_COV;
    let mut velocityo = Twist { nu, omega };

    /* create object */
    let mut robot = Robot::new(pose, nu, omega);
    let mut sensor_data: [SensorData; 6] = Sensor::data_init();
    let mut kf = kf::KFilterPose::new(&pose, init_cov);
    let mut sensor = Sensor::new();

    // /* main loop */
    for _i in 0..config::LOOP_NUM {
        /* update robot */
        /* update velocity */
        robot.update_velocity(nu, omega, delta);
        /* update robot position */
        robot.update_state(delta);

        /* receive sensor data */
        sensor_data = sensor.sensor_receive(&robot.state.pose, landmarks);

        /* kalman filter */
        kf.kf_predict(velocityo, delta);

        /* predict state transition */
        /* calculate kalman filtered pose */
        kf.kf_update(&sensor_data, landmarks);

        /* file */
        file::pose_write(&robot.state, &kf.belief.mean, &sensor_data);

        /* update robot */
        robot.update_time(delta); /* update robot time */
        velocityo = robot.state.velocity.clone(); /* update robot old velocity */
    }
    (kf.belief.mean, sensor_data)

    /* slam */
    /* slamの結果をFBする */
    // let (hat_xs, zlist, land) = slamagent::slam(file::KFPATH);
}
