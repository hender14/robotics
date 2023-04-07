use crate::domain::{
    kalman_filter as kf,
    sensor_data::{Landmark, SensorData},
    state,
    state::{State, Twist},
};
use crate::infrastructure::{config, file, sensor_data::Sensor};
use nalgebra as na;

pub fn state_estimate(
    nu: f32,
    omega: f32,
    delta: f32, /* control period */
    pose: na::Vector3<f32>,
    landmarks: &[Landmark; 6],
) -> (na::Vector3<f32>, Vec<SensorData>) {
    /* 初期値はインフラ層で定義すべき */
    let init_cov = config::INIT_COV;
    let mut velocityo = Twist { nu, omega };

    /* create object */
    let mut robot = State::new(pose, nu, omega);
    let mut sensor = Sensor::new();
    let mut kf = kf::KFilterPose::new(&pose, init_cov);

    // /* main loop */
    for _i in 0..config::LOOP_NUM {
        /* update robot */
        /* update velocity */
        robot.calc_velocity(nu, omega);
        /* update robot position */
        robot.pose = state::state_transition(&robot.velocity, delta, &robot.pose);

        /* receive sensor data */
        sensor.sensor_receive(&robot.pose, landmarks);
        // sensor_data = sensor.sensor_receive(&pose, &landmarks);

        /* kalman filter */
        kf.kf_predict(velocityo, delta);

        /* predict state transition */
        /* calculate kalman filtered pose */
        kf.kf_update(&sensor.sensor_data, landmarks);

        /* file */
        file::pose_write(&robot, &kf.belief.mean, &sensor.sensor_data);

        /* update robot */
        robot.time += delta; /* update robot time */
        velocityo = robot.velocity.clone(); /* update robot old velocity */
    }
    (kf.belief.mean, sensor.sensor_data)

    /* slam */
    /* slamの結果をFBする */
    // let (hat_xs, zlist, land) = slamagent::slam(file::KFPATH);
}
