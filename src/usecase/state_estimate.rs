use crate::domain::kalman_filter as kf;
use crate::domain::localize as loc;
use crate::domain::sensor_data::{Landmark, SensorData};
use crate::infrastructure::sensor_data::Sensor;
use crate::infrastructure::{config, file};
use nalgebra as na;

pub fn state_estimate<'a>(
    nu: f32,
    omega: f32,
    delta: f32,
    init_pose: na::Vector3<f32>,
    landmarks: &[Landmark; 6],
) -> (f32, f32, f32, na::Vector3<f32>, Vec<SensorData>) {
    /* 初期値はインフラ層で定義すべき */
    let mut pose = init_pose;
    let mut nuo = config::INIT_NU; /* preliminary */
    let mut omegao = config::INIT_OMEGA; /* preliminary */
    let init_cov = config::INIT_COV;
    let mut time = 0.;
    let mut kf_pose: na::Vector3<f32> = na::Vector3::zeros();

    /* create object */
    let mut sensor = Sensor::new();
    let mut kf = kf::KFilterPose::new(&pose, init_cov);

    // /* main loop */
    for _i in 0..config::LOOP_NUM {
        pose = loc::state_transition(nu, omega, delta, &pose);
        /* receive sensor data */
        sensor.sensor_receive(&pose, &landmarks);
        // sensor_data = sensor.sensor_receive(&pose, &landmarks);

        /* kalman filter */
        kf.kf_predict(nuo, omegao, delta);

        /* predict state transition */
        /* calculate kfiltered pose */
        kf_pose = kf.kf_update(&sensor.sensor_data, &landmarks);

        // /* file */
        file::pose_write(time, nu, omega, &kf_pose, &sensor.sensor_data);

        /* nu, omega, timeについても構造体定義して受け取るべき */
        // /* nu, omegaの更新は? */
        (nuo, omegao) = (nu, omega);
        /* update robot time */
        time += delta;
        /* update robot position */
    }
    (time, nu, omega, kf_pose, sensor.sensor_data)

    /* slam */
    // let (hat_xs, zlist, land) = slamagent::slam(file::KFPATH);
}
