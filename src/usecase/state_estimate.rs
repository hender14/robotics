use super::super::domain::kalman_filter as kf;
use super::super::domain::localize as loc;
use super::super::infrastructure::config;
use super::super::infrastructure::file;
use super::super::infrastructure::sensor_data as sd;
use nalgebra as na;

pub fn state_estimate(
    nu: f32,
    omega: f32,
    delta: f32,
    init_pose: na::Vector3<f32>,
    lpose: [[f32; 3]; 6],
    landsize: usize,
) -> (f32, f32, f32, na::Vector3<f32>, [bool; 6], [[f32; 3]; 6]) {
    /* 初期値はインフラ層で定義すべき */
    let mut pose = init_pose;
    let mut nuo = config::INIT_NU; /* preliminary */
    let mut omegao = config::INIT_OMEGA; /* preliminary */
    let init_cov = config::INIT_COV;
    let mut zlist: [[f32; 3]; 6] = [[0.; 3]; 6];
    let mut zres: [bool; 6] = [false; 6];
    let mut obj_dis: na::Matrix2x6<f32> = na::Matrix2x6::zeros();
    let mut time = 0.;
    let mut kf_pose: na::Vector3<f32> = na::Vector3::zeros();
    /* create object */
    let mut kf = kf::KFilterPose::new(&pose, init_cov);

    // /* main loop */
    for _i in 0..config::LOOP_NUM {
        /* 初期化 */
        zlist = [[0.; 3]; 6];
        zres = [false; 6];

        /* update robot position */
        pose = loc::state_transition(nu, omega, delta, &pose);

        /* calculate landmark */
        /* calculate landmark distance/direct */
        /* nu, omega, timeについても構造体定義して受け取るべき */
        (obj_dis, zlist, zres) = sd::sensor_receive(&pose, &lpose, landsize);

        /* kalman filter */
        kf.kf_predict(nuo, omegao, delta);

        /* predict state transition */
        /* calculate kfiltered pose*/
        kf_pose = kf.kf_update(&obj_dis, &lpose, landsize);

        // /* update old value */
        time += delta;
        // /* nu, omegaの更新は? */
        (nuo, omegao) = (nu, omega);
        // // pose = pose;
        // // zlist = zlist;

        // /* file */
        file::pose_write(time, nu, omega, &kf_pose, &zres, &zlist);
    }
    (time, nu, omega, kf_pose, zres, zlist)
    /* slam */
    // let (hat_xs, zlist, land) = slamagent::slam(file::KFPATH);
}
