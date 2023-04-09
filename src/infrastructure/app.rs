use super::{config, file, plot};
use crate::usecase::{map_create as map, state_estimate as estimate};

pub fn start_app() {
    /* init */
    let landmarks_kf = config::init();

    /* main task */
    /* kalmanfilter */
    let (_, _) = estimate::state_estimate(
        config::INIT_NU,
        config::INIT_OMEGA,
        config::TIME,
        config::INIT_POSE,
        &landmarks_kf,
    );

    /* slam */
    let (_, _, landmarks_slam) = map::slam(file::KF_PATH);

    /* plot */
    plot::plot_generic(file::KF_PATH, plot::KF_PATH, &landmarks_kf);
    plot::plot_generic(file::SLAM_PATH, plot::SLAM_PATH, &landmarks_slam);
}
