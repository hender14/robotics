use super::config;
use super::file;
use super::plot;
use crate::usecase::map_create as map;
use crate::usecase::state_estimate as estimate;

pub fn start_app() {
    /* init */
    let landmarks_kf = config::init();

    /* main task */
    let (time, nu, omega, pose, _) = estimate::state_estimate(
        config::INIT_NU,
        config::INIT_OMEGA,
        config::TIME,
        config::INIT_POSE,
        &landmarks_kf,
    );

    let (hat_xs, zlist, landmarks_slam) = map::slam(file::KFPATH);

    /* plot */
    plot::plot_kf(file::KFPATH, &landmarks_kf);
    plot::plot_slam(file::SLAMPATH, &landmarks_slam);
}
