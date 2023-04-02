use super::super::usecase::map_create as map;
use super::super::usecase::state_estimate as estimate;
use super::config;
use super::file;
use super::plot;

pub fn start_app() {
    /* init */
    let (lpose, landsize) = config::init();

    /* main task */
    let (time, nu, omega, pose, zres, zlist) = estimate::state_estimate(
        config::INIT_NU,
        config::INIT_OMEGA,
        config::TIME,
        config::INIT_POSE,
        lpose,
        landsize,
    );

    let (hat_xs, zlist, land) = map::slam(file::KFPATH);

    /* plot */
    plot::plot_kf(file::KFPATH, &lpose);
    plot::plot_slam(file::SLAMPATH, &land);
}
