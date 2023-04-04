#[cfg(test)]
mod tests {
    use nalgebra as na;
    use robotics;
    use robotics::infrastructure::config;
    use robotics::usecase::state_estimate as estimate;
    use std::f32::consts::PI;

    #[test]
    fn test_kf() {
        /* init */
        config::init();

        /* Output specification */
        let ans = na::Vector3::new(0., 0., 2. * PI);

        /* input condition */
        let (nu, omega) = (0.2, 2. * PI * 10. / 360.);
        let delta = 1.;
        let landmarks = config::get_landmark();
        // let loop_num = 36;
        let pose = na::Vector3::zeros();

        /* initial */
        // let out = na::Vector3::<f32>::zeros();

        let (_, _, _, out, _) = estimate::state_estimate(nu, omega, delta, pose, &landmarks);

        /* validate */
        let res = validate(&ans, &out);
        /* test */
        assert!(res, "\nans:{}  out:{}", &ans, &out);
    }

    /* validate */
    fn validate(ans: &na::Vector3<f32>, out: &na::Vector3<f32>) -> bool {
        let mut flag = true;
        let len = ans.len();
        for i in 0..(len - 1) {
            if (ans[i] - out[i]).abs() > 0.2 {
                flag = false;
                break;
            }
        }
        flag
    }
}
