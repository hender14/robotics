#[cfg(test)]
mod tests {
    use nalgebra as na;
    use robotics;
    use robotics::common::landmark;
    use robotics::kf::kfagent;
    use robotics::kf::kfilter;
    use std::f32::consts::PI;

    #[test]
    fn test_kf() {
        /* Output specification */
        let ans = na::Vector3::new(0., 0., 2. * PI);

        /* input condition */
        let (nu, omega) = (0.2, 2. * PI * 10. / 360.);
        let time = 1.;
        let (lpose, landsize) = landmark::dec_landmark();
        let loop_num = 36;
        let mut pose = na::Vector3::zeros();

        /* initial */
        let mut out = na::Vector3::zeros();

        /* create object */
        let mut agent = kfagent::Agent::new(&pose, landsize);

        /* main loop */
        for _i in 0..loop_num {
            /* update robot position */
            pose = kfilter::KFilterPose::state_transition(nu, omega, time, &pose);

            /* kalman filter */
            agent.pose_estimate(nu, omega, &lpose, time, &pose);
            out = agent.pose;
        }

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
            if (ans[i] - out[i]).abs() > 0.1 {
                flag = false;
            }
        }
        flag
    }
}
