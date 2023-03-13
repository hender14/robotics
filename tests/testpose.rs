#[cfg(test)]
mod tests {
    //     use super::*;
    use robotics::kf::pose;
    use nalgebra as na;
    use std::f32::consts::PI;
    
    #[test]
    fn test_mat_m() {
        let ans = na::SMatrix::<f32, 2, 2>::new(
            0.07220017, 0.,
            0.        , 0.08294961
        );

        let nu = 2.;
        let omega = 2.*PI*10./360.;
        let time = 1.;
        let system_cov = pose::Stds {nn:0.19, no:0.001, oo:0.13, on:0.2,};
        // let result = pose::mat_m(nu, omega, time);
        let result = pose::mat_m(nu, omega, time ,&system_cov);
        println!("{}", result);  
        assert_eq!(result, ans);
    }
}