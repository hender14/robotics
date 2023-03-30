use nalgebra as na;


fn noise( pose: &mut [f32; 3], nu: f32, omega: f32, time_interval: f32) -> [f32; 3] {
    let distance_until_noise -= (nu.abs()*(time_interval)) + r*(omega.abs()*(time_interval));
    if distance_until_noise <= 0.0 {
        distance_until_noise += noise_pdf.rvs();
        pose[2] += theta_noise.rvs();
    }
    
    return pose
}

fn bias(&mut  nu: f32, omega: f32) -> (f32, f32) {
    return (nu*bias_rate_nu, omega*bias_rate_omega)
}

fn stuck(&mut  nu: f32, omega: f32, time_interval: f32) -> (f32, f32) {
    if is_stuck {
        time_until_escape -= time_interval;
        if time_until_escape <= 0.0 {
            time_until_escape += escape_pdf.rvs();
            is_stuck = false;
        }
    } else {
        time_until_stuck -= time_interval;
        if time_until_stuck <= 0.0 {
            time_until_stuck += stuck_pdf.rvs();
            is_stuck = true;
        }
    }
    
    return (nu*(not is_stuck), omega*(not is_stuck))
}

fn kidnap(&mut  pose: &mut [f32; 3], time_interval: f32) -> [f32; 3] {
    time_until_kidnap -= time_interval;
    if time_until_kidnap <= 0.0 {
        time_until_kidnap += kidnap_pdf.rvs();
        return kidnap_dist.rvs();
    } else {
        return pose;
    }
}