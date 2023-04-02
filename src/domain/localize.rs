use nalgebra as na;

pub fn state_transition(
    nu: f32,
    omega: f32,
    time: f32,
    pose: &na::Vector3<f32>,
) -> na::Vector3<f32> {
    let t0 = pose[2];
    if omega.abs() < 1e-10 {
        return pose + na::Vector3::new(nu * t0.cos(), nu * t0.sin(), omega) * time;
    } else {
        return pose
            + na::Vector3::new(
                nu / omega * ((t0 + omega * time).sin() - t0.sin()),
                nu / omega * (-(t0 + omega * time).cos() + t0.cos()),
                omega * time,
            );
    }
}
