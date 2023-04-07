use nalgebra as na;

#[derive(Clone)]
pub struct Twist {
    pub nu: f32,
    pub omega: f32,
}

pub struct State {
    pub time: f32,
    pub pose: na::Vector3<f32>,
    pub velocity: Twist,
}

impl State {
    pub fn new(pose: na::Vector3<f32>, nu: f32, omega: f32) -> Self {
        Self {
            time: 0.,
            pose,
            velocity: Twist { nu, omega },
        }
    }

    pub fn calc_velocity(&mut self, nu: f32, omega: f32) {
        self.velocity = Twist { nu, omega };
    }

    pub fn update_time(&mut self, delta: f32) {
        self.time += delta;
    }
}

pub fn state_transition(velocity: &Twist, time: f32, pose: &na::Vector3<f32>) -> na::Vector3<f32> {
    let t0 = pose[2];
    if velocity.omega.abs() < 1e-10 {
        pose + na::Vector3::new(
            velocity.nu * t0.cos(),
            velocity.nu * t0.sin(),
            velocity.omega,
        ) * time
    } else {
        pose + na::Vector3::new(
            velocity.nu / velocity.omega * ((t0 + velocity.omega * time).sin() - t0.sin()),
            velocity.nu / velocity.omega * (-(t0 + velocity.omega * time).cos() + t0.cos()),
            velocity.omega * time,
        )
    }
}
