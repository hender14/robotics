use nalgebra as na;
use rand_distr::{Distribution, Exp, Normal, Uniform};
use std::f32::consts::PI;

/* Twist struct represents the robot's linear and angular velocity. */
#[derive(Clone, Debug)]
pub struct Twist {
    pub nu: f32,
    pub omega: f32,
}

/* State struct, represents the robot's current state. */
#[derive(Clone, Debug)]
pub struct State {
    pub time: f32,
    pub pose: na::Vector3<f32>,
    pub velocity: Twist,
}

/* create a new State with the given pose and velocity. */
pub fn new(pose: na::Vector3<f32>, nu: f32, omega: f32) -> State {
    State {
        time: 0.,
        pose,
        velocity: Twist { nu, omega },
    }
}

/* Robot struct, which represents the robot and its properties. */
pub struct Robot {
    pub state: State,
    r: f32,
    noise_pdf: Exp<f32>,
    theta_noise: Normal<f32>,
    bias_rate_nu: f32,
    bias_rate_omega: f32,
    escape_pdf: Exp<f32>,
    stuck_pdf: Exp<f64>,
    kidnap_range_x: (f32, f32),
    kidnap_range_y: (f32, f32),
    distance_until_noise: f32,
    is_stuck: bool,
    time_until_stuck: f64,
    time_until_escape: f32,
    kidnap_pdf: Exp<f64>,
    time_until_kidnap: f64,
}

impl Robot {
    /* create a new Robot with the given pose and velocity. */
    pub fn new(pose: na::Vector3<f32>, nu: f32, omega: f32) -> Self {
        let mut rng = rand::thread_rng();
        let noise_pdf = Exp::new(1.0 / (1e-100 + 5.)).unwrap();
        let theta_noise = Normal::new(0.0, 2. * PI * 3. / 360.).unwrap();
        let bias_rate_nu = Normal::new(1.0, 0.1).unwrap().sample(&mut rng);
        let bias_rate_omega = Normal::new(1.0, 0.1).unwrap().sample(&mut rng);

        let stuck_pdf = Exp::new(1.0e100).unwrap();
        let escape_pdf = Exp::new(1e-100).unwrap();
        let is_stuck = false;
        let time_until_stuck = stuck_pdf.sample(&mut rng);
        let time_until_escape = escape_pdf.sample(&mut rng);

        let kidnap_pdf = Exp::new(1.0e100).unwrap();
        let time_until_kidnap = kidnap_pdf.sample(&mut rng);
        Self {
            r: 0.2,
            state: new(pose, nu, omega),
            noise_pdf,
            theta_noise,
            bias_rate_nu,
            bias_rate_omega,
            escape_pdf,
            stuck_pdf,
            kidnap_range_x: (-5., 5.),
            kidnap_range_y: (-5., 5.),
            distance_until_noise: noise_pdf.sample(&mut rng),
            is_stuck,
            time_until_stuck,
            time_until_escape,
            kidnap_pdf,
            time_until_kidnap,
        }
    }

    /* update the robot's velocity with the given linear and angular velocity. */
    pub fn update_velocity(&mut self, mut nu: f32, mut omega: f32, delta: f32) {
        (nu, omega) = self.bias(nu, omega);
        // (nu, omega) = self.stuck(nu, omega, delta);
        self.state.velocity = Twist { nu, omega };
    }
    /* update the robot's state (pose) based on its current velocity and time interval. */
    pub fn update_state(&mut self, delta: f32) {
        self.state.pose = state_transition(&self.state.velocity, delta, &self.state.pose);
        self.noise(delta);
        // self.kidnap(delta);
    }

    /* update the robot's time with the given time interval. */
    pub fn update_time(&mut self, delta: f32) {
        self.state.time += delta;
    }

    /* apply bias to the robot's linear and angular velocity. */
    pub fn bias(&self, nu: f32, omega: f32) -> (f32, f32) {
        (nu * self.bias_rate_nu, omega * self.bias_rate_omega)
    }

    /* simulate the robot getting stuck or escaping. */
    pub fn stuck(&mut self, nu: f32, omega: f32, time_interval: f32) -> (f32, f32) {
        if self.is_stuck {
            self.time_until_escape -= time_interval;
            if self.time_until_escape <= 0.0 {
                let mut rng = rand::thread_rng();
                self.time_until_escape += self.escape_pdf.sample(&mut rng);
                self.is_stuck = false;
            }
        } else {
            self.time_until_stuck -= time_interval as f64;
            if self.time_until_stuck <= 0.0 {
                let mut rng = rand::thread_rng();
                self.time_until_stuck += self.stuck_pdf.sample(&mut rng);
                self.is_stuck = true;
            }
        }

        (
            nu * (!self.is_stuck as i32 as f32),
            omega * (!self.is_stuck as i32 as f32),
        )
    }

    /* add noise to the robot's orientation based on the distance traveled. */
    pub fn noise(&mut self, time_interval: f32) {
        self.distance_until_noise -= self.state.velocity.nu.abs() * time_interval
            + self.r * self.state.velocity.omega.abs() * time_interval;
        if self.distance_until_noise <= 0.0 {
            let mut rng = rand::thread_rng();
            self.distance_until_noise += self.noise_pdf.sample(&mut rng);
            self.state.pose[2] += self.theta_noise.sample(&mut rng);
        }
    }

    pub fn kidnap(&mut self, time_interval: f32) {
        self.time_until_kidnap -= time_interval as f64;
        if self.time_until_kidnap <= 0.0 {
            let mut rng = rand::thread_rng();
            self.time_until_kidnap += self.kidnap_pdf.sample(&mut rng);
            let kidnap_dist_x = Uniform::new(self.kidnap_range_x.0, self.kidnap_range_x.1);
            let kidnap_dist_y = Uniform::new(self.kidnap_range_y.0, self.kidnap_range_y.1);
            self.state.pose = na::Vector3::new(
                kidnap_dist_x.sample(&mut rng),
                kidnap_dist_y.sample(&mut rng),
                2. * PI,
            );
        }
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
