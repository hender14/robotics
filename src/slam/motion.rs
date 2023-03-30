use super::super::kf::kfilter;
// use super::sensor::SensorEdge;
use nalgebra as na;

pub struct MotionEdge {
    t1: usize,
    t2: usize,
    hat_x1: na::Vector3<f32>,
    hat_x2: na::Vector3<f32>,
    omega: na::Matrix3<f32>,
    omega_upperleft: na::Matrix3<f32>,
    omega_upperright: na::Matrix3<f32>,
    omega_bottomleft: na::Matrix3<f32>,
    omega_bottomright: na::Matrix3<f32>,
    xi_upper: na::Vector3<f32>,
    xi_bottom: na::Vector3<f32>,
}

impl MotionEdge {
    pub fn new(t1: usize, t2: usize, xs_vec: Vec<(f32, f32, f32)>) -> Self {
        let hat_x1 = na::Vector3::new(xs_vec[t1].0, xs_vec[t1].1, xs_vec[t1].2);
        let hat_x2 = na::Vector3::new(xs_vec[t2].0, xs_vec[t2].1, xs_vec[t2].2);

        Self {
            t1,
            t2,
            hat_x1,
            hat_x2,
            omega: Default::default(),
            omega_upperleft: Default::default(),
            omega_upperright: Default::default(),
            omega_bottomleft: Default::default(),
            omega_bottomright: Default::default(),
            xi_upper: Default::default(),
            xi_bottom: Default::default(),
        }
    }

    pub fn action_matrix(&mut self, us_vec: Vec<(f32, f32)>, delta: f32) {
        let motion_noise_stds = kfilter::Stds {
            nn: 0.19,
            no: 0.001,
            oo: 0.13,
            on: 0.2,
        };
        let (nu, mut omega) = (us_vec[self.t1].0, us_vec[self.t1].1);

        if omega.abs() < 1e-5 {
            omega = 1e-5;
        }

        let m = kfilter::mat_m(nu, omega, delta, &motion_noise_stds);
        let a = kfilter::mat_a(nu, omega, delta, self.hat_x1[2]);
        let f = kfilter::mat_f(nu, omega, delta, self.hat_x1[2]);

        self.omega = (a * m * a.transpose() + na::Matrix3::<f32>::identity() * 0.0001)
            .try_inverse()
            .unwrap();

        self.omega_upperleft = f.transpose() * self.omega * f;
        self.omega_upperright = -f.transpose() * self.omega;
        self.omega_bottomleft = self.omega * f;
        self.omega_bottomright = self.omega;

        let x2 = kfilter::KFilterPose::state_transition(nu, omega, delta, &self.hat_x1);
        self.xi_upper = f.transpose() * self.omega * (self.hat_x2 - x2);
        self.xi_bottom = -self.omega * (self.hat_x2 - x2);
    }
    pub fn add_edge(
        self,
        mut omega: na::DMatrix<f32>,
        mut xi: na::DVector<f32>,
    ) -> (na::DMatrix<f32>, na::DVector<f32>) {
        let f1 = self.t1 * 3;
        let f2 = self.t2 * 3;
        let t1 = f1 + 3;
        let t2 = f2 + 3;

        omega = Self::add_edge_omega(omega, &self.omega_upperleft, f1, f1, t1, t1);
        omega = Self::add_edge_omega(omega, &self.omega_upperright, f1, f2, t1, t2);
        omega = Self::add_edge_omega(omega, &self.omega_bottomleft, f2, f1, t2, t1);
        omega = Self::add_edge_omega(omega, &self.omega_bottomright, f2, f2, t2, t2);

        xi = Self::add_edge_xi(xi, &self.xi_upper, f1, t1);
        xi = Self::add_edge_xi(xi, &self.xi_bottom, f2, t2);

        (omega, xi)
    }

    fn add_edge_omega(
        mut omega: na::DMatrix<f32>,
        omega_part: &na::Matrix3<f32>,
        f1: usize,
        f2: usize,
        t1: usize,
        t2: usize,
    ) -> na::DMatrix<f32> {
        for i in f1..t1 {
            for j in f2..t2 {
                omega[(i, j)] += omega_part[(i - f1, j - f2)];
            }
        }
        omega
    }

    fn add_edge_xi(
        mut xi: na::DVector<f32>,
        xi_upper: &na::Vector3<f32>,
        f1: usize,
        t1: usize,
    ) -> na::DVector<f32> {
        for i in f1..t1 {
            xi[i] += xi_upper[i - f1];
        }
        xi
    }
}
