use super::{
    constraint::Constraint,
    state::{self, Twist},
    utils as ut,
};
use nalgebra as na;

/* MotionEdge struct represents a motion edge in the Graph SLAM problem. */
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
    /* Creates a new MotionEdge. */
    pub fn new(t1: usize, t2: usize, state_vec: &[(f32, f32, f32)]) -> Self {
        let hat_x1 = na::Vector3::new(state_vec[t1].0, state_vec[t1].1, state_vec[t1].2);
        let hat_x2 = na::Vector3::new(state_vec[t2].0, state_vec[t2].1, state_vec[t2].2);

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

    /* Computes the action matrix for the MotionEdge based on the given velocity vector and time delta. */
    pub fn calc_action_matrix(&mut self, velocity_vec: &[Twist], delta: f32) {
        let motion_noise_stds = ut::MotionNoise {
            nn: 0.19,
            no: 0.001,
            oo: 0.13,
            on: 0.2,
        };
        let velocity = velocity_vec[self.t1].clone();
        let (nu, mut omega) = (velocity.nu, velocity.omega);

        if omega.abs() < 1e-5 {
            omega = 1e-5;
        }

        let m = ut::motion_noise_cov(nu, omega, delta, &motion_noise_stds);
        let a = ut::state_trans_model(nu, omega, delta, self.hat_x1[2]);
        let f = ut::state_trans_jac(nu, omega, delta, self.hat_x1[2]);

        /* Compute omega matrix and its partitions */
        self.omega = (a * m * a.transpose() + na::Matrix3::<f32>::identity() * 0.0001)
            .try_inverse()
            .unwrap();

        self.omega_upperleft = f.transpose() * self.omega * f;
        self.omega_upperright = -f.transpose() * self.omega;
        self.omega_bottomleft = self.omega * f;
        self.omega_bottomright = self.omega;

        /* Compute xi vector */
        let x2 = state::state_transition(&velocity, delta, &self.hat_x1);
        self.xi_upper = f.transpose() * self.omega * (self.hat_x2 - x2);
        self.xi_bottom = -self.omega * (self.hat_x2 - x2);
    }

    /* Add the current edge to the global information matrix and vector */
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

    /* Helper function to add the current edge to the information matrix */
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

    /* Helper function to add the current edge to the information vector */
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

/* The SensorEdge structure represents information about robot pose and landmark observations */
/* using sensor data at two times t1 and t2. */
pub struct SensorEdge {
    t1: usize,
    t2: usize,
    x1: na::Vector3<f32>,
    x2: na::Vector3<f32>,
    z1: na::Vector3<f32>,
    z2: na::Vector3<f32>,
    omega_upperleft: na::Matrix3<f32>,
    omega_upperright: na::Matrix3<f32>,
    omega_bottomleft: na::Matrix3<f32>,
    omega_bottomright: na::Matrix3<f32>,
    xi_upper: na::Vector3<f32>,
    xi_bottom: na::Vector3<f32>,
}

impl SensorEdge {
    /* Constructor for SensorEdge struct */
    pub fn new(
        t1: usize,
        t2: usize,
        z1_cr: Constraint,
        z2_cr: Constraint,
        state_vec: &[(f32, f32, f32)],
    ) -> Self {
        assert_eq!(z1_cr.id, z2_cr.id);
        let x1 = na::Vector3::new(state_vec[t1].0, state_vec[t1].1, state_vec[t1].2);
        let x2 = na::Vector3::new(state_vec[t2].0, state_vec[t2].1, state_vec[t2].2);
        let z1 = na::Vector3::new(z1_cr.data.polor[0], z1_cr.data.polor[1], z1_cr.data.psi);
        let z2 = na::Vector3::new(z2_cr.data.polor[0], z2_cr.data.polor[1], z2_cr.data.psi);

        Self {
            t1,
            t2,
            x1,
            x2,
            z1,
            z2,
            omega_upperleft: Default::default(),
            omega_upperright: Default::default(),
            omega_bottomleft: Default::default(),
            omega_bottomright: Default::default(),
            xi_upper: Default::default(),
            xi_bottom: Default::default(),
        }
    }

    /* Calculate the precision matrix for the constraint */
    pub fn calc_precision_matrix(&mut self) {
        let sensor_noise_rate = [0.14, 0.05, 0.05];
        let s1 = (self.x1[2] + self.z1[1]).sin();
        let c1 = (self.x1[2] + self.z1[1]).cos();
        let s2 = (self.x2[2] + self.z2[1]).sin();
        let c2 = (self.x2[2] + self.z2[1]).cos();

        /* Calculate error between expected and measured states */
        let mut hat_e = self.x2 - self.x1
            + na::Vector3::new(
                self.z2[0] * c2 - self.z1[0] * c1,
                self.z2[0] * s2 - self.z1[0] * s1,
                self.z2[1] - self.z2[2] - self.z1[1] + self.z1[2],
            );
        /* normalize states */
        hat_e[2] = ut::normalize_angle(hat_e[2]);

        /* Define noise covariance matrices q1 and q2 */
        let q1_diag = na::Matrix3x1::new(
            (self.z1[0] * sensor_noise_rate[0]).powi(2),
            sensor_noise_rate[1].powi(2),
            sensor_noise_rate[2].powi(2),
        );
        let q1 = na::Matrix3::from_diagonal(&q1_diag);
        let r1 = -na::Matrix3::new(
            c1,
            -self.z1[0] * s1,
            0.,
            s1,
            self.z1[0] * c1,
            0.,
            0.,
            1.,
            -1.,
        );

        let q2_diag = na::Vector3::new(
            (self.z2[0] * sensor_noise_rate[0]).powi(2),
            sensor_noise_rate[1].powi(2),
            sensor_noise_rate[2].powi(2),
        );
        let q2 = na::Matrix3::from_diagonal(&q2_diag);
        let r2 = -na::Matrix3::new(
            c2,
            -self.z2[0] * s2,
            0.,
            s2,
            self.z2[0] * c2,
            0.,
            0.,
            1.,
            -1.,
        );

        /* Compute the combined covariance matrix and its inverse (precision matrix) */
        let sigma1 = r1 * q1 * r1.transpose() + r2 * q2 * r2.transpose();
        let omega1 = sigma1.try_inverse().unwrap();

        /* Define the Jacobian matrices b1 and b2 */
        let b1 = -na::Matrix3::new(
            1.,
            0.,
            -self.z1[0] * s1,
            0.,
            1.,
            self.z1[0] * c1,
            0.,
            0.,
            1.,
        );
        let b2 = -na::Matrix3::new(
            1.,
            0.,
            -self.z2[0] * s2,
            0.,
            1.,
            self.z2[0] * c2,
            0.,
            0.,
            1.,
        );

        /* Compute the submatrices of the information matrix */
        self.omega_upperleft = b1.transpose() * omega1 * b1;
        self.omega_upperright = b1.transpose() * omega1 * b2;
        self.omega_bottomleft = b2.transpose() * omega1 * b1;
        self.omega_bottomright = b2.transpose() * omega1 * b2;

        /* Compute the subvectors of the information vector */
        self.xi_upper = -b1.transpose() * omega1 * hat_e;
        self.xi_bottom = -b2.transpose() * omega1 * hat_e;
    }

    /* Add the current edge to the global information matrix and vector */
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

        xi = SensorEdge::add_edge_xi(xi, &self.xi_upper, f1, t1);
        xi = SensorEdge::add_edge_xi(xi, &self.xi_bottom, f2, t2);

        (omega, xi)
    }

    /* Helper function to add the current edge to the information matrix */
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

    /* Helper function to add the current edge to the information vector */
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
