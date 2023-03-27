use super::super::common::file;
use super::super::kf::kf;
use itertools::Itertools;
use nalgebra as na;
use std::f32::consts::PI;

pub fn slam() {
    for _n in 0..10 {
        let mut count = 0;
        let (hat_xs, zlist, us) = file::pose_read();
        let mut land_klist: [Vec<(f32, (f32, f32, f32, f32))>; 6] = Default::default();

        for row in zlist {
            for z in row {
                let l_id = z.0 as usize;
                land_klist[l_id].push((count as f32, z));
            }
            count += 1;
        }

        let xdim = hat_xs.len() * 3;
        let mut omega = na::DMatrix::<f32>::zeros(xdim, xdim);
        for i in 0..3 {
            omega[(i, i)] = 1000000.;
        }
        let mut xi = na::DVector::<f32>::zeros(xdim);

        for index in 0..land_klist.len() {
            for comb in land_klist[index].iter().combinations(2) {
                let mut obsedges =
                    ObsEdge::new(comb[0].0, comb[1].0, comb[0].1, comb[1].1, hat_xs.clone());
                obsedges.precision_matrix();
                (omega, xi) = obsedges.add_edge(omega, xi);

                for i in 0..hat_xs.len() - 1 {
                    let mut motionedges = MotionEdge::new(i, i + 1, hat_xs.clone());
                    motionedges.action_matrix(us.clone(), 1.);
                    (omega, xi) = motionedges.add_edge(omega, xi);
                }
            }
        }
        // println!("omega:{}, xi:{}", omega, xi);

        let delta_xs = (omega.try_inverse().unwrap()) * xi;
        for i in 0..hat_xs.len() {
            let _x = hat_xs[i].0 + delta_xs[i * 3];
            let _y = hat_xs[i].1 + delta_xs[i * 3 + 1];
            let _theta = hat_xs[i].2 + delta_xs[i * 3 + 2];
            // println!("x:{}, y:{}, theta:{}", x, y, theta);
        }

        let diff = delta_xs.norm();
        println!("diff:{}", diff);
        if diff < 0.02 {
            println!("収束");
            break;
        }
    }
}

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
        let motion_noise_stds = kf::Stds {
            nn: 0.19,
            no: 0.001,
            oo: 0.13,
            on: 0.2,
        };
        let (nu, mut omega) = (us_vec[self.t1].0, us_vec[self.t1].1);

        if omega.abs() < 1e-5 {
            omega = 1e-5;
        }

        let m = kf::mat_m(nu, omega, delta, &motion_noise_stds);
        let a = kf::mat_a(nu, omega, delta, self.hat_x1[2]);
        let f = kf::mat_f(nu, omega, delta, self.hat_x1[2]);

        self.omega = (a * m * a.transpose() + na::Matrix3::<f32>::identity() * 0.0001)
            .try_inverse()
            .unwrap();

        self.omega_upperleft = f.transpose() * self.omega * f;
        self.omega_upperright = -f.transpose() * self.omega;
        self.omega_bottomleft = self.omega * f;
        self.omega_bottomright = self.omega;

        let x2 = kf::KFilterPose::state_transition(nu, omega, delta, &self.hat_x1);
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

        omega = ObsEdge::add_edge_omega(omega, &self.omega_upperleft, f1, f1, t1, t1);
        omega = ObsEdge::add_edge_omega(omega, &self.omega_upperright, f1, f2, t1, t2);
        omega = ObsEdge::add_edge_omega(omega, &self.omega_bottomleft, f2, f1, t2, t1);
        omega = ObsEdge::add_edge_omega(omega, &self.omega_bottomright, f2, f2, t2, t2);

        xi = ObsEdge::add_edge_xi(xi, &self.xi_upper, f1, t1);
        xi = ObsEdge::add_edge_xi(xi, &self.xi_bottom, f2, t2);

        (omega, xi)
    }

    pub fn add_edge_omega(
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

    pub fn add_edge_xi(
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

pub struct ObsEdge {
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

impl ObsEdge {
    pub fn new(
        t1_f32: f32,
        t2_f32: f32,
        z1_vec: (f32, f32, f32, f32),
        z2_vec: (f32, f32, f32, f32),
        xs_vec: Vec<(f32, f32, f32)>,
    ) -> Self {
        assert_eq!(z1_vec.0, z2_vec.0);
        let t1 = t1_f32 as usize;
        let t2 = t2_f32 as usize;
        let x1 = na::Vector3::new(xs_vec[t1].0, xs_vec[t1].1, xs_vec[t1].2);
        let x2 = na::Vector3::new(xs_vec[t2].0, xs_vec[t2].1, xs_vec[t2].2);
        let z1 = na::Vector3::new(z1_vec.1, z1_vec.2, z1_vec.3);
        let z2 = na::Vector3::new(z2_vec.1, z2_vec.2, z2_vec.3);

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

    pub fn precision_matrix(&mut self) {
        let sensor_noise_rate = [0.14, 0.05, 0.05];
        let s1 = (self.x1[2] + self.z1[1]).sin();
        let c1 = (self.x1[2] + self.z1[1]).cos();
        let s2 = (self.x2[2] + self.z2[1]).sin();
        let c2 = (self.x2[2] + self.z2[1]).cos();

        let mut hat_e = self.x2 - self.x1
            + na::Vector3::new(
                self.z2[0] * c2 - self.z1[0] * c1,
                self.z2[0] * s2 - self.z1[0] * s1,
                self.z2[1] - self.z2[2] - self.z1[1] + self.z1[2],
            );
        while hat_e[2] >= PI {
            hat_e[2] -= PI * 2.0;
        }
        while hat_e[2] < PI {
            hat_e[2] += PI * 2.0;
        }

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
        let sigma1 = r1 * q1 * r1.transpose() + r2 * q2 * r2.transpose();
        let omega1 = sigma1.try_inverse().unwrap();

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

        self.omega_upperleft = b1.transpose() * omega1 * b1;
        self.omega_upperright = b1.transpose() * omega1 * b2;
        self.omega_bottomleft = b2.transpose() * omega1 * b1;
        self.omega_bottomright = b2.transpose() * omega1 * b2;

        self.xi_upper = -b1.transpose() * omega1 * hat_e;
        self.xi_bottom = -b2.transpose() * omega1 * hat_e;
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

        omega = ObsEdge::add_edge_omega(omega, &self.omega_upperleft, f1, f1, t1, t1);
        omega = ObsEdge::add_edge_omega(omega, &self.omega_upperright, f1, f2, t1, t2);
        omega = ObsEdge::add_edge_omega(omega, &self.omega_bottomleft, f2, f1, t2, t1);
        omega = ObsEdge::add_edge_omega(omega, &self.omega_bottomright, f2, f2, t2, t2);

        xi = ObsEdge::add_edge_xi(xi, &self.xi_upper, f1, t1);
        xi = ObsEdge::add_edge_xi(xi, &self.xi_bottom, f2, t2);

        (omega, xi)
    }

    pub fn add_edge_omega(
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
