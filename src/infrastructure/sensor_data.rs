use crate::domain::{
    sensor_data::{Landmark, LandmarkData, SensorData},
    utils as ut,
};
use nalgebra as na;
use rand::prelude::{thread_rng, Distribution};
use rand_distr::Normal;
use std::f32::consts::PI;

pub struct Sensor {
    distance_bias_rate_std: f32,
    direction_bias: f32,
    distance_noise_rate: f32,
    direction_noise: f32,
    distance_range: [f32; 2],
    direction_range: [f32; 2],
    pub sensor_data: Vec<SensorData>,
    time: usize,
}

impl Default for Sensor {
    fn default() -> Self {
        Self::new()
    }
}

impl Sensor {
    pub fn new() -> Self {
        Self {
            distance_bias_rate_std: 0.1,
            direction_bias: 2. * PI * 2. / 360.,
            distance_noise_rate: 0.1,
            direction_noise: 2. * PI * 2. / 360.,
            distance_range: [0.5, 6.],
            direction_range: [-(PI / 3.), PI / 3.],
            sensor_data: Self::object_init(),
            time: 0,
        }
    }

    fn object_init() -> Vec<SensorData> {
        let mut sensor_data = vec![];
        for i in 0..6 {
            sensor_data.push(SensorData {
                id: i,
                timestamp: 0,
                result: false,
                data: LandmarkData {
                    polor: na::Matrix2x1::new(0., 0.),
                    psi: 0.,
                },
            })
        }

        sensor_data
    }

    pub fn sensor_receive(&mut self, pose: &na::Vector3<f32>, landmarks: &[Landmark; 6]) {
        /* find landmark */
        self.find_landmark(landmarks, pose);

        /* time increment */
        self.time += 1;
    }

    pub fn find_landmark(&mut self, landmarks: &[Landmark; 6], pose: &na::Vector3<f32>) {
        /* Process by landmark */
        for landmark in landmarks {
            /* get timestamp */
            self.sensor_data[landmark.id].id = landmark.id;
            self.sensor_data[landmark.id].timestamp = self.time;

            /* calculate landmark */
            let mut polar_landmark = ut::polar_trans(pose, &landmark.pose);
            let psi = ut::psi_predict(pose, &landmark.pose);

            /* refrect noize etc */
            self.sensor_data[landmark.id].result = self.visible(&polar_landmark);
            polar_landmark = self.exter_dist(polar_landmark, landmark.id);
            self.sensor_data[landmark.id].data = LandmarkData {
                polor: polar_landmark,
                psi,
            };
        }
    }

    pub fn visible(&self, polarpos: &na::Matrix2x1<f32>) -> bool {
        self.distance_range[0] <= polarpos[0]
            && polarpos[0] <= self.distance_range[1]
            && self.direction_range[0] <= polarpos[1]
            && polarpos[1] <= self.direction_range[1]
    }

    pub fn exter_dist(&self, mut obj_dis: na::Matrix2x1<f32>, id: usize) -> na::Matrix2x1<f32> {
        if self.sensor_data[id].result {
            obj_dis = Sensor::bias(self, &obj_dis);
            obj_dis = Sensor::noise(self, &obj_dis);
        } else {
            obj_dis = na::Matrix2x1::new(0.0, 0.0);
        }
        obj_dis
    }
    fn bias(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mut mat = na::Matrix2x1::zeros();
        mat[0] = obj_dis[0] + obj_dis[0] * self.distance_bias_rate_std;
        mat[1] = obj_dis[1] + self.direction_bias;
        mat
    }

    fn noise(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mut rng = thread_rng();
        let normal_ell = Normal::new(obj_dis[0], obj_dis[0] * self.distance_noise_rate).unwrap();
        let normal_phi = Normal::new(obj_dis[1], self.direction_noise).unwrap();
        let ell = normal_ell.sample(&mut rng);
        let phi = normal_phi.sample(&mut rng);
        na::Matrix2x1::new(ell, phi)
    }
}
