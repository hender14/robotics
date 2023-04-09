use crate::domain::{
    sensor_data::{data_init, Landmark, LandmarkData, SensorData},
    utils as ut,
};
use nalgebra as na;
use rand::distributions::{Distribution, Uniform};
use rand::prelude::thread_rng;
use rand_distr::Normal;
use std::f32::consts::PI;

pub struct Sensor {
    distance_bias_rate_std: f32,
    direction_bias: f32,
    distance_noise_rate: f32,
    direction_noise: f32,
    distance_range: [f32; 2],
    direction_range: [f32; 2],
    phantom_dist_x: Uniform<f32>,
    phantom_dist_y: Uniform<f32>,
    phantom_prob: f32,
    oversight_prob: f32,
    occlusion_prob: f32,
    pub sensor_data: [SensorData; 6],
    time: usize,
}

impl Default for Sensor {
    fn default() -> Self {
        Self::new()
    }
}

impl Sensor {
    pub fn new() -> Self {
        let mut rng = rand::thread_rng();
        let normal_dist = Normal::new(0.0, 1.0).unwrap();
        let phantom_range_x = (-5., 5.);
        let phantom_range_y = (-5., 5.);
        Self {
            distance_bias_rate_std: normal_dist.sample(&mut rng) * 0.1,
            direction_bias: normal_dist.sample(&mut rng) * 2. * PI * 2. / 360.,
            distance_noise_rate: 0.1,
            direction_noise: 2. * PI * 2. / 360.,
            distance_range: [0.5, 6.],
            direction_range: [-(PI / 3.), PI / 3.],
            phantom_dist_x: Uniform::new(phantom_range_x.0, phantom_range_x.1),
            phantom_dist_y: Uniform::new(phantom_range_y.0, phantom_range_y.1),
            phantom_prob: 0.,
            oversight_prob: 0.1,
            occlusion_prob: 0.,
            sensor_data: Sensor::data_init(),
            time: 0,
        }
    }

    pub fn data_init() -> [SensorData; 6] {
        let sensor_data: [SensorData; 6] = [0, 1, 2, 3, 4, 5]
            .iter()
            .map(|&id| SensorData {
                id,
                timestamp: 0,
                result: false,
                data: LandmarkData {
                    polor: na::Matrix2x1::new(0., 0.),
                    psi: 0.,
                },
            })
            .collect::<Vec<SensorData>>()
            .try_into()
            .unwrap_or_else(|_| panic!("Failed to initialize sensor_data_array."));
        sensor_data
    }

    pub fn sensor_receive(
        &mut self,
        pose: &na::Vector3<f32>,
        landmarks: &[Landmark],
        delta: f32,
    ) -> [SensorData; 6] {
        /* sensor init */
        self.sensor_data = data_init();

        /* find landmark */
        self.find_landmark(landmarks, pose);

        /* time increment */
        self.time += delta as usize;
        self.sensor_data.clone()
    }

    pub fn find_landmark(&mut self, landmarks: &[Landmark], pose: &na::Vector3<f32>) {
        /* Process by landmark */
        for landmark in landmarks {
            /* get timestamp */
            self.sensor_data[landmark.id].id = landmark.id;
            self.sensor_data[landmark.id].timestamp = self.time;

            /* calculate landmark */
            let mut polar_landmark = ut::cartesian_to_polar(pose, &landmark.pose);
            let psi = ut::psi_predict(pose, &landmark.pose);

            /* refrect noize etc */
            polar_landmark = self.phantom(pose, &polar_landmark);
            polar_landmark = self.occlusion(&polar_landmark);
            let polar_some = self.oversight(&polar_landmark);

            if self.visible(&polar_some) {
                polar_landmark = self.exter_dist(polar_landmark);
                self.sensor_data[landmark.id].data = LandmarkData {
                    polor: polar_landmark,
                    psi,
                };
                self.sensor_data[landmark.id].result = true;
            } else {
                self.sensor_data[landmark.id].result = false;
            }
            // println!("{}: {:?}", landmark.id, self.sensor_data[landmark.id]);
        }
    }

    fn phantom(&self, pose: &na::Vector3<f32>, relpos: &na::Vector2<f32>) -> na::Vector2<f32> {
        let mut rng = rand::thread_rng();
        let uniform_dist = Uniform::new(0.0, 1.0);

        if uniform_dist.sample(&mut rng) < self.phantom_prob {
            let pos = na::Vector3::new(
                self.phantom_dist_x.sample(&mut rng),
                self.phantom_dist_y.sample(&mut rng),
                0.,
            );
            ut::cartesian_to_polar(pose, &pos)
        } else {
            *relpos
        }
    }

    fn occlusion(&self, relpos: &na::Vector2<f32>) -> na::Vector2<f32> {
        let mut rng = rand::thread_rng();
        let uniform_dist = Uniform::new(0.0, 1.0);

        if uniform_dist.sample(&mut rng) < self.occlusion_prob {
            let ell =
                relpos[0] + uniform_dist.sample(&mut rng) * (self.distance_range[1] - relpos[0]);
            na::Vector2::new(ell, relpos[1])
        } else {
            *relpos
        }
    }

    fn oversight(&self, relpos: &na::Vector2<f32>) -> Option<na::Vector2<f32>> {
        let mut rng = rand::thread_rng();
        let uniform_dist = Uniform::new(0.0, 1.0);

        if uniform_dist.sample(&mut rng) < self.oversight_prob {
            None
        } else {
            Some(*relpos)
        }
    }

    pub fn visible(&self, polarpos: &Option<na::Vector2<f32>>) -> bool {
        match polarpos {
            None => false,
            Some(polarpos) => {
                self.distance_range[0] <= polarpos[0]
                    && polarpos[0] <= self.distance_range[1]
                    && self.direction_range[0] <= polarpos[1]
                    && polarpos[1] <= self.direction_range[1]
            }
        }
    }

    pub fn exter_dist(&self, mut obj_dis: na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        obj_dis = self.bias(&obj_dis);
        obj_dis = self.noise(&obj_dis);
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
