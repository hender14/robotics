use crate::domain::{
    sensor_data::{data_init, Landmark, LandmarkData, SensorData},
    utils as ut,
};
use nalgebra as na;
use rand::distributions::{Distribution, Uniform};
use rand::prelude::thread_rng;
use rand_distr::Normal;
use std::f32::consts::PI;

/* enum sensor effect */
enum SensorEffect {
    Phantom,
    Occlusion,
    Oversight,
}

/* The Sensor struct represents a sensor that can receive data from landmarks. */
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
    /* Initialize a new Sensor with default values. */
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

    /* Initialize the sensor_data array with default values. */
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

    /* This method receives sensor data, processes it, and updates the sensor_data array. */
    pub fn sensor_receive(
        &mut self,
        pose: &na::Vector3<f32>,
        landmarks: &[Landmark],
        delta: f32,
    ) -> [SensorData; 6] {
        /* Initialize sensor data. */
        self.sensor_data = data_init();

        /* Find visible landmarks. */
        self.find_landmark(landmarks, pose);

        /* Increment the timestamp. */
        self.time += delta as usize;
        self.sensor_data.clone()
    }

    /* This method processes the landmarks and updates the sensor_data array based on their visibility. */
    pub fn find_landmark(&mut self, landmarks: &[Landmark], pose: &na::Vector3<f32>) {
        /* Process by landmark */
        for landmark in landmarks {
            /* Set the landmark ID and timestamp. */
            self.sensor_data[landmark.id].id = landmark.id;
            self.sensor_data[landmark.id].timestamp = self.time;

            /* Calculate the polar coordinates of the landmark relative to the sensor's pose. */
            let mut polar_landmark = ut::cartesian_to_polar(pose, &landmark.pose);
            let psi = ut::psi_predict(pose, &landmark.pose);

            /* Apply phantom, occlusion, and oversight probabilities. */
            polar_landmark = self
                .apply_sensor_effects(SensorEffect::Phantom, pose, &polar_landmark)
                .unwrap();
            polar_landmark = self
                .apply_sensor_effects(SensorEffect::Occlusion, pose, &polar_landmark)
                .unwrap();
            let polar_some =
                self.apply_sensor_effects(SensorEffect::Oversight, pose, &polar_landmark);

            /* If the landmark is visible, update the sensor_data array. */
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

    fn apply_sensor_effects(
        &self,
        effect_type: SensorEffect,
        pose: &na::Vector3<f32>,
        relpos: &na::Vector2<f32>,
    ) -> Option<na::Vector2<f32>> {
        let mut rng = rand::thread_rng();
        let uniform_dist = Uniform::new(0.0, 1.0);
        let probability = match effect_type {
            SensorEffect::Phantom => self.phantom_prob,
            SensorEffect::Occlusion => self.occlusion_prob,
            SensorEffect::Oversight => self.oversight_prob,
        };

        if uniform_dist.sample(&mut rng) < probability {
            match effect_type {
                /* simulates the appearance of phantom landmarks by introducing random noise. */
                SensorEffect::Phantom => {
                    let pos = na::Vector3::new(
                        self.phantom_dist_x.sample(&mut rng),
                        self.phantom_dist_y.sample(&mut rng),
                        0.,
                    );
                    Some(ut::cartesian_to_polar(pose, &pos))
                }
                /* simulates occlusion by modifying the distance component of the polar coordinates. */
                SensorEffect::Occlusion => {
                    let ell = relpos[0]
                        + uniform_dist.sample(&mut rng) * (self.distance_range[1] - relpos[0]);
                    Some(na::Vector2::new(ell, relpos[1]))
                }
                /* simulates oversight by returning None if the probability is met. */
                SensorEffect::Oversight => None,
            }
        } else {
            Some(*relpos)
        }
    }

    /* This method checks if the landmark is within the sensor's visible range. */
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

    /* This method applies external noise and bias to the distance and direction of the observed landmark. */
    pub fn exter_dist(&self, mut obj_dis: na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        obj_dis = self.bias(&obj_dis);
        obj_dis = self.noise(&obj_dis);
        obj_dis
    }

    /* This method applies bias to the distance and direction components of the polar coordinates. */
    fn bias(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mut mat = na::Matrix2x1::zeros();
        mat[0] = obj_dis[0] + obj_dis[0] * self.distance_bias_rate_std;
        mat[1] = obj_dis[1] + self.direction_bias;
        mat
    }

    /* This method applies random noise to the distance and direction components of the polar coordinates. */
    fn noise(&self, &obj_dis: &na::Matrix2x1<f32>) -> na::Matrix2x1<f32> {
        let mut rng = thread_rng();
        let normal_ell = Normal::new(obj_dis[0], obj_dis[0] * self.distance_noise_rate).unwrap();
        let normal_phi = Normal::new(obj_dis[1], self.direction_noise).unwrap();
        let ell = normal_ell.sample(&mut rng);
        let phi = normal_phi.sample(&mut rng);
        na::Matrix2x1::new(ell, phi)
    }
}
