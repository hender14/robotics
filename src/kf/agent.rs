use super::kf;
use super::sensor;
use nalgebra as na;
use std::f32::consts::PI;

pub struct Agent<'a> {
    sensor: sensor::Sensor,
    kf: kf::KFilterPose<'a>,
    pub pose: na::Vector3<f32>,
    pub nuo: f32,
    pub omegao: f32,
    pub zres: [bool; 6],
    pub zlist: [[f32; 3]; 6],
    pub time: f32,
    landsize: usize,
}

impl<'a> Agent<'a> {
    pub fn new(initial_pose: &na::Vector3<f32>, size: usize) -> Self {
        let init_cov: f32 = 1e-10;
        let sensor = sensor::Sensor::new();
        let kf = kf::KFilterPose::new(initial_pose, init_cov);
        Self {
            sensor,
            kf,
            pose: *initial_pose,
            nuo: 0.2,                     /* preliminary */
            omegao: 2. * PI * 10. / 360., /* preliminary */
            landsize: size,
            zres: Default::default(),
            zlist: Default::default(),
            time: 0.,
        }
    }

    pub fn pose_estimate(
        &mut self,
        nu: f32,
        omega: f32,
        lpose: &na::Matrix6x3<f32>,
        time: f32,
        pose: &na::Vector3<f32>,
    ) {
        // let nu = (pose[1] - self.belief.mean[1]).hypot(pose[0] - self.belief.mean[0])/time;
        // let omega = pose[2] / time;
        let mut kf_state = na::Vector3::zeros();

        /* predict state transition */
        self.kf.kf_predict(self.nuo, self.omegao, time);

        let mut zlist: [[f32; 3]; 6] = Default::default();
        /* Process by landmark */
        for i in 0..self.landsize {
            /* calculate landmark */
            let index: [usize; 3] = [i, self.landsize + i, self.landsize * 2 + i];
            let lpose_row: na::Vector3<f32> =
                na::Vector3::new(lpose[index[0]], lpose[index[1]], lpose[index[2]]);

            /* calculate landmark distance/direct */
            let mut obj_dis = self.sensor.observation_predict(pose, &lpose_row);
            let obj = self.sensor.psi_predict(&self.pose, &lpose_row);
            zlist[i] = [obj_dis[0], obj_dis[1], obj];
            /* refrect noize etc */
            self.zres[i] = self.sensor.visible(&obj_dis);
            obj_dis = self.sensor.exter_dist(obj_dis);

            /* calculate kfiltered pose*/
            kf_state = self.kf.kf_update(&obj_dis, &lpose_row);
        }

        /* update old value */
        self.time += time;
        self.nuo = nu;
        self.omegao = omega;
        self.pose = kf_state;
        self.zlist = zlist;

        /* robot */
        // robot_noise(&kf_state, time);
        // robot_move(&kf_state, time);
        // robot_noise2(&kf_state, time); //sensor処理にて代替
    }
}
