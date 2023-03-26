use super::kf;
use super::sensor;
use nalgebra as na;
use std::f32::consts::PI;
use std::fs::File;
use std::fs::OpenOptions;
use std::io::{BufWriter, Write};

pub struct Agent<'a> {
    sensor: sensor::Sensor,
    kf: kf::KFilterPose<'a>,
    pose: na::Vector3<f32>,
    nuo: f32,
    omegao: f32,
    zlist: [[f32; 3]; 6],
    time: f32,
    landsize: usize,
}

impl<'a> Agent<'a> {
    pub fn new(initial_pose: &na::Vector3<f32>, size: usize) -> Self {
        let init_cov: f32 = 1e-10;
        let sensor = sensor::Sensor::new();
        let kf = kf::KFilterPose::new(initial_pose, init_cov);
        // ファイルを開く
        File::create("output.txt").expect("ファイルを作成できませんでした");
        Self {
            sensor,
            kf,
            pose: *initial_pose,
            nuo: 0.2,                     /* preliminary */
            omegao: 2. * PI * 10. / 360., /* preliminary */
            landsize: size,
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
    ) -> na::Vector3<f32> {
        // let nu = (pose[1] - self.belief.mean[1]).hypot(pose[0] - self.belief.mean[0])/time;
        // let omega = pose[2] / time;
        // let mut obj_dis  = na::Matrix2x1::zeros();
        let mut kf_state = na::Vector3::zeros();

        /* predict state transition */
        self.kf.kf_predict(self.nuo, self.omegao, time);

        let mut zlist: [[f32; 3]; 6] = Default::default();
        let mut zres: [bool; 6] = Default::default();
        /* Process by landmark */
        for i in 0..self.landsize {
            /* calculate landmark */
            let index: [usize; 3] = [i, self.landsize + i, self.landsize*2 + i];
            let lpose_row: na::Vector3<f32> =
                na::Vector3::new(lpose[index[0]], lpose[index[1]], lpose[index[2]]);

            /* calculate landmark distance/direct */
            let mut obj_dis = self.sensor.observation_predict(pose, &lpose_row);
            let obj = self.sensor.psi_predict(&self.pose, &lpose_row);
            zlist[i] = [obj_dis[0], obj_dis[1], obj];
            /* refrect noize etc */
            zres[i] = self.sensor.visible(&obj_dis);
            obj_dis = self.sensor.exter_dist(&obj_dis);

            /* calculate kfiltered pose*/
            kf_state = self.kf.kf_update(&obj_dis, &lpose_row);
        }

        /* update old value */
        self.time += time;
        self.nuo = nu;
        self.omegao = omega;
        self.pose = kf_state;
        self.zlist = zlist;

        // let file = File::open("output.txt").expect("ファイルを作成できませんでした");
        let file = OpenOptions::new()
            .write(true)
            .append(true)
            .open("output.txt")
            .expect("ファイルを開けませんでした");
        // let file = File::create("output.txt").expect("ファイルを作成できませんでした");
        let mut writer = BufWriter::new(file);

        // 変数をファイルに書き込む
        writeln!(writer, "0 {} {} {}", self.time, nu, omega).expect("書き込みエラー");
        writeln!(
            writer,
            "1 {} {} {} {}",
            self.time, kf_state[0], kf_state[1], kf_state[2]
        )
        .expect("書き込みエラー");
        for i in 0..self.landsize {
            if zres[i] {
                writeln!(
                    writer,
                    "2 {} {} {} {} {}",
                    self.time, i, zlist[i][0], zlist[i][1], zlist[i][2]
                )
                .expect("書き込みエラー");
            }
        }

        /* robot */
        // robot_noise(&kf_state, time);
        // robot_move(&kf_state, time);
        // robot_noise2(&kf_state, time); //sensor処理にて代替
        kf_state
    }
}
