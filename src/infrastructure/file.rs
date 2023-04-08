use crate::domain::{
    sensor_data::{Landmark, LandmarkData, SensorData},
    state::{State, Twist},
};
use nalgebra as na;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;
use std::{
    fs,
    fs::{File, OpenOptions},
};

pub const DIRECTRY: &str = "out";
pub const KFPATH: &str = "out/kfoutput.txt";
pub const SLAMPATH: &str = "out/slamout.txt";

pub fn pose_read(path: &str) -> (Vec<(f32, f32, f32)>, Vec<Vec<SensorData>>, Vec<Twist>) {
    let mut hat_xs: Vec<(f32, f32, f32)> = vec![];
    let mut sensor_data: Vec<Vec<SensorData>> = vec![vec![]];
    let mut velocity: Vec<Twist> = vec![];
    let file = File::open(path).expect("Failed to open file");
    let reader = BufReader::new(file);
    for line in reader.lines() {
        let line = line.unwrap();
        let array: Vec<f32> = line
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();
        let step: usize = array[1] as usize;
        if array[0] == 0. {
            if step >= velocity.len() {
                velocity.resize(
                    step + 1,
                    Twist {
                        nu: 0.0,
                        omega: 0.0,
                    },
                );
            }
            velocity[step] = Twist {
                nu: array[2],
                omega: array[3],
            };
        } else if array[0] == 1. {
            if step >= hat_xs.len() {
                hat_xs.resize(step + 1, (0.0, 0.0, 0.0));
            }
            hat_xs[step] = (array[2], array[3], array[4]);
        } else if array[0] == 2. {
            if step >= sensor_data.len() {
                sensor_data.resize(
                    step + 1,
                    vec![SensorData {
                        id: array[2] as usize,
                        timestamp: step,
                        result: true,
                        data: LandmarkData {
                            polor: na::Matrix2x1::new(array[3], array[4]),
                            psi: array[5],
                        },
                    }],
                );
            } else {
                sensor_data[step].push(SensorData {
                    id: array[2] as usize,
                    timestamp: step,
                    result: true,
                    data: LandmarkData {
                        polor: na::Matrix2x1::new(array[3], array[4]),
                        psi: array[5],
                    },
                });
            }
        }
    }
    (hat_xs, sensor_data, velocity)
}

/* confirm directry */
pub fn directry_init() {
    let dir_path = DIRECTRY;
    let path = Path::new(dir_path);
    if !path.exists() {
        fs::create_dir_all(path).unwrap();
    }
}

/* create file */
pub fn file_init() {
    File::create(KFPATH).expect("can not create file");
    File::create(SLAMPATH).expect("can not create file");
}

pub fn pose_write(robot: &State, pose: &na::Vector3<f32>, sensor_data: &[SensorData; 6]) {
    let file = OpenOptions::new()
        .write(true)
        .append(true)
        .open(KFPATH)
        .expect("can not open file");
    let mut writer = BufWriter::new(file);

    // 変数をファイルに書き込む
    writeln!(
        writer,
        "0 {} {} {}",
        robot.time, robot.velocity.nu, robot.velocity.omega
    )
    .expect("err write");
    writeln!(
        writer,
        "1 {} {} {} {}",
        robot.time, pose[0], pose[1], pose[2]
    )
    .expect("err write");
    for data in sensor_data {
        if data.result {
            writeln!(
                writer,
                "2 {} {} {} {} {}",
                data.timestamp, data.id, data.data.polor[0], data.data.polor[1], data.data.psi,
            )
            .expect("err write");
        }
    }
}

pub fn slam_write(
    hat_xs: &[(f32, f32, f32)],
    sensor_data: &Vec<Vec<SensorData>>,
    landmarks: &[Landmark; 6],
) {
    let file = OpenOptions::new()
        .write(true)
        .append(true)
        .open(SLAMPATH)
        .expect("can not open file");
    let mut writer = BufWriter::new(file);

    // 変数をファイルに書き込む
    for (i, xs) in hat_xs.iter().enumerate() {
        writeln!(writer, "1 {} {} {} {}", i, xs.0, xs.1, xs.2).expect("err write");
    }
    for landmark in landmarks {
        writeln!(
            writer,
            "0 {} {} {}",
            landmark.id, landmark.pose[0], landmark.pose[1]
        )
        .expect("err write");
    }
    for data in sensor_data {
        // for i in 0..sensor_data.len() {
        for d in data {
            writeln!(
                writer,
                "2 {} {} {} {} {}",
                d.timestamp,
                d.id,
                d.data.polor[0],
                d.data.polor[1],
                d.data.psi,
                // i, sensor_data[i][j].0, sensor_data[i][j].1, sensor_data[i][j].2, sensor_data[i][j].3,
            )
            .expect("err write");
        }
    }
}
