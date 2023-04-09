use crate::domain::{
    sensor_data::{Landmark, LandmarkData, SensorData},
    state::{State, Twist},
};
use nalgebra as na;
use std::fs::{self, File, OpenOptions};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

/* Constants for the output file paths */
pub const KF_PATH: &str = concat!(crate::OUT_DIRECTORY!(), "/", crate::OUT_KF!(), "output.txt");
pub const SLAM_PATH: &str = concat!(crate::OUT_DIRECTORY!(), "/", crate::OUT_SLAM!(), "out.txt");

/* Function for initializing the output directory */
pub fn directry_init() {
    let dir_path = crate::OUT_DIRECTORY!();
    let path = Path::new(dir_path);
    if !path.exists() {
        fs::create_dir_all(path).unwrap();
    }
}

/* Function for initializing the output files */
pub fn file_init() {
    File::create(KF_PATH).expect("can not create file");
    File::create(SLAM_PATH).expect("can not create file");
}

/*  Function for reading the robot poses, sensor data, and velocities from a file */
pub fn pose_read(path: &str) -> (Vec<(f32, f32, f32)>, Vec<Vec<SensorData>>, Vec<Twist>) {
    /* Initialize the vectors for storing robot poses, sensor data, and velocities */
    let mut velocity: Vec<Twist> = vec![];
    let mut state: Vec<(f32, f32, f32)> = vec![];
    let mut sensor_data: Vec<Vec<SensorData>> = vec![vec![]];

    /* Open the file and create a BufReader */
    let file = File::open(path).expect("Failed to open file");
    let reader = BufReader::new(file);

    /* Iterate through the lines in the file */
    for line in reader.lines() {
        let line = line.unwrap();
        let data_type: usize = line.split_whitespace().next().unwrap().parse().unwrap();
        match data_type {
            0 => parse_velocity(&line, &mut velocity),
            1 => parse_robot_pose(&line, &mut state),
            2 => parse_sensor_data(&line, &mut sensor_data),
            _ => panic!("Invalid data type in the input file"),
        }
    }
    /* Return the vectors containing robot poses, sensor data, and velocities */
    (state, sensor_data, velocity)
}

/* Function for writing the robot state, pose, and sensor data to the Kalman Filter output file */
pub fn pose_write(state: &State, pose: &na::Vector3<f32>, sensor_data: &[SensorData; 6]) {
    /* Open the Kalman Filter output file and create a BufWriter */
    let file = OpenOptions::new()
        .write(true)
        .append(true)
        .open(KF_PATH)
        .expect("can not open file");
    let mut writer = BufWriter::new(file);

    /* Write the state, pose, and sensor data to the file */
    /* 0: write velocity data(nu omega) */
    writeln!(
        writer,
        "0 {} {} {}",
        state.time, state.velocity.nu, state.velocity.omega
    )
    .expect("err write");
    /* 1: write estimate_state data(x, y, theta) */
    writeln!(
        writer,
        "1 {} {} {} {}",
        state.time, pose[0], pose[1], pose[2]
    )
    .expect("err write");
    /* 2: write sendor data(id, polor, psi) */
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

/* Function for writing the robot states, sensor data, and landmarks to the Graph SLAM output file */
pub fn slam_write(
    states: &[(f32, f32, f32)],
    sensor_data: &Vec<Vec<SensorData>>,
    landmarks: &[Landmark],
) {
    let file = OpenOptions::new()
        .write(true)
        .append(true)
        .open(SLAM_PATH)
        .expect("can not open file");
    let mut writer = BufWriter::new(file);

    /* Write the state, pose, and sensor data to the file */
    /* 0: write velocity data(nu omega) */
    for landmark in landmarks {
        writeln!(
            writer,
            "0 {} {} {}",
            landmark.id, landmark.pose[0], landmark.pose[1]
        )
        .expect("err write");
    }
    /* 1: write estimate_state data(x, y, theta) */
    for (i, xs) in states.iter().enumerate() {
        writeln!(writer, "1 {} {} {} {}", i, xs.0, xs.1, xs.2).expect("err write");
    }
    /* 2: write sendor data(id, polor, psi) */
    for data in sensor_data {
        for d in data {
            writeln!(
                writer,
                "2 {} {} {} {} {}",
                d.timestamp, d.id, d.data.polor[0], d.data.polor[1], d.data.psi,
            )
            .expect("err write");
        }
    }
}

/* parse the velocity vector */
fn parse_velocity(line: &str, velocity: &mut Vec<Twist>) {
    let array: Vec<f32> = line
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    let step: usize = array[1] as usize;
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
}

/* parse the velocity vector */
fn parse_robot_pose(line: &str, state: &mut Vec<(f32, f32, f32)>) {
    let array: Vec<f32> = line
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    let step: usize = array[1] as usize;
    if step >= state.len() {
        state.resize(step + 1, (0.0, 0.0, 0.0));
    }
    state[step] = (array[2], array[3], array[4]);
}

/* parse the sensor data vector */
fn parse_sensor_data(line: &str, sensor_data: &mut Vec<Vec<SensorData>>) {
    let array: Vec<f32> = line
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    let step: usize = array[1] as usize;
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
