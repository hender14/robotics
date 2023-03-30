use nalgebra as na;
use std::fs::File;
use std::fs::OpenOptions;
use std::io::{BufRead, BufReader, BufWriter, Write};

pub const KFPATH: &str = "out/output.txt";
pub const SLAMPATH: &str = "out/slamout.txt";

pub fn pose_read(
    path: &str,
) -> (
    Vec<(f32, f32, f32)>,
    Vec<Vec<(f32, f32, f32, f32)>>,
    Vec<(f32, f32)>,
) {
    let mut hat_xs: Vec<(f32, f32, f32)> = vec![];
    let mut zlist: Vec<Vec<(f32, f32, f32, f32)>> = vec![vec![]];
    let mut us: Vec<(f32, f32)> = vec![];
    let file = File::open(path).expect("Failed to open file");
    let reader = BufReader::new(file);
    for line in reader.lines() {
        let line = line.unwrap();
        let array: Vec<f32> = line
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();
        // println!("{:?}", array);
        let step: usize = array[1] as usize;
        if array[0] == 0. {
            if step >= us.len() {
                us.resize(step + 1, (0.0, 0.0));
            }
            us[step] = (array[2], array[3]);
        } else if array[0] == 1. {
            if step >= hat_xs.len() {
                hat_xs.resize(step + 1, (0.0, 0.0, 0.0));
            }
            hat_xs[step] = (array[2], array[3], array[4]);
        } else if array[0] == 2. {
            if step >= zlist.len() {
                zlist.resize(step + 1, vec![(array[2], array[3], array[4], array[5])]);
            } else {
                zlist[step].push((array[2], array[3], array[4], array[5]));
            }
        }
    }
    (hat_xs, zlist, us)
}

pub fn write_init() {
    File::create(&KFPATH).expect("can not create file");
    File::create(&SLAMPATH).expect("can not create file");
}

pub fn pose_write(
    time: f32,
    nu: f32,
    omega: f32,
    pose: &na::Vector3<f32>,
    zres: &[bool; 6],
    zlist: &[[f32; 3]; 6],
) {
    let file = OpenOptions::new()
        .write(true)
        .append(true)
        .open(&KFPATH)
        .expect("can not open file");
    let mut writer = BufWriter::new(file);

    // 変数をファイルに書き込む
    writeln!(writer, "0 {} {} {}", time, nu, omega).expect("err write");
    writeln!(writer, "1 {} {} {} {}", time, pose[0], pose[1], pose[2]).expect("err write");
    for i in 0..zlist.len() {
        if zres[i] {
            writeln!(
                writer,
                "2 {} {} {} {} {}",
                time, i, zlist[i][0], zlist[i][1], zlist[i][2]
            )
            .expect("err write");
        }
    }
}

pub fn slam_write(
    time: f32,
    hat_xs: Vec<(f32, f32, f32)>,
    zlist: Vec<Vec<(f32, f32, f32, f32)>>,
    land: [[f32; 3]; 6],
) {
    let file = OpenOptions::new()
        .write(true)
        .append(true)
        .open(&SLAMPATH)
        .expect("can not open file");
    let mut writer = BufWriter::new(file);

    // 変数をファイルに書き込む
    for i in 0..hat_xs.len() {
        writeln!(
            writer,
            "1 {} {} {} {}",
            i, hat_xs[i].0, hat_xs[i].1, hat_xs[i].2
        )
        .expect("err write");
    }
    for i in 0..land.len() {
        writeln!(writer, "0 {} {} {}", i, land[i][0], land[i][1]).expect("err write");
    }
    for i in 0..zlist.len() {
        for j in 0..zlist[i].len() {
            writeln!(
                writer,
                "2 {} {} {} {} {}",
                i, zlist[i][j].1, zlist[i][j].1, zlist[i][j].1, zlist[i][j].1,
            )
            .expect("err write");
        }
    }
}
