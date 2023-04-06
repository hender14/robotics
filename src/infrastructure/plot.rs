use crate::domain::sensor_data::{Landmark, SensorData};

use super::file;
use plotters::{coord::types::RangedCoordf64, prelude::*};

pub fn plot_kf(path: &str, landmarks: &[Landmark; 6]) {
    let root = BitMapBackend::new("out/kfplot.png", (640, 480)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .caption("Point Cloud", ("sans-serif", 30))
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-5.0..5.0, -5.0..5.0)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    let (hat_xs, sensor_data, _) = file::pose_read(path);

    plot_pose(&mut chart, &hat_xs);
    plot_landmark(&mut chart, &landmarks);
    plot_edge(&mut chart, &hat_xs, &sensor_data);
}

pub fn plot_slam(path: &str, landmarks: &[Landmark; 6]) {
    let root = BitMapBackend::new("out/slamplot.png", (640, 480)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .caption("Point Cloud", ("sans-serif", 30))
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-5.0..5.0, -5.0..5.0)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    let (hat_xs, sensor_data, _) = file::pose_read(path);

    plot_pose(&mut chart, &hat_xs);
    plot_landmark(&mut chart, &landmarks);
    plot_edge(&mut chart, &hat_xs, &sensor_data);
}

pub fn plot_pose(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    hat_xs: &Vec<(f32, f32, f32)>,
) {
    let xs: Vec<f64> = hat_xs.iter().map(|(x, _, _)| *x as f64).collect();
    let ys: Vec<f64> = hat_xs.iter().map(|(_, y, _)| *y as f64).collect();

    let data: Vec<(f64, f64)> = xs.iter().zip(ys.iter()).map(|(&x, &y)| (x, y)).collect();

    let line_series = LineSeries::new(data.iter().copied(), &RED); //直線

    chart.draw_series(line_series).unwrap();
}

pub fn plot_landmark(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    array: &[Landmark; 6],
) {
    let point_series = array.iter().map(|landmark| {
        Circle::new(
            (landmark.pose.x as f64, landmark.pose.y as f64),
            5,
            ShapeStyle::from(&BLACK).filled(),
        )
    });

    chart.draw_series(point_series).unwrap();
}

pub fn plot_edge(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    hat_xs: &Vec<(f32, f32, f32)>,
    sensor_data: &Vec<Vec<SensorData>>,
) {
    for ar in 1..hat_xs.len() {
        let x1 = hat_xs[ar].0;
        let y1 = hat_xs[ar].1;
        let theta = hat_xs[ar].2;
        let data = &sensor_data[ar];
        for d in data {
            let ell = d.data.polor[0];
            let phi = d.data.polor[1];
            let x2: f32 = x1 + ell * ((theta + phi).cos());
            let y2: f32 = y1 + ell * ((theta + phi).sin());

            let line_series = LineSeries::new(
                [(x1 as f64, y1 as f64), (x2 as f64, y2 as f64)]
                    .iter()
                    .copied(),
                &BLUE,
            );

            chart.draw_series(line_series).unwrap();
        }
    }
}
