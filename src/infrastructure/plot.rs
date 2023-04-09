use super::file;
use crate::domain::sensor_data::{Landmark, SensorData};
use plotters::{coord::types::RangedCoordf64, prelude::*};

pub const KF_PATH: &str = concat!(crate::OUT_DIRECTORY!(), "/", crate::OUT_KF!(), "plot.png");
pub const SLAM_PATH: &str = concat!(crate::OUT_DIRECTORY!(), "/", crate::OUT_SLAM!(), "plot.png");

/* plotting the results of either the Kalman Filter or Graph SLAM algorithms. */
pub fn plot_generic(input_path: &str, output_path: &str, landmarks: &[Landmark]) {
    /* Create a new drawing area with the specified filename and dimensions */
    let root = BitMapBackend::new(output_path, (640, 480)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    /* Create a new cartesian 2D chart with the specified title and axis labels */
    let mut chart = ChartBuilder::on(&root)
        .caption("Point Cloud", ("sans-serif", 30))
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-5.0..5.0, -5.0..5.0)
        .unwrap();

    /* Configure and draw the chart mesh */
    chart.configure_mesh().draw().unwrap();

    /* Read the pose data from the specified file */
    let (estimate_states, sensor_data, _) = file::pose_read(input_path);

    /* Plot the robot poses, landmarks, and edges */
    plot_pose(&mut chart, &estimate_states);
    plot_landmark(&mut chart, landmarks);
    plot_edge(&mut chart, &estimate_states, &sensor_data);
}

/* plotting the results of the Kalman Filter algorithm of pose. */
pub fn plot_pose(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    states: &[(f32, f32, f32)],
) {
    /* Convert the robot poses to arrays of x and y coordinates */
    let xs: Vec<f64> = states.iter().map(|(x, _, _)| *x as f64).collect();
    let ys: Vec<f64> = states.iter().map(|(_, y, _)| *y as f64).collect();

    /* Combine the x and y coordinates into a single array of tuples */
    let data: Vec<(f64, f64)> = xs.iter().zip(ys.iter()).map(|(&x, &y)| (x, y)).collect();

    /* Create a line series from the data and draw it on the chart */
    let line_series = LineSeries::new(data.iter().copied(), &RED); //直線

    chart.draw_series(line_series).unwrap();
}

/* plotting the results of the Kalman Filter algorithm of landmark. */
pub fn plot_landmark(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    array: &[Landmark],
) {
    /* Create a point series from the landmarks and draw it on the chart */
    let point_series = array.iter().map(|landmark| {
        Circle::new(
            (landmark.pose.x as f64, landmark.pose.y as f64),
            5,
            ShapeStyle::from(&BLACK).filled(),
        )
    });

    chart.draw_series(point_series).unwrap();
}

/* plotting the results of the Kalman Filter algorithm of edge */
pub fn plot_edge(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    states: &Vec<(f32, f32, f32)>,
    sensor_data: &[Vec<SensorData>],
) {
    /* Iterate over the robot poses and sensor data */
    for ar in 1..states.len() {
        let x1 = states[ar].0;
        let y1 = states[ar].1;
        let theta = states[ar].2;
        let data = &sensor_data[ar];
        for d in data {
            /* Calculate the coordinates of the observed landmarks */
            let ell = d.data.polor[0];
            let phi = d.data.polor[1];
            let x2: f32 = x1 + ell * ((theta + phi).cos());
            let y2: f32 = y1 + ell * ((theta + phi).sin());

            /* Create a line series between the robot pose and the observed landmark, and draw it on the chart */
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
