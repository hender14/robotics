use crate::domain::{
    constraint::constraint,
    graph_slam::graph_slam,
    map::MapEdge,
    sensor_data::{init_landmark, Landmark, SensorData},
};
use crate::infrastructure::file;
use nalgebra as na;

pub fn slam(path: &str) -> (Vec<(f32, f32, f32)>, Vec<Vec<SensorData>>, [Landmark; 6]) {
    let (mut hat_xs, sensor_data, velocity) = file::pose_read(path);
    let mut delta_xs;

    let landmark_list = constraint(&sensor_data);
    /* slamの最適化サイクルについて、定数化要 */
    for _n in 0..3 {
        delta_xs = graph_slam(&hat_xs, &velocity, &landmark_list);
        for i in 0..hat_xs.len() {
            hat_xs[i].0 += delta_xs[i * 3];
            hat_xs[i].1 += delta_xs[i * 3 + 1];
            hat_xs[i].2 += delta_xs[i * 3 + 2];
        }
        let diff = delta_xs.norm();
        println!("diff:{}", diff);
        if diff < 0.02 {
            break;
        }
    }

    let mut landmarks: [Landmark; 6] = init_landmark();
    for (id, landmark) in landmark_list.iter().enumerate() {
        let mut omega = na::Matrix3::<f32>::zeros();
        let mut xi = na::Vector3::<f32>::zeros();
        let head_t = landmark[0].0;
        let head_z = landmark[0].1.data.clone();
        for mark in landmark.iter().take(landmark_list.len()) {
            let mut edge = MapEdge::new();
            let t = mark.0;
            let z = mark.1.data.clone();
            edge.land_matrix(t, z, head_t, head_z.clone(), &hat_xs);
            omega += edge.omega;
            xi += edge.xi;
        }
        landmarks[id].id = id;
        landmarks[id].pose = (omega.try_inverse().unwrap()) * xi;
    }

    file::slam_write(&hat_xs, &sensor_data, &landmarks);

    (hat_xs, sensor_data, landmarks)
}
