use super::super::domain::map::MapEdge;
use super::super::infrastructure::file;
use crate::domain::constraint::constraint;
use crate::domain::graph_slam::graph_slam;
use crate::domain::sensor_data::{init_landmark, Landmark};
use nalgebra as na;

pub fn slam(
    path: &str,
) -> (
    Vec<(f32, f32, f32)>,
    Vec<Vec<(f32, f32, f32, f32)>>,
    [Landmark; 6],
) {
    let (mut hat_xs, zlist, us) = file::pose_read(path);
    let mut delta_xs;

    let mut land_klist = constraint(&zlist);
    for _n in 0..10 {
        (delta_xs, land_klist) = graph_slam(&hat_xs, &us, &land_klist);
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
    for id in 0..6 {
        let mut omega = na::Matrix3::<f32>::zeros();
        let mut xi = na::Vector3::<f32>::zeros();
        let head_t = land_klist[id][0].0;
        let head_z = land_klist[id][0].1;
        for i in 0..land_klist[id].len() {
            let mut edge = MapEdge::new();
            let t = land_klist[id][i].0;
            let z = land_klist[id][i].1;
            edge.land_matrix(t as usize, z, head_t as usize, head_z, &hat_xs);
            omega += edge.omega;
            xi += edge.xi;
        }
        landmarks[id].id = id;
        landmarks[id].pose = (omega.try_inverse().unwrap()) * xi;
        // landmarks[id] = ((omega.try_inverse().unwrap()) * xi).into();
    }

    file::slam_write(hat_xs.clone(), zlist.clone(), &landmarks);

    (hat_xs, zlist, landmarks)
}
