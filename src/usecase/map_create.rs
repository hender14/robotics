use crate::domain::{
    constraint::constraint,
    graph_slam::graph_slam,
    map::MapEdge,
    sensor_data::{init_landmark, Landmark, SensorData},
};
use crate::infrastructure::file;
use nalgebra as na;

/* run slam */
pub fn slam(path: &str) -> (Vec<(f32, f32, f32)>, Vec<Vec<SensorData>>, [Landmark; 6]) {
    /* Read pose data from the file */
    let (mut estimate_states, sensor_data, velocity) = file::pose_read(path);
    let mut delta_states;

    /* Compute landmark constraints */
    let landmark_list = constraint(&sensor_data);

    /* Optimize the SLAM problem using a fixed number of iterations. This constant could be parameterized */
    for _n in 0..3 {
        /* Run Graph SLAM algorithm */
        delta_states = graph_slam(&estimate_states, &velocity, &landmark_list);

        /* Update the estimated states using the computed delta states */
        for i in 0..estimate_states.len() {
            estimate_states[i].0 += delta_states[i * 3];
            estimate_states[i].1 += delta_states[i * 3 + 1];
            estimate_states[i].2 += delta_states[i * 3 + 2];
        }

        /* Check the convergence criteria and break the loop if it is satisfied */
        let diff = delta_states.norm();
        println!("diff:{}", diff);
        if diff < 0.02 {
            break;
        }
    }

    /* Initialize the landmarks */
    let mut landmarks: [Landmark; 6] = init_landmark();

    /* Iterate through the landmark list */
    for (id, landmark) in landmark_list.iter().enumerate() {
        let mut omega = na::Matrix3::<f32>::zeros();
        let mut xi = na::Vector3::<f32>::zeros();
        let head_t = landmark[0].0;
        let head_z = landmark[0].1.data.clone();

        /* Iterate through the landmarks and create MapEdges */
        for mark in landmark.iter().take(landmark_list.len()) {
            let mut edge = MapEdge::new();
            let t = mark.0;
            let z = mark.1.data.clone();
            edge.land_matrix(t, z, head_t, head_z.clone(), &estimate_states);
            omega += edge.omega;
            xi += edge.xi;
        }

        /* Update the landmark poses */
        landmarks[id].id = id;
        landmarks[id].pose = (omega.try_inverse().unwrap()) * xi;
    }

    /* Write the SLAM results to a file */
    file::slam_write(&estimate_states, &sensor_data, &landmarks);

    (estimate_states, sensor_data, landmarks)
}
