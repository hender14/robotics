use std::vec;

// use core::slice::Iter;
// use itertools::{Combinations, Itertools};
use super::sensor_data::{LandmarkData, SensorData};

// pub struct Constraint {
//     pub from_t: usize,
//     pub to_t: usize,
//     pub from_vector: Vec<Vec<(f32, f32, f32, f32)>>,
//     pub to_vector: Vec<Vec<(f32, f32, f32, f32)>>,
// }

#[derive(Clone)]
pub struct Constraint {
    pub id: usize,
    pub data: LandmarkData,
}

// // 制約に関連する他の構造体や関数もここで定義
pub fn constraint(sensor_data: &Vec<Vec<SensorData>>) -> [Vec<(usize, Constraint)>; 6] {
    let mut landmark_list: [Vec<(usize, Constraint)>; 6] = Default::default();

    /* 以下処理はループに入れる必要がないのでは? */
    for (i, row) in sensor_data.iter().enumerate() {
        for data in row {
            landmark_list[data.id].push((
                i,
                (Constraint {
                    id: data.id,
                    data: data.data.clone(),
                }),
            ));
        }
    }
    landmark_list
}
