use super::sensor_data::{LandmarkData, SensorData};

#[derive(Clone)]
pub struct Constraint {
    pub id: usize,
    pub data: LandmarkData,
}

// // 制約に関連する他の構造体や関数もここで定義
pub fn constraint(sensor_data: &[Vec<SensorData>]) -> [Vec<(usize, Constraint)>; 6] {
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
