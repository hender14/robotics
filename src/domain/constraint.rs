use super::sensor_data::{LandmarkData, SensorData};

/* The Constraint struct represents a constraint for a landmark observed from sensor data. */
#[derive(Clone)]
pub struct Constraint {
    pub id: usize,
    pub data: LandmarkData,
}

/* processes the input sensor data and generates a list of constraints for each observed landmark. */
pub fn constraint(sensor_data: &[Vec<SensorData>]) -> [Vec<(usize, Constraint)>; 6] {
    /* Initialize the list of landmark constraints */
    let mut landmark_list: [Vec<(usize, Constraint)>; 6] = Default::default();

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
