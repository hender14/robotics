use nalgebra as na;

/* LandmarkData holds polar coordinates and orientation of a landmark relative to the robot */
#[derive(Clone, Debug)]
pub struct LandmarkData {
    pub polor: na::Matrix2x1<f32>,
    pub psi: f32,
}

/* SensorData holds data obtained from sensors for a specific landmark */
#[derive(Clone, Debug)]
pub struct SensorData {
    pub id: usize,
    pub timestamp: usize,
    pub result: bool,
    pub data: LandmarkData,
}

/* Landmark represents a landmark with an ID and a pose (x, y, theta) */
#[derive(Debug)]
pub struct Landmark {
    pub id: usize,
    pub pose: na::Vector3<f32>,
}

/* init_landmark initializes an array of Landmark instances with default values */
pub fn init_landmark() -> [Landmark; 6] {
    let pose = [0, 1, 2, 3, 4, 5]
        .iter()
        .map(|&id| Landmark {
            id,
            pose: na::Vector3::new(0., 0., 0.),
        })
        .collect::<Vec<Landmark>>()
        .try_into()
        .unwrap_or_else(|_| panic!("Failed to initialize sensor_data_array."));
    pose
}

/* data_init initializes an array of SensorData instances with default values */
pub fn data_init() -> [SensorData; 6] {
    let sensor_data: [SensorData; 6] = [0, 1, 2, 3, 4, 5]
        .iter()
        .map(|&id| SensorData {
            id,
            timestamp: 0,
            result: false,
            data: LandmarkData {
                polor: na::Matrix2x1::new(0., 0.),
                psi: 0.,
            },
        })
        .collect::<Vec<SensorData>>()
        .try_into()
        .unwrap_or_else(|_| panic!("Failed to initialize sensor_data_array."));

    sensor_data
}
