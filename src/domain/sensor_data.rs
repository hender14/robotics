use nalgebra as na;

#[derive(Clone, Debug)]
pub struct LandmarkData {
    pub polor: na::Matrix2x1<f32>,
    pub psi: f32,
}

#[derive(Clone, Debug)]
pub struct SensorData {
    pub id: usize,
    pub timestamp: usize,
    pub result: bool,
    pub data: LandmarkData,
}

#[derive(Debug)]
pub struct Landmark {
    pub id: usize,
    pub pose: na::Vector3<f32>,
}

pub fn init_landmark() -> [Landmark; 6] {
    let pose: [Landmark; 6] = [
        Landmark {
            id: 0,
            pose: na::Vector3::new(0., 0., 0.),
        },
        Landmark {
            id: 1,
            pose: na::Vector3::new(0., 0., 0.),
        },
        Landmark {
            id: 2,
            pose: na::Vector3::new(0., 0., 0.),
        },
        Landmark {
            id: 3,
            pose: na::Vector3::new(0., 0., 0.),
        },
        Landmark {
            id: 4,
            pose: na::Vector3::new(0., 0., 0.),
        },
        Landmark {
            id: 5,
            pose: na::Vector3::new(0., 0., 0.),
        },
    ];
    pose
}

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
