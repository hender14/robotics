use nalgebra as na;

#[derive(Clone)]
pub struct LandmarkData {
    pub polor: na::Matrix2x1<f32>,
    pub psi: f32,
}

#[derive(Clone)]
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
