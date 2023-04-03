use nalgebra as na;

pub struct LandmarkData {
    pub distance: f64,
    pub angle: f64,
    pub psi: f64,
}

pub struct SensorData {
    pub timestamp: u64,
    pub landmark_data: LandmarkData,
}

// impl SensorData {
//   pub fn new(timestamp: u64, range_data: Vec<f64>) -> Self {
//       Self {
//           timestamp,
//           range_data,
//       }
//   }
// }

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
