use super::super::domain::map::MapEdge;
use super::super::domain::solver::MotionEdge;
use super::super::domain::solver::SensorEdge;
use super::super::infrastructure::file;
use itertools::Itertools;
use nalgebra as na;

pub fn slam(
    path: &str,
) -> (
    Vec<(f32, f32, f32)>,
    Vec<Vec<(f32, f32, f32, f32)>>,
    [[f32; 3]; 6],
) {
    let (mut hat_xs, zlist, us) = file::pose_read(path);
    let mut land_klist: [Vec<(f32, (f32, f32, f32, f32))>; 6] = Default::default();
    let xdim = hat_xs.len() * 3;
    for _n in 0..5 {
        let mut count = 0;

        for row in &zlist {
            for z in row {
                let l_id = z.0 as usize;
                land_klist[l_id].push((count as f32, *z));
            }
            count += 1;
        }

        let mut omega = na::DMatrix::<f32>::zeros(xdim, xdim);
        for i in 0..3 {
            omega[(i, i)] = 1000000.;
        }
        let mut xi = na::DVector::<f32>::zeros(xdim);

        for index in 0..land_klist.len() {
            for comb in land_klist[index].iter().combinations(2) {
                let mut sensordges =
                    SensorEdge::new(comb[0].0, comb[1].0, comb[0].1, comb[1].1, hat_xs.clone());
                sensordges.precision_matrix();
                (omega, xi) = sensordges.add_edge(omega, xi);

                for i in 0..hat_xs.len() - 1 {
                    let mut motionedges = MotionEdge::new(i, i + 1, hat_xs.clone());
                    motionedges.action_matrix(us.clone(), 1.);
                    (omega, xi) = motionedges.add_edge(omega, xi);
                }
            }
        }
        // println!("omega:{}, xi:{}", omega, xi);

        let delta_xs = (omega.try_inverse().unwrap()) * xi;
        for i in 0..hat_xs.len() {
            hat_xs[i].0 += delta_xs[i * 3];
            hat_xs[i].1 += delta_xs[i * 3 + 1];
            hat_xs[i].2 += delta_xs[i * 3 + 2];
            // println!("x:{}, y:{}, theta:{}", x, y, theta);
        }

        let diff = delta_xs.norm();
        println!("diff:{}", diff);
        if diff < 0.02 {
            break;
        }
    }

    let mut land: [[f32; 3]; 6] = Default::default();
    for id in 0..land_klist.len() {
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
        land[id] = ((omega.try_inverse().unwrap()) * xi).into();
    }

    file::slam_write(hat_xs.clone(), zlist.clone(), land);

    (hat_xs, zlist, land)
}
