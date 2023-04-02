// use core::slice::Iter;
// use itertools::{Combinations, Itertools};

pub struct Constraint {
    pub from_t: usize,
    pub to_t: usize,
    pub from_vector: Vec<Vec<(f32, f32, f32, f32)>>,
    pub to_vector: Vec<Vec<(f32, f32, f32, f32)>>,
}

// impl Constraint {
//   pub fn new(from_id: usize, to_id: usize, transformation: Transformation, information_matrix: Matrix6) -> Self {
//       Self {
//           from_id,
//           to_id,
//           transformation,
//           information_matrix,
//       }
//   }

//   // その他の制約に関するメソッドを定義
// }

// // 制約に関連する他の構造体や関数もここで定義
pub fn constraint(zlist: &Vec<Vec<(f32, f32, f32, f32)>>) -> [Vec<(f32, (f32, f32, f32, f32))>; 6] {
    let mut land_klist: [Vec<(f32, (f32, f32, f32, f32))>; 6] = Default::default();
    let mut count = 0;

    /* 以下処理はループに入れる必要がないのでは? */
    for row in zlist {
        for z in row {
            let l_id = z.0 as usize;
            land_klist[l_id].push((count as f32, *z));
        }
        count += 1;
    }
    land_klist
}
