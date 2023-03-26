use std::fs::File;
use std::io::{BufRead, BufReader};

pub fn pose_read() -> (
  Vec<(f32, f32, f32)>,
  Vec<Vec<(f32, f32, f32, f32)>>,
  Vec<(f32, f32)>,
) {
  let mut hat_xs: Vec<(f32, f32, f32)> = vec![];
  let mut zlist: Vec<Vec<(f32, f32, f32, f32)>> = vec![vec![]];
  let mut us: Vec<(f32, f32)> = vec![];
  let file = File::open("output.txt").expect("Failed to open file");
  let reader = BufReader::new(file);
  for line in reader.lines() {
      let line = line.unwrap();
      let array: Vec<f32> = line
          .split_whitespace()
          .map(|s| s.parse().unwrap())
          .collect();
      // println!("{:?}", array);
      let step: usize = array[1] as usize;
      if array[0] == 0. {
          if step >= us.len() {
              us.resize(step + 1, (0.0, 0.0));
          }
          us[step] = (array[2], array[3]);
      } else if array[0] == 1. {
          if step >= hat_xs.len() {
              hat_xs.resize(step + 1, (0.0, 0.0, 0.0));
          }
          hat_xs[step] = (array[2], array[3], array[4]);
      } else if array[0] == 2. {
          if step >= zlist.len() {
              zlist.resize(step + 1, vec![(array[2], array[3], array[4], array[5])]);
          } else {
              zlist[step].push((array[2], array[3], array[4], array[5]));
          }
      }
  }
  (hat_xs, zlist, us)
}