use plotters::{prelude::*, coord::types::RangedCoordf64};
// use plotters::{prelude::*, coord::types::RangedCoordf64};
use itertools::Itertools;
use std::fs::File;
use std::io::{BufRead, BufReader};

// pub struct Plot <'a> {
//   pchart: ChartContext<'a, BitMapBackend<'a>, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
// //   chart: ChartContext<'a, BitMapBackend<'a>, Cartesian2d<f64, f64>>
// }

// impl <'a> Plot<'a> {
// 	pub fn new(root: DrawingArea<BitMapBackend<'a>, Shift>) -> Result<Self, Box<dyn std::error::Error>> {
// 		// let root:  = BitMapBackend::new("plot.png", (640, 480)).into_drawing_area();
// 		root.fill(&WHITE)?;

// 		let mut chart = ChartBuilder::on(&root)
// 			.caption("sin function", ("sans-serif", 30))
// 			.x_label_area_size(30)
// 			.y_label_area_size(30)
// 			.build_cartesian_2d(-5. ..5., -5. ..5.)?;

// 		chart.configure_mesh().draw()?;
		
// 		Ok(Self { pchart: chart })
// 	}
// }
	pub fn plot(chart: & mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>) {
		// let array = get_data();
        let array = read();
		    // x軸 : 日付のVector
        let xs: Vec<f64> = array.0.iter()
        .map(|(x, _, _)| *x as f64)
        .collect();
		// y軸: 値のVector
		let ys: Vec<f64> = array.0.iter()
		.map(|(_, y, _)| *y as f64)
		.collect();

        let data: Vec<(f64, f64)> = xs.iter().zip(ys.iter()).map(|(&x, &y)| (x, y)).collect();
		// let data = (-3..=3)
		// 	.map(|x| x as f64 )
		// 	.map(|x| (x, x.sin()))
		// 	.collect::<Vec<_>>();

        let line_series = LineSeries::new(data.iter().copied(), &RED); //直線
        // chart.draw_series(LineSeries::new(data, &RED)).unwrap(); //直線
		// let line_series = LineSeries::new(xs.iter().zip(ys.iter()).map(|(x, y)| (*x, *y)), &RED); //直線
		// chart.draw_series(data.iter().map(|(x, y)| Circle::new((*x, *y), 3, RED.filled())),)?;//線無し
        chart.draw_series(line_series).unwrap();
	}

	pub fn plot2(chart: & mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>) {
		let array = get_data();
        // let array = read();
		    // x軸 : 日付のVector
        let xs: Vec<f64> = array.iter()
        .map(|(x, _)| *x as f64)
        .collect();
		// y軸: 値のVector
		let ys: Vec<f64> = array.iter()
		.map(|(_, y)| *y as f64)
		.collect();

        let data: Vec<(f64, f64)> = xs.iter().zip(ys.iter()).map(|(&x, &y)| (x, y)).collect();
		// let data = (-3..=3)
		// 	.map(|x| x as f64 )
		// 	.map(|x| (x, x.sin()))
		// 	.collect::<Vec<_>>();

        // let line_series = Circle::new(data.iter().copied(), 2, &RED); //直線
        // chart.draw_series(line_series).unwrap();
        let point_series = PointSeries::of_element(
            data,
            5,
            &BLACK,
            &|c, s, st| {
                return EmptyElement::at(c)  // Position
                    + Circle::new((0, 0), s, st.filled()); // Circle shape
            },
        );
    
        chart.draw_series(point_series).unwrap();
	}

    pub fn plot3(chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>) {
        let array = read();
        for ar in 1..array.0.len() {
            let x1 = array.0[ar].0;
            let y1 = array.0[ar].1;
            let theta = array.0[ar].2;
            let zarray = &array.1[ar];
            // let zsize =  std::mem::size_of_val(zarray) / std::mem::size_of::<f32>();
            let zsize = zarray.len();
            for i in 0..zsize {
                let ell = zarray[i].1;
                let phi = zarray[i].2;
                let x2: f32 = x1 + ell * ((theta + phi).cos());
                let y2: f32 = y1 + ell * ((theta + phi).sin());
                println!("x1:{}, y1:{}, x2:{}, y2:{}, ell:{}, phi:{}", x1, y1, x2, y2, ell, phi);
                // let poly: Polyline = [(x1 as f64, y1 as f64), (x2 as f64, y2 as f64)].iter().copied().into_polyline();
                // let line_style = BLACK.stroke_width(2);
                // let line_series = PathElement::new(poly, line_style);

                let line_series = LineSeries::new([(x1 as f64, y1 as f64), (x2 as f64, y2 as f64)].iter().copied(), &BLUE); //直線

                // let line_series = LineSeries::new(vec![(x1_64, y1_64), (x2_64, y2_64)], &BLACK); //直線
                // chart.draw_series(std::iter::once(line_series)).unwrap();
                chart.draw_series(line_series).unwrap();

            }
        }
    }

    pub fn make_edges() {
        let mut count = 0;
        let array = read();
        // let mut land_klist: [Vec<(f32, f32, f32, f32)>;6] = Default::default();
        let mut land_klist: [Vec<(f32, (f32, f32, f32, f32))>;6] = Default::default();

        for row in array.1 {
            for z in row {
                let l_id = z.0 as usize;
                land_klist[l_id].push((count as f32, z));
            }
            count += 1;
        }
        // println!("land_klist: {:?}", land_klist);

        for index in 0..land_klist.len() {
            for comb in land_klist[index].iter().combinations(2) {
                println!("test: {:?}", comb[0].0);
                println!("test: {:?}", comb[1].1);
            }
        }
    }

    /// データの取得（固定値）
    pub fn get_data() -> Vec<(f32, f32)> {
        return vec![
            (-4., 2.), (2., -3.),
            (3., 3.), (0., 4.),
            (1., 1.), (-3., -1.),
        ];
    }

    pub fn read() -> ( Vec<(f32, f32, f32)>, Vec<Vec<(f32, f32, f32, f32)>>, Vec<(f32, f32)>){
        let mut hat_xs: Vec<(f32, f32, f32)> = vec![];
        let mut zlist: Vec<Vec<(f32, f32, f32, f32)>> = vec![vec![]];
        let mut us: Vec<(f32, f32)> = vec![];
        let file = File::open("output.txt").expect("Failed to open file");
        let reader = BufReader::new(file);
        for line in reader.lines() {
            let line = line.unwrap();
            let array: Vec<f32> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
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
                }
                else {
                    zlist[step].push((array[2], array[3], array[4], array[5]));
                }
                // let new_tuple = (array[2], array[3], array[4], array[5]);
                // zlist[step].1.push(new_tuple.0);
                // zlist[step].1.extend(new_tuple.1);
                // zlist[step].push(vec![(array[2], vec![array[3], array[4], array[5]])]);
            }
        }
        // println!("{:?}", hat_xs);
        // println!("{:?}", zlist);
        // println!("{:?}", us);
        // println!("{:?}", zlist[1].1);
        // println!("{:?}", zlist[1].1[1]);
        (hat_xs, zlist, us)
    }
    
