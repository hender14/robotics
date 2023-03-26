use super::super::common::landmark;
use super::super::common::read;
use plotters::{coord::types::RangedCoordf64, prelude::*};

pub fn plot_pose(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
) {
    // let array = get_data();
    let array = read::pose_read();
    // x軸 : 日付のVector
    let xs: Vec<f64> = array.0.iter().map(|(x, _, _)| *x as f64).collect();
    // y軸: 値のVector
    let ys: Vec<f64> = array.0.iter().map(|(_, y, _)| *y as f64).collect();

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

pub fn plot_land(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
) {
    let array = landmark::get_land();
    // let array = read();
    // x軸 : 日付のVector
    let xs: Vec<f64> = array.iter().map(|(x, _)| *x as f64).collect();
    // y軸: 値のVector
    let ys: Vec<f64> = array.iter().map(|(_, y)| *y as f64).collect();

    let data: Vec<(f64, f64)> = xs.iter().zip(ys.iter()).map(|(&x, &y)| (x, y)).collect();
    // let data = (-3..=3)
    // 	.map(|x| x as f64 )
    // 	.map(|x| (x, x.sin()))
    // 	.collect::<Vec<_>>();

    // let line_series = Circle::new(data.iter().copied(), 2, &RED); //直線
    // chart.draw_series(line_series).unwrap();
    let point_series = PointSeries::of_element(data, 5, &BLACK, &|c, s, st| {
        return EmptyElement::at(c)  // Position
                    + Circle::new((0, 0), s, st.filled()); // Circle shape
    });

    chart.draw_series(point_series).unwrap();
}

pub fn plot_edge(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
) {
    let array = read::pose_read();
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
            println!(
                "x1:{}, y1:{}, x2:{}, y2:{}, ell:{}, phi:{}",
                x1, y1, x2, y2, ell, phi
            );
            // let poly: Polyline = [(x1 as f64, y1 as f64), (x2 as f64, y2 as f64)].iter().copied().into_polyline();
            // let line_style = BLACK.stroke_width(2);
            // let line_series = PathElement::new(poly, line_style);

            let line_series = LineSeries::new(
                [(x1 as f64, y1 as f64), (x2 as f64, y2 as f64)]
                    .iter()
                    .copied(),
                &BLUE,
            ); //直線

            // let line_series = LineSeries::new(vec![(x1_64, y1_64), (x2_64, y2_64)], &BLACK); //直線
            // chart.draw_series(std::iter::once(line_series)).unwrap();
            chart.draw_series(line_series).unwrap();
        }
    }
}
