#[cfg(test)]
mod tests {
    use robotics;
    use robotics::domain::sensor_data::Landmark;
    use robotics::infrastructure::config;
    use robotics::infrastructure::file;
    use robotics::usecase::map_create as map;

    #[test]
    fn test_slam() {
        /* init */
        file::directry_init();

        /* Output specification */
        let ans = config::get_landmark();

        let (_, _, out) = map::slam(file::KFPATH);
        println!("{:?}", out);

        /* validate */
        let res = validate(&ans, &out);
        /* test */
        assert!(res, "\nans:{:?}  out:{:?}", &ans, &out);
    }

    /* validate */
    fn validate(ans: &[Landmark], out: &[Landmark]) -> bool {
        let mut flag = true;
        for i in 0..6 {
            for j in 0..2 {
                if (ans[i].pose[j] - out[i].pose[j]).abs() > 1. {
                    flag = false;
                    println!(
                        "ans: {} out: {} err: {}",
                        ans[i].pose[j],
                        out[i].pose[j],
                        (ans[i].pose[j] - out[i].pose[j]).abs()
                    );
                    break;
                }
            }
        }
        flag
    }
}
