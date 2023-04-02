#[cfg(test)]
mod tests {
    use robotics;
    use robotics::infrastructure::config;
    use robotics::infrastructure::file;
    use robotics::usecase::map_create as map;

    #[test]
    fn test_slam() {
        /* init */
        config::init();

        /* Output specification */
        let (ans, _) = config::dec_landmark();

        let (_, _, out) = map::slam(file::KFPATH);

        /* validate */
        println!("{:?}", out);
        let res = validate(&ans, &out);
        /* test */
        assert!(res, "\nans:{:?}  out:{:?}", &ans, &out);
    }

    /* validate */
    fn validate(ans: &[[f32; 3]; 6], out: &[[f32; 3]; 6]) -> bool {
        let mut flag = true;
        for i in 0..6 {
            for j in 0..3 {
                if (ans[i][j] - out[i][j]).abs() > 0.5 {
                    flag = false;
                    break;
                }
            }
        }
        flag
    }
}
