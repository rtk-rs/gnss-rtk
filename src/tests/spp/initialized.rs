use crate::{
    prelude::{Almanac, Config, Solver, EARTH_J2000},
};

#[test]
fn gps_l1() {
    
    let almanac = Almanac::until_2035().unwrap();
    let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

    // Testing multiple presets
    // Default preset should always deploy and give results 
    let cfg = Config::default();


    let mut solver = Solver::new(cfg, gps_data.clone(), None)
        .unwrap_or_else(|e| panic!("Failed to deploy with default setup: {}", e));
    
    for t in J2020_06_25_GPS_EPOCHS.iter().map(|t_str| Epoch::from_str(t_str).unwrap()) {

        let candidates = gps_data.co

        solver.resolve(t, &pool);
    }
    
}