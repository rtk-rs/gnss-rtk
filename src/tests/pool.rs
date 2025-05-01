use crate::{
    pool::Pool,
    prelude::{Config, Epoch, Frame},
    tests::{data::CandidatesBuilder, init_logger, time::NullTime, OrbitsData},
};

use rstest::*;
use std::str::FromStr;

#[fixture]
fn build_earth_frame() -> Frame {
    use crate::tests::test_earth_frame;
    test_earth_frame()
}

#[fixture]
fn build_orbit_source() -> OrbitsData {
    use crate::tests::test_orbits;
    test_orbits()
}

#[test]
fn test_simple_pool_fit() {
    init_logger();

    let null_time = NullTime {};
    let default_cfg = Config::default();

    let earth_frame = build_earth_frame();
    let mut orbits_data = build_orbit_source();

    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    let candidates = CandidatesBuilder::build_at(t0_gpst);

    let mut pool = Pool::allocate(0, earth_frame);

    pool.new_epoch(&candidates);

    assert_eq!(
        pool.candidates().len(),
        candidates.len(),
        "builder dropped some data!"
    );

    pool.pre_fit(&default_cfg, &null_time);

    pool.orbital_states(&default_cfg, &mut orbits_data);

    assert_eq!(
        pool.candidates().len(),
        candidates.len(),
        "builder dropped some data!"
    );
}
