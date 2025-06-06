use anise::{
    constants::frames::EARTH_J2000,
    prelude::{Almanac, Epoch, Frame, Orbit},
};

use crate::navigation::apriori::Apriori;

pub mod bias;
pub mod ephemeris;

mod bancroft;
mod candidate;
mod data;
mod pool;
mod ppp;
mod pseudo_range;
mod time;
// mod navi;
mod spp;

pub use data::*;

use log::LevelFilter;
use std::sync::Once;

static INIT: Once = Once::new();

pub fn init_logger() {
    INIT.call_once(|| {
        env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .init();
    });
}

pub fn test_almanac() -> Almanac {
    Almanac::until_2035().unwrap_or_else(|e| panic!("Failed to build test Almanac: {}", e))
}

pub fn test_earth_frame() -> Frame {
    Almanac::until_2035()
        .unwrap_or_else(|e| panic!("Failed to build test Almanac: {}", e))
        .frame_from_uid(EARTH_J2000)
        .unwrap_or_else(|e| panic!("Failed to build test EARTH-J2000 frame: {}", e))
}

pub fn test_orbits() -> OrbitsData {
    let earth_frame = test_earth_frame();
    OrbitsData::new(earth_frame)
}

pub const REFERENCE_COORDS_ECEF_M: (f64, f64, f64) = (3628427.9118, 562059.0936, 5197872.2150);

/// Express [REFERENCE_COORDS_ECEF_M] as ANISE [Orbit].
pub fn test_reference_orbit(t: Epoch, frame: Frame) -> Orbit {
    Orbit::from_position(
        REFERENCE_COORDS_ECEF_M.0 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.1 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.2 / 1.0E3,
        t,
        frame,
    )
}

/// Builds reference [Apriori] position
pub fn test_reference_apriori() -> Apriori {
    let earth_frame = test_earth_frame();
    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();
    let ref_orbit = test_reference_orbit(t0_gpst, earth_frame);
    let apriori = Apriori::from_orbit(&ref_orbit, earth_frame);
    apriori
}

use std::str::FromStr;

/// Expresse [REFERENCE_COORDS_ECEF_M] as ANISE [Orbit] at initial reference [Epoch].
pub fn test_reference_orbit_t0_gpst(frame: Frame) -> Orbit {
    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    Orbit::from_position(
        REFERENCE_COORDS_ECEF_M.0 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.1 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.2 / 1.0E3,
        t0_gpst,
        frame,
    )
}

#[test]
fn verify_t0_gpst_reference_orbit() {
    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();
    let earth_frame = test_earth_frame();
    let reference_orbit = test_reference_orbit_t0_gpst(earth_frame);
    assert_eq!(reference_orbit.epoch, t0_gpst, "invalid T0 GPST Epoch!");
}
