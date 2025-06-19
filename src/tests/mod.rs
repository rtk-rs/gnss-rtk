use anise::{
    constants::frames::EARTH_J2000,
    prelude::{Almanac, Epoch, Frame, Orbit},
};

use std::str::FromStr;

use crate::navigation::apriori::Apriori;

pub mod bias;
pub mod ephemeris;
pub mod fuzz;

mod bancroft;
mod candidate;
mod data;
mod number;
mod pool;
mod ppp;
mod pseudo_range;
mod time;
// mod navi;
mod cpp;
mod spp;

pub use data::*;
pub use number::TestNumber;

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

pub fn almanac() -> Almanac {
    Almanac::until_2035().unwrap_or_else(|e| panic!("Failed to build test Almanac: {}", e))
}

pub fn earth_frame() -> Frame {
    Almanac::until_2035()
        .unwrap_or_else(|e| panic!("Failed to build test Almanac: {}", e))
        .frame_from_uid(EARTH_J2000)
        .unwrap_or_else(|e| panic!("Failed to build test EARTH-J2000 frame: {}", e))
}

pub fn test_orbits() -> OrbitsData {
    let earth_frame = earth_frame();
    OrbitsData::new(earth_frame)
}

pub const REFERENCE_COORDS_ECEF_M: (f64, f64, f64) = (3582105.2910, 532589.7313, 5232754.8054);

pub fn reference_epoch() -> Epoch {
    Epoch::from_str("2020-06-25T00:00:00 GPST")
        .unwrap_or_else(|e| panic!("internal error: failed to build reference epoch: {}", e))
}

/// Builds reference [Apriori] position
pub fn reference_apriori(t: Epoch) -> Apriori {
    let earth_frame = earth_frame();
    let ref_orbit = reference_orbit(t, earth_frame);
    let apriori = Apriori::from_orbit(&ref_orbit, earth_frame);
    apriori
}

/// Builds reference [Apriori] position at reference [Epoch]
pub fn reference_apriori_at_ref_epoch() -> Apriori {
    reference_apriori(reference_epoch())
}

/// Expresses [REFERENCE_COORDS_ECEF_M] as ANISE [Orbit] at any [Epoch].
pub fn reference_orbit(t: Epoch, frame: Frame) -> Orbit {
    Orbit::from_position(
        REFERENCE_COORDS_ECEF_M.0 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.1 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.2 / 1.0E3,
        t,
        frame,
    )
}

/// Expresses [REFERENCE_COORDS_ECEF_M] as ANISE [Orbit] at initial reference [Epoch].
pub fn reference_orbit_at_ref_epoch(frame: Frame) -> Orbit {
    reference_orbit(reference_epoch(), frame)
}

#[test]
fn verify_reference_orbit() {
    let earth_frame = earth_frame();
    let reference_orbit = reference_orbit_at_ref_epoch(earth_frame);
    assert_eq!(
        reference_orbit.epoch,
        reference_epoch(),
        "invalid reference Epoch!"
    );
}
