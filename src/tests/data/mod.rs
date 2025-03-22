pub mod epochs;
pub mod gps;

pub mod orbits;

use crate::prelude::{Epoch, Frame, Orbit};
use std::str::FromStr;

pub const REFERENCE_COORDS_ECEF_M: (f64, f64, f64) = (3582105.2910, 532589.7313, 5232754.8054);

pub fn reference_orbit(fr: Frame) -> Orbit {
    Orbit::from_position(
        REFERENCE_COORDS_ECEF_M.0 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.1 / 1.0E3,
        REFERENCE_COORDS_ECEF_M.2 / 1.0E3,
        Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap(),
        fr,
    )
}
