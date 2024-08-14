use crate::prelude::{Constellation, Epoch, Orbit, EARTH_J2000, SV};
use std::str::FromStr;

pub fn interp_data() -> [(SV, Orbit); 1] {
    [(
        SV::new(Constellation::GPS, 1),
        Orbit::from_position(
            10996.104343,
            -19841.200560,
            -13758.983598,
            Epoch::from_str("2020-06-25T12:00:0 GPST").unwrap(),
            EARTH_J2000,
        ),
    )]
}
