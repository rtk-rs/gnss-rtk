use crate::prelude::{Constellation, Epoch, InterpolationResult, SV};
use std::str::FromStr;

pub fn interp_data() -> [(Epoch, SV, InterpolationResult); 6] {
    [
        (
            Epoch::from_str("2020-06-25T12:00:0 GPST").unwrap(),
            SV::new(Constellation::GPS, 1),
            InterpolationResult::from_position((10996.104343, -19841.200560, -13758.983598)),
        ),
        (
            Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
            SV::new(Constellation::GPS, 2),
            InterpolationResult::from_position((-21763.192092, 13697.004574, -5902.542198)),
        ),
        (
            Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
            SV::new(Constellation::GPS, 3),
            InterpolationResult::from_position((1812.402225, -15421.072633, -21621.927836)),
        ),
        (
            Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
            SV::new(Constellation::GPS, 5),
            InterpolationResult::from_position((-20632.475811, 4434.893522, 16106.178530)),
        ),
        (
            Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
            SV::new(Constellation::GPS, 6),
            InterpolationResult::from_position((-20945.449299, 2452.339321, -16121.005873)),
        ),
        (
            Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
            SV::new(Constellation::GPS, 7),
            InterpolationResult::from_position((-6945.099222, -14068.115087, 21704.860378)),
        ),
    ]
}
