use crate::prelude::{Ephemeris, EphemerisSource, Epoch, SV};

pub struct NullEph {}

impl EphemerisSource for NullEph {
    fn ephemeris_data(&self, _: Epoch, _: SV) -> Option<Ephemeris> {
        None
    }
}
