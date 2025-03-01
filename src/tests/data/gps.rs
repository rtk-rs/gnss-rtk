use crate::prelude::{Constellation, SV};

pub const G01: SV = SV::new(Constellation::GPS, 1);
pub const G05: SV = SV::new(Constellation::GPS, 5);
pub const G07: SV = SV::new(Constellation::GPS, 7);
pub const G09: SV = SV::new(Constellation::GPS, 9);
pub const G13: SV = SV::new(Constellation::GPS, 13);
pub const G15: SV = SV::new(Constellation::GPS, 15);

pub const GPS_EPOCHS: [&str; 1] = ["2020-06-25T00:00:00 GPST"];

#[cfg(test)]
mod test {
    use super::GPS_EPOCHS;
    use crate::prelude::Epoch;
    use std::str::FromStr;
    #[test]
    fn epochs_validity() {
        for t in GPS_EPOCHS.iter() {
            let _ = Epoch::from_str(*t).unwrap();
        }
    }
}
