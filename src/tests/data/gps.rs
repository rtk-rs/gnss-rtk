use crate::prelude::{Constellation, SV};

pub const G01: SV = SV::new(Constellation::GPS, 1);

pub const GPS_EPOCHS: [&str; 1] = ["2020-06-25T00:00:00 GPST"];

// const GPS_DATA_POINTS : Vec<DataPoint> = vec![
//     DataPoint
// ];

// const GPS_CANDIDATES : Vec<Candidate> = vec![

// ]
// const GPS_EPOCHS : Vec<Epoch> = vec![
//     Epoch::str("2020-06-25T12:00:00 GPST").unwrap(),
//     Epoch::str("2020-06-25T12:00:30 GPST").unwrap(),
//     Epoch::str("2020-06-25T12:01:00 GPST").unwrap(),
//     Epoch::str("2020-06-25T12:01:30 GPST").unwrap(),
// ];

// const G01 : SV = SV::new(Constellation::GPS, 1);
// const G02 : SV = SV::new(Constellation::GPS, 1);
// const G03 : SV = SV::new(Constellation::GPS, 1);
// const G04 : SV = SV::new(Constellation::GPS, 1);

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
