pub mod gps;

mod complex;
pub use complex::ComplexItem;

mod error;
pub use error::ParsingError;

mod observable;
pub use observable::Observable;

use crate::prelude::{Epoch, Observation, SV};
use std::str::FromStr;

#[derive(Debug, Clone, PartialEq)]
pub struct DataPoint {
    t: Epoch,
    sv: SV,
    observations: Vec<Observation>,
}

impl DataPoint {
    /// Build a new test [DataPoint] at Epoch description, SV description,
    /// and desc_csv: "l1:pr:10.0,l1:cp:20.0,l2:pr:20.0"
    pub fn new(t: &str, sv: &str, desc_csv: &str) -> Self {
        let t = Epoch::from_str(t).unwrap_or_else(|_| {
            panic!(
                "Bad epoch description: \"{}\" (for sv={}/desc={}",
                t, sv, desc_csv
            )
        });
        let sv = SV::from_str(sv.trim()).unwrap_or_else(|_| {
            panic!(
                "Bad SV description: \"{}\" (for t={}/desc={})",
                sv, t, desc_csv
            )
        });

        let mut observations = Vec::<Observation>::new();

        for content in desc_csv.split(',') {
            let trimmed = content.trim();
            let complex =
                ComplexItem::from_str(trimmed).unwrap_or_else(|e| panic!("Parsing error: {}", e));

            let observation = complex.to_observation();

            if let Some(obs) = observations
                .iter_mut()
                .find(|ob| ob.carrier == observation.carrier)
            {
                if let Some(pr) = observation.pseudo_range_m {
                    obs.pseudo_range_m = Some(pr);
                } else if let Some(cp) = observation.phase_range_m {
                    obs.phase_range_m = Some(cp);
                } else if let Some(dop) = observation.doppler {
                    obs.doppler = Some(dop);
                } else if let Some(snr) = observation.snr_dbhz {
                    obs.snr_dbhz = Some(snr);
                }
            } else {
                observations.push(observation);
            }
        }
        Self {
            t,
            sv,
            observations,
        }
    }
}

use crate::prelude::Carrier;
use crate::tests::data::gps::G01;

#[test]
fn test_data_parser() {
    let dp = DataPoint::new("2020-01-01T00:00:00 GPST", "G01", "L1:cp:10.0");
    assert_eq!(dp.sv, G01);
    assert_eq!(dp.observations.len(), 1);
    let observation = &dp.observations[0];
    assert_eq!(observation.carrier, Carrier::L1);
    assert_eq!(observation.phase_range_m, Some(10.0));
}
