use crate::prelude::Epoch;
use std::str::FromStr;

const SIZE: usize = 4;

pub const EPOCHS_DESCRIPTOR: [&str; SIZE] = [
    "2020-06-25T00:00:00 GPST",
    "2020-06-25T00:00:30 GPST",
    "2020-06-25T00:01:00 GPST",
    "2020-06-25T00:01:30 GPST",
];

pub struct EpochDataSet {}

impl EpochDataSet {
    pub fn build() -> Vec<Epoch> {
        EPOCHS_DESCRIPTOR
            .iter()
            .map(|s| Epoch::from_str(s).unwrap())
            .collect()
    }
}

#[test]
fn validity() {
    for t_str in EPOCHS_DESCRIPTOR {
        let _ = Epoch::from_str(t_str).unwrap();
    }
}
