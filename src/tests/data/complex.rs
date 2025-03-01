use crate::{
    prelude::{Carrier, Observation},
    tests::{Observable, ParsingError},
};

use std::str::FromStr;

#[derive(Debug, Clone, PartialEq)]
pub struct ComplexItem {
    pub freq: Carrier,
    pub observable: Observable,
    pub value: f64,
    pub snr: Option<f64>,
}

impl Default for ComplexItem {
    fn default() -> Self {
        Self {
            freq: Default::default(),
            observable: Observable::PseudoRange,
            value: Default::default(),
            snr: Default::default(),
        }
    }
}

impl std::fmt::Display for ComplexItem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(snr) = self.snr {
            write!(
                f,
                "{}:{}:{}/{}",
                self.freq, self.observable, self.value, snr
            )
        } else {
            write!(f, "{}:{}:{}", self.freq, self.observable, self.value)
        }
    }
}

impl std::str::FromStr for ComplexItem {
    type Err = ParsingError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let trimmed = s.trim();
        let mut items = 0;
        let mut s = Self::default();
        for (nth, item) in trimmed.split(':').enumerate() {
            let trimmed = item.trim();
            if nth == 0 {
                let freq = Carrier::from_str(trimmed)
                    .unwrap_or_else(|e| panic!("invalid freq: {} [{}] - {}", trimmed, s, e));

                s.freq = freq;
            } else if nth == 1 {
                let obs = Observable::from_str(trimmed)
                    .unwrap_or_else(|e| panic!("invalid obs: {} [{}] - {} ", trimmed, s, e));

                s.observable = obs;
            } else if nth == 2 {
                let value = trimmed
                    .parse::<f64>()
                    .unwrap_or_else(|e| panic!("obs value parsing: {} [{}] - {}", trimmed, s, e));

                s.value = value;
            } else if nth == 3 {
                let value = trimmed
                    .parse::<f64>()
                    .unwrap_or_else(|e| panic!("snr value parsing: {} [{}] - {}", trimmed, s, e));

                s.snr = Some(value);
            }
            items += 1;
        }
        if items > 1 {
            Ok(s)
        } else {
            panic!("invalid ComplexItem descriptor: \"{}\"", s);
        }
    }
}

impl ComplexItem {
    pub fn to_observation(&self) -> Observation {
        Observation {
            carrier: self.freq,
            pseudo_range_m: if self.observable == Observable::PseudoRange {
                Some(self.value)
            } else {
                None
            },
            phase_range_m: if matches!(
                self.observable,
                Observable::AmbiguousPhaseRange | Observable::UnambiguousPhaseRange
            ) {
                Some(self.value)
            } else {
                None
            },
            doppler: if self.observable == Observable::Doppler {
                Some(self.value)
            } else {
                None
            },
            ambiguity: if self.observable == Observable::AmbiguousPhaseRange {
                // TODO
                Some(0.0)
            } else {
                None
            },
            snr_dbhz: self.snr,
        }
    }
}

#[test]
fn test_parser() {
    let item = ComplexItem::from_str("L1:pr:10.0").unwrap();

    assert_eq!(item.freq, Carrier::L1);
    assert_eq!(item.observable, Observable::PseudoRange);
    assert_eq!(item.value, 10.0);
    assert!(item.snr.is_none());

    let item = ComplexItem::from_str("L5:cp:12.0").unwrap();

    assert_eq!(item.freq, Carrier::L5);
    assert_eq!(item.observable, Observable::AmbiguousPhaseRange);
    assert_eq!(item.value, 12.0);
    assert!(item.snr.is_none());

    let observation = item.to_observation();
    assert_eq!(observation.ambiguity, Some(0.0));

    let item = ComplexItem::from_str("E1:dop:12.0").unwrap();

    assert_eq!(item.freq, Carrier::E1);
    assert_eq!(item.observable, Observable::Doppler);
    assert_eq!(item.value, 12.0);
    assert!(item.snr.is_none());

    let item = ComplexItem::from_str("E5:dop:12.0:3.0").unwrap();

    assert_eq!(item.freq, Carrier::E5);
    assert_eq!(item.observable, Observable::Doppler);
    assert_eq!(item.value, 12.0);
    assert_eq!(item.snr, Some(3.0));
}
