use crate::{
    prelude::{
        Candidate, Carrier, ClockCorrection, Constellation, Duration, Epoch, IonoComponents,
        Observation, TropoComponents, SV,
    },
    tests::SolverInput,
};

use std::str::FromStr;

/*
 * Test Dataset
 */
pub fn test_data() -> [SolverInput; 2] {
    [
        SolverInput {
            t_rx: Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
            pool: vec![
                Candidate::new(
                    SV::new(Constellation::GPS, 1),
                    Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
                    ClockCorrection::without_relativistic_correction(Duration::from_seconds(
                        0.162520179759E-04,
                    )),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        carrier: Carrier::L1,
                        pseudo: Some(1.0E6_f64),
                        snr: None,
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                    IonoComponents::Unknown,
                    TropoComponents::Unknown,
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 2),
                    Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
                    ClockCorrection::without_relativistic_correction(Duration::from_seconds(
                        -0.477580320500E-03,
                    )),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        carrier: Carrier::L1,
                        pseudo: Some(1.0E6_f64),
                        snr: None,
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                    IonoComponents::Unknown,
                    TropoComponents::Unknown,
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 3),
                    Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
                    ClockCorrection::without_relativistic_correction(Duration::from_seconds(
                        -0.220043185257E-03,
                    )),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        carrier: Carrier::L1,
                        pseudo: Some(1.0E6_f64),
                        snr: None,
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                    IonoComponents::Unknown,
                    TropoComponents::Unknown,
                ),
            ],
        },
        SolverInput {
            t_rx: Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
            pool: vec![
                Candidate::new(
                    SV::new(Constellation::GPS, 1),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    ClockCorrection::without_relativistic_correction(Duration::from_seconds(
                        0.162520179759E-04,
                    )),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        carrier: Carrier::L1,
                        pseudo: Some(1.0E6_f64),
                        snr: None,
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                    IonoComponents::Unknown,
                    TropoComponents::Unknown,
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 2),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    ClockCorrection::without_relativistic_correction(Duration::from_seconds(
                        -0.477580320500E-03,
                    )),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        carrier: Carrier::L1,
                        pseudo: Some(1.0E6_f64),
                        snr: None,
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                    IonoComponents::Unknown,
                    TropoComponents::Unknown,
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 3),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    ClockCorrection::without_relativistic_correction(Duration::from_seconds(
                        -0.220043185257E-03,
                    )),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        snr: None,
                        carrier: Carrier::L1,
                        pseudo: Some(1.0E6_f64),
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                    IonoComponents::Unknown,
                    TropoComponents::Unknown,
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 5),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    ClockCorrection::without_relativistic_correction(Duration::from_seconds(
                        -0.153530275954E-04,
                    )),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        carrier: Carrier::L1,
                        pseudo: Some(1.0E6_f64),
                        snr: None,
                        phase: None,
                        doppler: None,
                        ambiguity: None,
                    }],
                    IonoComponents::Unknown,
                    TropoComponents::Unknown,
                ),
            ],
        },
    ]
}
