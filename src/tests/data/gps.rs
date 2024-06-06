use crate::{
    prelude::{
        Candidate, Carrier, Constellation, Duration, Epoch, IonosphereBias, PhaseRange,
        PseudoRange, TroposphereBias, SV,
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
                    Duration::from_seconds(0.162520179759E-04),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 2),
                    Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
                    Duration::from_seconds(-0.477580320500E-03),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 3),
                    Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
                    Duration::from_seconds(-0.220043185257E-03),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 5),
                    Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
                    Duration::from_seconds(-0.153530275954E-04),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
            ],
            iono_bias: IonosphereBias {
                kb_model: None,
                bd_model: None,
                ng_model: None,
                stec_meas: None,
            },
            tropo_bias: TroposphereBias {
                total: None,
                zwd_zdd: None,
            },
        },
        SolverInput {
            t_rx: Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
            pool: vec![
                Candidate::new(
                    SV::new(Constellation::GPS, 1),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    Duration::from_seconds(0.162520179759E-04),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            ambiguity: None,
                            snr: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            ambiguity: None,
                            snr: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 2),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    Duration::from_seconds(-0.477580320500E-03),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            ambiguity: None,
                            snr: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 3),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    Duration::from_seconds(-0.220043185257E-03),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
                Candidate::new(
                    SV::new(Constellation::GPS, 5),
                    Epoch::from_str("2020-06-25T12:00:30 GPST").unwrap(),
                    Duration::from_seconds(-0.153530275954E-04),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![
                        PseudoRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                        PseudoRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                        },
                    ],
                    vec![
                        PhaseRange {
                            carrier: Carrier::L1,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L2,
                            value: 1.0E6_f64,
                            ambiguity: None,
                            snr: None,
                        },
                        PhaseRange {
                            carrier: Carrier::L5,
                            value: 1.0E6_f64,
                            snr: None,
                            ambiguity: None,
                        },
                    ],
                ),
            ],
            iono_bias: IonosphereBias {
                kb_model: None,
                bd_model: None,
                ng_model: None,
                stec_meas: None,
            },
            tropo_bias: TroposphereBias {
                total: None,
                zwd_zdd: None,
            },
        },
    ]
}
