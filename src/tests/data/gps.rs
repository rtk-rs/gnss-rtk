use std::str::FromStr;

use crate::{
    prelude::{Candidate, Carrier, Constellation, Epoch, Observation, SV},
    tests::epochs::EPOCHS_DESCRIPTOR,
};

pub const G01: SV = SV::new(Constellation::GPS, 1);
pub const G02: SV = SV::new(Constellation::GPS, 2);
pub const G03: SV = SV::new(Constellation::GPS, 3);
pub const G04: SV = SV::new(Constellation::GPS, 4);
pub const G05: SV = SV::new(Constellation::GPS, 5);
pub const G07: SV = SV::new(Constellation::GPS, 7);
pub const G08: SV = SV::new(Constellation::GPS, 8);
pub const G09: SV = SV::new(Constellation::GPS, 9);
pub const G13: SV = SV::new(Constellation::GPS, 13);
pub const G15: SV = SV::new(Constellation::GPS, 15);

pub struct Candidates {}

impl Candidates {
    /// GPS (exclusively) [Candidate] pool.
    pub fn builder() -> Vec<Candidate> {
        vec![
            Candidate::new(
                G02,
                Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                vec![Observation {
                    carrier: Carrier::L1,
                    pseudo_range_m: Some(25847357.745),
                    doppler: Some(-3123.088),
                    ambiguity: None,
                    snr_dbhz: None,
                    phase_range_m: None,
                }],
            ),
            Candidate::new(
                G05,
                Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                vec![
                    Observation {
                        carrier: Carrier::L1,
                        pseudo_range_m: Some(20947300.931),
                        phase_range_m: Some(110078836.389),
                        doppler: Some(-1037.205),
                        snr_dbhz: None,
                        ambiguity: None,
                    },
                    Observation {
                        carrier: Carrier::L2,
                        pseudo_range_m: Some(20947300.413),
                        phase_range_m: Some(85775729.71809),
                        doppler: Some(-808.180),
                        snr_dbhz: None,
                        ambiguity: None,
                    },
                ],
            ),
            Candidate::new(
                G07,
                Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                vec![
                    Observation {
                        carrier: Carrier::L1,
                        pseudo_range_m: Some(21777182.297),
                        phase_range_m: Some(114439911.635),
                        doppler: Some(-1843.922),
                        snr_dbhz: None,
                        ambiguity: None,
                    },
                    Observation {
                        carrier: Carrier::L2,
                        pseudo_range_m: Some(21777181.716),
                        phase_range_m: Some(89173970.254),
                        doppler: Some(-1436.857),
                        snr_dbhz: None,
                        ambiguity: None,
                    },
                ],
            ),
            Candidate::new(
                G08,
                Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                vec![
                    Observation {
                        carrier: Carrier::L1,
                        pseudo_range_m: Some(21777182.297),
                        phase_range_m: Some(114439911.635),
                        doppler: Some(-1843.922),
                        snr_dbhz: None,
                        ambiguity: None,
                    },
                    Observation {
                        carrier: Carrier::L2,
                        pseudo_range_m: Some(21777181.716),
                        phase_range_m: Some(89173970.254),
                        doppler: Some(-1436.857),
                        snr_dbhz: None,
                        ambiguity: None,
                    },
                ],
            ),
            Candidate::new(
                G09,
                Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                vec![
                    Observation {
                        carrier: Carrier::L1,
                        snr_dbhz: None,
                        ambiguity: None,
                        pseudo_range_m: Some(24545460.880),
                        doppler: Some(-3465.631),
                        phase_range_m: None,
                    },
                    Observation {
                        carrier: Carrier::L2,
                        pseudo_range_m: Some(24545462.948),
                        phase_range_m: Some(128987295.999),
                        doppler: Some(-2700.368),
                        ambiguity: None,
                        snr_dbhz: None,
                    },
                    Observation {
                        carrier: Carrier::L5,
                        pseudo_range_m: Some(24545458.453),
                        phase_range_m: Some(96321695.874),
                        doppler: Some(-2700.489),
                        ambiguity: None,
                        snr_dbhz: None,
                    },
                ],
            ),
            Candidate::new(
                G13,
                Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                vec![
                    Observation {
                        carrier: Carrier::L1,
                        snr_dbhz: None,
                        ambiguity: None,
                        doppler: None,
                        pseudo_range_m: Some(21695570.939),
                        phase_range_m: None,
                    },
                    Observation {
                        carrier: Carrier::L2,
                        snr_dbhz: None,
                        ambiguity: None,
                        pseudo_range_m: None,
                        phase_range_m: None,
                        doppler: None,
                    },
                    Observation {
                        carrier: Carrier::L5,
                        snr_dbhz: None,
                        ambiguity: None,
                        pseudo_range_m: None,
                        phase_range_m: None,
                        doppler: None,
                    },
                ],
            ),
            Candidate::new(
                G15,
                Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                vec![
                    Observation {
                        carrier: Carrier::L1,
                        snr_dbhz: None,
                        ambiguity: None,
                        pseudo_range_m: Some(24050353.947),
                        phase_range_m: Some(126385473.468),
                        doppler: None,
                    },
                    Observation {
                        carrier: Carrier::L2,
                        snr_dbhz: None,
                        ambiguity: None,
                        doppler: None,
                        phase_range_m: None,
                        pseudo_range_m: None,
                    },
                ],
            ),
        ]
    }
}
