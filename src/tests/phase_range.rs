
use crate::prelude::{
    Candidate, Carrier, Duration, Epoch, PseudoRange, PseudoRangeCombination, PhaseRange, SV
};


#[test]
fn l1_widelane() {
        let phases = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::L1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::L2,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            vec![],
            phases,
        );
        let pw = cd.phase_wl_combination();
        assert!(pw.is_some(), "failed to form Ph_wide combination");
        let pw = pw.unwrap();
        assert_eq!(pw.carrier, Carrier::L1);
        assert_eq!(
            pw.value,
            (Carrier::L1.frequency() * 64.0 - Carrier::L2.frequency() * 128.0)
                / (Carrier::L1.frequency() - Carrier::L2.frequency())
        );

        let phases = vec![Observation {
            snr: None,
            value: 64.0,
            carrier: Carrier::L1,
        }];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            vec![],
            phases,
        );

        assert!(
            cd.phase_wl_combination().is_none(),
            "Ph_wide should not be feasible!"
        );
}
    
    #[test]
    fn e1_widelane() {
        let phases = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::E1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::E5,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            vec![],
            phases,
        );
        let pw = cd.phase_wl_combination();
        assert!(pw.is_some(), "failed to form Ph_wide combination");
        let pw = pw.unwrap();
        assert_eq!(pw.carrier, Carrier::E1);
        assert_eq!(
            pw.value,
            (Carrier::E1.frequency() * 64.0 - Carrier::E5.frequency() * 128.0)
                / (Carrier::E1.frequency() - Carrier::E5.frequency())
        );
    }
    
    #[test]
    fn l1_l5_mw_combination() {
        let codes = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::L1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::L5,
            },
        ];
        let phases = vec![
            Observation {
                snr: None,
                value: 16.0,
                carrier: Carrier::L1,
            },
            Observation {
                snr: None,
                value: 32.0,
                carrier: Carrier::L5,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            phases,
            vec![],
        );
        let mw = cd.mw_combination();
        assert!(mw.is_some(), "failed to form MW combination");
        let mw = mw.unwrap();
        assert_eq!(mw.lhs, Carrier::L5);
        assert_eq!(mw.reference, Carrier::L1);
        let pw = (Carrier::L1.frequency() * 16.0 - Carrier::L5.frequency() * 32.0)
            / (Carrier::L1.frequency() - Carrier::L5.frequency());
        let cn = (Carrier::L1.frequency() * 64.0 + Carrier::L5.frequency() * 128.0)
            / (Carrier::L1.frequency() + Carrier::L5.frequency());
        assert_eq!(mw.value, pw - cn);
    }
    
    #[test]
    fn e1_e5_mw_combination() {
        let codes = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::E1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::E5,
            },
        ];
        let phases = vec![
            Observation {
                snr: None,
                value: 16.0,
                carrier: Carrier::E1,
            },
            Observation {
                snr: None,
                value: 32.0,
                carrier: Carrier::E5,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            phases,
        );
        let mw = cd.mw_combination();
        assert!(mw.is_some(), "failed to form MW combination");
        let mw = mw.unwrap();
        assert_eq!(mw.lhs, Carrier::E5);
        assert_eq!(mw.reference, Carrier::E1);
        let pw = (Carrier::E1.frequency() * 16.0 - Carrier::E5.frequency() * 32.0)
            / (Carrier::E1.frequency() - Carrier::E5.frequency());
        let cn = (Carrier::E1.frequency() * 64.0 + Carrier::E5.frequency() * 128.0)
            / (Carrier::E1.frequency() + Carrier::E5.frequency());
        assert_eq!(mw.value, pw - cn);
    }

    #[test]
    fn best_snr() {
        for (codes, best_snr) in [
            (vec![
            PseudoRange {
                value: 1.0,
                snr: None,
                carrier: Carrier::L1,
            },
            PseudoRange {
                value: 2.0,
                snr: None,
                carrier: Carrier::L2,
            },
            PseudoRange {
                value: 3.0,
                snr: Some(10.0),
                carrier: Carrier::L5,
            },
            PseudoRange {
                value: 4.0,
                snr: Some(11.0),
                carrier: Carrier::L2,
            },
            PseudoRange {
                value: 5.0,
                snr: Some(9.0),
                carrier: Carrier::L2,
            }], 11.0),
            (vec![
            PseudoRange {
                value: 1.0,
                snr: Some(1.0),
                carrier: Carrier::L1,
            },
            PseudoRange {
                value: 2.0,
                snr: Some(1.1),
                carrier: Carrier::L2,
            },
            PseudoRange {
                value: 3.0,
                snr: Some(1.2),
                carrier: Carrier::L5,
            }], 12.0),
        ] {
            let cd = Candidate::new(
                SV::default(),
                Epoch::default(),
                Duration::default(),
                None,
                codes,
                vec![],
            );
            assert_eq!(cd.pseudorange_best_snr(), Some(best_snr), "failed for {}", best_snr);
        }
    }
