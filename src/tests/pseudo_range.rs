use crate::prelude::{Candidate, Carrier, Epoch, Observation, SV};

#[test]
fn best_snr_pseudorange() {
    for (observations, prefered) in [(
        vec![
            Observation {
                snr_dbhz: Some(20.0),
                phase_range_m: None,
                pseudo_range_m: Some(1.0),
                ambiguity: None,
                doppler: None,
                carrier: Carrier::L1,
            },
            Observation {
                snr_dbhz: Some(30.0),
                phase_range_m: None,
                pseudo_range_m: Some(2.0),
                ambiguity: None,
                doppler: None,
                carrier: Carrier::L2,
            },
        ],
        Observation {
            snr_dbhz: Some(30.0),
            phase_range_m: None,
            pseudo_range_m: Some(2.0),
            ambiguity: None,
            doppler: None,
            carrier: Carrier::L2,
        },
    )] {
        let cd = Candidate::new(SV::default(), Epoch::default(), observations);
        assert_eq!(cd.best_snr_observation(), Some(prefered));
    }
}

#[test]
fn l1_l2_narrowlane() {
    let codes = vec![
        Observation {
            snr_dbhz: None,
            phase_range_m: None,
            doppler: None,
            ambiguity: None,
            pseudo_range_m: Some(64.0),
            carrier: Carrier::L1,
        },
        Observation {
            snr_dbhz: None,
            phase_range_m: None,
            doppler: None,
            ambiguity: None,
            pseudo_range_m: Some(128.0),
            carrier: Carrier::L2,
        },
    ];
    let cd = Candidate::new(SV::default(), Epoch::default(), codes);
    let cn = cd.code_nl_combination();
    assert!(cn.is_some(), "failed to form Cn_narrow combination");
    let cn = cn.unwrap();
    assert_eq!(cn.rhs, Carrier::L1);
    assert_eq!(
        cn.value,
        (Carrier::L1.frequency() * 64.0 + Carrier::L2.frequency() * 128.0)
            / (Carrier::L1.frequency() + Carrier::L2.frequency())
    );

    let codes = vec![Observation {
        snr_dbhz: None,
        phase_range_m: None,
        doppler: None,
        ambiguity: None,
        pseudo_range_m: Some(64.0),
        carrier: Carrier::L1,
    }];
    let cd = Candidate::new(SV::default(), Epoch::default(), codes);

    assert!(
        cd.code_nl_combination().is_none(),
        "Cn_narrow should not be feasible!"
    );
}

#[test]
fn e1_e5_narrowlane() {
    let obs = vec![
        Observation {
            snr_dbhz: None,
            phase_range_m: None,
            doppler: None,
            pseudo_range_m: Some(64.0),
            ambiguity: None,
            carrier: Carrier::E1,
        },
        Observation {
            snr_dbhz: None,
            phase_range_m: None,
            doppler: None,
            pseudo_range_m: Some(128.0),
            ambiguity: None,
            carrier: Carrier::E5,
        },
    ];
    let cd = Candidate::new(SV::default(), Epoch::default(), obs);
    let cn = cd.code_nl_combination();
    assert!(cn.is_some(), "failed to form Cn_narrow combination");
    let cn = cn.unwrap();

    assert_eq!(cn.rhs, Carrier::E1);

    assert_eq!(
        cn.value,
        (Carrier::E1.frequency() * 64.0 + Carrier::E5.frequency() * 128.0)
            / (Carrier::E1.frequency() + Carrier::E5.frequency())
    );
}
