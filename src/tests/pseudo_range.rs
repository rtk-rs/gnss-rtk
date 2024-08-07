use crate::prelude::{
    Candidate, Carrier, ClockCorrection, Epoch, IonoComponents, Observation, TropoComponents, SV,
};

#[test]
fn prefered_pseudorange() {
    for (observations, prefered) in [(
        vec![
            Observation {
                snr: None,
                phase: None,
                pseudo: Some(1.0),
                ambiguity: None,
                doppler: None,
                carrier: Carrier::L1,
            },
            Observation {
                snr: None,
                phase: None,
                pseudo: Some(2.0),
                ambiguity: None,
                doppler: None,
                carrier: Carrier::L2,
            },
            Observation {
                snr: None,
                phase: None,
                pseudo: Some(3.0),
                ambiguity: None,
                doppler: None,
                carrier: Carrier::L2,
            },
        ],
        Observation {
            snr: None,
            phase: None,
            doppler: None,
            ambiguity: None,
            pseudo: Some(1.0),
            carrier: Carrier::L1,
        },
    )] {
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            ClockCorrection::default(),
            None,
            observations,
            IonoComponents::Unknown,
            TropoComponents::Unknown,
        );
        assert_eq!(cd.prefered_pseudorange(), Some(prefered),);
    }
}

#[test]
fn l1_l2_narrowlane() {
    let codes = vec![
        Observation {
            snr: None,
            pseudo: Some(64.0),
            phase: None,
            doppler: None,
            ambiguity: None,
            carrier: Carrier::L1,
        },
        Observation {
            snr: None,
            phase: None,
            doppler: None,
            ambiguity: None,
            pseudo: Some(128.0),
            carrier: Carrier::L2,
        },
    ];
    let cd = Candidate::new(
        SV::default(),
        Epoch::default(),
        ClockCorrection::default(),
        None,
        codes,
        IonoComponents::Unknown,
        TropoComponents::Unknown,
    );
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
        snr: None,
        phase: None,
        doppler: None,
        ambiguity: None,
        pseudo: Some(64.0),
        carrier: Carrier::L1,
    }];
    let cd = Candidate::new(
        SV::default(),
        Epoch::default(),
        ClockCorrection::default(),
        None,
        codes,
        IonoComponents::Unknown,
        TropoComponents::Unknown,
    );

    assert!(
        cd.code_nl_combination().is_none(),
        "Cn_narrow should not be feasible!"
    );
}

#[test]
fn e1_e5_narrowlane() {
    let obs = vec![
        Observation {
            snr: None,
            phase: None,
            doppler: None,
            pseudo: Some(64.0),
            ambiguity: None,
            carrier: Carrier::E1,
        },
        Observation {
            snr: None,
            phase: None,
            doppler: None,
            pseudo: Some(128.0),
            ambiguity: None,
            carrier: Carrier::E5,
        },
    ];
    let cd = Candidate::new(
        SV::default(),
        Epoch::default(),
        ClockCorrection::default(),
        None,
        obs,
        IonoComponents::Unknown,
        TropoComponents::Unknown,
    );
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
