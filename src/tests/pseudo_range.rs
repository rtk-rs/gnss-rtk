use crate::prelude::{Candidate, Carrier, Duration, Epoch, PseudoRange, SV};

#[test]
fn prefered_pseudorange() {
    for (observations, prefered) in [
        (
            vec![
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
                    snr: None,
                    carrier: Carrier::L5,
                },
            ],
            PseudoRange {
                value: 1.0,
                snr: None,
                carrier: Carrier::L1,
            },
        ),
        (
            vec![
                PseudoRange {
                    value: 1.0,
                    snr: None,
                    carrier: Carrier::L1,
                },
                PseudoRange {
                    value: 2.0,
                    snr: Some(2.0),
                    carrier: Carrier::L2,
                },
                PseudoRange {
                    value: 3.0,
                    snr: None,
                    carrier: Carrier::L5,
                },
            ],
            PseudoRange {
                value: 2.0,
                snr: Some(2.0),
                carrier: Carrier::L2,
            },
        ),
    ] {
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            observations,
            vec![],
        );
        assert_eq!(cd.prefered_pseudorange(), Some(prefered),);
    }
}

#[test]
fn l1_l2_narrowlane() {
    let codes = vec![
        PseudoRange {
            snr: None,
            value: 64.0,
            carrier: Carrier::L1,
        },
        PseudoRange {
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
        codes,
        vec![],
    );
    let cn = cd.code_nl_combination();
    assert!(cn.is_some(), "failed to form Cn_narrow combination");
    let cn = cn.unwrap();
    assert_eq!(cn.reference, Carrier::L1);
    assert_eq!(
        cn.value,
        (Carrier::L1.frequency() * 64.0 + Carrier::L2.frequency() * 128.0)
            / (Carrier::L1.frequency() + Carrier::L2.frequency())
    );

    let codes = vec![PseudoRange {
        snr: None,
        value: 64.0,
        carrier: Carrier::L1,
    }];
    let cd = Candidate::new(
        SV::default(),
        Epoch::default(),
        Duration::default(),
        None,
        codes,
        vec![],
    );

    assert!(
        cd.code_nl_combination().is_none(),
        "Cn_narrow should not be feasible!"
    );
}

#[test]
fn e1_e5_narrowlane() {
    let codes = vec![
        PseudoRange {
            snr: None,
            value: 64.0,
            carrier: Carrier::E1,
        },
        PseudoRange {
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
        codes,
        vec![],
    );
    let cn = cd.code_nl_combination();
    assert!(cn.is_some(), "failed to form Cn_narrow combination");
    let cn = cn.unwrap();
    assert_eq!(cn.reference, Carrier::E1);
    assert_eq!(
        cn.value,
        (Carrier::E1.frequency() * 64.0 + Carrier::E5.frequency() * 128.0)
            / (Carrier::E1.frequency() + Carrier::E5.frequency())
    );
}
