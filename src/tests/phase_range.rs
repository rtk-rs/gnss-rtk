use crate::prelude::{Candidate, Carrier, Observation};

#[test]
fn l1_widelane() {
    let observations = vec![
        Observation {
            snr_dbhz: None,
            phase_range_m: Some(64.0),
            pseudo_range_m: None,
            carrier: Carrier::L1,
            doppler: None,
            ambiguity: None,
        },
        Observation {
            snr_dbhz: None,
            phase_range_m: Some(128.0),
            pseudo_range_m: None,
            carrier: Carrier::L2,
            doppler: None,
            ambiguity: None,
        },
    ];

    let cd = Candidate::new(Default::default(), Default::default(), observations);
    let wl = cd.phase_wl_combination().unwrap();

    assert_eq!(wl.rhs, Carrier::L1);
    assert_eq!(wl.lhs, Carrier::L2);

    assert_eq!(
        wl.value,
        (Carrier::L1.frequency() * 64.0 - Carrier::L2.frequency() * 128.0)
            / (Carrier::L1.frequency() - Carrier::L2.frequency())
    );
}

#[test]
fn non_feasible_l1_widelane() {
    let observations = vec![
        Observation {
            snr_dbhz: None,
            phase_range_m: Some(64.0),
            pseudo_range_m: None,
            carrier: Carrier::L1,
            doppler: None,
            ambiguity: None,
        },
        Observation {
            snr_dbhz: None,
            pseudo_range_m: Some(128.0),
            phase_range_m: None,
            carrier: Carrier::L2,
            doppler: None,
            ambiguity: None,
        },
    ];

    let cd = Candidate::new(Default::default(), Default::default(), observations);
    assert!(
        cd.phase_wl_combination().is_none(),
        "Wl combination should not be feasible"
    );
}

#[test]
fn e1_widelane() {
    let observations = vec![
        Observation {
            snr_dbhz: None,
            phase_range_m: Some(64.0),
            pseudo_range_m: None,
            carrier: Carrier::E1,
            doppler: None,
            ambiguity: None,
        },
        Observation {
            snr_dbhz: None,
            phase_range_m: Some(128.0),
            pseudo_range_m: None,
            carrier: Carrier::E5,
            doppler: None,
            ambiguity: None,
        },
    ];

    let cd = Candidate::new(Default::default(), Default::default(), observations);
    let wl = cd.phase_wl_combination().unwrap();

    assert_eq!(wl.rhs, Carrier::E1);
    assert_eq!(wl.lhs, Carrier::E5);

    assert_eq!(
        wl.value,
        (Carrier::E1.frequency() * 64.0 - Carrier::E5.frequency() * 128.0)
            / (Carrier::E1.frequency() - Carrier::E5.frequency())
    );
}

#[test]
fn l1_l5_mw_combination() {
    let observations = vec![
        Observation {
            snr_dbhz: None,
            pseudo_range_m: Some(64.0),
            phase_range_m: Some(16.0),
            carrier: Carrier::L1,
            doppler: None,
            ambiguity: None,
        },
        Observation {
            snr_dbhz: None,
            pseudo_range_m: Some(128.0),
            phase_range_m: Some(32.0),
            carrier: Carrier::L5,
            doppler: None,
            ambiguity: None,
        },
    ];

    let cd = Candidate::new(Default::default(), Default::default(), observations);

    let mw = cd.mw_combination().unwrap();

    assert_eq!(mw.rhs, Carrier::L1);
    assert_eq!(mw.lhs, Carrier::L5);

    let pw = (Carrier::L1.frequency() * 16.0 - Carrier::L5.frequency() * 32.0)
        / (Carrier::L1.frequency() - Carrier::L5.frequency());

    let cn = (Carrier::L1.frequency() * 64.0 + Carrier::L5.frequency() * 128.0)
        / (Carrier::L1.frequency() + Carrier::L5.frequency());

    assert_eq!(mw.value, pw - cn);
}
