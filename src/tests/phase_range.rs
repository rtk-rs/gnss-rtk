use crate::prelude::{Candidate, Carrier, Epoch, Observation, SV};

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
        (Carrier::L1.frequency_hz() * 64.0 - Carrier::L2.frequency_hz() * 128.0)
            / (Carrier::L1.frequency_hz() - Carrier::L2.frequency_hz())
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
            carrier: Carrier::L1,
            doppler: None,
            ambiguity: None,
        },
        Observation {
            snr_dbhz: None,
            phase_range_m: Some(128.0),
            pseudo_range_m: None,
            carrier: Carrier::L5,
            doppler: None,
            ambiguity: None,
        },
    ];

    let cd = Candidate::new(Default::default(), Default::default(), observations);
    let wl = cd.phase_wl_combination().unwrap();

    assert_eq!(wl.rhs, Carrier::L1);
    assert_eq!(wl.lhs, Carrier::L5);

    assert_eq!(
        wl.value,
        (Carrier::L1.frequency_hz() * 64.0 - Carrier::L5.frequency_hz() * 128.0)
            / (Carrier::L1.frequency_hz() - Carrier::L5.frequency_hz())
    );
}

// #[test]
// fn l1_l5_mw_combination() {
//     let observations = vec![
//         Observation {
//             snr_dbhz: None,
//             pseudo_range_m: Some(64.0),
//             phase_range_m: Some(16.0),
//             carrier: Carrier::L1,
//             doppler: None,
//             ambiguity: None,
//         },
//         Observation {
//             snr_dbhz: None,
//             pseudo_range_m: Some(128.0),
//             phase_range_m: Some(32.0),
//             carrier: Carrier::L5,
//             doppler: None,
//             ambiguity: None,
//         },
//     ];

//     let cd = Candidate::new(Default::default(), Default::default(), observations);

//     let mw = cd.mw_combination().unwrap();

//     assert_eq!(mw.rhs, Carrier::L1);
//     assert_eq!(mw.lhs, Carrier::L5);

//     let pw = (Carrier::L1.frequency_hz() * 16.0 - Carrier::L5.frequency_hz() * 32.0)
//         / (Carrier::L1.frequency_hz() - Carrier::L5.frequency_hz());

//     let cn = (Carrier::L1.frequency_hz() * 64.0 + Carrier::L5.frequency_hz() * 128.0)
//         / (Carrier::L1.frequency_hz() + Carrier::L5.frequency_hz());

//     assert_eq!(mw.value, pw - cn);
// }

#[test]
fn l1_l5_phase_if() {
    let obs = vec![
        Observation {
            snr_dbhz: None,
            doppler: None,
            pseudo_range_m: None,
            phase_range_m: Some(64.0),
            ambiguity: None,
            carrier: Carrier::L1,
        },
        Observation {
            snr_dbhz: None,
            doppler: None,
            pseudo_range_m: None,
            phase_range_m: Some(128.0),
            ambiguity: None,
            carrier: Carrier::L5,
        },
    ];

    let cd = Candidate::new(SV::default(), Epoch::default(), obs);

    let l_if = cd.phase_if_combination().unwrap_or_else(|| {
        panic!("Failed to form L_if combination");
    });

    assert_eq!(l_if.lhs, Carrier::L5);
    assert_eq!(l_if.rhs, Carrier::L1);

    let (f1, f2) = (Carrier::L1.frequency_hz(), Carrier::L5.frequency_hz());
    let (f1pow, f2pow) = (f1.powi(2), f2.powi(2));

    assert_eq!(l_if.value, (f1pow * 64.0 - f2pow * 128.0) / (f1pow - f2pow));
}
