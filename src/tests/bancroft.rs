use crate::bancroft::Bancroft;
use crate::prelude::{
    Candidate, Carrier, Constellation, Duration, Epoch, Observation, OrbitalState, SV,
};

#[test]
fn test() {
    let (x0, y0, z0) = (3582105.291, 532589.7313, 5232754.8054);

    let pr = Observation {
        snr: None,
        phase: None,
        doppler: None,
        ambiguity: None,
        pseudo: Some(28776032.260),
        carrier: Carrier::E1,
    };

    let mut cd0 = Candidate::new(
        SV::new(Constellation::default(), 2),
        Epoch::default(),
        Duration::from_seconds(142.784E-6),
        None, // TGD
        vec![pr],
    );
    let st =
        OrbitalState::from_position((24170352.34904016, -16029029.85873581, -5905924.153143198));
    cd0.set_state(st);

    let pr = Observation {
        snr: None,
        phase: None,
        doppler: None,
        ambiguity: None,
        pseudo: Some(24090441.364),
        carrier: Carrier::E1,
    };

    let mut cd1 = Candidate::new(
        SV::new(Constellation::default(), 3),
        Epoch::default(),
        Duration::from_seconds(-313.533E-6),
        None, // TGD
        vec![pr],
    );
    let st =
        OrbitalState::from_position((16069642.946692571, -8992001.827692423, 23184746.654093638));
    cd1.set_state(st);

    let pr = Observation {
        snr: None,
        phase: None,
        doppler: None,
        ambiguity: None,
        pseudo: Some(24762903.616),
        carrier: Carrier::E1,
    };

    let mut cd2 = Candidate::new(
        SV::new(Constellation::default(), 5),
        Epoch::default(),
        Duration::from_seconds(-368.749E-6),
        None, // TGD
        vec![pr],
    );
    let st =
        OrbitalState::from_position((26119621.94656989, 7791422.617964384, 11558902.718228433));
    cd2.set_state(st);

    let pr = Observation {
        snr: None,
        phase: None,
        doppler: None,
        ambiguity: None,
        pseudo: Some(25537644.454),
        carrier: Carrier::E1,
    };

    let mut cd3 = Candidate::new(
        SV::new(Constellation::default(), 8),
        Epoch::default(),
        Duration::from_seconds(6.158955E-3),
        None, // TGD
        vec![pr],
    );
    let st =
        OrbitalState::from_position((-3601205.0295727667, -20311399.087870672, 21230831.216778148));
    cd3.set_state(st);

    let pool = vec![cd0, cd1, cd2, cd3];
    let solver = Bancroft::new(&pool);
    assert!(
        solver.is_ok(),
        "failed to create bancroft solver: {}",
        solver.err().unwrap()
    );
    let solver = solver.unwrap();
    let output = solver.resolve();
    assert!(
        output.is_ok(),
        "bancroft solver failure: {}",
        output.err().unwrap()
    );
    let output = output.unwrap();

    let x_err = (output[0] - x0).abs();
    let y_err = (output[1] - y0).abs();
    let z_err = (output[2] - z0).abs();
    assert!(x_err < 100.0, "bancroft solver error: x error too large");
    assert!(y_err < 100.0, "bancroft solver error: x error too large");
    assert!(z_err < 100.0, "bancroft solver error: x error too large");
}
