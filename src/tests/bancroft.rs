use crate::bancroft::Bancroft;
use crate::prelude::{
    Candidate, Carrier, ClockCorrection, Constellation, Duration, Epoch, IonoComponents,
    Observation, Orbit, TropoComponents, EARTH_J2000, SV,
};

use std::str::FromStr;

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
        ClockCorrection::without_relativistic_correction(Duration::from_seconds(142.784E-6)),
        None, // TGD
        vec![pr],
        IonoComponents::Unknown,
        TropoComponents::Unknown,
    );
    let orbit = Orbit::from_position(
        24170352.34904016 / 1.0E3,
        -16029029.85873581 / 1.0E3,
        -5905924.153143198 / 1.0E3,
        Epoch::from_str("2020-06-20T00:00:00 GPST").unwrap(),
        EARTH_J2000, //wrong ! wgs84
    );
    cd0.set_orbit(orbit);

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
        ClockCorrection::without_relativistic_correction(Duration::from_seconds(-313.533E-6)),
        None, // TGD
        vec![pr],
        IonoComponents::Unknown,
        TropoComponents::Unknown,
    );
    let orbit = Orbit::from_position(
        16069642.946692571 / 1.0E3,
        -8992001.827692423 / 1.0E3,
        23184746.654093638 / 1.0E3,
        Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap(),
        EARTH_J2000,
    );
    cd1.set_orbit(orbit);

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
        ClockCorrection::without_relativistic_correction(Duration::from_seconds(-368.749E-6)),
        None, // TGD
        vec![pr],
        IonoComponents::Unknown,
        TropoComponents::Unknown,
    );
    let orbit = Orbit::from_position(
        26119621.94656989 / 1.0E3,
        7791422.617964384 / 1.0E3,
        11558902.718228433 / 1.0E3,
        Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap(),
        EARTH_J2000, // TODO WRONG! GPS
    );
    cd2.set_orbit(orbit);

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
        ClockCorrection::without_relativistic_correction(Duration::from_seconds(6.158955E-3)),
        None, // TGD
        vec![pr],
        IonoComponents::Unknown,
        TropoComponents::Unknown,
    );
    let orbit = Orbit::from_position(
        -3601205.0295727667 / 1.0E3,
        -20311399.087870672 / 1.0E3,
        21230831.216778148 / 1.0E3,
        Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap(),
        EARTH_J2000,
    );
    cd3.set_orbit(orbit);

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
