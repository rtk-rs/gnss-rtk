use crate::bancroft::Bancroft;
use crate::prelude::{
    Candidate, Carrier, ClockCorrection, Constellation, Duration, Epoch, Observation, Orbit,
    EARTH_J2000, SV,
};
use hifitime::Unit;

use std::str::FromStr;

#[test]
fn test() {
    let mut pool = Vec::<Candidate>::new();
    let t0 = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();
    let (x0, y0, z0) = (3582105.291, 532589.7313, 5232754.8054);
    for (i, (pr, dt, sv_x_m, sv_y_m, sv_z_m)) in [
        (
            28776032.260,
            Duration::from_microseconds(142.784),
            24170352.34904016,
            -16029029.85873581,
            -5905924.153143198,
        ),
        (
            24090441.364,
            Duration::from_microseconds(-313.533),
            16069642.946692571,
            -8992001.827692423,
            23184746.654093638,
        ),
        (
            24762903.616,
            Duration::from_microseconds(-368.749),
            26119621.94656989,
            7791422.617964384,
            11558902.718228433,
        ),
        (
            25537644.454,
            Duration::from_milliseconds(-6.158955),
            -3601205.0295727667,
            -20311399.087870672,
            21230831.216778148,
        ),
    ]
    .iter()
    .enumerate()
    {
        let pr = Observation::pseudo_range(Carrier::E1, *pr, None);
        let mut cd = Candidate::new(SV::new(Constellation::default(), i as u8), t0, vec![pr]);
        cd.set_clock_correction(ClockCorrection::without_relativistic_correction(*dt));
        cd.set_orbit(Orbit::from_position(
            *sv_x_m,
            *sv_y_m,
            *sv_z_m,
            t0,
            EARTH_J2000,
        ));
        pool.push(cd);
    }

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
    assert!(
        x_err < 100.0,
        "bancroft solver error: x error too large: {}",
        x_err
    );
    assert!(
        y_err < 100.0,
        "bancroft solver error: y error too large: {}",
        y_err
    );
    assert!(
        z_err < 100.0,
        "bancroft solver error: z error too large: {}",
        z_err
    );
}
