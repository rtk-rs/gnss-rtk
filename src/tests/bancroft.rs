use crate::bancroft::Bancroft;
use crate::prelude::{
    Candidate, Carrier, ClockCorrection, Constellation, Duration, Epoch, Observation, Orbit,
    EARTH_J2000, SV,
};

use std::str::FromStr;

#[test]
fn test() {
    let mut pool = Vec::<Candidate>::new();
    let t0 = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();
    let (x0, y0, z0) = (3628427.9118, 562059.0936, 5197872.215);
    for (i, (pr, dt, sv_x_km, sv_y_km, sv_z_km)) in [
        (
            26952639.751,
            Duration::from_microseconds(-313.498),
            4577.077035843635,
            -22996.125649966143,
            18062.46236437641,
        ),
        (
            23595077.027,
            Duration::from_microseconds(-368.775),
            16576.946499220812,
            -4619.715035111092,
            24092.50915107983,
        ),
        (
            22579938.261,
            Duration::from_milliseconds(6.017694),
            18846.557032585508,
            16144.709835080192,
            16160.045068828074,
        ),
        (
            27896986.615,
            Duration::from_microseconds(401.846),
            -15921.905530334785,
            -5399.928036329342,
            24360.75165958442,
        ),
    ]
    .iter()
    .enumerate()
    {
        let pr = Observation::pseudo_range(Carrier::E1, *pr, None);
        let mut cd = Candidate::new(
            SV::new(Constellation::default(), (i + 1) as u8),
            t0,
            vec![pr],
        );
        cd.set_clock_correction(ClockCorrection::without_relativistic_correction(*dt));
        cd.set_orbit(Orbit::from_position(
            *sv_x_km,
            *sv_y_km,
            *sv_z_km,
            t0,
            EARTH_J2000,
        ));
        pool.push(cd);
    }

    let solver =
        Bancroft::new(&pool).unwrap_or_else(|e| panic!("failed to create Bancroft solver: {}", e));

    let output = solver
        .resolve()
        .unwrap_or_else(|e| panic!("Bancroft solver failure: {}", e));

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
