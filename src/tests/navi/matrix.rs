use std::str::FromStr;

use anise::{constants::frames::EARTH_J2000, math::Vector6};
use hifitime::Duration;
use nyx::cosmic::SPEED_OF_LIGHT_M_S;

use crate::{
    cfg::Modeling,
    constants::Constants,
    navigation::{Navigation, State},
    prelude::{
        Almanac, Candidate, Carrier, ClockCorrection, Config, Epoch, Error, Observation, Orbit,
        Vector3, SV,
    },
    tests::{data::gps::*, DataPoint},
};

use nalgebra::{Matrix4, Vector4};

#[test]
fn pvt_failures() {
    let cfg = Config::default();

    let (x0_m, y0_m, z0_m) = (0.0_f64, 0.0_f64, 0.0_f64);

    let almanac = Almanac::until_2035().unwrap();
    let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

    let state =
        &State::from_ecef_m(Vector3::new(x0_m, y0_m, z0_m), Default::default(), frame).unwrap();

    let candidates = [DataPoint::new(GPS_EPOCHS[0], "G01", "L1:pr:1.0").to_candidate()]
        .iter()
        .cloned()
        .collect::<Vec<_>>();

    match Navigation::new(&cfg, state, &candidates) {
        Err(e) => match e {
            Error::MatrixMinimalDimension => {},
            e => panic!("failed with invalid error: {}", e),
        },
        _ => panic!("should have failed 1x4"),
    }

    let candidates = [
        DataPoint::new(GPS_EPOCHS[0], "G01", "L1:pr:1.0").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G02", "L1:pr:2.0").to_candidate(),
    ]
    .iter()
    .cloned()
    .collect::<Vec<_>>();

    match Navigation::new(&cfg, state, &candidates) {
        Err(e) => match e {
            Error::MatrixMinimalDimension => {},
            e => panic!("failed with invalid error: {}", e),
        },
        _ => panic!("should have failed 2x4"),
    }

    let candidates = [
        DataPoint::new(GPS_EPOCHS[0], "G01", "L1:pr:1.0").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G02", "L1:pr:2.0").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G03", "L1:pr:3.0").to_candidate(),
    ]
    .iter()
    .cloned()
    .collect::<Vec<_>>();

    match Navigation::new(&cfg, state, &candidates) {
        Err(e) => match e {
            Error::MatrixMinimalDimension => {},
            e => panic!("failed with invalid error: {}", e),
        },
        _ => panic!("should have failed 3x4"),
    }

    let candidates = [
        DataPoint::new(GPS_EPOCHS[0], "G01", "L1:pr:1.0").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G02", "L1:pr:2.0").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G03", "L1:pr:3.0").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G04", "L1:pr:4.0").to_candidate(),
    ]
    .iter()
    .cloned()
    .collect::<Vec<_>>();

    let nav = Navigation::new(&cfg, state, &candidates)
        .unwrap_or_else(|e| panic!("Matrix formation should be feasible: {}", e));

    assert_eq!(nav.b, Vector4::zeros(), "Only unresolved states!");
    assert_eq!(nav.h, Matrix4::zeros(), "Only unresolved states!");
}

#[test]
fn pvt_matrix() {
    let cfg = Config::default().with_modeling(Modeling::no_modeling());

    let (x0_m, y0_m, z0_m) = (1000.0_f64, 2000.0_f64, 3000.0_f64);
    let r_0 = (x0_m.powi(2) + y0_m.powi(2) + z0_m.powi(2)).sqrt();

    let almanac = Almanac::until_2035().unwrap();
    let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

    let state =
        &State::from_ecef_m(Vector3::new(x0_m, y0_m, z0_m), Default::default(), frame).unwrap();

    let t0_gpst = Epoch::from_str(GPS_EPOCHS[0]).unwrap();

    let mut candidates = [
        DataPoint::new(GPS_EPOCHS[0], "G01", "L1:pr:21401234.5").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G02", "L1:pr:21421234.8").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G03", "L1:pr:21391235.1").to_candidate(),
        DataPoint::new(GPS_EPOCHS[0], "G04", "L1:pr:21411234.6").to_candidate(),
    ]
    .iter()
    .cloned()
    .collect::<Vec<_>>();

    let sv_coords_m = vec![
        (15600.0, 7540.0, 20140.0),
        (18760.0, 2750.0, 18610.0),
        (17610.0, 14630.0, 13480.0),
        (19170.0, 610.0, 18390.0),
    ];

    for (nth, coords) in sv_coords_m.iter().enumerate() {
        let pos_vel_m = Vector6::new(coords.0, coords.1, coords.2, 0.0, 0.0, 0.0);
        let orbit = Orbit::from_cartesian_pos_vel(pos_vel_m / 1.0E3, t0_gpst, frame);
        candidates[nth].set_orbit(orbit);
        assert!(candidates[nth].orbit.is_some());
    }

    let mut nav = Navigation::new(&cfg, state, &candidates).unwrap();

    let r_i = vec![
        candidates[0].l1_pseudo_range().unwrap().1,
        candidates[1].l1_pseudo_range().unwrap().1,
        candidates[2].l1_pseudo_range().unwrap().1,
        candidates[3].l1_pseudo_range().unwrap().1,
    ];

    assert_eq!(
        r_i,
        vec![21401234.5, 21421234.8, 21391235.1, 21411234.6],
        "incorrect test values"
    );

    let rho = vec![
        ((x0_m - sv_coords_m[0].0).powi(2)
            + (y0_m - sv_coords_m[0].1).powi(2)
            + (z0_m - sv_coords_m[0].2).powi(2))
        .sqrt(),
        ((x0_m - sv_coords_m[1].0).powi(2)
            + (y0_m - sv_coords_m[1].1).powi(2)
            + (z0_m - sv_coords_m[1].2).powi(2))
        .sqrt(),
        ((x0_m - sv_coords_m[2].0).powi(2)
            + (y0_m - sv_coords_m[2].1).powi(2)
            + (z0_m - sv_coords_m[2].2).powi(2))
        .sqrt(),
        ((x0_m - sv_coords_m[3].0).powi(2)
            + (y0_m - sv_coords_m[3].1).powi(2)
            + (z0_m - sv_coords_m[3].2).powi(2))
        .sqrt(),
    ];

    for i in 0..4 {
        let (dx_m, dy_m, dz_m) = (
            (x0_m - sv_coords_m[i].0) / rho[i],
            (y0_m - sv_coords_m[i].1) / rho[i],
            (z0_m - sv_coords_m[i].2) / rho[i],
        );

        assert_eq!(nav.h[(i, 0)], dx_m, "x test failed [({},{})]", i, 0);
        assert_eq!(nav.h[(i, 1)], dy_m, "y test failed [({},{})]", i, 1);
        assert_eq!(nav.h[(i, 2)], dz_m, "z test failed [({},{})]", i, 2);
        assert_eq!(nav.h[(i, 3)], 1.0, "dt test failed [({},{})]", i, 3);
        assert_eq!(nav.b[i], r_i[i] - rho[i], "b (noclock) test failed [{}]", i);

        nav.iterate().unwrap();
    }

    assert_eq!(nav.iter, 4);

    // Clock definitions
    let mut cfg = Config::default().with_modeling(Modeling::no_modeling());

    cfg.modeling.sv_clock_bias = true;

    match Navigation::new(&cfg, state, &candidates) {
        Ok(_) => panic!("Should have failed (noclock!)"),
        Err(e) => match e {
            Error::MatrixMinimalDimension => {},
            e => panic!("Failed with invalid error: {}", e),
        },
    }

    for (nth, cd) in candidates.iter_mut().enumerate() {
        let nanos = (nth + 100) as i128;
        let dt = Duration::from_total_nanoseconds(nanos);

        cd.set_clock_correction(ClockCorrection::without_relativistic_correction(dt));

        assert!(
            cd.clock_corr.is_some(),
            "failed to define a clock correction for {}@{}",
            cd.sv,
            cd.t
        );
        assert_eq!(
            cd.clock_corr.unwrap().duration.total_nanoseconds(),
            nanos,
            "invalid clock duration"
        );
    }

    let mut nav = Navigation::new(&cfg, state, &candidates).unwrap();

    for i in 0..4 {
        let (dx_m, dy_m, dz_m) = (
            (x0_m - sv_coords_m[i].0) / rho[i],
            (y0_m - sv_coords_m[i].1) / rho[i],
            (z0_m - sv_coords_m[i].2) / rho[i],
        );

        assert_eq!(nav.h[(i, 0)], dx_m, "x test failed [({},{})]", i, 0);
        assert_eq!(nav.h[(i, 1)], dy_m, "y test failed [({},{})]", i, 1);
        assert_eq!(nav.h[(i, 2)], dz_m, "z test failed [({},{})]", i, 2);
        assert_eq!(nav.h[(i, 3)], 1.0, "dt test failed [({},{})]", i, 3);

        let dt_s = ((i + 100) as f64) * 1E-9;
        assert_eq!(nav.b[i], r_i[i] - rho[i] + SPEED_OF_LIGHT_M_S * dt_s);

        nav.iterate().unwrap();
    }

    cfg.modeling.relativistic_path_range = true;

    let mut nav = Navigation::new(&cfg, state, &candidates).unwrap();

    for i in 0..4 {
        let r_sat =
            (sv_coords_m[i].0.powi(2) + sv_coords_m[i].1.powi(2) + sv_coords_m[i].2.powi(2)).sqrt();
        let r_sat_0 = r_0 - r_sat;

        let dr = 2.0 * Constants::EARTH_GRAVITATION / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S
            * ((r_sat + r_0 + r_sat_0) / (r_sat + r_0 - r_sat_0)).ln();

        let (dx_m, dy_m, dz_m) = (
            (x0_m - sv_coords_m[i].0) / (rho[i] + dr),
            (y0_m - sv_coords_m[i].1) / (rho[i] + dr),
            (z0_m - sv_coords_m[i].2) / (rho[i] + dr),
        );

        assert_eq!(nav.h[(i, 0)], dx_m, "x test failed [({},{})]", i, 0);
        assert_eq!(nav.h[(i, 1)], dy_m, "y test failed [({},{})]", i, 1);
        assert_eq!(nav.h[(i, 2)], dz_m, "z test failed [({},{})]", i, 2);
        assert_eq!(nav.h[(i, 3)], 1.0, "dt test failed [({},{})]", i, 3);

        let dt_s = ((i + 100) as f64) * 1E-9;
        let b_model = r_i[i] - (rho[i] + dr) + SPEED_OF_LIGHT_M_S * dt_s;
        let err = (nav.b[i] - b_model).abs();
        assert!(err < 1E-6);

        nav.iterate().unwrap();
    }
}
