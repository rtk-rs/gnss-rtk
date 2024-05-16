use crate::bias::RuntimeParam;
use log::debug;
use std::f64::consts::PI;

#[derive(Copy, Clone, Debug)]
pub(crate) enum UNB3Param {
    // pressure in mBar
    Pressure = 0,
    // temperature in Kelvin
    Temperature = 1,
    // water vapour pressure in mBar
    WaterVapourPressure = 2,
    // beta is temperature lapse rate (Kelvin/m)
    Beta = 3,
    // lambda is wvp height factor (N/A)
    Lambda = 4,
}

fn unb3_annual_average(prm: UNB3Param, lat_ddeg: f64, nearest: usize) -> f64 {
    const LUT: [(f64, [f64; 5]); 5] = [
        (15.0, [1013.25, 299.65, 26.31, 6.30E-3, 2.77]),
        (30.0, [1017.25, 294.15, 21.79, 6.05E-3, 3.15]),
        (45.0, [1015.75, 283.15, 11.66, 5.58E-3, 2.57]),
        (60.0, [1011.75, 272.15, 6.78, 5.39E-3, 1.81]),
        (75.0, [1013.00, 263.65, 4.11, 4.53E-3, 1.55]),
    ];
    let prm = (prm as u8) as usize;
    if lat_ddeg <= 15.0 {
        LUT[0].1[prm]
    } else if lat_ddeg >= 75.0 {
        LUT[4].1[prm]
    } else {
        LUT[nearest - 1].1[prm]
            + (LUT[nearest].1[prm] - LUT[nearest - 1].1[prm]) / 15.0_f64
                * (lat_ddeg - LUT[nearest - 1].0)
    }
}

fn unb3_average_amplitude(prm: UNB3Param, lat_ddeg: f64, nearest: usize) -> f64 {
    const LUT: [(f64, [f64; 5]); 5] = [
        (15.0, [0.0, 0.0, 0.0, 0.0, 0.0]),
        (30.0, [-3.75, 7.0, 8.85, 0.25E-3, 0.33]),
        (45.0, [-2.25, 11.0, 7.24, 0.32E-3, 0.46]),
        (60.0, [-1.75, 15.0, 5.36, 0.81E-3, 0.74]),
        (75.0, [-0.50, 14.5, 3.39, 0.62E-3, 0.3]),
    ];
    let prm = (prm as u8) as usize;
    if lat_ddeg <= 15.0 {
        LUT[0].1[prm]
    } else if lat_ddeg >= 75.0 {
        LUT[4].1[prm]
    } else {
        LUT[nearest - 1].1[prm]
            + (LUT[nearest].1[prm] - LUT[nearest - 1].1[prm]) / 15.0_f64
                * (lat_ddeg - LUT[nearest - 1].0)
    }
}

fn unb3_parameter(prm: UNB3Param, lat_ddeg: f64, day_of_year: f64, nearest: usize) -> f64 {
    let dmin = match lat_ddeg.is_sign_positive() {
        true => 28.0_f64,
        false => 211.0_f64,
    };
    let annual = unb3_annual_average(prm, lat_ddeg, nearest);
    let amplitude = unb3_average_amplitude(prm, lat_ddeg, nearest);
    annual - amplitude * ((day_of_year - dmin) * 2.0_f64 * PI / 365.25_f64).cos()
}

/*
 * Evaluate ZWD and ZDD at given Epoch "t" and given latitude
 * This method is infaillible and will work at any Epoch, for any latitude
 */
pub(crate) fn unb3_model(rtm: &RuntimeParam) -> (f64, f64) {
    const K_1: f64 = 77.604;
    const K_2: f64 = 382000.0_f64;
    const R_D: f64 = 287.054;
    const G: f64 = 9.80665_f64;
    const G_M: f64 = 9.784_f64;

    let day_of_year = rtm.t.day_of_year();
    let (lat_ddeg, _, h) = rtm.apriori_geo;

    let mut lat = 15.0_f64;
    let mut min_delta = 180.0_f64;
    let mut nearest_index: usize = 1;

    while lat < 75.0 {
        if lat > lat_ddeg {
            break;
        }
        let delta = lat_ddeg - lat;
        if delta < min_delta {
            min_delta = delta;
            nearest_index += 1;
        }
        lat += 15.0;
    }

    let beta = unb3_parameter(UNB3Param::Beta, lat_ddeg, day_of_year, nearest_index);
    let p = unb3_parameter(UNB3Param::Pressure, lat_ddeg, day_of_year, nearest_index);
    let lambda = unb3_parameter(UNB3Param::Lambda, lat_ddeg, day_of_year, nearest_index);
    let temp = unb3_parameter(UNB3Param::Temperature, lat_ddeg, day_of_year, nearest_index);
    let e = unb3_parameter(
        UNB3Param::WaterVapourPressure,
        lat_ddeg,
        day_of_year,
        nearest_index,
    );

    let z0_zdd = 10.0E-6 * K_1 * R_D * p / G_M;
    let denom = (lambda + 1.0_f64) * G_M - beta * R_D;
    let z0_zwd = 10.0E-6 * K_2 * R_D * e / temp / denom;
    let value = 1.0_f64 - beta * h / temp;

    let zdd = (value).powf(G / R_D / beta) * z0_zdd;
    let zwd = (value).powf((lambda + 1.0_f64) * G / R_D / beta - 1.0_f64) * z0_zwd;

    debug!(
        "{}: unb3 - [beta: {:.3}, p: {:.3}, temp: {:.3}, e: {:.3}, lambda: {:.3}",
        rtm.t, beta, p, temp, e, lambda
    );

    debug!(
        "{}: unb3 - zdd(h=0) {:.3} zwd(h=0) {:.3}",
        rtm.t, z0_zdd, z0_zwd,
    );
    debug!(
        "{}: unb3 - zdd(h={:.3}) {} zwd(h={:.3}) {:.3}",
        rtm.t, h, zdd, h, zwd
    );

    (zwd, zdd)
}
