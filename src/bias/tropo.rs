use crate::bias::RuntimeParams;
use crate::cfg::Error;
use log::debug;
use std::f64::consts::PI;

#[derive(Default, Copy, Clone, Debug)]
pub enum TropoModel {
    #[default]
    Niel,
    UNB3,
}

#[derive(Copy, Clone, Debug)]
enum UNB3Param {
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

impl std::str::FromStr for TropoModel {
    type Err = Error;
    fn from_str(s: &str) -> Result<TropoModel, Error> {
        let c = s.trim().to_lowercase();
        match c.as_str() {
            "niel" => Ok(TropoModel::Niel),
            "unb3" => Ok(TropoModel::UNB3),
            _ => Err(Error::UnknownTropoModel),
        }
    }
}

/// Tropospheric delay components you can provide, to improve the internal model.
#[derive(Default, Copy, Clone, Debug)]
pub enum TropoComponents {
    /// Use this to use the internal meteorological model
    #[default]
    Unknown,
    /// Use this to define total Zenith (Wet + Dry) delay components, in meters.
    Total(f64),
    /// Use this to define both Zenith Dry and Wet delay components, in meters.
    WetDry((f64, f64)),
}

impl TropoComponents {
    /// returns Zwd, Zdd from UNB3 model
    fn unb3_model(rtm: &RuntimeParams) -> (f64, f64) {
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

        let beta = Self::unb3_parameter(UNB3Param::Beta, lat_ddeg, day_of_year, nearest_index);
        let p = Self::unb3_parameter(UNB3Param::Pressure, lat_ddeg, day_of_year, nearest_index);
        let lambda = Self::unb3_parameter(UNB3Param::Lambda, lat_ddeg, day_of_year, nearest_index);
        let temp =
            Self::unb3_parameter(UNB3Param::Temperature, lat_ddeg, day_of_year, nearest_index);
        let e = Self::unb3_parameter(
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
    fn unb3_parameter(prm: UNB3Param, lat_ddeg: f64, day_of_year: f64, nearest: usize) -> f64 {
        let dmin = match lat_ddeg.is_sign_positive() {
            true => 28.0_f64,
            false => 211.0_f64,
        };
        let annual = Self::unb3_annual_average(prm, lat_ddeg, nearest);
        let amplitude = Self::unb3_average_amplitude(prm, lat_ddeg, nearest);
        annual - amplitude * ((day_of_year - dmin) * 2.0_f64 * PI / 365.25_f64).cos()
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

    fn niel_model(prm: &RuntimeParams) -> f64 {
        const NS: f64 = 324.8;

        let (_, _, h) = prm.apriori_geo;
        let elev_rad = prm.elevation_rad;
        let h_km = h / 1000.0;

        let f = match prm.elevation_deg < 90.0 {
            true => 1.0_f64 / (elev_rad.sin() + 0.00143 / (elev_rad.tan() + 0.0455)),
            false => 1.0,
        };

        let delta_n = -7.32 * (0.005577 * NS).exp();

        let delta_r =
            (NS + 0.5 * delta_n - NS * h_km - 0.5 * delta_n * h_km.powi(2) + 1430.0 + 732.0)
                * 0.001;

        f * delta_r
    }
    pub(crate) fn value(&self, model: TropoModel, rtm: &RuntimeParams) -> f64 {
        match self {
            Self::Unknown => match model {
                TropoModel::Niel => Self::niel_model(rtm),
                TropoModel::UNB3 => {
                    let (zwd, zdd) = Self::unb3_model(rtm);
                    (zwd + zdd) * 1.001_f64
                        / (0.002001_f64 + rtm.elevation_rad.sin().powi(2)).sqrt()
                },
            },
            Self::Total(tot) => {
                tot * 1.001_f64 / (0.002001_f64 + rtm.elevation_rad.sin().powi(2)).sqrt()
            },
            Self::WetDry((zwd, zdd)) => {
                (zwd + zdd) * 1.001_f64 / (0.002001_f64 + rtm.elevation_rad.sin().powi(2)).sqrt()
            },
        }
    }
}
