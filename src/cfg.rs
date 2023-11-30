use thiserror::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::prelude::{Mode, TimeScale};
use nalgebra::{DMatrix, DVector, Matrix3, MatrixXx4, Vector3};

/// Configuration Error
#[derive(Debug, Error)]
pub enum Error {
    #[error("unknown tropo model")]
    UnknownTropoModel(String),
}

#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct ElevationMappingFunction {
    /// a + b * e-elev/c
    pub a: f64,
    /// a + b * e-elev/c
    pub b: f64,
    /// a + b * e-elev/c
    pub c: f64,
}

impl ElevationMappingFunction {
    pub(crate) fn eval(&self, elev_sv: f64) -> f64 {
        self.a + self.b * (elev_sv / self.c).exp()
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum LSQWeight {
    /// a + b e-elev/c
    LSQWeightMappingFunction(ElevationMappingFunction),
    /// Advanced measurement noise covariance matrix
    LSQWeightCovar,
}

fn default_timescale() -> TimeScale {
    TimeScale::GPST
}

fn default_interp() -> usize {
    11
}

fn default_max_sv() -> usize {
    10
}

fn default_smoothing() -> bool {
    false
}

fn default_sv_clock() -> bool {
    true
}

fn default_sv_tgd() -> bool {
    true
}

fn default_iono() -> bool {
    true
}

fn default_tropo() -> bool {
    true
}

fn default_earth_rot() -> bool {
    true
}

fn default_relativistic_clock_bias() -> bool {
    true
}

fn default_relativistic_path_range() -> bool {
    false
}

fn default_lsq_weight() -> Option<LSQWeight> {
    None
}

#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
/// System Internal Delay as defined by BIPM in
/// "GPS Receivers Accurate Time Comparison" : the (frequency dependent)
/// time delay introduced by the combination of:
///  + the RF cable (up to several nanoseconds)
///  + the distance between the antenna baseline and its APC:
///    a couple picoseconds, and is frequency dependent
///  + the GNSS receiver inner delay (hardware and frequency dependent)
pub struct InternalDelay {
    /// Delay [s]
    pub delay: f64,
    /// Carrier frequency [Hz]
    pub frequency: f64,
}

/// Atmospherical, Physical and Environmental modeling
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Modeling {
    #[cfg_attr(feature = "serde", serde(default))]
    pub sv_clock_bias: bool,
    #[cfg_attr(feature = "serde", serde(default))]
    pub sv_total_group_delay: bool,
    #[cfg_attr(feature = "serde", serde(default))]
    pub relativistic_clock_bias: bool,
    #[cfg_attr(feature = "serde", serde(default))]
    pub relativistic_path_range: bool,
    #[cfg_attr(feature = "serde", serde(default))]
    pub tropo_delay: bool,
    #[cfg_attr(feature = "serde", serde(default))]
    pub iono_delay: bool,
    #[cfg_attr(feature = "serde", serde(default))]
    pub earth_rotation: bool,
}

impl Default for Modeling {
    fn default() -> Self {
        Self {
            sv_clock_bias: default_sv_clock(),
            iono_delay: default_iono(),
            tropo_delay: default_tropo(),
            sv_total_group_delay: default_sv_tgd(),
            earth_rotation: default_earth_rot(),
            relativistic_clock_bias: default_relativistic_clock_bias(),
            relativistic_path_range: default_relativistic_path_range(),
        }
    }
}

impl From<Mode> for Modeling {
    fn from(mode: Mode) -> Self {
        let s = Self::default();
        match mode {
            Mode::PPP => {
                // TODO ?
            },
            _ => {},
        }
        s
    }
}

#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct Config {
    /// Time scale
    #[cfg_attr(feature = "serde", serde(default = "default_timescale"))]
    pub timescale: TimeScale,
    /// (Position) interpolation filter order.
    /// A minimal order must be respected for correct results.
    /// -  7 when working with broadcast ephemeris
    /// - 11 when working with SP3
    #[cfg_attr(feature = "serde", serde(default = "default_interp"))]
    pub interp_order: usize,
    /// Fixed altitude : reduces the need of 4 to 3 SV to resolve a solution
    #[cfg_attr(feature = "serde", serde(default))]
    pub fixed_altitude: Option<f64>,
    /// PR code smoothing filter before moving forward
    #[cfg_attr(feature = "serde", serde(default = "default_smoothing"))]
    pub code_smoothing: bool,
    /// Internal delays
    #[cfg_attr(feature = "serde", serde(default))]
    pub int_delay: Vec<InternalDelay>,
    /// Weight Matrix in LSQ solving process
    #[cfg_attr(feature = "serde", serde(default = "default_lsq_weight"))]
    pub lsq_weight: Option<LSQWeight>,
    /// Time Reference Delay, as defined by BIPM in
    /// "GPS Receivers Accurate Time Comparison" : the time delay
    /// between the GNSS receiver external reference clock and the sampling clock
    /// (once again can be persued as a cable delay). This one is typically
    /// only required in ultra high precision timing applications
    #[cfg_attr(feature = "serde", serde(default))]
    pub externalref_delay: Option<f64>,
    /// Minimal percentage ]0; 1[ of Sun light to be received by an SV
    /// for not to be considered in Eclipse.
    /// A value closer to 0 means we tolerate fast Eclipse exit.
    /// A value closer to 1 is a stringent criteria: eclipse must be totally exited.
    #[cfg_attr(feature = "serde", serde(default))]
    pub min_sv_sunlight_rate: Option<f64>,
    /// Minimal elevation angle. SV below that angle will not be considered.
    #[cfg_attr(feature = "serde", serde(default))]
    pub min_sv_elev: Option<f64>,
    /// Minimal SNR for an SV to be considered.
    #[cfg_attr(feature = "serde", serde(default))]
    pub min_snr: Option<f64>,
    /// modeling
    #[cfg_attr(feature = "serde", serde(default))]
    pub modeling: Modeling,
    /// Max. number of vehicules to consider.
    /// The more the merrier, but it also means heavier computations
    #[cfg_attr(feature = "serde", serde(default = "default_max_sv"))]
    pub max_sv: usize,
}

impl Config {
    pub fn default(solver: Mode) -> Self {
        match solver {
            Mode::SPP => Self {
                timescale: default_timescale(),
                fixed_altitude: None,
                interp_order: default_interp(),
                code_smoothing: default_smoothing(),
                min_sv_sunlight_rate: None,
                min_sv_elev: Some(15.0),
                min_snr: Some(30.0),
                modeling: Modeling::default(),
                max_sv: default_max_sv(),
                int_delay: Default::default(),
                externalref_delay: Default::default(),
                lsq_weight: None,
            },
            Mode::LSQSPP => Self {
                timescale: default_timescale(),
                fixed_altitude: None,
                interp_order: default_interp(),
                code_smoothing: default_smoothing(),
                min_sv_sunlight_rate: None,
                min_sv_elev: Some(10.0),
                min_snr: Some(30.0),
                modeling: Modeling::default(),
                max_sv: default_max_sv(),
                int_delay: Default::default(),
                externalref_delay: Default::default(),
                lsq_weight: Some(LSQWeight::LSQWeightMappingFunction(
                    ElevationMappingFunction {
                        a: 0.03,
                        b: 0.03,
                        c: 100.0,
                    },
                )),
            },
            Mode::PPP => Self {
                timescale: default_timescale(),
                fixed_altitude: None,
                interp_order: 11,
                code_smoothing: default_smoothing(),
                min_sv_sunlight_rate: Some(0.75),
                min_sv_elev: Some(10.0),
                min_snr: Some(30.0),
                //TODO min_snr: Some(SNR::from_str("strong").unwrap()),
                modeling: Modeling::default(),
                max_sv: default_max_sv(),
                int_delay: Default::default(),
                externalref_delay: Default::default(),
                lsq_weight: default_lsq_weight(),
            },
        }
    }
    /*
     * form the weight matrix to be used in the solving process
     */
    pub(crate) fn lsq_weight_matrix(&self, nb_rows: usize, sv_elev: Vec<f64>) -> DMatrix<f64> {
        let mut mat = DMatrix::identity(sv_elev.len(), sv_elev.len());
        match &self.lsq_weight {
            Some(LSQWeight::LSQWeightCovar) => panic!("not implemented yet"),
            Some(LSQWeight::LSQWeightMappingFunction(mapf)) => {
                for i in 0..sv_elev.len() - 1 {
                    mat[(i, i)] = mapf.a + mapf.b * ((-sv_elev[i]) / mapf.c).exp();
                }
            },
            None => {},
        }
        mat
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum SolverMode {
    /// Receiver is kept at fixed location
    #[default]
    Static,
    /// Receiver is not static
    Kinematic,
}
