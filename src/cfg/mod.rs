use thiserror::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{
    navigation::Filter,
    prelude::{PVTSolutionType, Position, TimeScale},
};

use nalgebra::{base::dimension::U8, OMatrix};

mod method;
pub use method::Method;

/// Configuration Error
#[derive(Debug, Error)]
pub enum Error {
    #[error("unknown tropo model")]
    UnknownTropoModel,
}

/// Geometry strategy
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum GeometryStrategy {
    /// Algorithm selects best elevation angles
    #[default]
    BestElevation,
    /// Spread geometry algorithm (aims at minimizing geometric error)
    SpreadAzimuth,
}

/// Rover or receiver use case Profile, to the [Solver]
/// selects appropriate settings. Failing to select
/// the apropriate [Profile] will degrade the solutions.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub enum Profile {
    /// Receiver held in static.
    /// Typically used in Geodetic surveys (GNSS stations Referencing)
    /// and laboratories applications.
    #[default]
    Static,
    /// Roaming: Pedestrian (5 to 10 km/h)
    Pedestrian,
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
pub enum WeightMatrix {
    /// a + b e-elev/c
    MappingFunction(ElevationMappingFunction),
    /// Advanced measurement noise covariance matrix
    Covar,
}

fn default_timescale() -> TimeScale {
    TimeScale::GPST
}

fn default_interp() -> usize {
    11
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
    true
}

fn default_phase_windup() -> bool {
    false
}

fn default_solid_tides() -> bool {
    false
}

fn default_cable_delay() -> bool {
    true
}

fn default_postfit_kf() -> bool {
    false
}

fn default_weight_matrix() -> Option<WeightMatrix> {
    None
    //Some(WeightMatrix::MappingFunction(
    //    ElevationMappingFunction {
    //        a: 5.0,
    //        b: 0.0,
    //        c: 10.0,
    //    },
    //))
}

fn max_tropo_bias() -> f64 {
    30.0
}

fn max_iono_bias() -> f64 {
    10.0
}

fn default_filter_opts() -> Option<FilterOpts> {
    Some(FilterOpts {
        weight_matrix: default_weight_matrix(),
    })
}

fn default_gdop_threshold() -> Option<f64> {
    None
}

fn default_tdop_threshold() -> Option<f64> {
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

#[derive(Default, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct SolverOpts {
    /// GDOP threshold to invalidate ongoing GDOP
    #[cfg_attr(feature = "serde", serde(default = "default_gdop_threshold"))]
    pub gdop_threshold: Option<f64>,
    /// TDOP threshold to invalidate ongoing TDOP
    #[cfg_attr(feature = "serde", serde(default = "default_tdop_threshold"))]
    pub tdop_threshold: Option<f64>,
    /// Filter to use
    #[cfg_attr(feature = "serde", serde(default))]
    pub filter: Filter,
    /// Filter options
    #[cfg_attr(feature = "serde", serde(default = "default_filter_opts"))]
    pub filter_opts: Option<FilterOpts>,
    /// Deploy a post-fit denoising Kalman Filter to denoise and further improve PVT solutions,
    /// at the expense of more calculations.
    #[cfg_attr(feature = "serde", serde(default = "default_postfit_kf"))]
    pub postfit_kf: bool,
}

#[derive(Default, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct FilterOpts {
    /// Weight Matrix
    #[cfg_attr(feature = "serde", serde(default = "default_weight_matrix"))]
    pub weight_matrix: Option<WeightMatrix>,
}

impl SolverOpts {
    /*
     * form the weight matrix to be used in the solving process
     */
    pub(crate) fn weight_matrix(&self) -> OMatrix<f64, U8, U8> {
        let mat = OMatrix::<f64, U8, U8>::identity();
        if let Some(opts) = &self.filter_opts {
            match &opts.weight_matrix {
                Some(WeightMatrix::Covar) => panic!("not implemented yet"),
                Some(WeightMatrix::MappingFunction(_)) => panic!("mapf: not implemented yet"),
                //                Some(WeightMatrix::MappingFunction(mapf)) => {
                //                    for i in 0..8 {
                //                        let sigma = mapf.a + mapf.b * ((-sv_elev[i]) / mapf.c).exp();
                //                        mat[(i, i)] = 1.0 / sigma.powi(2);
                //                    }
                //                },
                None => {},
            }
        }
        mat
    }
}

/// Atmospherical, Physical and Environmental modeling
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Modeling {
    /// Compensate for onboard clock offset to system time (+/- 100km)
    #[cfg_attr(feature = "serde", serde(default))]
    pub sv_clock_bias: bool,
    /// Compensate for onboard circuitry delay (+/- 1m)
    #[cfg_attr(feature = "serde", serde(default))]
    pub sv_total_group_delay: bool,
    /// Compensate for relativistic effect on onboard clock (+/- 1m)
    #[cfg_attr(feature = "serde", serde(default))]
    pub relativistic_clock_bias: bool,
    /// Compensate for relativistic effect on signal propagation (+/- 0.1 m)
    #[cfg_attr(feature = "serde", serde(default))]
    pub relativistic_path_range: bool,
    /// Compensate for troposphere negative impact (+/- 10m)
    #[cfg_attr(feature = "serde", serde(default))]
    pub tropo_delay: bool,
    /// Compensate for ionosphere negative impact (+/- 10m).
    /// If Method is not [Method::SPP], this is
    /// natively taken care of and option is actually disregarded.
    #[cfg_attr(feature = "serde", serde(default))]
    pub iono_delay: bool,
    /// Compensate for Earth rotation during signal propagation
    /// (static +5/+10m eastern error).
    #[cfg_attr(feature = "serde", serde(default))]
    pub earth_rotation: bool,
    /// Compensate for signal phase windup. This only impacts
    /// strategies that use raw phase like [Method::PPP].
    #[cfg_attr(feature = "serde", serde(default))]
    pub phase_windup: bool,
    /// Setup cable delay compensation.
    /// Only effective if the (RF) cable delay of your setup
    /// are known and defined in [Config]. Only careful
    /// cable delay specs will allow differential timing analysis.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cable_delay: bool,
    /// Compensate to crust (solid body) deformation due to moon and star
    /// gravitational effect.
    pub solid_tides: bool,
}

impl Default for Modeling {
    fn default() -> Self {
        Self {
            sv_clock_bias: default_sv_clock(),
            iono_delay: default_iono(),
            tropo_delay: default_tropo(),
            sv_total_group_delay: default_sv_tgd(),
            earth_rotation: default_earth_rot(),
            phase_windup: default_phase_windup(),
            solid_tides: default_solid_tides(),
            cable_delay: default_cable_delay(),
            relativistic_clock_bias: default_relativistic_clock_bias(),
            relativistic_path_range: default_relativistic_path_range(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct Config {
    /// Type of solutions to form.
    #[cfg_attr(feature = "serde", serde(default))]
    pub sol_type: PVTSolutionType,
    /// Time scale in which we express the PVT solutions,
    /// [TimeScale::GPST] is the default value.
    #[cfg_attr(feature = "serde", serde(default = "default_timescale"))]
    pub timescale: TimeScale,
    /// Solver method (strategy) used.
    #[cfg_attr(feature = "serde", serde(default))]
    pub method: Method,
    /// [Profile] defines the type of application.
    #[cfg_attr(feature = "serde", serde(default))]
    pub profile: Profile,
    /// Possible reference site static coordinates.
    /// Must be defined in case RTK navigation is selected.
    pub remote_site: Option<Position>,
    /// Interpolation order
    #[cfg_attr(feature = "serde", serde(default = "default_interp"))]
    pub interp_order: usize,
    /// Fixed altitude: reduces the need of 4 to 3 SV to obtain 3D solutions.
    #[cfg_attr(feature = "serde", serde(default))]
    pub fixed_altitude: Option<f64>,
    /// Pseudo Range smoothing. Use this to improve solutions accuracy.
    /// This applies to all positioning strategies.
    #[cfg_attr(feature = "serde", serde(default = "default_smoothing"))]
    pub code_smoothing: bool,
    /// Internal delays to compensate for (total summation, in [s]).
    /// Compensation is only effective if [Modeling.cable_delay]
    /// is also turned on.
    #[cfg_attr(feature = "serde", serde(default))]
    pub int_delay: Vec<InternalDelay>,
    /// Antenna Reference Point (ARP) expressed as ENU offset [m]
    #[cfg_attr(feature = "serde", serde(default))]
    pub arp_enu: Option<(f64, f64, f64)>,
    /// Solver customization
    #[cfg_attr(feature = "serde", serde(default))]
    pub solver: SolverOpts,
    /// Time Reference Delay. According to BIPM ""GPS Receivers Accurate Time Comparison""
    /// this is the time delay between the receiver external reference clock
    /// and the internal sampling clock. This is typically needed in
    /// ultra high precision timing applications or geodetic surveys.
    /// Compensation is only effective if [Modeling.cable_delay]
    /// is also turned on.
    #[cfg_attr(feature = "serde", serde(default))]
    pub externalref_delay: Option<f64>,
    /// Minimal rate of Sun light rate one SV must receive for not to be considered Eclipsed from the Sun by Earth.
    /// Closer to 0 means we exit Eclipsed state faster.
    /// Closer to 1 means stringent condition and only completely illuminated SV are considered.
    /// This criteria is not meaningful unless you're using centimetric positioning strategies like [Method::PPP].
    #[cfg_attr(feature = "serde", serde(default))]
    pub min_sv_sunlight_rate: Option<f64>,
    /// Minimal SV elevation angle for an SV to contribute to the solution.
    /// Use this as a simple quality criteria.
    #[cfg_attr(feature = "serde", serde(default))]
    pub min_sv_elev: Option<f64>,
    /// Minimal SV Azimuth angle for an SV to contribute to the solution.
    /// SV below that angle will not be considered.
    /// Use this is in special navigation scenarios.
    #[cfg_attr(feature = "serde", serde(default))]
    pub min_sv_azim: Option<f64>,
    /// Maximal SV Azimuth angle for an SV to contribute to the solution.
    /// SV below that angle will not be considered.
    /// Use this is in special navigation scenarios.
    #[cfg_attr(feature = "serde", serde(default))]
    pub max_sv_azim: Option<f64>,
    /// Minimal SNR for an SV to contribute to the solution.
    #[cfg_attr(feature = "serde", serde(default))]
    pub min_snr: Option<f64>,
    /// Maximal tropo bias that we tolerate (in [m]).
    /// Has no effect if modeling.tropo_delay is disabled.
    #[cfg_attr(feature = "serde", serde(default = "max_tropo_bias"))]
    pub max_tropo_bias: f64,
    /// Maximal iono bias that we tolerate (in [m]).
    /// Has no effect if modeling.iono_delay is disabled.
    #[cfg_attr(feature = "serde", serde(default = "max_iono_bias"))]
    pub max_iono_bias: f64,
    /// Atmospherical and Physical [Modeling] used to improve the accuracy of solution.
    #[cfg_attr(feature = "serde", serde(default))]
    pub modeling: Modeling,
}

impl Config {
    /// Returns a basic [Config] that is consistent with given Positioning [Method]
    /// and a static GNSS receiver.
    pub fn static_preset(method: Method) -> Self {
        match method {
            Method::SPP => Self {
                method,
                profile: Profile::Static,
                sol_type: PVTSolutionType::default(),
                remote_site: None,
                arp_enu: None,
                fixed_altitude: None,
                timescale: default_timescale(),
                interp_order: default_interp(),
                code_smoothing: default_smoothing(),
                min_snr: None,
                min_sv_elev: Some(10.0),
                min_sv_azim: None,
                max_sv_azim: None,
                min_sv_sunlight_rate: None,
                max_tropo_bias: max_tropo_bias(),
                max_iono_bias: max_iono_bias(),
                modeling: Modeling::default(),
                int_delay: Default::default(),
                externalref_delay: Default::default(),
                solver: SolverOpts {
                    filter: Filter::LSQ,
                    gdop_threshold: default_gdop_threshold(),
                    tdop_threshold: default_tdop_threshold(),
                    filter_opts: default_filter_opts(),
                    postfit_kf: default_postfit_kf(),
                },
            },
            Method::CPP => Self {
                method,
                profile: Profile::Static,
                sol_type: PVTSolutionType::default(),
                arp_enu: None,
                remote_site: None,
                fixed_altitude: None,
                timescale: default_timescale(),
                interp_order: default_interp(),
                code_smoothing: default_smoothing(),
                min_snr: None,
                min_sv_elev: Some(10.0),
                min_sv_azim: None,
                max_sv_azim: None,
                min_sv_sunlight_rate: None,
                max_tropo_bias: max_tropo_bias(),
                max_iono_bias: max_iono_bias(),
                modeling: Modeling::default(),
                int_delay: Default::default(),
                externalref_delay: Default::default(),
                solver: SolverOpts {
                    filter: Filter::LSQ,
                    gdop_threshold: default_gdop_threshold(),
                    tdop_threshold: default_tdop_threshold(),
                    filter_opts: default_filter_opts(),
                    postfit_kf: default_postfit_kf(),
                },
            },
            Method::PPP => Self {
                method,
                profile: Profile::Static,
                sol_type: PVTSolutionType::default(),
                arp_enu: None,
                remote_site: None,
                fixed_altitude: None,
                timescale: default_timescale(),
                interp_order: default_interp(),
                code_smoothing: default_smoothing(),
                min_snr: None,
                min_sv_elev: Some(10.0),
                min_sv_azim: None,
                max_sv_azim: None,
                min_sv_sunlight_rate: None,
                max_tropo_bias: max_tropo_bias(),
                max_iono_bias: max_iono_bias(),
                modeling: Modeling::default(),
                int_delay: Default::default(),
                externalref_delay: Default::default(),
                solver: SolverOpts {
                    filter: Filter::LSQ,
                    gdop_threshold: default_gdop_threshold(),
                    tdop_threshold: default_tdop_threshold(),
                    filter_opts: default_filter_opts(),
                    postfit_kf: default_postfit_kf(),
                },
            },
        }
    }
}
