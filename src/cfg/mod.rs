use thiserror::Error;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

mod method;
mod modeling;
mod profile;
mod solver;

pub use crate::{
    carrier::Signal,
    cfg::solver::SolverOpts,
    cfg::{method::Method, modeling::Modeling, profile::Profile},
    prelude::TimeScale,
};

pub(crate) use crate::cfg::solver::LoopExitCriteria;

/// Configuration Error
#[derive(Debug, Error)]
pub enum Error {
    #[error("invalid troposphere model")]
    InvalidTroposphereModel,
}

fn default_timescale() -> TimeScale {
    TimeScale::GPST
}

fn default_smoothing() -> bool {
    false
}

fn max_tropo_bias() -> f64 {
    30.0
}

fn max_iono_bias() -> f64 {
    10.0
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

#[derive(Default, Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Deserialize))]
pub struct Config {
    /// Time scale in which we express the PVT solutions,
    /// [TimeScale::GPST] is the default value.
    #[cfg_attr(feature = "serde", serde(default = "default_timescale"))]
    pub timescale: TimeScale,
    /// Navigation [Method] (technique) to be used.
    #[cfg_attr(feature = "serde", serde(default))]
    pub method: Method,
    /// [Profile] defines the type of application.
    #[cfg_attr(feature = "serde", serde(default))]
    pub profile: Profile,
    /// Select a prefered signal.
    /// When defined, this signal will strictly be used in the navigation process.
    /// When undefined, the algorithm will prefer the best SNR available, and the
    /// signal frequency being used might change.
    /// When [Method] is [Method::SPP] this should be a single frequency.
    /// When [Method] is not [Method::SPP] and [Modeling] enables Ionospheric
    /// bias compensation,
    #[cfg_attr(feature = "serde", serde(default))]
    pub prefered_signal: Option<Signal>,
    /// Possible remote reference site coordinates, in ECEF [m].
    /// Must be defined in case RTK navigation is selected.
    pub remote_site: Option<(f64, f64, f64)>,
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
    /// Maximal Earth / Sun occultation tolerated for each satellite in orbit.
    /// This is percentage, > 99.9 means total darkness.
    /// 20.0% for example, would mean partially eclipsed satellites are to be discarded
    /// by the solver.
    #[cfg_attr(feature = "serde", serde(default))]
    pub max_sv_occultation_percent: Option<f64>,
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
    /// Returns [Config] for static PPP positioning, with desired [Method].
    /// You can then customize [Self] as you will.
    pub fn static_ppp_preset(method: Method) -> Self {
        let mut s = Self::default();
        s.profile = Profile::Static;
        s.method = method;
        s.min_sv_elev = Some(15.0);
        s.max_tropo_bias = max_tropo_bias();
        s.max_iono_bias = max_iono_bias();
        s
    }

    /// Returns [Config] for dynamic PPP positioning, with desired [Method]
    /// and rover [Profile]. You can then customize [Self] as you will.
    pub fn dynamic_ppp_preset(profile: Profile, method: Method) -> Self {
        let mut s = Self::default();
        s.profile = profile;
        s.method = method;
        s.min_sv_elev = Some(15.0);
        s.max_tropo_bias = max_tropo_bias();
        s.max_iono_bias = max_iono_bias();
        s
    }

    /// Returns [Config] for static RTK positioning, with desired [Method],
    /// Remote site coordinates expressed in meters ECEF.
    /// You can then customize [Self] as you will.
    pub fn static_rtk_preset(method: Method, remote_site_ecef_m: (f64, f64, f64)) -> Self {
        let mut s = Self::default();
        s.profile = Profile::Static;
        s.method = method;
        s.remote_site = Some(remote_site_ecef_m);
        s.min_sv_elev = Some(15.0);
        s.max_tropo_bias = max_tropo_bias();
        s.max_iono_bias = max_iono_bias();
        s
    }

    /// Returns [Config] for dynamic RTK positioning, with desired [Method],
    /// rover [Profile] and reference Remote Site coordinates expressed
    /// as meters ECEF. You can then customize [Self] as you will.
    pub fn dynamic_rtk_preset(
        profile: Profile,
        method: Method,
        remote_site_ecef_m: (f64, f64, f64),
    ) -> Self {
        let mut s = Self::default();
        s.profile = profile;
        s.method = method;
        s.remote_site = Some(remote_site_ecef_m);
        s.min_sv_elev = Some(15.0);
        s.max_tropo_bias = max_tropo_bias();
        s.max_iono_bias = max_iono_bias();
        s
    }

    /// Returns new [Config] with desired navigation [Method]
    pub fn with_navigation_method(&self, method: Method) -> Self {
        let mut s = self.clone();
        s.method = method;
        s
    }

    /// Copies and returns [Config] with desired [Modeling], that you can
    /// tune to improve the accuracy of your solution, or for learning purposes.
    pub fn with_modeling(&self, modeling: Modeling) -> Self {
        let mut s = self.clone();
        s.modeling = modeling;
        s
    }
}
