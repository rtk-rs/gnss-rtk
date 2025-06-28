use crate::prelude::{BiasRuntime, Duration};

#[cfg(doc)]
use crate::prelude::Method;

#[derive(Default, Debug, Copy, Clone)]
pub struct SatelliteClockCorrection {
    /// Correction to associated timescale, expressed as [Duration]
    pub duration: Duration,

    /// True when relativistic effect has not been corrected.
    pub(crate) needs_relativistic_correction: bool,
}

impl SatelliteClockCorrection {
    /// Define a new [SatelliteClockCorrection] that already integrates relativistic corrections
    pub fn with_relativistic_correction(duration: Duration) -> Self {
        Self {
            duration,
            needs_relativistic_correction: false,
        }
    }

    /// Define a new [SatelliteClockCorrection] that does not integrate relativistic corrections
    pub fn without_relativistic_correction(duration: Duration) -> Self {
        Self {
            duration,
            needs_relativistic_correction: true,
        }
    }
}

/// [SpacebornBias] must be implemented to determine all on-board biases.
/// Basic navigation only requires [SpacebornBias::satellite_clock_bias],
/// but precise navigation requires all of it.
pub trait SpacebornBias {
    /// Provide the [SatelliteClockCorrection] for requested satellite,
    /// as epoch of observation.
    /// If this is not known, simply return [Default::default()], but
    /// it will dramatically impact the accuracy of any aboslute solution (>100km).
    /// This value is disregarded when sv_clock_bias is not being modeled.
    /// This value does not impact the accuracy of RTK solutions.
    fn clock_bias(&self, rtm: &BiasRuntime) -> SatelliteClockCorrection;

    /// Provide the Satellite specific on-board group delay,
    /// referece to the L1 frequency.
    /// This value will be disregarded when sv_total_group_delay
    /// is not being modeled. If this value is not known,
    /// simply return Duration::ZERO.
    /// This is mandatory when sv_total_group_delay is being modeled,
    /// otherwise this [SV] will get dropped.
    fn group_delay(&self, rtm: &BiasRuntime) -> Duration;

    /// Provide the Satellite MW internal bias for requested [SV].
    /// This is mandatory in [Method::PPP] technique is being used,
    /// otherwise this [SV] will get dropped.
    fn mw_bias(&self, rtm: &BiasRuntime) -> f64;
}
