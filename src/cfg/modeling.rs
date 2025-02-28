#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

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
    /// Enable Ionospheric bias compensation.
    /// When [Method] is not set to [Method::SPP], this is actually taken care
    /// of physically and is disregarded, as long as your observations fit this
    /// requirement.
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
    /// Compensate for crust (solid body) deformation due to moon and star
    /// gravitational effect.
    #[cfg_attr(feature = "serde", serde(default))]
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
