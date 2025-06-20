#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

fn default_sv_clock() -> bool {
    true
}

fn default_group_delay() -> bool {
    true
}

fn default_iono_delay() -> bool {
    true
}

fn default_tropo_delay() -> bool {
    true
}

fn default_earth_rot() -> bool {
    true
}

fn default_relativistic_clock() -> bool {
    true
}

fn default_relativistic_path() -> bool {
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

/// Atmospherical, Physical and Environmental modeling
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Modeling {
    /// Compensate for onboard clock offset to system time (+/- 100km)
    #[cfg_attr(feature = "serde", serde(default = "default_sv_clock"))]
    pub sv_clock_bias: bool,

    /// Compensate for onboard transmission delay (+/- 1m)
    #[cfg_attr(feature = "serde", serde(default = "default_group_delay"))]
    pub sv_total_group_delay: bool,

    /// Compensate for relativistic effect on onboard clock (+/- 1m)
    #[cfg_attr(feature = "serde", serde(default = "default_relativistic_clock"))]
    pub relativistic_clock_bias: bool,

    /// Compensate for relativistic effect on signal propagation (+/- 0.1 m)
    #[cfg_attr(feature = "serde", serde(default = "default_relativistic_path"))]
    pub relativistic_path_range: bool,

    /// Compensate for troposphere negative impact (+/- 10m)
    #[cfg_attr(feature = "serde", serde(default = "default_tropo_delay"))]
    pub tropo_delay: bool,

    /// Enable Ionospheric bias compensation.
    /// When [Method] is not set to [Method::SPP], this is actually taken care
    /// of physically and is disregarded, as long as your observations fit this
    /// requirement.
    #[cfg_attr(feature = "serde", serde(default = "default_iono_delay"))]
    pub iono_delay: bool,

    /// Compensate for Earth rotation during signal propagation
    /// (static +5/+10m eastern error).
    #[cfg_attr(feature = "serde", serde(default = "default_earth_rot"))]
    pub earth_rotation: bool,

    /// Compensate for signal phase windup. This only impacts
    /// strategies that use raw phase like [Method::PPP].
    #[cfg_attr(feature = "serde", serde(default = "default_phase_windup"))]
    pub phase_windup: bool,

    /// Setup cable delay compensation.
    /// Only effective if the (RF) cable delay of your setup
    /// are known and defined in [Config]. Only careful
    /// cable delay specs will allow differential timing analysis.
    #[cfg_attr(feature = "serde", serde(default = "default_cable_delay"))]
    pub cable_delay: bool,

    /// Compensate for crust (solid body) deformation due to moon and star
    /// gravitational effect.
    #[cfg_attr(feature = "serde", serde(default = "default_solid_tides"))]
    pub solid_tides: bool,
}

impl Default for Modeling {
    fn default() -> Self {
        Self {
            iono_delay: default_iono_delay(),
            sv_clock_bias: default_sv_clock(),
            solid_tides: default_solid_tides(),
            cable_delay: default_cable_delay(),
            tropo_delay: default_tropo_delay(),
            earth_rotation: default_earth_rot(),
            phase_windup: default_phase_windup(),
            sv_total_group_delay: default_group_delay(),
            relativistic_clock_bias: default_relativistic_clock(),
            relativistic_path_range: default_relativistic_path(),
        }
    }
}

impl Modeling {
    /// Defines a null [Modeling] structure where all physical
    /// perturbations and phenomena are not accounted for.
    /// This is not the default value! Use this for teaching
    /// purposes only.
    pub fn no_modeling() -> Modeling {
        Modeling {
            sv_clock_bias: false,
            sv_total_group_delay: false,
            relativistic_clock_bias: false,
            relativistic_path_range: false,
            tropo_delay: false,
            iono_delay: false,
            earth_rotation: false,
            phase_windup: false,
            cable_delay: false,
            solid_tides: false,
        }
    }
}
