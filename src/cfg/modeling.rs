#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

fn default_sv_clock() -> bool {
    true
}

fn default_group_delay() -> bool {
    true
}

fn default_ionospheric_bias() -> bool {
    true
}

fn default_tropospheric_bias() -> bool {
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
    #[cfg_attr(feature = "serde", serde(default = "default_tropospheric_bias"))]
    pub tropospheric_bias: bool,

    /// Compensate for ionospheric negative impact (+/- 20m)
    /// Starting from [Method::CPP] and on, this value is disregarded as the
    /// measurement strategy allows to physically canceled this phenomenon,
    /// which is always prefered.
    #[cfg_attr(feature = "serde", serde(default = "default_ionospheric_bias"))]
    pub ionospheric_bias: bool,

    /// Compensate for Earth rotation during signal propagation
    /// (static +5/+10m eastern error).
    #[cfg_attr(feature = "serde", serde(default = "default_earth_rot"))]
    pub earth_rotation: bool,

    /// Compensate for signal phase windup. This only impacts
    /// strategies that use raw phase like [Method::PPP].
    #[cfg_attr(feature = "serde", serde(default = "default_phase_windup"))]
    pub phase_windup: bool,

    /// Compensate for crust (solid body) deformation due to moon and star
    /// gravitational effect.
    #[cfg_attr(feature = "serde", serde(default = "default_solid_tides"))]
    pub solid_tides: bool,
}

impl Default for Modeling {
    fn default() -> Self {
        Self {
            sv_clock_bias: default_sv_clock(),
            solid_tides: default_solid_tides(),
            earth_rotation: default_earth_rot(),
            phase_windup: default_phase_windup(),
            sv_total_group_delay: default_group_delay(),
            ionospheric_bias: default_ionospheric_bias(),
            tropospheric_bias: default_tropospheric_bias(),
            relativistic_clock_bias: default_relativistic_clock(),
            relativistic_path_range: default_relativistic_path(),
        }
    }
}

impl Modeling {
    /// Defines a new [Modeling] structure without any of the
    /// compensation enabled, that you can then customize.
    /// This is intended for teaching purposes only.
    /// Until [Modeling.sv_clock_bias] is enabled, your solution
    /// will be off by hundreds of kilometers.
    pub fn no_modeling() -> Modeling {
        Modeling {
            sv_clock_bias: false,
            sv_total_group_delay: false,
            relativistic_clock_bias: false,
            relativistic_path_range: false,
            tropospheric_bias: false,
            ionospheric_bias: false,
            earth_rotation: false,
            phase_windup: false,
            solid_tides: false,
        }
    }
}
