use crate::prelude::{Epoch, SV};

pub(crate) mod private;

#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct Ephemeris {
    /// [SV]
    pub sv: SV,

    /// Time of Issue of [Ephemeris] that must be expressed in correct timescale
    pub toe: Epoch,

    /// Time of Clock that must be expressed in correct timescale
    pub toc: Epoch,

    /// Semi-major axis (in meters)
    pub semi_major_axis_m: f64,

    /// Eccentricity
    pub eccentricity: f64,

    /// m0 (in radians)
    pub m0_rad: f64,

    /// (in radians)
    pub i0_rad: f64,

    /// (in radians/s)
    pub idot_rad_s: f64,

    /// (in radians)
    pub dn_rad: f64,

    /// (in radians)
    pub omega0_rad: f64,

    /// (in radians)
    pub omega_rad: f64,

    /// (in radians/s)
    pub omega_dot_rad_s: f64,

    /// Sine Cosine (in radians)
    pub cus_cuc_rad: (f64, f64),

    /// Sine / Cosine (in radians)
    pub cis_cic_rad: (f64, f64),

    /// Sine / Cosine (in meters)
    pub crs_crc_m: (f64, f64),
}

/// [EphemerisSource] to provie [Ephemeris] data and contribute to the solving process.
pub trait EphemerisSource {
    /// :warning: this is work in progress, it is not possible to navigate using
    /// ephemeris using current version.
    ///
    /// Provide [Ephemeris] frame for requested [SV]. Provided [Epoch] is the ongoing
    /// [Epoch] being processed by the solver.
    /// It is possible to navigate without proposing any [Orbit]al state, if you maintain
    /// up-to-date [Ephemeris] data.
    fn ephemeris_data(&self, epoch: Epoch, sv: SV) -> Option<Ephemeris>;
}
