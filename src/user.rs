use crate::cfg::Error;
use crate::prelude::{Duration, SPEED_OF_LIGHT_M_S};

use std::f64::consts::PI;

use nalgebra::{allocator::Allocator, DefaultAllocator, DimName, OMatrix};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Default acceleration PSD
const fn default_accel_psd() -> f64 {
    0.5_f64 * 0.5_f64
}

/// Default clock PSD
const fn default_clock_psd() -> f64 {
    0.5_f64 * 2.0E-19
}

/// Default clock PSD
const fn default_clock_drift_psd() -> f64 {
    2.0_f64 * PI * PI * 2.0E-20
}

/// [UserProfile] can be used to generate a set of [UserParameters] easily.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum UserProfile {
    /// [UserProfile::Static] applies to user applications where
    /// the receiver antenna is held static at all times.
    /// This is not our prefered mode, because this apply to particular use cases.
    Static,

    /// [UserProfile::Pedestrian]: < 10 km/h very low velocity.
    /// This is our default mode.
    #[cfg_attr(feature = "serde", serde(alias = "pedestrian", alias = "Pedestrian"))]
    #[default]
    Pedestrian,

    /// [UserProfile::Car]: < 100 km/h slow velocity
    #[cfg_attr(feature = "serde", serde(alias = "car", alias = "Car"))]
    Car,

    /// [UserProfile::Airplane]: < 1000 km/h high velocity
    #[cfg_attr(feature = "serde", serde(alias = "airplane", alias = "airplane"))]
    Airplane,

    /// [UserProfile::Rocket]: > 1000 km/h ultra high velocity
    #[cfg_attr(feature = "serde", serde(alias = "rocket", alias = "rocket"))]
    Rocket,
}

impl UserProfile {
    pub(crate) fn psd(&self) -> f64 {
        match self {
            Self::Static => 0.0,
            Self::Pedestrian => 0.5f64.powi(2),
            Self::Car => 2.0f64.powi(2),
            Self::Airplane => 50.0f64.powi(2),
            Self::Rocket => 1000.0f64.powi(2),
        }
    }
}

impl std::str::FromStr for UserProfile {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.to_lowercase();
        let trimmed = s.trim();
        match trimmed {
            "static" => Ok(Self::Static),
            "pedestrian" => Ok(Self::Pedestrian),
            "car" => Ok(Self::Car),
            "airplane" => Ok(Self::Airplane),
            "rocket" => Ok(Self::Rocket),
            _ => Err(Error::InvalidUserProfile),
        }
    }
}

impl std::fmt::Display for UserProfile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Static => write!(f, "Static"),
            Self::Pedestrian => write!(f, "Pedestrian"),
            Self::Car => write!(f, "car"),
            Self::Airplane => write!(f, "airplane"),
            Self::Rocket => write!(f, "rocket"),
        }
    }
}

pub enum ClockProfile {
    /// [ClockProfile::Quartz] low quality clock
    Quartz,

    /// [ClockProfile::Oscillator] medium quality clock
    Oscillator,

    /// [ClockProfile::Atomic] high quality clock
    Atomic,
}

impl ClockProfile {
    pub(crate) fn bias_psd(&self) -> f64 {
        match self {
            Self::Quartz => 0.5 * 2.0E-19,
            Self::Oscillator => 0.5 * 2.0E-19,
            Self::Atomic => 0.5 * 2.0E-19,
        }
    }

    pub(crate) fn drift_psd(&self) -> f64 {
        match self {
            Self::Quartz => 2.0 * 2.0E-20,
            Self::Oscillator => 2.0 * 2.0E-20,
            Self::Atomic => 2.0 * 2.0E-20,
        }
    }
}

/// [UserParameters] are applications dependent.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UserParameters {
    /// Acceleration Power Spectral Density (PSD) in m².s⁻.¹
    /// Directly tied to your application profile and attitude.
    /// For cars we recommend 2.0. For pedestrian 0.5,
    /// and obviously 0.0 for static applications.
    #[cfg_attr(feature = "serde", serde(default = "default_accel_psd"))]
    pub accel_psd: f64,

    /// Clock bias PSD
    #[cfg_attr(feature = "serde", serde(default = "default_clock_psd"))]
    pub clock_psd: f64,

    /// Clock drift PSD
    #[cfg_attr(feature = "serde", serde(default = "default_clock_drift_psd"))]
    pub clock_drift_psd: f64,
}

impl std::fmt::Display for UserParameters {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "a={}m².s⁻¹, offset={}s, drift={}s.s⁻¹",
            self.accel_psd, self.clock_psd, self.clock_drift_psd
        )
    }
}

impl Default for UserParameters {
    fn default() -> Self {
        Self {
            accel_psd: default_accel_psd(),
            clock_psd: default_clock_psd(),
            clock_drift_psd: default_clock_drift_psd(),
        }
    }
}

impl UserParameters {
    /// Creates a [UserParameters] set from [UserProfile] and [ClockProfile]
    pub fn new(user_profile: UserProfile, clock_profile: ClockProfile) -> Self {
        Self {
            accel_psd: user_profile.psd(),
            clock_psd: clock_profile.bias_psd(),
            clock_drift_psd: clock_profile.drift_psd(),
        }
    }

    pub(crate) fn q_matrix<D: DimName>(&self, dt: Duration) -> OMatrix<f64, D, D>
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
    {
        let mut q_k = OMatrix::<f64, D, D>::zeros();
        let dt_s = dt.to_seconds();
        let dt_s3 = dt_s.powi(3);

        q_k[(0, 0)] = self.accel_psd * dt_s3 / 3.0;

        q_k[(0, 0)] = 1.0;
        q_k[(1, 1)] = 1.0;
        q_k[(2, 2)] = 1.0;

        if D::USIZE == 4 {
            q_k[(3, 3)] = SPEED_OF_LIGHT_M_S.powi(2)
                * (self.clock_psd * dt_s + self.clock_drift_psd * dt_s3 / 3.0);
            //  self.q_k[(Self::clock_index(), Self::clock_index())] =
            //      (user.clock_sigma_s * SPEED_OF_LIGHT_M_S).powi(2);
            q_k[(3, 3)] = (100e-3 * SPEED_OF_LIGHT_M_S).powi(2);
        }

        q_k
    }
}
