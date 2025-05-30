use crate::cfg::Error;
use crate::prelude::{SPEED_OF_LIGHT_M_S, Duration};

use nalgebra::{DimName, OMatrix, allocator::Allocator, DefaultAllocator};

use std::f64::consts::PI;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Default acceleration PSD
const fn default_accel_psd() -> f64 {
    0.5_f64 * 0.5_f64 // compatible with pedestrian profile
}

/// Default clock PSD
const fn default_clock_psd() -> f64 {
    0.5_f64 * 2.0E-19
}

/// Default clock PSD
const fn default_clock_drift_psd() -> f64 {
    2.0_f64 * PI * PI * 2.0E-20
}

/// [UserPreset] can be used to generate a basic [UserProfile] easily.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum UserPreset {
    /// [Profile::Static] applies to user applications where
    /// the receiver antenna is held static at all times.
    /// This is not our prefered mode, because this apply to particular use cases.
    Static,

    /// [Profile::Pedestrian]: < 10 km/h very low velocity.
    /// This is our default mode.
    #[cfg_attr(feature = "serde", serde(alias = "pedestrian", alias = "Pedestrian"))]
    #[default]
    Pedestrian,

    /// [Profile::Car]: < 100 km/h slow velocity
    #[cfg_attr(feature = "serde", serde(alias = "car", alias = "Car"))]
    Car,

    /// [Profile::Airplane]: < 1000 km/h high velocity
    #[cfg_attr(feature = "serde", serde(alias = "airplane", alias = "airplane"))]
    Airplane,

    /// [Profile::Rocket]: > 1000 km/h ultra high velocity
    #[cfg_attr(feature = "serde", serde(alias = "rocket", alias = "rocket"))]
    Rocket,
}

impl UserPreset {
    /// Generate a [UserProfile] from this [UserPreset].
    /// Note that the clock profile is still the default preset
    /// and requires fine tuning.
    pub fn profiling(&self) -> UserProfile {
        match self {
            Self::Static => {
                UserProfile {
                    accel_psd: 0.0,
                    clock_psd: default_clock_psd(),
                    clock_drift_psd: default_clock_drift_psd(),
                }
            },
            Self::Pedestrian => {
                UserProfile {
                    accel_psd: 0.5,
                    clock_psd: default_clock_psd(),
                    clock_drift_psd: default_clock_drift_psd(),
                }
            },
            Self::Car => {
                UserProfile {
                    accel_psd: 2.0,
                    clock_psd: default_clock_psd(),
                    clock_drift_psd: default_clock_drift_psd(),
                }
            },
            Self::Airplane => {
                UserProfile {
                    accel_psd: 50.0,
                    clock_psd: default_clock_psd(),
                    clock_drift_psd: default_clock_drift_psd(),
                }
            },
            Self::Rocket => {
                UserProfile {
                    accel_psd: 1000.0,
                    clock_psd: default_clock_psd(),
                    clock_drift_psd: default_clock_drift_psd(),
                }
            },
        }
    }
}

impl std::str::FromStr for UserPreset {
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

impl std::fmt::Display for UserPreset {
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

/// [UserProfile] are applications dependent.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UserProfile {
    /// Acceleration Power Spectral Density (PSD) in m^2 m s-1.
    /// Directly tied to your application profile and attitude.
    /// For cars we recommend 2.0. For pedestrian 0.5,
    /// and obviously Null for static applications.
    #[cfg_attr(feature = "serde", serde(default = "default_accel_psd"))]
    pub accel_psd: f64,

    /// Clock bias PSD
    #[cfg_attr(feature = "serde", serde(default = "default_clock_psd"))]
    pub clock_psd: f64,

    /// Clock drift PSD
    #[cfg_attr(feature = "serde", serde(default = "default_clock_drift_psd"))]
    pub clock_drift_psd: f64,
}

impl std::fmt::Display for UserProfile {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "accel-psd={}, clock-psd={}, clock-drift-psd={}", self.accel_psd, self.clock_psd, self.clock_drift_psd)
    }
}

impl Default for UserProfile {
    fn default() -> Self {
        Self {
            accel_psd: default_accel_psd(),
            clock_psd: default_clock_psd(),
            clock_drift_psd: default_clock_drift_psd(),
        }
    }
}

impl UserProfile {
    pub(crate) fn q_matrix<D: DimName>(&self, dt: Duration) -> OMatrix<f64, D, D> where
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

           q_k[(3, 3)] = SPEED_OF_LIGHT_M_S.powi(2) * (self.clock_psd * dt_s + self.clock_drift_psd * dt_s3 / 3.0);
           //  self.q_k[(Self::clock_index(), Self::clock_index())] =
           //      (user.clock_sigma_s * SPEED_OF_LIGHT_M_S).powi(2);
            q_k[(3, 3)] = (100e-3 * SPEED_OF_LIGHT_M_S).powi(2);
        }
        
        q_k
    }
}
