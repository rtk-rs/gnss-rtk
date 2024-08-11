/// 3D Position coordinates
use crate::prelude::Vector3;
use map_3d::{ecef2geodetic, geodetic2ecef, Ellipsoid};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Position is used as the (optional) [Solver] initial point.
#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Position {
    /// ECEF coordinates in meters
    pub(crate) ecef: Vector3<f64>,
    /// Geodetic coordinates in radians
    pub(crate) geodetic: Vector3<f64>,
}

impl Position {
    /// Builds new [Position] from ECEF coordinates expressed in meter.
    pub fn from_ecef(ecef: Vector3<f64>) -> Self {
        let (x, y, z) = (ecef[0], ecef[1], ecef[2]);
        let (lat, lon, h) = ecef2geodetic(x, y, z, Ellipsoid::WGS84);
        Self {
            ecef,
            geodetic: Vector3::new(lat, lon, h),
        }
    }
    /// Builds new [Position] from Geodetic coordinates
    /// - latitude [rad]
    /// - longitude [rad]
    /// - altitude above sea level [m]
    pub fn from_geo_rad(geodetic: Vector3<f64>) -> Self {
        let (lat, lon, alt) = (geodetic[0], geodetic[1], geodetic[2]);
        let (x, y, z) = geodetic2ecef(lat, lon, alt, Ellipsoid::WGS84);
        Self {
            geodetic,
            ecef: Vector3::new(x, y, z),
        }
    }
    /// Builds new [Position] from Geodetic coordinates
    /// - latitude [ddeg]
    /// - longitude [ddeg]
    /// - altitude above sea level [m]
    pub fn from_geo_ddeg(geodetic: Vector3<f64>) -> Self {
        Self::from_geo_rad(Vector3::new(
            geodetic[0].to_radians(),
            geodetic[1].to_radians(),
            geodetic[2].to_radians(),
        ))
    }
    /// Returns ECEF coordinates.
    pub fn ecef(&self) -> Vector3<f64> {
        self.ecef
    }
    /// Returns Geodetic coordinates
    /// - latitude [rad]
    /// - longitude [rad]
    /// - altitude above sea levl [m]
    pub fn geodetic(&self) -> Vector3<f64> {
        self.geodetic
    }
}
