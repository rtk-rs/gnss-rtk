use crate::prelude::Vector3;
use map_3d::{ecef2geodetic, geodetic2ecef, Ellipsoid};

#[derive(Default, Debug, Clone, PartialEq)]
pub struct AprioriPosition {
    pub ecef: Vector3<f64>,
    pub geodetic: Vector3<f64>,
}

impl AprioriPosition {
    /// Builds Self from ECEF position [m]
    pub fn from_ecef(ecef: Vector3<f64>) -> Self {
        let (x, y, z) = (ecef[0], ecef[1], ecef[2]);
        let (lat, lon, h) = ecef2geodetic(x, y, z, Ellipsoid::WGS84);
        Self {
            ecef,
            geodetic: Vector3::new(lat, lon, h),
        }
    }
    /// Builds Self from Geodetic coordinates:
    /// latitude [ddeg], longitude [ddeg] and altitude above sea [m].
    pub fn from_geo(geodetic: Vector3<f64>) -> Self {
        let (lat, lon, alt) = (geodetic[0], geodetic[1], geodetic[2]);
        let (x, y, z) = geodetic2ecef(lat, lon, alt, Ellipsoid::WGS84);
        Self {
            geodetic,
            ecef: Vector3::new(x, y, z),
        }
    }
}
