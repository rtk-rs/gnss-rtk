pub struct Url;
pub struct Constants;

use anise::almanac::metaload::MetaFile;

impl Url {
    pub fn nyx_anise_de440s_bsp() -> MetaFile {
        MetaFile {
            crc32: Some(1921414410),
            uri: String::from("http://public-data.nyxspace.com/anise/de440s.bsp"),
        }
    }
    pub fn nyx_anise_pck11_pca() -> MetaFile {
        MetaFile {
            crc32: Some(0x8213b6e9),
            uri: String::from("http://public-data.nyxspace.com/anise/v0.4/pck11.pca"),
        }
    }
    pub fn jpl_latest_high_prec_bpc() -> MetaFile {
        MetaFile {
            crc32: None,
            uri:
                "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc"
                    .to_string(),
        }
    }
}

impl Constants {
    /// Earth angular velocity, in WGS84 frame rad/s
    pub const EARTH_ANGULAR_VEL_RAD: f64 = 7.2921151467E-5;
    /// Earth gravitational constant (m^3 s-2)
    pub const EARTH_GRAVITATION: f64 = 3986004.418 * 10.0E8;
    // /// Sun gravitational constant (m^3 s-2)
    // pub const SUN_GRAVITATION: f64 = 1.32712440018 * 10.0E14;
    // /// Earth's moon gravitational constant (m^3 s-2)
    // pub const MOON_GRAVITATION: f64 = 4.9028695 * 10.0E6;
    /// Earth equatorial radis [km]
    pub const EARTH_EQUATORIAL_RADIUS_KM: f64 = 6378.1366;
    /// WGS84 Earth Frame Ellipsoid semi-major axis
    pub const EARTH_SEMI_MAJOR_AXIS_WGS84: f64 = 6378137.0_f64;
    // /// Love degree^2 term
    // pub const LOVE_DEGREE2: f64 = 0.6078;
    // /// Shida degree^2 term
    // pub const SHIDA_DEGREE2: f64 = 0.0847;
}
