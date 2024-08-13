use crate::{
    constants::Constants,
    prelude::{Almanac, Epoch, Error, Frame, Vector3},
};
use anise::{
    constants::frames::{EARTH_J2000, MOON_J2000, SUN_J2000},
    prelude::Orbit,
};

/// Calculates local site displacement vector (crust deformation)
/// for given site coordinates [ECEF m], located on [Frame] body
/// due to moon an star gravitational interaction.
pub fn solid_tides(
    t: Epoch,
    almanac: &Almanac,
    site_ecef_m: Vector3<f64>,
) -> Result<Vector3<f64>, Error> {
    let (site_ecef_x_m, site_ecef_y_m, site_ecef_z_m) =
        (site_ecef_m[0], site_ecef_m[1], site_ecef_m[2]);

    let r_earth_m = Constants::EARTH_EQUATORIAL_RADIUS_KM * 1.0E3;
    let (h2, l2) = (Constants::LOVE_DEGREE2, Constants::SHIDA_DEGREE2);
    let (g_earth, g_moon, g_sun) = (
        Constants::EARTH_GRAVITATION,
        Constants::EARTH_GRAVITATION,
        Constants::EARTH_GRAVITATION,
    );

    let earth_sun = almanac
        .transform(EARTH_J2000, SUN_J2000, t, None)
        .map_err(Error::Almanac)?;

    let earth_moon = almanac
        .transform(EARTH_J2000, MOON_J2000, t, None)
        .map_err(Error::Almanac)?;

    let earth_moon_m = Vector3::new(
        earth_moon.radius_km.x * 1.0E3,
        earth_moon.radius_km.y * 1.0E3,
        earth_moon.radius_km.z * 1.0E3,
    );

    let earth_sun_m = Vector3::new(
        earth_sun.radius_km.x * 1.0E3,
        earth_sun.radius_km.y * 1.0E3,
        earth_sun.radius_km.z * 1.0E3,
    );

    let station_orb = Orbit::from_position(
        site_ecef_x_m / 1.0E3,
        site_ecef_y_m / 1.0E3,
        site_ecef_z_m / 1.0E3,
        t,
        EARTH_J2000,
    );

    let earth_sun_mag = earth_sun_m.magnitude();
    let earth_moon_mag = earth_sun_m.magnitude();

    let site_r = Vector3::new(
        station_orb.radius_km.x * 1.0E3,
        station_orb.radius_km.y * 1.0E3,
        station_orb.radius_km.z * 1.0E3,
    );

    let site_r_mag = site_r.magnitude();

    // first term is body<->moon interaction
    let body_moon_const = g_moon * r_earth_m.powi(4) / g_earth / earth_moon_mag.powi(3);
    let rj_r = earth_moon_m.dot(&site_r);
    let body_moon = h2 * site_r * (3.0 / 2.0 * rj_r.powi(2) - 0.5);
    let body_moon = body_moon + 3.0 * l2 * rj_r * (earth_moon_m - rj_r * earth_moon_m);
    let body_moon = body_moon_const * body_moon;

    // second term is body<->star interaction
    let body_sun_const = g_sun * r_earth_m.powi(4) / g_earth / earth_sun_mag.powi(3);
    let rj_r = earth_sun_m.dot(&site_r);
    let body_sun = h2 * site_r * (3.0 / 2.0 * rj_r.powi(2) - 0.5);
    let body_sun = body_sun + 3.0 * l2 * rj_r * (earth_sun_m - rj_r * earth_sun_m);
    let body_sun = body_sun_const * body_sun;

    Ok(body_moon + body_sun)
}

#[cfg(feature)]
mod test {
    use super::*;
    #[test]
    fn test_solid_tides() {
        // solid tidal effect is said to be between [-2mm;+2mm]
        for t in [] {}
    }
}
