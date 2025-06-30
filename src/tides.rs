use crate::{
    constants::Constants,
    prelude::{Almanac, Epoch, Error, Frame, Vector3},
};
use anise::{
    math::cartesian::CartesianState,
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
        0.01230002,
        332946.0,
        //Constants::MOON_GRAVITATION,
        //Constants::SUN_GRAVITATION,
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

    let site_orbit = Orbit::from_position(
        site_ecef_x_m / 1.0E3,
        site_ecef_y_m / 1.0E3,
        site_ecef_z_m / 1.0E3,
        t,
        EARTH_J2000,
    );

    //let site_latitude = site_orbit.latitude_deg()
    //    .map_err(|e| Error::Physics(e))?
    //    .to_radians();

    let earth_sun_mag = earth_sun_m.magnitude();
    let earth_moon_mag = earth_sun_m.magnitude();

    let site_r = Vector3::new(
        site_orbit.radius_km.x * 1.0E3,
        site_orbit.radius_km.y * 1.0E3,
        site_orbit.radius_km.z * 1.0E3,
    );

    let site_cartesian = CartesianState::from_cartesian_pos_vel(
        site_orbit.to_cartesian_pos_vel(),
        t,
        EARTH_J2000,
    );

    let site_r_mag = site_r.magnitude();

    // first term is body<->moon interaction
    let body_moon_const = g_moon * r_earth_m.powi(4) / g_earth / earth_moon_mag.powi(3);
    let rj_r = earth_moon_m.dot(&site_r);
    let body_moon = h2 * site_r * (3.0 / 2.0 * rj_r.powi(2) - 0.5);
    let body_moon = body_moon + 3.0 * l2 * rj_r * (earth_moon_m - rj_r * site_r);
    let body_moon = body_moon_const * body_moon;

    // second term is body<->star interaction
    let body_sun_const = g_sun * r_earth_m.powi(4) / g_earth / earth_sun_mag.powi(3);
    let rj_r = earth_sun_m.dot(&site_r);
    let body_sun = h2 * site_r * (3.0 / 2.0 * rj_r.powi(2) - 0.5);
    let body_sun = body_sun + 3.0 * l2 * rj_r * (earth_sun_m - rj_r * site_r);
    let body_sun = body_sun_const * body_sun;

    // only for three bodies (one star, one moon)
    // more complex systems require involve more terms
    let sum = body_moon + body_sun;
    Ok(sum)
}

#[cfg(test)]
mod test {
    use super::*;
    use hifitime::{TimeSeries, Duration, Unit};
    
    #[test]
    fn earth_france_solid_tides() {
        // solid tidal effect is said to be between [-2mm;+2mm]
        let max_absolute_mm = 2.0;
        let france_ecef_m = Vector3::<f64>::new(
            4696989.6880,
            723994.1970,
            4239678.3040,
        );
        let almanac = Almanac::until_2035().unwrap();
        let t0 = Epoch::from_gregorian_utc_at_midnight(2000, 1, 1);
        let t1 = t0 + 24.0 * Unit::Day;
        let dt = Duration::from_seconds(30.0 * 60.0);
        for t in TimeSeries::inclusive(t0, t1, dt).into_iter() { 
            let (dr_x, dr_y, dr_z) = solid_tides(
                t,
                &almanac,
                france_ecef_m, 
            ) / 1.0E3; // mm
            assert!(dr_x_mm.abs() < max_absolute_mm);
            assert!(dr_y_mm.abs() < max_absolute_mm);
            assert!(dr_z_mm.abs() < max_absolute_mm);
            println!("solid tide: {:?}", dr);
        }
    }
    
    #[test]
    fn earth_north_pole_tides() {
        let almanac = Almanac::until_2035().unwrap();
        // solid tidal effect is larger @ poles than equatorial latitudes
    }

    #[test]
    fn earth_south_pole_tides() {
        let almanac = Almanac::until_2035().unwrap();
        // solid tidal effect is larger @ poles than equatorial latitudes
    }
}
