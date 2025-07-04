use crate::{
    prelude::{Almanac, Epoch, Error, Vector3},
    constants::{
        EARTH_EQUATORIAL_RADIUS_M,
        LOVE_DEGREE2,
        SHIDA_DEGREE2,
        EARTH_GRAVITATION_MU_M3_S2,
        SUN_GRAVITATION_MU_M3_S2,
        MOON_GRAVITATION_MU_M3_S2,
    },
};

use anise::{
    constants::{
        frames::{EARTH_J2000, MOON_J2000, SUN_J2000},
    },
};

/// Calculates local crust displacement [Vector3] (all axis),
/// at specified position position expressed in meters (ECEF).
///
/// ## Input
/// - epoch: [Epoch] of displacement calculation
/// - almanac: [Almanac]
/// - position_ecef_m: position as [Vector3] ECEF (meters)
/// - latitude_rad: latitude (in radians) of location
/// at which the displacement is to be calculated.
pub fn solid_body_tidal_displacement(
    epoch: Epoch,
    almanac: &Almanac,
    position_ecef_m: Vector3<f64>,
    latitude_rad: f64,
) -> Result<Vector3<f64>, Error> {

     let r_earth_moon_unit = almanac
        .unit_vector(
            EARTH_J2000,
            MOON_J2000,
            epoch,
            None,
        )
        .map_err(|e| Error::Almanac(e))?
        * 1.0E3;
    
     let r_earth_sun_unit = almanac
        .unit_vector(
            SUN_J2000,
            EARTH_J2000,
            epoch,
            None,
        )
        .map_err(|e| Error::Almanac(e))?
        * 1.0E3;
    
    let position_ecef_mag = position_ecef_m.magnitude();
    let position_ecef_unit = position_ecef_m / position_ecef_mag;

    let r_earth_moon_unit_mag = r_earth_moon_unit.magnitude();
    let r_earth_sun_unit_mag = r_earth_moon_unit.magnitude();

    let moon_num = MOON_GRAVITATION_MU_M3_S2
        * EARTH_EQUATORIAL_RADIUS_M.powi(4);
    
    let sun_num = SUN_GRAVITATION_MU_M3_S2
        * EARTH_EQUATORIAL_RADIUS_M.powi(4);
   
    let moon_denom = EARTH_GRAVITATION_MU_M3_S2 *
        r_earth_moon_unit_mag.powi(3);

    let sun_denom = EARTH_GRAVITATION_MU_M3_S2
        * r_earth_sun_unit_mag.powi(3);
    
    let r_earth_moon_unit_dot = r_earth_moon_unit
        .dot(&position_ecef_unit);

    let r_earth_sun_unit_dot = r_earth_sun_unit
        .dot(&position_ecef_unit);

    let h2 = LOVE_DEGREE2
        - 0.0006
        * ((3.0 * latitude_rad.sin().powi(2) - 1.0)  /2.0);

    let l2 = SHIDA_DEGREE2
        + 0.0002 
        * ((3.0 * latitude_rad.sin().powi(2) - 1.0) / 2.0);

    let moon_lhs = h2
        * position_ecef_unit
        * (3.0 / 2.0 * r_earth_moon_unit_dot.powi(2) - 0.5);

    let sun_lhs = h2
        * position_ecef_unit
        * (3.0 / 2.0 * r_earth_sun_unit_dot.powi(2) - 0.5);

    let moon_rhs = 3.0 * l2
        * r_earth_moon_unit_dot
        * (r_earth_moon_unit - r_earth_moon_unit_dot * position_ecef_unit);
    
    let sun_rhs = 3.0 * l2
        * r_earth_sun_unit_dot
        * (r_earth_sun_unit - r_earth_sun_unit_dot * position_ecef_unit);
    
    let mut dr = moon_num / moon_denom * (moon_lhs + moon_rhs);
    
    // dr += sun_num / sun_denom * (sun_lhs + sun_rhs);

    Ok(dr)
}

#[cfg(test)]
mod test {
    use super::*;

    use crate::{
        tides::solid_body_tidal_displacement,
        prelude::{
            Vector3,
            Almanac,
            Epoch,
            Duration,
        },
        tests::init_logger,
    };

    use hifitime::{
        TimeSeries,
        Unit,
    };

    use log::info;

    use rstest::*;

    #[fixture]
    fn build_almanac() -> Almanac {
        use crate::tests::almanac;
        almanac()
    }

    #[test]
    fn solid_crust_tidal_deformation() {
        init_logger();

        let position_ecef_m = Vector3::<f64>::new(
            4696989.6880,
            723994.1970,
            4239678.3040,
        );

        let latitude_rad = 48.855338_f64.to_radians();
    
        let almanac = build_almanac();

        let t0 = Epoch::from_gregorian_utc_at_midnight(2000, 1, 1);

        let t1 = t0 + 24.0 * Unit::Day;
        let dt = Duration::from_seconds(30.0 * 60.0);

        for epoch in TimeSeries::inclusive(t0, t0, dt).into_iter() { 
            
            let dr = solid_body_tidal_displacement(
                epoch,
                &almanac,
                position_ecef_m,
                latitude_rad,
            ).unwrap_or_else(|e| {
                panic!("failed to calculate tidal displacement");
            });

            info!("tidal displacement dr={}m", dr);
        }
    }
}
