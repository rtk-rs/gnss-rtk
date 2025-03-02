use crate::{
    prelude::{Almanac, Constellation, Epoch, Frame, Orbit, OrbitSource, EARTH_J2000, SV},
    tests::{
        gps::{G01, G05},
        reference_orbit,
    },
};

use std::{collections::HashMap, str::FromStr};

use anise::astro::AzElRange;
use sp3::prelude::SP3;

/// Dummy structure that allows deploying and iterating the solver
/// infinitely, but cannot be used for calculations verification.
pub struct NullOrbits {}

impl OrbitSource for NullOrbits {
    fn next_at(&mut self, t: Epoch, _: SV, fr: Frame) -> Option<Orbit> {
        let (x_km, y_km, z_km) = (15600.0, 7540.0, 20140.0);
        Some(Orbit::from_position(x_km, y_km, z_km, t, fr))
    }
}

pub struct OrbitDataSet<'a> {
    sp3: SP3,
    almanac: &'a Almanac,
}

impl<'a> OrbitDataSet<'a> {
    /// Builds new test [OrbitDataSet] from [SP3] GPS data, to be used in the solving process.
    pub fn from_sp3(fp: &str, almanac: &'a Almanac) -> Self {
        let sp3 = SP3::from_gzip_file(fp).unwrap();
        Self {
            sp3,
            almanac: almanac,
        }
    }
}

impl<'a> OrbitSource for OrbitDataSet<'a> {
    fn next_at(&mut self, t: Epoch, sv: SV, fr: Frame) -> Option<Orbit> {
        let (x_km, y_km, z_km) = match self
            .sp3
            .satellites_position_km_iter()
            .filter_map(|(t_i, sv_i, (x_km, y_km, z_km))| {
                if t_i == t && sv_i == sv {
                    Some((x_km, y_km, z_km))
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
        {
            Some((x_km, y_km, z_km)) => Some((x_km, y_km, z_km)),
            None => self
                .sp3
                .satellite_position_lagrangian_11_interpolation(sv, t),
        }?;

        let orbit = Orbit::from_position(x_km, y_km, z_km, t, fr);

        if let Ok(azelrange) =
            self.almanac
                .azimuth_elevation_range_sez(orbit, reference_orbit(fr), None, None)
        {
            if azelrange.elevation_deg.is_sign_positive() {
                Some(orbit)
            } else {
                None
            }
        } else {
            None
        }
    }
}

#[test]
fn gps_validity() {
    let almanac = Almanac::until_2035().unwrap();
    let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

    let mut gps_orbits =
        OrbitDataSet::from_sp3("data/GRG0MGXFIN_20201770000_01D_15M_ORB.SP3.gz", &almanac);

    let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    assert!(
        gps_orbits.next_at(t0_gpst, G01, frame).is_none(),
        "not seen from station"
    );

    let tx_orbit = gps_orbits
        .next_at(t0_gpst, G05, frame)
        .expect("Missing T0/G05 data");

    let pos_vel = tx_orbit.to_cartesian_pos_vel();
    let pos_km = (pos_vel[0], pos_vel[1], pos_vel[2]);

    assert_eq!(pos_km, (20403.407951, -4547.528919, 16359.977231));
}
