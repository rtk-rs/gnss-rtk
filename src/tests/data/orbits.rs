use crate::{
    prelude::{Almanac, Constellation, Epoch, Frame, Orbit, OrbitSource, EARTH_J2000, SV},
    tests::{
        gps::{G01, G05, G13},
        reference_orbit,
    },
};

use std::{collections::HashMap, str::FromStr};

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
    buffer: HashMap<(Epoch, SV), Orbit>,
    iter: Box<dyn Iterator<Item = (Epoch, SV, Orbit)> + 'a>,
}

impl<'a> OrbitDataSet<'a> {
    // /// Builds new test [OrbitDataSet] from [SP3] data, to be used in the solving process.
    // pub fn from_sp3(sp3: &'a SP3, almanac: &'a Almanac, frame: Frame) -> Self {
    //     let rx = reference_orbit(frame);
    //     let iter = sp3.satellites_orbit_iter(frame);
    //     Self {
    //         buffer: HashMap::new(),
    //         iter: Box::new(
    //             iter.filter_map(move |(t, sv, tx_orbit)| {
    //                 if let Ok(azelrange) = almanac.azimuth_elevation_range_sez(tx_orbit, rx, None, None) {
    //                     if azelrange.elevation_deg.is_sign_positive() {
    //                         Some((t, sv, tx_orbit))
    //                     } else {
    //                         None
    //                     }
    //                 } else {
    //                     None
    //                 }
    //             })
    //         ),
    //     }
    // }

    /// Builds new test [OrbitDataSet] from [SP3] GPS data, to be used in the solving process.
    pub fn from_sp3_gps(sp3: &'a SP3, almanac: &'a Almanac, frame: Frame) -> Self {
        let rx = reference_orbit(frame);
        let iter = sp3.satellites_orbit_iter(frame);
        Self {
            buffer: HashMap::new(),
            iter: Box::new(iter.filter_map(move |(t, sv, tx_orbit)| {
                if sv.constellation == Constellation::GPS {
                    if let Ok(azelrange) =
                        almanac.azimuth_elevation_range_sez(tx_orbit, rx, None, None)
                    {
                        if azelrange.elevation_deg.is_sign_positive() {
                            Some((t, sv, tx_orbit))
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                } else {
                    None
                }
            })),
        }
    }
}

impl<'a> OrbitSource for OrbitDataSet<'a> {
    fn next_at(&mut self, t: Epoch, sv: SV, _: Frame) -> Option<Orbit> {
        // discard past data points
        self.buffer.retain(|(t_i, _), _| *t_i >= t);

        // Return if already buffered
        for ((t_i, sv_i), orbit_i) in self.buffer.iter() {
            if *sv_i == sv && *t_i == t {
                return Some(*orbit_i);
            }
        }

        // consume and store until past epoch
        loop {
            if let Some((t_i, sv_i, orbit_i)) = self.iter.next() {
                self.buffer.insert((t_i, sv_i), orbit_i);

                if t_i == t && sv_i == sv {
                    return Some(orbit_i);
                }
            } else {
                return None;
            }
        }
    }
}

#[test]
fn gps_validity() {
    let almanac = Almanac::until_2035().unwrap();
    let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

    let sp3 = SP3::from_gzip_file("data/GRG0MGXFIN_20201770000_01D_15M_ORB.SP3.gz")
        .unwrap_or_else(|e| panic!("Failed to load test data: {}", e));

    let mut gps_orbits = OrbitDataSet::from_sp3_gps(&sp3, &almanac, frame);

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
