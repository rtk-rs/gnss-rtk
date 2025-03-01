use std::str::FromStr;

use crate::{
    prelude::{Almanac, Epoch, Frame, Orbit, OrbitSource, EARTH_J2000, SV},
    tests::reference_orbit,
};

/// Dummy structure that allows deploying and iterating the solver
/// infinitely, but cannot be used for calculations verification.
pub struct SinglestaticSVOrbits {}

impl OrbitSource for SinglestaticSVOrbits {
    fn next_at(&mut self, t: Epoch, _: SV, fr: Frame) -> Option<Orbit> {
        let (x_km, y_km, z_km) = (15600.0, 7540.0, 20140.0);
        Some(Orbit::from_position(x_km, y_km, z_km, t, fr))
    }
}

pub type NullOrbits = SinglestaticSVOrbits;

/// A few orbital states that we can use to iterate & verify the solver
/// using GPS Constellation and GPST
pub struct GPSOrbits {}

impl GPSOrbits {
    const DESCRIPTOR: [(&str, &str, f64, f64, f64); 7] = [
        (
            "2020-06-25T00:00:00 GPST",
            "G02",
            21815.313784,
            -13786.051880,
            -5530.292407, //   -477.325536
        ),
        (
            "2020-06-25T00:00:00 GPST",
            "G05",
            20403.407951,
            -4547.528919,
            16359.977231, //    -15.320222
        ),
        (
            "2020-06-25T00:00:00 GPST",
            "G07",
            7216.464981,
            13874.448927,
            21747.416323, //   -312.212568
        ),
        (
            "2020-06-25T00:00:00 GPST",
            "G08",
            -7492.550168,
            20537.976443,
            14911.094048, //    -38.703947
        ),
        (
            "2020-06-25T00:00:00 GPST",
            "G09",
            8106.486739,
            24398.526847,
            6586.681092, //   -242.279193
        ),
        (
            "2020-06-25T00:00:00 GPST",
            "G13",
            13008.717968,
            -13353.750095,
            18762.067067, //     21.151577
        ),
        (
            "2020-06-25T00:00:00 GPST",
            "G15",
            5550.690261,
            -21648.534281,
            13744.298178, //   -221.978679
        ),
    ];

    fn sv_orbit_by_index(index: usize, frame: Frame) -> (SV, Orbit) {
        let (t_str, sv_str, x_km, y_km, z_km) = Self::DESCRIPTOR[index];
        let t = Epoch::from_str(t_str).unwrap();
        let sv = SV::from_str(sv_str).unwrap();
        let orbit = Orbit::from_position(x_km, y_km, z_km, t, frame);
        (sv, orbit)
    }

    pub fn find_orbit(t: Epoch, sv: SV, frame: Frame) -> Option<Orbit> {
        for i in 0..Self::DESCRIPTOR.len() {
            let (sv_i, orbit_i) = Self::sv_orbit_by_index(i, frame);
            if sv_i == sv && orbit_i.epoch == t {
                return Some(orbit_i);
            }
        }
        None
    }

    // pub fn collect(fr: Frame) -> Vec<(SV, Orbit)> {
    //     Self::DESCRIPTOR.iter().enumerate().map(|(i, _)| {
    //         Self::sv_orbit_by_index(i, fr)
    //     }).collect()
    // }

    pub fn collect_epoch(t: &str, fr: Frame) -> Vec<(SV, Orbit)> {
        Self::DESCRIPTOR
            .iter()
            .enumerate()
            .filter_map(|(i, (t_str, _, _, _, _))| {
                if t_str.eq(&t) {
                    Some(Self::sv_orbit_by_index(i, fr))
                } else {
                    None
                }
            })
            .collect()
    }
}

impl OrbitSource for GPSOrbits {
    fn next_at(&mut self, t: Epoch, sv: SV, fr: Frame) -> Option<Orbit> {
        Self::find_orbit(t, sv, fr)
    }
}

#[test]
fn gps_orbits_validity() {
    let al = Almanac::until_2035().unwrap();
    let frame = al.frame_from_uid(EARTH_J2000).unwrap();
    for i in 0..GPSOrbits::DESCRIPTOR.len() {
        let (sv, orbit) = GPSOrbits::sv_orbit_by_index(i, frame);

        let _azelrange = al
            .azimuth_elevation_range_sez(orbit, reference_orbit(frame), None, None)
            .unwrap_or_else(|e| {
                panic!(
                    "az_el_range failure: {} | invalid GPS Orbit [sv={} i={}]",
                    e, sv, i
                )
            });

        // println!(
        //     "sv={} i={} azim={}° elev={}°",
        //     sv, i, azelrange.azimuth_deg, azelrange.elevation_deg
        // );
    }
}
