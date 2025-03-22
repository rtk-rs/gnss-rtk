use crate::{
    prelude::{Almanac, Epoch, Frame, Orbit, OrbitSource, EARTH_J2000, SV},
    tests::{
        epochs::EPOCHS_DESCRIPTOR,
        gps::{G02, G05, G07, G08, G09, G13, G15},
        init_logger, REFERENCE_COORDS_ECEF_M,
    },
};

use hifitime::Unit;

use log::warn;
use std::str::FromStr;

/// Dummy structure that allows deploying and iterating the solver
/// infinitely, but cannot be used for calculations verification.
pub struct NullOrbits {}

impl OrbitSource for NullOrbits {
    fn next_at(&mut self, t: Epoch, _: SV, fr: Frame) -> Option<Orbit> {
        let (x_km, y_km, z_km) = (15600.0, 7540.0, 20140.0);
        Some(Orbit::from_position(x_km, y_km, z_km, t, fr))
    }
}

pub struct OrbitData {
    pub t: Epoch,
    pub sv: SV,
    pub pos_km: (f64, f64, f64),
}

pub struct GpsOrbits {
    buffer: [OrbitData; 7],
}

impl GpsOrbits {
    pub fn build() -> Self {
        Self {
            buffer: [
                OrbitData {
                    t: Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                    sv: G02,
                    pos_km: (21815.313784, -13786.051880, -5530.292407),
                },
                OrbitData {
                    t: Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                    sv: G05,
                    pos_km: (20403.407951, -4547.528919, 16359.977231),
                },
                OrbitData {
                    t: Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                    sv: G07,
                    pos_km: (7216.464981, 13874.448927, 21747.416323),
                },
                OrbitData {
                    t: Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                    sv: G08,
                    pos_km: (-7492.550168, 20537.976443, 14911.094048),
                },
                OrbitData {
                    t: Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                    sv: G09,
                    pos_km: (8106.486739, 24398.526847, 6586.681092),
                },
                OrbitData {
                    t: Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                    sv: G13,
                    pos_km: (13008.717968, -13353.750095, 18762.067067),
                },
                OrbitData {
                    t: Epoch::from_str(EPOCHS_DESCRIPTOR[0]).unwrap(),
                    sv: G15,
                    pos_km: (5550.690261, -21648.534281, 13744.298178),
                },
            ],
        }
    }
}

impl OrbitSource for GpsOrbits {
    fn next_at(&mut self, t: Epoch, sv: SV, frame: Frame) -> Option<Orbit> {
        let point = self
            .buffer
            .iter()
            .filter(|pt| pt.sv == sv && (pt.t - t).abs() < 1.0 * Unit::Nanosecond)
            .reduce(|k, _| k)?;

        Some(Orbit::from_position(
            point.pos_km.0,
            point.pos_km.1,
            point.pos_km.2,
            t,
            frame,
        ))
    }
}

#[test]
fn validity() {
    init_logger();

    let almanac = Almanac::until_2035().unwrap();
    let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

    let mut gps_orbits = GpsOrbits::build();

    for t in EPOCHS_DESCRIPTOR.iter() {
        let t_gpst = Epoch::from_str(t).unwrap();

        let rx_orbit = Orbit::from_position(
            REFERENCE_COORDS_ECEF_M.0 / 1.0E3,
            REFERENCE_COORDS_ECEF_M.1 / 1.0E3,
            REFERENCE_COORDS_ECEF_M.2 / 1.0E3,
            t_gpst,
            frame,
        );

        match *t {
            "2020-06-25T00:00:00 GPST" => {
                for sv in [G05] {
                    let orbit = gps_orbits
                        .next_at(t_gpst, sv, frame)
                        .expect(&format!("undetermined orbital state: {}({})", t_gpst, sv));

                    let azelrange = almanac
                        .azimuth_elevation_range_sez(orbit, rx_orbit, None, None)
                        .unwrap_or_else(|e| {
                            panic!(
                                "Physical error, invalid orbital state? {}({}): {}",
                                t_gpst, sv, e
                            )
                        });

                    assert!(
                        azelrange.elevation_deg.is_sign_positive(),
                        "not seen by station: invalid data point"
                    );
                }
            },
            t => warn!("{}: orbits not verified", t),
        }
    }
}
