use crate::{
    prelude::{Almanac, Duration, Epoch, Frame, Orbit, OrbitSource, SV},
    tests::data::{E01, E03, E05, E09, E13, E15, E24, E31},
};

use rstest::*;

use itertools::Itertools;

use std::{collections::HashMap, str::FromStr};

const MIN_ORBITS_PER_EPOCH: usize = 4 * 2;

#[fixture]
fn build_almanac() -> Almanac {
    use crate::tests::test_almanac;
    test_almanac()
}

#[fixture]
fn build_earth_frame() -> Frame {
    use crate::tests::test_earth_frame;
    test_earth_frame()
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Eq, Hash)]
pub struct OrbitsDataKey {
    pub sv: SV,
    pub epoch: Epoch,
}

pub struct OrbitsData {
    pub map: HashMap<OrbitsDataKey, Orbit>,
}

impl OrbitsData {
    pub fn new(frame: Frame) -> Self {
        let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

        let map = HashMap::from_iter(
            [
                (
                    OrbitsDataKey {
                        sv: E01,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(-11562.163582, 14053.114306, 23345.128269, t0_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E03,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(4577.136069, -22995.974895, 18062.640686, t0_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E05,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(16577.017768, -4619.539763, 24092.494804, t0_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E09,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(18846.610510, 16144.830741, 16159.863309, t0_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E13,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(-15921.765341, -5400.108297, 24360.804625, t0_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E15,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(-409.147663, -21456.140629, 20391.202816, t0_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E24,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(26947.918853, 9320.740084, 7908.580547, t0_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E31,
                        epoch: t0_gpst,
                    },
                    Orbit::from_position(11195.440434, 16391.022663, 21968.198345, t0_gpst, frame),
                ),
            ]
            .into_iter(),
        );

        Self { map }
    }
}

impl OrbitSource for OrbitsData {
    fn next_at(&self, epoch: Epoch, sv: SV, _: Frame) -> Option<Orbit> {
        let orbit = self
            .map
            .iter()
            .filter_map(|(k, v)| {
                // TODO : missing state interpolation !
                if k.sv == sv && (k.epoch - epoch).abs() < Duration::from_seconds(15.0 * 60.0) {
                    Some(v)
                } else {
                    None
                }
            })
            .reduce(|k, _| k)?;

        Some(*orbit)
    }
}

#[test]
fn verify_min_test_orbits_per_epoch() {
    let earth_frame = build_earth_frame();

    let data = OrbitsData::new(earth_frame);

    for epoch in data.map.keys().map(|k| k.epoch).unique() {
        let collected = data
            .map
            .iter()
            .filter(|(k, _)| k.epoch == epoch)
            .collect::<Vec<_>>();

        assert!(
            collected.len() >= MIN_ORBITS_PER_EPOCH,
            "not enough valid test orbits @ {}",
            epoch
        );
    }
}
