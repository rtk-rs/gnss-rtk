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
    use crate::tests::almanac;
    almanac()
}

#[fixture]
fn build_earth_frame() -> Frame {
    use crate::tests::earth_frame;
    earth_frame()
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
        let t1_gpst = Epoch::from_str("2020-06-25T00:15:00 GPST").unwrap();
        let t2_gpst = Epoch::from_str("2020-06-25T00:30:00 GPST").unwrap();
        let t3_gpst = Epoch::from_str("2020-06-25T00:45:00 GPST").unwrap();
        let t4_gpst = Epoch::from_str("2020-06-25T01:00:00 GPST").unwrap();

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
                (
                    OrbitsDataKey {
                        sv: E01,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(-13618.625154, 13865.251337, 22325.739925, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E03,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(5249.050928, -21427.275276, 19733.53934, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E05,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(17451.377203, -2644.578813, 23770.958635, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E09,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(19437.621892, 17372.955840, 14048.13756, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E13,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(-14620.483100, -7198.531437, 24709.457156, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E15,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(549.559454, -22943.716677, 18698.434257, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E24,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(26132.488800, 9133.294721, 10452.358017, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E31,
                        epoch: t1_gpst,
                    },
                    Orbit::from_position(9177.338648, 16128.279322, 23070.547459, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E01,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(-15578.906571, 13828.422191, 21028.690065, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E03,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(6089.334963, -19783.358422, 21158.979160, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E05,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(18436.719493, -770.324223, 23154.020400, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E09,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(20017.686424, 18382.862213, 11761.922747, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E13,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(-13443.345950, -9101.204022, 24750.808535, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E15,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(1313.090113, -24357.643799, 16773.179621, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E24,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(25082.310095, 8993.926987, 12865.989009, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E31,
                        epoch: t2_gpst,
                    },
                    Orbit::from_position(7059.263844, 16006.140471, 23886.192153, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E01,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(-17408.137167, 13927.983030, 19470.096085, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E03,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(7098.698927, -18099.833762, 22321.267348, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E05,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(19509.930356, 974.904114, 22249.348845, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E09,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(20553.093095, 19169.983128, 9329.608871, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E13,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(
                        -12411.471925,
                        -11074.923521,
                        24484.357047,
                        t1_gpst,
                        frame,
                    ),
                ),
                (
                    OrbitsDataKey {
                        sv: E15,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(1887.325160, -25661.135185, 14639.374285, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E24,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(23814.294414, 8933.516592, 15119.417055, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E31,
                        epoch: t3_gpst,
                    },
                    Orbit::from_position(4877.703417, 16036.340787, 24404.993554, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E01,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(-19074.795786, 14143.814957, 17669.329791, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E03,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(8271.797755, -16412.322261, 23205.987823, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E05,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(20643.741422, 2567.147607, 21068.184349, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E09,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(21010.298836, 19735.979210, 6781.401622, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E13,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(
                        -11540.233821,
                        -13083.511580,
                        23913.427261,
                        t1_gpst,
                        frame,
                    ),
                ),
                (
                    OrbitsDataKey {
                        sv: E15,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(2284.385592, -26819.485051, 12323.546523, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E24,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(22350.983090, 8979.707681, 17184.581953, t1_gpst, frame),
                ),
                (
                    OrbitsDataKey {
                        sv: E31,
                        epoch: t4_gpst,
                    },
                    Orbit::from_position(2670.799462, 16224.467677, 24620.497432, t1_gpst, frame),
                ),
            ]
            .into_iter(),
        );

        Self { map }
    }
}

impl OrbitSource for OrbitsData {
    fn state_at(&self, epoch: Epoch, sv: SV, _: Frame) -> Option<Orbit> {
        let dt_15min = Duration::from_hours(0.25);

        let orbit = self
            .map
            .iter()
            .filter_map(|(k, v)| {
                if k.sv == sv && (k.epoch - epoch).abs() < dt_15min {
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
