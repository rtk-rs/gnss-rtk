use std::str::FromStr;

use crate::{
    prelude::{Candidate, Carrier, ClockCorrection, Duration, Epoch, Observation},
    tests::data::{E01, E03, E05, E09, E13, E15, E24, E31},
};

pub struct CandidatesBuilder {}

impl CandidatesBuilder {
    pub fn build_at(t: Epoch) -> Vec<Candidate> {
        Self::build_data()
            .into_iter()
            .filter(|cd| cd.t == t)
            .collect()
    }

    pub fn build_data() -> Vec<Candidate> {
        let t0_gpst = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

        let mut dataset = Vec::<Candidate>::with_capacity(16);

        for cd in [
            Candidate::new(
                E01,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27506424.743, None),
                    Observation::pseudo_range(Carrier::E5b, 27506425.902, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.707516),
            )),
            Candidate::new(
                E03,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26952639.751, None),
                    Observation::pseudo_range(Carrier::E5b, 26952641.150, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.499771),
            )),
            Candidate::new(
                E05,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23595077.027, None),
                    Observation::pseudo_range(Carrier::E5b, 23595078.180, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.776159),
            )),
            Candidate::new(
                E09,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22579938.261, None),
                    Observation::pseudo_range(Carrier::E5b, 22579939.154, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.693914),
            )),
            Candidate::new(
                E13,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27896986.615, None),
                    Observation::pseudo_range(Carrier::E5b, 27896987.493, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847044),
            )),
            Candidate::new(
                E15,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26795887.300, None),
                    Observation::pseudo_range(Carrier::E5b, 26795890.201, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.331400),
            )),
            Candidate::new(
                E24,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23442817.742, None),
                    Observation::pseudo_range(Carrier::E5b, 23442829.814, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-512.649778),
            )),
            Candidate::new(
                E31,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24412335.986, None),
                    Observation::pseudo_range(Carrier::E5b, 24412337.730, None),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.987972),
            )),
        ] {
            dataset.push(cd);
        }

        dataset
    }
}
