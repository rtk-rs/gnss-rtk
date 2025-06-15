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
                    Observation::pseudo_range(Carrier::L1, 27616185.992, None)
                        .with_ambiguous_phase_range_m(145124050.106),
                    Observation::pseudo_range(Carrier::E5b, 27616184.819, None)
                        .with_ambiguous_phase_range_m(108371872.760),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.707516),
            )),
            Candidate::new(
                E03,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27055946.391, None)
                        .with_ambiguous_phase_range_m(142179967.778),
                    Observation::pseudo_range(Carrier::E5b, 27055945.532, None)
                        .with_ambiguous_phase_range_m(106173364.686),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.499771),
            )),
            Candidate::new(
                E05,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23730317.923, None)
                        .with_ambiguous_phase_range_m(124703702.220),
                    Observation::pseudo_range(Carrier::E5b, 23730316.788, None)
                        .with_ambiguous_phase_range_m(93122915.921),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.776159),
            )),
            Candidate::new(
                E09,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22756243.562, None)
                        .with_ambiguous_phase_range_m(119584910.611),
                    Observation::pseudo_range(Carrier::E5b, 22756242.295, None)
                        .with_ambiguous_phase_range_m(89300442.791),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.693914),
            )),
            Candidate::new(
                E13,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27978718.264, None)
                        .with_ambiguous_phase_range_m(147029158.327),
                    Observation::pseudo_range(Carrier::E5b, 27978716.335, None)
                        .with_ambiguous_phase_range_m(109794501.184),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847044),
            )),
            Candidate::new(
                E15,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26889610.666, None)
                        .with_ambiguous_phase_range_m(141305866.869),
                    Observation::pseudo_range(Carrier::E5b, 26889610.961, None)
                        .with_ambiguous_phase_range_m(105520617.365),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.331400),
            )),
            Candidate::new(
                E24,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23636670.553, None)
                        .with_ambiguous_phase_range_m(124211548.364),
                    Observation::pseudo_range(Carrier::E5b, 23636680.074, None)
                        .with_ambiguous_phase_range_m(92755422.744),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-512.649778),
            )),
            Candidate::new(
                E31,
                t0_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24566160.392, None)
                        .with_ambiguous_phase_range_m(129096072.832),
                    Observation::pseudo_range(Carrier::E5b, 24566159.582, None)
                        .with_ambiguous_phase_range_m(96402914.192),
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
