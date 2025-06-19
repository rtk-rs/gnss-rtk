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
        let t1_gpst = Epoch::from_str("2020-06-25T00:15:00 GPST").unwrap();
        let t2_gpst = Epoch::from_str("2020-06-25T00:30:00 GPST").unwrap();
        let t3_gpst = Epoch::from_str("2020-06-25T00:45:00 GPST").unwrap();
        let t4_gpst = Epoch::from_str("2020-06-25T01:00:00 GPST").unwrap();

        [
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
            Candidate::new(
                E01,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27631168.610, None)
                        .with_ambiguous_phase_range_m(145202784.624),
                    Observation::pseudo_range(Carrier::E5b, 27631166.786, None)
                        .with_ambiguous_phase_range_m(108430668.004),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.714669),
            )),
            // OK
            Candidate::new(
                E03,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27040742.707, None)
                        .with_ambiguous_phase_range_m(142100071.288),
                    Observation::pseudo_range(Carrier::E5b, 27040742.124, None)
                        .with_ambiguous_phase_range_m(106113701.732),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.503498),
            )),
            // OK
            Candidate::new(
                E05,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23725981.640, None)
                        .with_ambiguous_phase_range_m(124680915.120),
                    Observation::pseudo_range(Carrier::E5b, 23725980.604, None)
                        .with_ambiguous_phase_range_m(93105899.595),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.773276),
            )),
            // OK
            Candidate::new(
                E09,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22766834.452, None)
                        .with_ambiguous_phase_range_m(119640566.190),
                    Observation::pseudo_range(Carrier::E5b, 22766833.144, None)
                        .with_ambiguous_phase_range_m(89342003.768),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.682886),
            )),
            // OK
            Candidate::new(
                E13,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27970957.396, None)
                        .with_ambiguous_phase_range_m(146988374.919),
                    Observation::pseudo_range(Carrier::E5b, 22766833.144, None)
                        .with_ambiguous_phase_range_m(109764046.039),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847317),
            )),
            // OK
            Candidate::new(
                E15,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26896117.363, None)
                        .with_ambiguous_phase_range_m(141340060.771),
                    Observation::pseudo_range(Carrier::E5b, 26896117.647, None)
                        .with_ambiguous_phase_range_m(105546151.754),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.330144),
            )),
            // OK
            Candidate::new(
                E24,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23622060.696, None)
                        .with_ambiguous_phase_range_m(124134773.341),
                    Observation::pseudo_range(Carrier::E5b, 23622070.342, None)
                        .with_ambiguous_phase_range_m(92698090.750),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5385.017319),
            )),
            // OK
            Candidate::new(
                E31,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24567140.313, None)
                        .with_ambiguous_phase_range_m(129101222.589),
                    Observation::pseudo_range(Carrier::E5b, 24567139.562, None)
                        .with_ambiguous_phase_range_m(96406759.805),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988096),
            )),
            // OK
            Candidate::new(
                E01,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27646192.922, None)
                        .with_ambiguous_phase_range_m(145281739.119),
                    Observation::pseudo_range(Carrier::E5b, 27646192.331, None)
                        .with_ambiguous_phase_range_m(108489627.494),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.721810),
            )),
            // OK
            Candidate::new(
                E03,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27025553.361, None)
                        .with_ambiguous_phase_range_m(142020249.814),
                    Observation::pseudo_range(Carrier::E5b, 27025552.715, None)
                        .with_ambiguous_phase_range_m(106054094.834),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.507266),
            )),
            // OK
            Candidate::new(
                E05,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23721698.975, None)
                        .with_ambiguous_phase_range_m(124658409.765),
                    Observation::pseudo_range(Carrier::E5b, 23721697.897, None)
                        .with_ambiguous_phase_range_m(93089093.648),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.770391),
            )),
            // OK
            Candidate::new(
                E09,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22777470.182, None)
                        .with_ambiguous_phase_range_m(119696457.450),
                    Observation::pseudo_range(Carrier::E5b, 22777468.871, None)
                        .with_ambiguous_phase_range_m(89383740.738),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.671834),
            )),
            // OK
            Candidate::new(
                E13,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27963274.550, None)
                        .with_ambiguous_phase_range_m(146948003.533),
                    Observation::pseudo_range(Carrier::E5b, 27963273.367, None)
                        .with_ambiguous_phase_range_m(109733898.616),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847606),
            )),
            // OK
            Candidate::new(
                E15,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26902705.809, None)
                        .with_ambiguous_phase_range_m(141374681.651),
                    Observation::pseudo_range(Carrier::E5b, 26902705.793, None)
                        .with_ambiguous_phase_range_m(105572005.022),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.328904),
            )),
            // OK
            Candidate::new(
                E24,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23607504.612, None)
                        .with_ambiguous_phase_range_m(124058281.040),
                    Observation::pseudo_range(Carrier::E5b, 23607514.182, None)
                        .with_ambiguous_phase_range_m(92640969.900),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5384.999417),
            )),
            // OK
            Candidate::new(
                E31,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24568204.755, None)
                        .with_ambiguous_phase_range_m(129106816.911),
                    Observation::pseudo_range(Carrier::E5b, 24568204.040, None)
                        .with_ambiguous_phase_range_m(96410937.404),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988276),
            )),
            // OK
            Candidate::new(
                E01,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27661258.503, None)
                        .with_ambiguous_phase_range_m(145360907.686),
                    Observation::pseudo_range(Carrier::E5b, 27661257.497, None)
                        .with_ambiguous_phase_range_m(108548746.851),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.728967),
            )),
            // OK
            Candidate::new(
                E03,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27010377.430, None)
                        .with_ambiguous_phase_range_m(141940500.113),
                    Observation::pseudo_range(Carrier::E5b, 27010376.711, None)
                        .with_ambiguous_phase_range_m(105994541.535),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.511002),
            )),
            // OK
            Candidate::new(
                E05,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23717469.331, None)
                        .with_ambiguous_phase_range_m(124636182.671),
                    Observation::pseudo_range(Carrier::E5b, 23717468.180, None)
                        .with_ambiguous_phase_range_m(93072495.500),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.767474),
            )),
            // OK
            Candidate::new(
                E09,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22788149.916, None)
                        .with_ambiguous_phase_range_m(119752579.857),
                    Observation::pseudo_range(Carrier::E5b, 22788148.659, None)
                        .with_ambiguous_phase_range_m(89425650.328),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.660829),
            )),
            // OK
            Candidate::new(
                E13,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27955670.294, None)
                        .with_ambiguous_phase_range_m(146908041.217),
                    Observation::pseudo_range(Carrier::E5b, 27955669.386, None)
                        .with_ambiguous_phase_range_m(109704056.642),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847841),
            )),
            // OK
            Candidate::new(
                E15,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26909374.208, None)
                        .with_ambiguous_phase_range_m(141409724.237),
                    Observation::pseudo_range(Carrier::E5b, 26909374.364, None)
                        .with_ambiguous_phase_range_m(105598173.225),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.327697),
            )),
            // OK
            Candidate::new(
                E24,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23593001.901, None)
                        .with_ambiguous_phase_range_m(123982068.745),
                    Observation::pseudo_range(Carrier::E5b, 23593011.562, None)
                        .with_ambiguous_phase_range_m(92584058.144),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5384.981517),
            )),
            // OK
            Candidate::new(
                E31,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24569353.107, None)
                        .with_ambiguous_phase_range_m(129112851.210),
                    Observation::pseudo_range(Carrier::E5b, 24569352.356, None)
                        .with_ambiguous_phase_range_m(96415443.541),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988448),
            )),
            // OK
            Candidate::new(
                E01,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27676365.168, None)
                        .with_ambiguous_phase_range_m(145440290.795),
                    Observation::pseudo_range(Carrier::E5b, 27676363.634, None)
                        .with_ambiguous_phase_range_m(108608026.449),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.736121),
            )),
            // OK
            Candidate::new(
                E03,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26995215.170, None)
                        .with_ambiguous_phase_range_m(141860825.142),
                    Observation::pseudo_range(Carrier::E5b, 26995214.771, None)
                        .with_ambiguous_phase_range_m(105935044.000),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.514700),
            )),
            // OK
            Candidate::new(
                E05,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23713293.150, None)
                        .with_ambiguous_phase_range_m(124614236.703),
                    Observation::pseudo_range(Carrier::E5b, 23713292.035, None)
                        .with_ambiguous_phase_range_m(93056107.280),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.764558),
            )),
            // OK
            Candidate::new(
                E09,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22798874.048, None)
                        .with_ambiguous_phase_range_m(119808935.247),
                    Observation::pseudo_range(Carrier::E5b, 22798872.769, None)
                        .with_ambiguous_phase_range_m(89467733.904),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.649787),
            )),
            // OK
            Candidate::new(
                E13,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27948143.890, None)
                        .with_ambiguous_phase_range_m(146868491.293),
                    Observation::pseudo_range(Carrier::E5b, 27948143.178, None)
                        .with_ambiguous_phase_range_m(109674522.632),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.848117),
            )),
            // OK
            Candidate::new(
                E15,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26916123.181, None)
                        .with_ambiguous_phase_range_m(141445189.630),
                    Observation::pseudo_range(Carrier::E5b, 26916123.260, None)
                        .with_ambiguous_phase_range_m(105624657.098),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.326442),
            )),
            // OK
            Candidate::new(
                E24,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23578553.240, None)
                        .with_ambiguous_phase_range_m(123906140.078),
                    Observation::pseudo_range(Carrier::E5b, 23578562.924, None)
                        .with_ambiguous_phase_range_m(92527358.178),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5384.963584),
            )),
            // OK
            Candidate::new(
                E31,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24570585.451, None)
                        .with_ambiguous_phase_range_m(129119327.230),
                    Observation::pseudo_range(Carrier::E5b, 24570584.735, None)
                        .with_ambiguous_phase_range_m(96420279.532),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988574),
            )),
            // OK
        ]
        .into_iter()
        .collect::<Vec<_>>()
    }
}

#[cfg(test)]
mod test {

    use super::CandidatesBuilder;
    use itertools::Itertools;

    #[test]
    fn verify_data_correctness() {
        const NB_EXPECTED_EPOCHS: usize = 5;

        let data = CandidatesBuilder::build_data();

        let epoch_uniques = data.iter().map(|cd| cd.t).unique().collect::<Vec<_>>();
        let num_epochs = epoch_uniques.len();

        assert_eq!(
            num_epochs, NB_EXPECTED_EPOCHS,
            "incorrect number of epochs: {}, expecting {}",
            num_epochs, NB_EXPECTED_EPOCHS
        );

        let sv_uniques = data.iter().map(|cd| cd.sv).unique().collect::<Vec<_>>();

        for sv in sv_uniques.iter() {
            let sv_data = data.iter().filter(|cd| cd.sv == *sv).collect::<Vec<_>>();

            let epochs = sv_data.iter().map(|cd| cd.t).sorted().collect::<Vec<_>>();

            let unique_epochs = sv_data
                .iter()
                .map(|cd| cd.t)
                .unique()
                .sorted()
                .collect::<Vec<_>>();

            assert_eq!(
                epochs, unique_epochs,
                "{} temporal data is not unique: {:#?}",
                sv, epochs
            );
        }
    }
}
