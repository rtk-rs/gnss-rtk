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
                Duration::from_microseconds(5385.035201),
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
                    Observation::pseudo_range(Carrier::L1, 28082362.117, None)
                        .with_ambiguous_phase_range_m(147573817.753),
                    Observation::pseudo_range(Carrier::E5b, 28082361.005, None)
                        .with_ambiguous_phase_range_m(110201243.966),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.714669),
            )),
            Candidate::new(
                E03,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26606532.953, None)
                        .with_ambiguous_phase_range_m(139818287.147),
                    Observation::pseudo_range(Carrier::E5b, 26606532.001, None)
                        .with_ambiguous_phase_range_m(104409773.053),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.503498),
            )),
            Candidate::new(
                E05,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23623876.415, None)
                        .with_ambiguous_phase_range_m(124144348.824),
                    Observation::pseudo_range(Carrier::E5b, 23623875.215, None)
                        .with_ambiguous_phase_range_m(92705217.123),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.773276),
            )),
            Candidate::new(
                E09,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23092986.586, None)
                        .with_ambiguous_phase_range_m(121354506.223),
                    Observation::pseudo_range(Carrier::E5b, 23092985.298, None)
                        .with_ambiguous_phase_range_m(90621893.797),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.682886),
            )),
            Candidate::new(
                E13,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27780658.212, None)
                        .with_ambiguous_phase_range_m(145988350.070),
                    Observation::pseudo_range(Carrier::E5b, 27780656.979, None)
                        .with_ambiguous_phase_range_m(109017274.709),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847317),
            )),
            Candidate::new(
                E15,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27119058.610, None)
                        .with_ambiguous_phase_range_m(142511625.150),
                    Observation::pseudo_range(Carrier::E5b, 27119059.363, None)
                        .with_ambiguous_phase_range_m(106421021.309),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.330144),
            )),
            Candidate::new(
                E24,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23222677.117, None)
                        .with_ambiguous_phase_range_m(122035999.088),
                    Observation::pseudo_range(Carrier::E5b, 23222687.027, None)
                        .with_ambiguous_phase_range_m(91130824.527),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5385.017319),
            )),
            Candidate::new(
                E31,
                t1_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24631780.017, None)
                        .with_ambiguous_phase_range_m(129440908.053),
                    Observation::pseudo_range(Carrier::E5b, 24631779.306, None)
                        .with_ambiguous_phase_range_m(96660421.268),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988096),
            )),
            Candidate::new(
                E01,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 28577344.325, None)
                        .with_ambiguous_phase_range_m(150174966.356),
                    Observation::pseudo_range(Carrier::E5b, 28577343.483, None)
                        .with_ambiguous_phase_range_m(112143659.135),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.721810),
            )),
            Candidate::new(
                E03,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 26174086.791, None)
                        .with_ambiguous_phase_range_m(137545768.524),
                    Observation::pseudo_range(Carrier::E5b, 26174085.540, None)
                        .with_ambiguous_phase_range_m(102712763.137),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.507266),
            )),
            Candidate::new(
                E05,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23568449.038, None)
                        .with_ambiguous_phase_range_m(123853076.698),
                    Observation::pseudo_range(Carrier::E5b, 23568447.746, None)
                        .with_ambiguous_phase_range_m(92487708.863),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.770391),
            )),
            Candidate::new(
                E09,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23467567.543, None)
                        .with_ambiguous_phase_range_m(123322942.461),
                    Observation::pseudo_range(Carrier::E5b, 23467566.515, None)
                        .with_ambiguous_phase_range_m(92091829.787),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.671834),
            )),
            Candidate::new(
                E13,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27657598.527, None)
                        .with_ambiguous_phase_range_m(145341666.399),
                    Observation::pseudo_range(Carrier::E5b, 27657597.170, None)
                        .with_ambiguous_phase_range_m(108534361.838),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847606),
            )),
            Candidate::new(
                E15,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27414801.157, None)
                        .with_ambiguous_phase_range_m(144065761.257),
                    Observation::pseudo_range(Carrier::E5b, 27414800.782, None)
                        .with_ambiguous_phase_range_m(107581577.638),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.328904),
            )),
            Candidate::new(
                E24,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22863341.608, None)
                        .with_ambiguous_phase_range_m(120147679.864),
                    Observation::pseudo_range(Carrier::E5b, 22863351.054, None)
                        .with_ambiguous_phase_range_m(89720716.434),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5384.999417),
            )),
            Candidate::new(
                E31,
                t2_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24770072.612, None)
                        .with_ambiguous_phase_range_m(130167640.275),
                    Observation::pseudo_range(Carrier::E5b, 24770071.564, None)
                        .with_ambiguous_phase_range_m(97203111.17706),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988276),
            )),
            Candidate::new(
                E01,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 29092635.828, None)
                        .with_ambiguous_phase_range_m(152882837.719),
                    Observation::pseudo_range(Carrier::E5b, 29092635.926, None)
                        .with_ambiguous_phase_range_m(114165769.700),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-884.728967),
            )),
            Candidate::new(
                E03,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 25763722.247, None)
                        .with_ambiguous_phase_range_m(135389290.834),
                    Observation::pseudo_range(Carrier::E5b, 25763720.939, None)
                        .with_ambiguous_phase_range_m(101102406.973),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.511002),
            )),
            Candidate::new(
                E05,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23566872.078, None)
                        .with_ambiguous_phase_range_m(123844789.147),
                    Observation::pseudo_range(Carrier::E5b, 23566870.615, None)
                        .with_ambiguous_phase_range_m(92481520.116),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.767474),
            )),
            Candidate::new(
                E09,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23877328.596, None)
                        .with_ambiguous_phase_range_m(125476250.520),
                    Observation::pseudo_range(Carrier::E5b, 23877327.191, None)
                        .with_ambiguous_phase_range_m(93699819.409),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.660829),
            )),
            Candidate::new(
                E13,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27612730.591, None)
                        .with_ambiguous_phase_range_m(145105885.665),
                    Observation::pseudo_range(Carrier::E5b, 27612729.140, None)
                        .with_ambiguous_phase_range_m(108358292.140),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.847841),
            )),
            Candidate::new(
                E15,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27768772.741, None)
                        .with_ambiguous_phase_range_m(145925894.814),
                    Observation::pseudo_range(Carrier::E5b, 27768772.699, None)
                        .with_ambiguous_phase_range_m(108970638.656),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.327697),
            )),
            Candidate::new(
                E24,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22564462.203, None)
                        .with_ambiguous_phase_range_m(118577059.434),
                    Observation::pseudo_range(Carrier::E5b, 22564471.813, None)
                        .with_ambiguous_phase_range_m(88547850.917),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5384.981517),
            )),
            Candidate::new(
                E31,
                t3_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24976421.969, None)
                        .with_ambiguous_phase_range_m(131252014.059),
                    Observation::pseudo_range(Carrier::E5b, 24976421.047, None)
                        .with_ambiguous_phase_range_m(98012870.941),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988448),
            )),
            Candidate::new(
                E03,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 25381023.064, None)
                        .with_ambiguous_phase_range_m(133378195.781),
                    Observation::pseudo_range(Carrier::E5b, 25381022.036, None)
                        .with_ambiguous_phase_range_m(99600615.595),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-313.514700),
            )),
            Candidate::new(
                E05,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 23621365.068, None)
                        .with_ambiguous_phase_range_m(124131152.182),
                    Observation::pseudo_range(Carrier::E5b, 23621363.960, None)
                        .with_ambiguous_phase_range_m(92695362.567),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-368.764558),
            )),
            Candidate::new(
                E09,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 24319118.620, None)
                        .with_ambiguous_phase_range_m(127797872.373),
                    Observation::pseudo_range(Carrier::E5b, 24319117.427, None)
                        .with_ambiguous_phase_range_m(95433498.048),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(6017.649787),
            )),
            Candidate::new(
                E13,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 27647098.655, None)
                        .with_ambiguous_phase_range_m(145286490.231),
                    Observation::pseudo_range(Carrier::E5b, 27647097.054, None)
                        .with_ambiguous_phase_range_m(108493159.141),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(401.848117),
            )),
            Candidate::new(
                E15,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 28171693.206, None)
                        .with_ambiguous_phase_range_m(148043255.618),
                    Observation::pseudo_range(Carrier::E5b, 28171693.865, None)
                        .with_ambiguous_phase_range_m(110551784.578),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(862.326442),
            )),
            Candidate::new(
                E24,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 22330484.011, None)
                        .with_ambiguous_phase_range_m(117347497.248),
                    Observation::pseudo_range(Carrier::E5b, 22330493.787, None)
                        .with_ambiguous_phase_range_m(87629671.742),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(5384.963584),
            )),
            Candidate::new(
                E31,
                t4_gpst,
                vec![
                    Observation::pseudo_range(Carrier::L1, 25244908.716, None)
                        .with_ambiguous_phase_range_m(132662921.691),
                    Observation::pseudo_range(Carrier::E5b, 25244907.849, None)
                        .with_ambiguous_phase_range_m(99066470.826),
                ],
            )
            .with_clock_correction(ClockCorrection::without_relativistic_correction(
                Duration::from_microseconds(-472.988574),
            )),
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
