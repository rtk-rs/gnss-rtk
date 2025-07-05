use log::debug;

use crate::{
    candidate::differences::Difference,
    constants::SPEED_OF_LIGHT_M_S,
    prelude::{Candidate, Method},
};

impl Candidate {
    /// Runs the SD algorithm between [Self] and pivot [Self].
    pub(crate) fn single_difference(&self, method: Method, pivot: &Self) -> Difference {
        let mut sd = Difference::default();

        match method {
            Method::SPP => {
                if let Some((c_1, p_1)) = self.l1_pseudo_range() {
                    if let Some((c_2, p_2)) = pivot.l1_pseudo_range() {
                        if c_1 == c_2 {
                            sd = sd.with_code((c_1, p_1 - p_2));
                        }
                    }
                }
            },
            _ => {
                if let Some(c_1) = self.code_if_combination() {
                    if let Some(c_2) = pivot.code_if_combination() {
                        if c_1.lhs == c_2.lhs && c_1.rhs == c_2.rhs {
                            sd = sd.with_code((c_1.rhs, c_1.value - c_2.value));
                        }
                    }
                }
            },
        }

        if let Some(mw_1) = self.mw_combination() {
            if let Some(mw_2) = pivot.mw_combination() {
                sd = sd.with_mw((mw_1.rhs, mw_1.lambda, mw_1.value - mw_2.value));
            }
        }

        if let Some((c_1, l_1)) = self.subsidary_phase_range() {
            if let Some((c_2, l_2)) = pivot.subsidary_phase_range() {
                if c_1 == c_2 {
                    let lambda = SPEED_OF_LIGHT_M_S / c_1.frequency_hz();
                    sd = sd.with_phase_j((c_1, lambda, l_1 - l_2));
                }
            }
        }

        if let Some(l_1) = self.phase_if_combination() {
            if let Some(l_2) = pivot.phase_if_combination() {
                if l_1.lhs == l_2.lhs && l_1.rhs == l_2.rhs {
                    sd = sd.with_phase_if((l_1.rhs, l_1.lambda, l_1.value - l_2.value));
                }
            }
        }

        debug!("{}({}) - {}", self.epoch, self.sv, sd);
        sd
    }
}

#[cfg(test)]
mod test {

    use log::info;
    use std::str::FromStr;

    use crate::{
        constants::SPEED_OF_LIGHT_M_S,
        prelude::{Carrier, Epoch, Method},
        tests::{init_logger, CandidatesBuilder, E01, E03, E05},
    };

    #[test]
    fn single_difference_null() {
        for t_str in [
            "2020-06-25T00:00:00 GPST",
            "2020-06-25T00:15:00 GPST",
            "2020-06-25T00:30:00 GPST",
            "2020-06-25T00:45:00 GPST",
            "2020-06-25T01:00:00 GPST",
        ] {
            let t = Epoch::from_str(t_str).unwrap();

            let mut pool = CandidatesBuilder::build_rover_at(t);

            assert!(
                pool.len() > 0,
                "failed to propose any measurements at {}",
                t_str
            );

            // SPP test
            for cd in pool.iter() {
                let single_diff = cd.single_difference(Method::SPP, &cd);
                assert_eq!(single_diff.code, Some((Carrier::L1, 0.0)));
            }

            // CPP test
            for cd in pool.iter() {
                let single_diff = cd.single_difference(Method::CPP, &cd);
                assert_eq!(single_diff.code, Some((Carrier::L1, 0.0)));
            }

            // PPP test
            for cd in pool.iter() {
                let single_diff = cd.single_difference(Method::PPP, &cd);
                assert_eq!(single_diff.code, Some((Carrier::L1, 0.0)));

                let (f1, f2) = (Carrier::L1.frequency_hz(), Carrier::E5b.frequency_hz());
                let lambda = SPEED_OF_LIGHT_M_S / f2;

                assert_eq!(single_diff.phase_j, Some((Carrier::E5b, lambda, 0.0)));

                let (f1pow, f2pow) = (f1.powi(2), f2.powi(2));
                let freq = f1 * f2 / (f1pow + f2pow).sqrt();
                let lambda = SPEED_OF_LIGHT_M_S / freq;

                assert_eq!(single_diff.phase_if, Some((Carrier::L1, lambda, 0.0)));

                let lambda = SPEED_OF_LIGHT_M_S / (f1 - f2);

                assert_eq!(single_diff.mw, Some((Carrier::L1, lambda, 0.0)));
            }
        }
    }

    #[test]
    fn single_difference() {
        init_logger();

        let t_str = "2020-06-25T00:00:00 GPST";
        let t = Epoch::from_str(t_str).unwrap();

        let rover = CandidatesBuilder::build_rover_at(t);
        let e05 = CandidatesBuilder::build_rover_sv_at(E05, t);

        let (f1, fj) = (Carrier::L1.frequency_hz(), Carrier::E5b.frequency_hz());
        let (f1pow, fjpow) = (f1.powi(2), fj.powi(2));

        let mut e01_test_passed = false;
        let mut e03_test_passed = false;

        for rover in rover.iter() {
            let spp_sd = rover.single_difference(Method::SPP, &e05);
            let cpp_sd = rover.single_difference(Method::CPP, &e05);
            let ppp_sd = rover.single_difference(Method::PPP, &e05);

            if rover.sv == E01 {
                assert_eq!(
                    spp_sd.code,
                    Some((Carrier::L1, 27616185.992 - 23730317.923)),
                    "{}(E01/E05): invalid SD(C1-C1_ref)",
                    t_str
                );

                let p11 = 27616185.992;
                let p12 = 27616184.819;
                let pc_1 = (f1pow * p11 - fjpow * p12) / (f1pow - fjpow);

                let p51 = 23730317.923;
                let p52 = 23730316.788;
                let pc_5 = (f1pow * p51 - fjpow * p52) / (f1pow - fjpow);

                assert_eq!(
                    cpp_sd.code,
                    Some((Carrier::L1, pc_1 - pc_5)),
                    "{}(E01/E05): invalid SD(P1-P1_ref)",
                    t_str
                );

                info!("{}(E01/E05) : SD(C1-C1_ref)={}", t_str, pc_1 - pc_5);

                // let l11 = 145124050.106;
                // let l12 = 108371872.760;
                // let lc_1 = (f1pow * l11 - fjpow * l12) / (f1pow - fjpow);

                // let l51 = 124703702.220;
                // let l52 = 93122915.921;
                // let lc_5 = (f1pow * l51 - fjpow * l52) / (f1pow - fjpow);

                e01_test_passed = true;
            } else if rover.sv == E03 {
                assert_eq!(
                    spp_sd.code,
                    Some((Carrier::L1, 27055946.391 - 23730317.923)),
                    "{}(E03/E05): invalid SD(L1-L1_ref)",
                    t_str
                );

                let p31 = 27055946.391;
                let p32 = 27055945.532;
                let pc_3 = (f1pow * p31 - fjpow * p32) / (f1pow - fjpow);

                let p51 = 23730317.923;
                let p52 = 23730316.788;
                let pc_5 = (f1pow * p51 - fjpow * p52) / (f1pow - fjpow);

                assert_eq!(
                    cpp_sd.code,
                    Some((Carrier::L1, pc_3 - pc_5)),
                    "{}(E03/E05): invalid SD(P1-P1_ref)",
                    t_str
                );

                info!("{}(E03/E05) : SD(C1-C1_ref)={}", t_str, pc_3 - pc_5);

                // let l31 = 142179967.778;
                // let l32 = 106173364.686;
                // let lc_3 = (f1pow * l31 - fjpow * l32) / (f1pow - fjpow);

                // let l51 = 124703702.220;
                // let l52 = 93122915.921;
                // let lc_5 = (f1pow * l51 - fjpow * l52) / (f1pow - fjpow);
                e03_test_passed = true;
            }
        }

        assert!(e01_test_passed);
        assert!(e03_test_passed);
    }
}
