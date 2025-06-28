//! Position solving candidate
use log::debug;

use crate::prelude::{Candidate, Method, Observation};

impl Candidate {
    /// Runs the SD algorithm between [Self] and pivot counterpart.
    pub(crate) fn sd_mut(&mut self, method: Method, pivot: &Self) {
        let mut sd = Option::<Observation>::None;

        match method {
            Method::SPP => {
                if let Some((c_1, p_1)) = self.l1_pseudo_range() {
                    if let Some((c_2, p_2)) = pivot.l1_pseudo_range() {
                        if c_1 == c_2 {
                            sd = Some(Observation::pseudo_range(c_1, p_1 - p_2, None));
                        }
                    }
                }
            },
            Method::CPP => {
                if let Some(pc_1) = self.code_if_combination() {
                    if let Some(pc_2) = pivot.code_if_combination() {
                        if pc_1.lhs == pc_2.lhs && pc_1.rhs == pc_2.rhs {
                            sd = Some(Observation::pseudo_range(
                                pc_1.rhs,
                                pc_1.value - pc_2.value,
                                None,
                            ));
                        }
                    }
                }
            },
            Method::PPP | Method::PPP_AR => {
                if let Some(lc_1) = self.phase_if_combination() {
                    if let Some(lc_2) = pivot.phase_if_combination() {
                        if lc_1.lhs == lc_2.lhs && lc_1.rhs == lc_2.rhs {
                            sd = Some(Observation::ambiguous_phase_range(
                                lc_1.rhs,
                                lc_1.value - lc_2.value,
                                None,
                            ));
                        }
                    }
                }
            },
        }
        self.sd = sd;
    }
}

#[cfg(test)]
mod test {

    use log::info;
    use std::str::FromStr;

    use crate::{
        prelude::{Carrier, Epoch, Method, Observation},
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

            for method in [Method::SPP, Method::CPP, Method::PPP] {
                for cd in pool.iter() {
                    let mut cd_differenced = cd.clone();
                    cd_differenced.sd_mut(method, &cd);

                    assert!(
                        cd_differenced.sd_codes_are_null(),
                        "{} - {}-SD(self-self) is not null for code measurement!",
                        t_str,
                        method,
                    );

                    assert!(
                        cd_differenced.sd_phases_are_null(),
                        "{} - {}-SD(self-self) is not null for phase measurement!",
                        t_str,
                        method,
                    );
                }
            }
        }
    }

    #[test]
    fn spp_single_difference() {
        init_logger();
        let t_str = "2020-06-25T00:00:00 GPST";
        let t = Epoch::from_str(t_str).unwrap();

        let mut rover = CandidatesBuilder::build_rover_at(t);

        let e05 = CandidatesBuilder::build_rover_sv_at(E05, t);

        let mut e01_test_passed = false;
        let mut e03_test_passed = false;

        for rover in rover.iter() {
            let mut rover_differenced = rover.clone();

            rover_differenced.sd_mut(Method::SPP, &e05);

            if rover.sv == E01 {
                assert_eq!(
                    rover_differenced.sd,
                    Some(Observation::pseudo_range(
                        Carrier::L1,
                        27616185.992 - 23730317.923,
                        None
                    )),
                    "{}(E01): invalid SD(L1-L1_ref)",
                    t_str
                );

                e01_test_passed = true;
            } else if rover.sv == E03 {
                assert_eq!(
                    rover_differenced.sd,
                    Some(Observation::pseudo_range(
                        Carrier::L1,
                        27055946.391 - 23730317.923,
                        None
                    )),
                    "{}(E03): invalid SD(L1-L1_ref)",
                    t_str
                );

                e03_test_passed = true;
            }
        }

        assert!(e01_test_passed);
        assert!(e03_test_passed);
    }

    #[test]
    fn cpp_single_difference() {
        init_logger();
        let t_str = "2020-06-25T00:00:00 GPST";
        let t = Epoch::from_str(t_str).unwrap();

        let mut rover = CandidatesBuilder::build_rover_at(t);

        let e05 = CandidatesBuilder::build_rover_sv_at(E05, t);

        let mut e01_test_passed = false;
        let mut e03_test_passed = false;

        let (f1, fj) = (Carrier::L1.frequency_hz(), Carrier::E5b.frequency_hz());
        let (f1pow, fjpow) = (f1.powi(2), fj.powi(2));

        for rover in rover.iter() {
            let mut rover_differenced = rover.clone();

            rover_differenced.sd_mut(Method::CPP, &e05);

            if rover.sv == E01 {
                let p11 = 27616185.992;
                let p12 = 27616184.819;
                let pc_1 = (f1pow * p11 - fjpow * p12) / (f1pow - fjpow);

                let p51 = 23730317.923;
                let p52 = 23730316.788;
                let pc_5 = (f1pow * p51 - fjpow * p52) / (f1pow - fjpow);

                assert_eq!(
                    rover_differenced.sd,
                    Some(Observation::pseudo_range(Carrier::L1, pc_1 - pc_5, None)),
                    "{}(E01): invalid SD(P1-P1_ref)",
                    t_str
                );

                info!("{}(E01) : SD(C1-C1_ref)={}", t_str, pc_1 - pc_5);
                e01_test_passed = true;
            } else if rover.sv == E03 {
                let p31 = 27055946.391;
                let p32 = 27055945.532;
                let pc_3 = (f1pow * p31 - fjpow * p32) / (f1pow - fjpow);

                let p51 = 23730317.923;
                let p52 = 23730316.788;
                let pc_5 = (f1pow * p51 - fjpow * p52) / (f1pow - fjpow);

                assert_eq!(
                    rover_differenced.sd,
                    Some(Observation::pseudo_range(Carrier::L1, pc_3 - pc_5, None)),
                    "{}(E03): invalid SD(L1-L1_ref)",
                    t_str
                );

                info!("{}(E03) : SD(C1-C1_ref)={}", t_str, pc_3 - pc_5);
                e03_test_passed = true;
            }
        }

        assert!(e01_test_passed);
        assert!(e03_test_passed);
    }

    #[test]
    fn ppp_single_difference() {
        init_logger();
        let t_str = "2020-06-25T00:00:00 GPST";
        let t = Epoch::from_str(t_str).unwrap();

        let mut rover = CandidatesBuilder::build_rover_at(t);

        let e05 = CandidatesBuilder::build_rover_sv_at(E05, t);

        let mut e01_test_passed = false;
        let mut e03_test_passed = false;

        let (f1, fj) = (Carrier::L1.frequency_hz(), Carrier::E5b.frequency_hz());
        let (f1pow, fjpow) = (f1.powi(2), fj.powi(2));

        for rover in rover.iter() {
            let mut rover_differenced = rover.clone();

            rover_differenced.sd_mut(Method::PPP, &e05);

            if rover.sv == E01 {
                let l11 = 145124050.106;
                let l12 = 108371872.760;
                let lc_1 = (f1pow * l11 - fjpow * l12) / (f1pow - fjpow);

                let l51 = 124703702.220;
                let l52 = 93122915.921;
                let lc_5 = (f1pow * l51 - fjpow * l52) / (f1pow - fjpow);

                assert_eq!(
                    rover_differenced.sd,
                    Some(Observation::ambiguous_phase_range(
                        Carrier::L1,
                        lc_1 - lc_5,
                        None
                    )),
                    "{}(E01): invalid SD(L1-L1_ref)",
                    t_str
                );

                info!("{}(E01) : SD(L1-L1_ref)={}", t_str, lc_1 - lc_5);

                e01_test_passed = true;
            } else if rover.sv == E03 {
                let l31 = 142179967.778;
                let l32 = 106173364.686;
                let lc_3 = (f1pow * l31 - fjpow * l32) / (f1pow - fjpow);

                let l51 = 124703702.220;
                let l52 = 93122915.921;
                let lc_5 = (f1pow * l51 - fjpow * l52) / (f1pow - fjpow);

                assert_eq!(
                    rover_differenced.sd,
                    Some(Observation::ambiguous_phase_range(
                        Carrier::L1,
                        lc_3 - lc_5,
                        None
                    )),
                    "{}(E03): invalid SD(L1-L1_ref)",
                    t_str
                );

                info!("{}(E03) : SD(L1-L1_ref)={}", t_str, lc_3 - lc_5);
                e03_test_passed = true;
            }
        }

        assert!(e01_test_passed);
        assert!(e03_test_passed);
    }
}
