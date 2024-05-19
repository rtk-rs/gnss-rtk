//! Position solving candidate
use crate::prelude::{Carrier, Config, Duration, Epoch, Error, InterpolationResult, SV};
use hifitime::Unit;
use itertools::Itertools;
use log::debug;
use nyx::cosmic::SPEED_OF_LIGHT;
use std::cmp::Ordering;

/// Signal observation to attach to each candidate
#[derive(Debug, Default, PartialEq, Clone)]
pub struct Observation {
    /// Carrier signal
    pub carrier: Carrier,
    /// Measured value
    pub value: f64,
    /// Optional (but recommended) SNR in [dB]
    pub snr: Option<f64>,
}

/// Position solving candidate
#[derive(Debug, Clone)]
pub struct Candidate {
    /// SV
    pub sv: SV,
    /// Sampling Epoch
    pub t: Epoch,
    /// Tx Epoch
    pub t_tx: Epoch,
    /// State that needs to be resolved
    pub state: Option<InterpolationResult>,
    // SV group delay
    pub(crate) tgd: Option<Duration>,
    // SV clock correction
    pub(crate) clock_corr: Duration,
    // Pseudo Range Code observations
    pub(crate) pseudo_range: Vec<Observation>,
    // Phase range observations
    pub(crate) phase_range: Vec<Observation>,
    // Doppler observations
    pub(crate) doppler: Vec<Observation>,
}

impl Candidate {
    /// Creates a new candidate, to inject in the solver pool.
    /// ## Inputs
    /// - sv: [SV] Identity
    /// - t: sampling [Epoch]
    /// - clock_corr: SV onboard clock correction (mandatory)
    /// - tgd: possible onboard group delay
    /// - pseudo_range: provide as many Pseudo Range observations as you can
    /// - phase_range: provide as many Phase Range observations as you can.
    ///   NB: we do not accept raw "radians" here, but phase range [m] once again.
    /// - doppler: provide as many Doppler observations as you can
    pub fn new(
        sv: SV,
        t: Epoch,
        clock_corr: Duration,
        tgd: Option<Duration>,
        pseudo_range: Vec<Observation>,
        phase_range: Vec<Observation>,
        doppler: Vec<Observation>,
    ) -> Self {
        Self {
            sv,
            t,
            t_tx: t,
            clock_corr,
            tgd,
            pseudo_range,
            phase_range,
            doppler,
            state: None,
        }
    }
    /// Define state
    pub fn set_state(&mut self, state: InterpolationResult) {
        self.state = Some(state);
    }
    /*
     * Returns best observed SNR, whatever the signal
     */
    pub(crate) fn best_snr(&self) -> Option<f64> {
        self.pseudo_range
            .iter()
            .chain(self.phase_range.iter())
            .chain(self.doppler.iter())
            .max_by(|a, b| {
                if let Some(snr_a) = a.snr {
                    if let Some(snr_b) = b.snr {
                        snr_a.partial_cmp(&snr_b).unwrap()
                    } else {
                        Ordering::Greater
                    }
                } else {
                    Ordering::Less
                }
            })
            .map(|c| c.snr)?
    }
    /*
     * Returns one pseudo range observation [m], whatever the frequency.
     * Best SNR is preferred though (if such information was provided).
     */
    pub(crate) fn prefered_pseudorange(&self) -> Option<Observation> {
        let mut snr = Option::<f64>::None;
        let mut pr = Option::<Observation>::None;
        for c in &self.pseudo_range {
            if pr.is_none() {
                pr = Some(c.clone());
                snr = c.snr;
            } else {
                // prefer best SNR if possible
                if let Some(s1) = c.snr {
                    if snr.is_some() {
                        let s2 = snr.unwrap();
                        if s1 > s2 {
                            snr = Some(s1);
                            pr = Some(c.clone());
                        }
                    } else {
                        snr = Some(s1);
                        pr = Some(c.clone());
                    }
                }
            }
        }
        pr
    }
    // True if Self is Method::CPP compatible
    pub(crate) fn cpp_compatible(&self) -> bool {
        self.dual_pseudorange()
    }
    // True if Self is Method::PPP compatible
    pub(crate) fn ppp_compatible(&self) -> bool {
        self.dual_pseudorange() && self.dual_phase()
    }
    // True if dual PR is present
    pub(crate) fn dual_pseudorange(&self) -> bool {
        self.pseudo_range
            .iter()
            .map(|c| (c.carrier.frequency() / 1.0E6) as u16)
            .unique()
            .count()
            > 1
    }
    // True if dual phase is present
    pub(crate) fn dual_phase(&self) -> bool {
        self.phase_range
            .iter()
            .map(|c| (c.carrier.frequency() / 1.0E6) as u16)
            .unique()
            .count()
            > 1
    }
    pub(crate) fn pseudorange_combination(&self) -> Option<Observation> {
        let l1_pr = self
            .pseudo_range
            .iter()
            .filter(|p| {
                matches!(
                    p.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)?;

        let (c_l1, l1_signal) = (l1_pr.value, l1_pr.carrier);
        let freq_l1 = l1_signal.frequency();

        let lx_pr = self
            .pseudo_range
            .iter()
            .filter(|p| p.carrier != l1_signal)
            .reduce(|k, _| k)?;

        let (c_lx, lx_signal) = (lx_pr.value, lx_pr.carrier);
        let freq_lx = lx_signal.frequency();

        let alpha = 1.0 / (freq_l1.powi(2) - freq_lx.powi(2));
        let beta = freq_l1.powi(2);
        let gamma = freq_lx.powi(2);
        Some({
            Observation {
                snr: None,
                carrier: l1_signal,
                value: alpha * (beta * c_l1 - gamma * c_lx),
            }
        })
    }
    pub(crate) fn phase_combination(&self) -> Option<Observation> {
        let l1_ph = self
            .phase_range
            .iter()
            .filter(|p| {
                matches!(
                    p.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)?;

        let (c_l1, l1_signal) = (l1_ph.value, l1_ph.carrier);
        let f_l1 = l1_signal.frequency();

        let lx_ph = self
            .phase_range
            .iter()
            .filter(|p| p.carrier != l1_signal)
            .reduce(|k, _| k)?;

        let (c_lx, lx_signal) = (lx_ph.value, lx_ph.carrier);
        let f_lx = lx_signal.frequency();

        let alpha = 1.0 / (f_l1.powi(2) - f_lx.powi(2));
        let beta = f_l1.powi(2);
        let gamma = f_lx.powi(2);
        Some({
            Observation {
                snr: None,
                carrier: l1_signal,
                value: alpha * (beta * c_l1 - gamma * c_lx),
            }
        })
    }
    pub(crate) fn phase_wide_combination(&self) -> Option<Observation> {
        let l_1 = self
            .phase_range
            .iter()
            .filter(|p| matches!(p.carrier, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let l_j = self
            .phase_range
            .iter()
            .filter(|p| {
                p.carrier != Carrier::L1 && p.carrier != Carrier::E1 && p.carrier != Carrier::B1aB1c
            })
            .reduce(|k, _| k)?;

        let f_1 = l_1.carrier.frequency();
        let f_j = l_j.carrier.frequency();

        Some(Observation {
            snr: None,
            carrier: l_1.carrier,
            value: (f_1 * l_1.value - f_j * l_j.value) / (f_1 - f_j),
        })
    }
    pub(crate) fn code_narrow_combination(&self) -> Option<Observation> {
        let c_1 = self
            .pseudo_range
            .iter()
            .filter(|p| matches!(p.carrier, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let c_j = self
            .pseudo_range
            .iter()
            .filter(|p| {
                p.carrier != Carrier::L1 && p.carrier != Carrier::E1 && p.carrier != Carrier::B1aB1c
            })
            .reduce(|k, _| k)?;

        let f_1 = c_1.carrier.frequency();
        let f_j = c_j.carrier.frequency();

        Some(Observation {
            snr: None,
            carrier: c_1.carrier,
            value: (f_1 * c_1.value + f_j * c_j.value) / (f_1 + f_j),
        })
    }
    pub(crate) fn mw_combination(&self) -> Option<Observation> {
        let pw = self.phase_wide_combination()?;
        let cn = self.code_narrow_combination()?;
        Some(Observation {
            snr: None,
            carrier: pw.carrier,
            value: pw.value - cn.value,
        })
    }
    /*
     * Apply min SNR mask to mutable self
     */
    pub(crate) fn min_snr_mask(&mut self, min_snr: f64) {
        self.pseudo_range.retain(|c| {
            if let Some(snr) = c.snr {
                snr >= min_snr
            } else {
                false
            }
        });
        self.doppler.retain(|d| {
            if let Some(snr) = d.snr {
                snr >= min_snr
            } else {
                false
            }
        });
        self.phase_range.retain(|p| {
            if let Some(snr) = p.snr {
                snr >= min_snr
            } else {
                false
            }
        });
    }
    /*
     * Computes signal transmission time, expressed as [Epoch]
     * returns (t_tx, dt_ttx)
     * "t_tx": Epoch in given timescale
     * "dt_ttx": elapsed duration in seconds in given timescale
     */
    pub(crate) fn transmission_time(&self, cfg: &Config) -> Result<(Epoch, Duration), Error> {
        let (t, ts) = (self.t, self.t.time_scale);
        let seconds_ts = t.to_duration_in_time_scale(t.time_scale).to_seconds();

        let dt_tx = seconds_ts
            - self
                .prefered_pseudorange()
                .ok_or(Error::MissingPseudoRange)?
                .value
                / SPEED_OF_LIGHT;

        let mut e_tx = Epoch::from_duration(dt_tx * Unit::Second, ts);

        if cfg.modeling.sv_clock_bias {
            debug!("{} ({}) clock correction: {}", t, self.sv, self.clock_corr);
            e_tx -= self.clock_corr;
        }

        if cfg.modeling.sv_total_group_delay {
            if let Some(tgd) = self.tgd {
                debug!("{} ({}) tgd   : {}", t, self.sv, tgd);
                e_tx -= tgd;
            }
        }

        let dt_secs = (t - e_tx).to_seconds();
        let dt = Duration::from_seconds(dt_secs);
        assert!(
            dt_secs >= 0.0,
            "physical non sense - RX {:?} prior TX {:?}",
            t,
            e_tx
        );
        assert!(
            dt_secs <= 0.2,
            "{}({}): {} propagation delay (to Earth) is unrealistic: invalid input",
            t,
            self.sv,
            dt
        );
        Ok((e_tx, dt))
    }
}

#[cfg(test)]
mod test {
    use crate::prelude::{Candidate, Carrier, Duration, Epoch, Observation, SV};
    #[test]
    fn prefered_pseudorange() {
        let l1_freq = 1575.42_f64 * 1.0E6_f64;
        let l2_freq = 1176.45_f64 * 1.0E6_f64;
        let l5_freq = 1176.45_f64 * 1.0E6_f64;

        for (observations, prefered) in [
            (
                vec![
                    Observation {
                        value: 1.0,
                        snr: None,
                        carrier: Carrier::L1,
                    },
                    Observation {
                        value: 2.0,
                        snr: None,
                        carrier: Carrier::L2,
                    },
                    Observation {
                        value: 3.0,
                        snr: None,
                        carrier: Carrier::L5,
                    },
                ],
                Observation {
                    value: 1.0,
                    snr: None,
                    carrier: Carrier::L1,
                },
            ),
            (
                vec![
                    Observation {
                        value: 1.0,
                        snr: None,
                        carrier: Carrier::L1,
                    },
                    Observation {
                        value: 2.0,
                        snr: Some(2.0),
                        carrier: Carrier::L2,
                    },
                    Observation {
                        value: 3.0,
                        snr: None,
                        carrier: Carrier::L5,
                    },
                ],
                Observation {
                    value: 2.0,
                    snr: Some(2.0),
                    carrier: Carrier::L2,
                },
            ),
        ] {
            let cd = Candidate::new(
                SV::default(),
                Epoch::default(),
                Duration::default(),
                None,
                observations,
                vec![],
                vec![],
            );
            assert_eq!(cd.prefered_pseudorange(), Some(prefered),);
        }
    }
    #[test]
    fn best_snr() {
        let l1_freq = 1575.42_f64 * 1.0E6_f64;
        let l2_freq = 1176.45_f64 * 1.0E6_f64;
        let l5_freq = 1176.45_f64 * 1.0E6_f64;

        let codes = vec![
            Observation {
                value: 1.0,
                snr: None,
                carrier: Carrier::L1,
            },
            Observation {
                value: 2.0,
                snr: None,
                carrier: Carrier::L2,
            },
            Observation {
                value: 3.0,
                snr: Some(10.0),
                carrier: Carrier::L5,
            },
            Observation {
                value: 4.0,
                snr: Some(11.0),
                carrier: Carrier::L2,
            },
            Observation {
                value: 5.0,
                snr: Some(9.0),
                carrier: Carrier::L2,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            vec![],
            vec![],
        );
        assert_eq!(cd.best_snr(), Some(11.0));

        let codes = vec![
            Observation {
                value: 1.0,
                snr: Some(1.0),
                carrier: Carrier::L1,
            },
            Observation {
                value: 2.0,
                snr: Some(1.1),
                carrier: Carrier::L2,
            },
            Observation {
                value: 3.0,
                snr: Some(1.2),
                carrier: Carrier::L5,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            vec![],
            vec![],
        );
        assert_eq!(cd.best_snr(), Some(1.2));
    }
    #[test]
    fn ppp_compatibility() {
        for (pr_observations, phase_observations, cpp_compatible) in [
            (
                vec![Observation {
                    value: 1.0,
                    snr: Some(1.0),
                    carrier: Carrier::L1,
                }],
                vec![],
                false,
            ),
            (
                vec![
                    Observation {
                        value: 1.0,
                        snr: Some(1.0),
                        carrier: Carrier::L1,
                    },
                    Observation {
                        value: 2.0,
                        snr: Some(2.0),
                        carrier: Carrier::L2,
                    },
                ],
                vec![],
                true,
            ),
        ] {
            let cd = Candidate::new(
                SV::default(),
                Epoch::default(),
                Duration::default(),
                None,
                pr_observations,
                phase_observations,
                vec![],
            );
            assert_eq!(cd.cpp_compatible(), cpp_compatible);
        }
    }
    #[test]
    fn l1_phase_wide() {
        let phases = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::L1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::L2,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            vec![],
            phases,
            vec![],
        );
        let pw = cd.phase_wide_combination();
        assert!(pw.is_some(), "failed to form Ph_wide combination");
        let pw = pw.unwrap();
        assert_eq!(pw.carrier, Carrier::L1);
        assert_eq!(
            pw.value,
            (Carrier::L1.frequency() * 64.0 - Carrier::L2.frequency() * 128.0)
                / (Carrier::L1.frequency() - Carrier::L2.frequency())
        );

        let phases = vec![Observation {
            snr: None,
            value: 64.0,
            carrier: Carrier::L1,
        }];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            vec![],
            phases,
            vec![],
        );

        assert!(
            cd.phase_wide_combination().is_none(),
            "Ph_wide should not be feasible!"
        );
    }
    #[test]
    fn e1_phase_wide() {
        let phases = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::E1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::E5,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            vec![],
            phases,
            vec![],
        );
        let pw = cd.phase_wide_combination();
        assert!(pw.is_some(), "failed to form Ph_wide combination");
        let pw = pw.unwrap();
        assert_eq!(pw.carrier, Carrier::E1);
        assert_eq!(
            pw.value,
            (Carrier::E1.frequency() * 64.0 - Carrier::E5.frequency() * 128.0)
                / (Carrier::E1.frequency() - Carrier::E5.frequency())
        );
    }
    #[test]
    fn l1_code_narrow() {
        let codes = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::L1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::L2,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            vec![],
            vec![],
        );
        let cn = cd.code_narrow_combination();
        assert!(cn.is_some(), "failed to form Cn_narrow combination");
        let cn = cn.unwrap();
        assert_eq!(cn.carrier, Carrier::L1);
        assert_eq!(
            cn.value,
            (Carrier::L1.frequency() * 64.0 + Carrier::L2.frequency() * 128.0)
                / (Carrier::L1.frequency() + Carrier::L2.frequency())
        );

        let codes = vec![Observation {
            snr: None,
            value: 64.0,
            carrier: Carrier::L1,
        }];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            vec![],
            vec![],
        );

        assert!(
            cd.code_narrow_combination().is_none(),
            "Cn_narrow should not be feasible!"
        );
    }
    #[test]
    fn e1_code_narrow() {
        let codes = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::E1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::E5,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            vec![],
            vec![],
        );
        let cn = cd.code_narrow_combination();
        assert!(cn.is_some(), "failed to form Cn_narrow combination");
        let cn = cn.unwrap();
        assert_eq!(cn.carrier, Carrier::E1);
        assert_eq!(
            cn.value,
            (Carrier::E1.frequency() * 64.0 + Carrier::E5.frequency() * 128.0)
                / (Carrier::E1.frequency() + Carrier::E5.frequency())
        );
    }
    #[test]
    fn e1_e5_mw_combination() {
        let codes = vec![
            Observation {
                snr: None,
                value: 64.0,
                carrier: Carrier::E1,
            },
            Observation {
                snr: None,
                value: 128.0,
                carrier: Carrier::E5,
            },
        ];
        let phases = vec![
            Observation {
                snr: None,
                value: 16.0,
                carrier: Carrier::E1,
            },
            Observation {
                snr: None,
                value: 32.0,
                carrier: Carrier::E5,
            },
        ];
        let cd = Candidate::new(
            SV::default(),
            Epoch::default(),
            Duration::default(),
            None,
            codes,
            phases,
            vec![],
        );
        let mw = cd.mw_combination();
        assert!(mw.is_some(), "failed to form MW combination");
        let mw = mw.unwrap();
        assert_eq!(mw.carrier, Carrier::E1);
        let pw = (Carrier::E1.frequency() * 16.0 - Carrier::E5.frequency() * 32.0)
            / (Carrier::E1.frequency() - Carrier::E5.frequency());
        let cn = (Carrier::E1.frequency() * 64.0 + Carrier::E5.frequency() * 128.0)
            / (Carrier::E1.frequency() + Carrier::E5.frequency());
        assert_eq!(mw.value, pw - cn);
    }
}
