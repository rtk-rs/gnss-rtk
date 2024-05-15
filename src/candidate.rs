//! Position solving candidate

use hifitime::Unit;
use itertools::Itertools;

use log::debug;
use std::cmp::Ordering;

use crate::prelude::{Config, Duration, Epoch, Error, InterpolationResult, SV};

use nyx::cosmic::SPEED_OF_LIGHT;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum Carrier {
    // L1 (GPS/QZSS/SBAS)
    #[default]
    L1,
    // L2 (GPS/QZSS)
    L2,
    // L5 (GPS/QZSS/SBAS)
    L5,
    // L6 (GPS/QZSS)
    L6,
    // E1 (Galileo)
    E1,
    // E5 (Galileo)
    E5,
    // E6 (Galileo)
    E6,
}

impl std::fmt::Display for Carrier {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        match self {
            Self::L1 => write!(f, "L1"),
            Self::L2 => write!(f, "L2"),
            Self::L5 => write!(f, "L5"),
            Self::L6 => write!(f, "L6"),
            Self::E1 => write!(f, "E1"),
            Self::E5 => write!(f, "E5"),
            Self::E6 => write!(f, "E6"),
        }
    }
}

impl Carrier {
    pub(crate) fn frequency(&self) -> f64 {
        match self {
            Self::L1 | Self::E1 => 1575.42E6_f64,
            Self::L2 => 1227.60E6_f64,
            Self::L5 => 1176.45E6_f64,
            Self::E5 => 1191.795E6_f64,
            Self::L6 | Self::E6 => 1278.750E6_f64,
        }
    }
}

/// Signal used in [PVTSolution] resolution
#[derive(Debug, Clone)]
pub enum Signal {
    Single(Carrier),
    Dual((Carrier, Carrier)),
}

impl Signal {
    pub(crate) fn single(carrier: Carrier) -> Self {
        Self::Single(carrier)
    }
    pub(crate) fn dual(lhs: Carrier, rhs: Carrier) -> Self {
        Self::Dual((lhs, rhs))
    }
}

impl std::fmt::Display for Signal {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        match self {
            Self::Single(carrier) => write!(f, "{}", carrier),
            Self::Dual((lhs, rhs)) => write!(f, "{}/{}", rhs, lhs),
        }
    }
}

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
    // True if Self is Method::CodePPP compatible
    pub(crate) fn code_ppp_compatible(&self) -> bool {
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
    /*
     * Forms combination
     */
    pub(crate) fn pseudorange_combination(&self) -> Option<Observation> {
        let mut l1_pr = Option::<(Observation, Carrier)>::None;
        for code in &self.pseudo_range {
            match code.carrier {
                Carrier::L1 => {
                    l1_pr = Some((code.clone(), Carrier::L1));
                    break;
                },
                Carrier::E1 => {
                    l1_pr = Some((code.clone(), Carrier::E1));
                    break;
                },
                _ => {},
            }
        }

        let (c_l1, l1_signal) = l1_pr?;
        let freq_l1 = l1_signal.frequency();
        let to_match = match l1_signal {
            Carrier::L1 => vec![Carrier::L2, Carrier::L5],
            Carrier::E1 => vec![Carrier::E5, Carrier::E6],
            _ => unreachable!(),
        };

        let mut lx_pr = Option::<(Observation, Carrier)>::None;

        for code in &self.pseudo_range {
            if to_match.contains(&code.carrier) {
                lx_pr = Some((code.clone(), code.carrier));
                break;
            }
        }

        let (c_lx, lx_signal) = lx_pr?;
        let freq_lx = lx_signal.frequency();

        let alpha = 1.0 / (freq_l1.powi(2) - freq_lx.powi(2));
        let beta = freq_l1.powi(2);
        let gamma = freq_lx.powi(2);
        Some({
            Observation {
                snr: None,
                carrier: l1_signal,
                value: alpha * (beta * c_l1.value - gamma * c_lx.value),
            }
        })
    } /*
       * Forms combination
       */
    pub(crate) fn phase_combination(&self) -> Option<Observation> {
        let mut phases = (
            Option::<&Observation>::None,
            Option::<&Observation>::None,
            Option::<&Observation>::None,
        );
        for ph in &self.phase_range {
            let freq = (ph.carrier.frequency() / 1.0E6) as u16;
            if freq == 1575 {
                phases.0 = Some(ph);
            } else if freq == 1227 {
                phases.1 = Some(ph);
            } else if freq == 1176 {
                phases.2 = Some(ph);
            }
        }

        let c_l1 = phases.0?;
        let f_l1 = 1575.42_f64 * 1.0E6_f64;

        let (c_lx, f_lx) = match phases.1 {
            Some(ph) => (ph, 1227.6_f64 * 1.0E6_f64),
            None => match phases.2 {
                Some(ph) => (ph, 1176.45_f64 * 1.0E6_f64),
                None => {
                    return None;
                },
            },
        };

        let alpha = 1.0 / (f_l1.powi(2) - f_lx.powi(2));
        let beta = f_l1.powi(2);
        let gamma = f_lx.powi(2);
        Some({
            Observation {
                snr: None,
                carrier: c_l1.carrier,
                value: alpha * (beta * c_l1.value - gamma * c_lx.value),
            }
        })
    }
    /*
     * apply min SNR mask
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
            debug!(
                "{:?} ({}) clock correction: {}",
                t, self.sv, self.clock_corr
            );
            e_tx -= self.clock_corr;
        }

        if cfg.modeling.sv_total_group_delay {
            if let Some(tgd) = self.tgd {
                debug!("{:?} ({}) tgd   : {}", t, self.sv, tgd);
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
            dt_secs <= 0.1,
            "something's wrong - {} propagation delay is suspicious",
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
        for (pr_observations, phase_observations, code_ppp_compatible) in [
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
            assert_eq!(cd.code_ppp_compatible(), code_ppp_compatible);
        }
    }
}
