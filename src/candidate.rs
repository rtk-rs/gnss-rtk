//! Position solving candidate
use crate::prelude::{Carrier, Config, Duration, Epoch, Error, InterpolationResult, Vector3, SV};
use hifitime::Unit;
use itertools::Itertools;
use log::debug;
use nyx::cosmic::SPEED_OF_LIGHT;
use std::cmp::Ordering;

/// Phase range observation to attach to each candidate
#[derive(Debug, Default, PartialEq, Clone)]
pub struct PhaseRange {
    /// Carrier signal
    pub carrier: Carrier,
    /// Measured value
    pub value: f64,
    /// For navigation methods that use PhaseRange like [Method::PPP], phase range ambiguities
    /// need to be fixed at some point, otherwise, you end up with similar performances as
    /// [PseudoRange] based navigation methods.
    /// If you resolved the ambiguities yourself, set this value ahead of time, otherwise we will take care of it.
    pub ambiguity: Option<f64>,
    /// Optional (but recommended) SNR in [dB]
    pub snr: Option<f64>,
}

/// Pseudo range observation to attach to each candidate
#[derive(Debug, Default, PartialEq, Clone)]
pub struct PseudoRange {
    /// Carrier signal
    pub carrier: Carrier,
    /// Measured value
    pub value: f64,
    /// Optional (but recommended) SNR in [dB]
    pub snr: Option<f64>,
}

/// Combination of observations
pub struct PhaseCombination {
    /// LHS signal
    pub lhs: Carrier,
    /// RHS reference signal
    pub reference: Carrier,
    /// Value
    pub value: f64,
    /// Ambiguity
    pub ambiguity: Option<f64>,
}

/// Combination of observations
pub struct PseudoRangeCombination {
    /// LHS signal
    pub lhs: Carrier,
    /// RHS reference signal
    pub reference: Carrier,
    /// Value
    pub value: f64,
}

/// Position solving candidate
#[derive(Debug, Clone)]
pub struct Candidate {
    /// [SV]
    pub sv: SV,
    /// Sampling [Epoch]
    pub t: Epoch,
    /// Tx Epoch
    pub t_tx: Epoch,
    /// State that needs to be resolved
    pub state: Option<InterpolationResult>,
    // SV group delay expressed as a [Duration]
    pub(crate) tgd: Option<Duration>,
    // Windup term in cycles
    pub(crate) wind_up: f64,
    // SV clock correction
    pub(crate) clock_corr: Duration,
    // Pseudo Range observations
    pub(crate) pseudo_range: Vec<PseudoRange>,
    // Phase range observations
    pub(crate) phase_range: Vec<PhaseRange>,
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
    pub fn new(
        sv: SV,
        t: Epoch,
        clock_corr: Duration,
        tgd: Option<Duration>,
        pseudo_range: Vec<PseudoRange>,
        phase_range: Vec<PhaseRange>,
    ) -> Self {
        Self {
            sv,
            t,
            t_tx: t,
            clock_corr,
            tgd,
            pseudo_range,
            phase_range,
            state: None,
            wind_up: 0.0_f64,
        }
    }
    /*
     * Returns best observed SNR, whatever the signal
     */
    pub(crate) fn pseudorange_best_snr(&self) -> Option<f64> {
        self.pseudo_range
            .iter()
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
    pub fn prefered_pseudorange(&self) -> Option<PseudoRange> {
        let mut snr = Option::<f64>::None;
        let mut pr = Option::<PseudoRange>::None;
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
    // Returns the L1 Pseudo Range observation [m] if it exists
    pub(crate) fn l1_pseudorange(&self) -> Option<&PseudoRange> {
        self.pseudo_range
            .iter()
            .filter(|p| {
                matches!(
                    p.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the L1 Phase Range observation [m] if it exists
    pub(crate) fn l1_phaserange(&self) -> Option<&PhaseRange> {
        self.phase_range
            .iter()
            .filter(|p| {
                matches!(
                    p.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the Lj Pseudo Range observation [m] if it exists
    pub(crate) fn lj_pseudorange(&self) -> Option<&PseudoRange> {
        self.pseudo_range
            .iter()
            .filter(|p| {
                !matches!(
                    p.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the Lj Phase Range observation [m] if it exists
    pub(crate) fn lj_phaserange(&self) -> Option<&PhaseRange> {
        self.phase_range
            .iter()
            .filter(|p| {
                !matches!(
                    p.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    /// Returns IF code range combination
    pub fn code_if_combination(&self) -> Option<PseudoRangeCombination> {
        let l1_pr = self.l1_pseudorange()?;
        let (c_l1, l1_signal) = (l1_pr.value, l1_pr.carrier);
        let freq_l1 = l1_signal.frequency();

        let lx_pr = self
            .pseudo_range
            .iter()
            .filter(|p| p.carrier != l1_pr.carrier)
            .reduce(|k, _| k)?;

        let (c_lx, lx_signal) = (lx_pr.value, lx_pr.carrier);
        let freq_lx = lx_signal.frequency();

        let alpha = 1.0 / (freq_l1.powi(2) - freq_lx.powi(2));
        let beta = freq_l1.powi(2);
        let gamma = freq_lx.powi(2);
        Some({
            PseudoRangeCombination {
                lhs: lx_signal,
                reference: l1_signal,
                value: alpha * (beta * c_l1 - gamma * c_lx),
            }
        })
    }
    /// Returns IF phase range combination
    pub fn phase_if_combination(&self) -> Option<PhaseCombination> {
        let l1_ph = self.l1_phaserange()?;
        let (c_l1, l1_signal) = (l1_ph.value, l1_ph.carrier);
        let f_l1 = l1_signal.frequency();

        let lx_ph = self
            .phase_range
            .iter()
            .filter(|p| p.carrier != l1_ph.carrier)
            .reduce(|k, _| k)?;

        let (c_lx, lx_signal) = (lx_ph.value, lx_ph.carrier);
        let f_lx = lx_signal.frequency();

        let alpha = 1.0 / (f_l1.powi(2) - f_lx.powi(2));
        let beta = f_l1.powi(2);
        let gamma = f_lx.powi(2);
        Some(PhaseCombination {
            lhs: lx_signal,
            ambiguity: None,
            reference: l1_signal,
            value: alpha * (beta * c_l1 - gamma * c_lx),
        })
    }
    /// Returns phase wide lane combination
    pub(crate) fn phase_wl_combination(&self) -> Option<PhaseCombination> {
        let l_1 = self.l1_phaserange()?;
        let l_j = self
            .phase_range
            .iter()
            .filter(|p| p.carrier != l_1.carrier)
            .reduce(|k, _| k)?;

        let f_1 = l_1.carrier.frequency();
        let f_j = l_j.carrier.frequency();
        Some(PhaseCombination {
            lhs: l_j.carrier,
            ambiguity: None,
            reference: l_1.carrier,
            value: (f_1 * l_1.value - f_j * l_j.value) / (f_1 - f_j),
        })
    }
    /// Returns code narrow lane combination
    pub(crate) fn code_nl_combination(&self) -> Option<PseudoRangeCombination> {
        let c_1 = self.l1_pseudorange()?;
        let c_j = self
            .pseudo_range
            .iter()
            .filter(|p| p.carrier != c_1.carrier)
            .reduce(|k, _| k)?;

        let f_1 = c_1.carrier.frequency();
        let f_j = c_j.carrier.frequency();

        Some(PseudoRangeCombination {
            lhs: c_j.carrier,
            reference: c_1.carrier,
            value: (f_1 * c_1.value + f_j * c_j.value) / (f_1 + f_j),
        })
    }
    pub(crate) fn mw_combination(&self) -> Option<PhaseCombination> {
        let pw = self.phase_wl_combination()?;
        let cn = self.code_nl_combination()?;
        Some(PhaseCombination {
            lhs: pw.lhs,
            ambiguity: None,
            reference: pw.reference,
            value: pw.value - cn.value,
        })
    }
    // Form GF combination
    pub(crate) fn phase_gf_combination(&self) -> Option<PhaseCombination> {
        let c_1 = self
            .phase_range
            .iter()
            .filter(|p| matches!(p.carrier, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let c_j = self
            .phase_range
            .iter()
            .filter(|p| p.carrier != c_1.carrier)
            .reduce(|k, _| k)?;

        Some(PhaseCombination {
            lhs: c_j.carrier,
            reference: c_1.carrier,
            ambiguity: None,
            value: c_1.value - c_j.value,
        })
    }
    // Form GF combination
    pub(crate) fn code_gf_combination(&self) -> Option<PseudoRangeCombination> {
        let c_1 = self
            .pseudo_range
            .iter()
            .filter(|p| matches!(p.carrier, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let c_j = self
            .phase_range
            .iter()
            .filter(|p| p.carrier != c_1.carrier)
            .reduce(|k, _| k)?;

        Some(PseudoRangeCombination {
            lhs: c_j.carrier,
            reference: c_1.carrier,
            value: c_j.value - c_1.value,
        })
    }
    // Computes phase windup term. Self should be fully resolved, otherwse
    // will panic.
    pub(crate) fn windup_correction(&mut self, ref_enu: Vector3<f64>, sun: Vector3<f64>) -> f64 {
        0.0
        // let state = self.state.unwrap();
        // let r_sv = state.to_ecef();

        // let norm = (
        //     (sun[0] - r_sv[0]).powi(2)
        //     + (sun[1] - r_sv[1]).powi(2)
        //     + (sun[2] - r_sv[2]).powi(2)
        // ).sqrt();

        // let e = (r_sun - r_sv_mc ) / norm;
        // let j = k.cross(e);
        // let i = j.cross(k);

        // let d_prime_norm = d_prime.norm();
        // let d_norm = d.norm();
        // let psi = pho * (d_prime.cross(d));
        // let dphi = d_prime.dot(d) / d_prime.norm() / d.norm();

        // let n = (self.delta_phi.unwrap_or(0.0) / 2.0 / PI).round();
        // self.delta_phi = dphi + 2.0 * n;

        // self.delta_phi
        // self.wind_up =
    }
    // Retains only observations with SNR >= min_snr
    pub(crate) fn min_snr_mask(&mut self, min_snr: f64) {
        self.pseudo_range.retain(|c| {
            if let Some(snr) = c.snr {
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
        // self.doppler.retain(|d| {
        //     if let Some(snr) = d.snr {
        //         snr >= min_snr
        //     } else {
        //         false
        //     }
        // });
    }
    ///
    /// Computes signal transmission time, expressed as [Epoch]
    /// - returns (t_tx, dt_ttx)
    /// - "t_tx": Epoch in given timescale
    /// - "dt_ttx": elapsed duration in seconds in given timescale
    pub(crate) fn transmission_time(&self, cfg: &Config) -> Result<(Epoch, Duration), Error> {
        let (t, ts) = (self.t, self.t.time_scale);
        let seconds_ts = t.to_duration_in_time_scale(t.time_scale).to_seconds();

        // TODO: prefere (when possible) resolved phase range here ?
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
    #[cfg(test)]
    pub fn set_state(&mut self, state: InterpolationResult) {
        self.state = Some(state);
    }
}

#[cfg(test)]
mod test {
    use super::{PhaseCombination, PseudoRangeCombination};
    use crate::prelude::{Candidate, Carrier, Duration, Epoch, PhaseRange, PseudoRange, SV};
    #[test]
    fn cpp_compatibility() {
        for (pr_observations, phase_observations, cpp_compatible) in [
            (
                vec![PseudoRange {
                    value: 1.0,
                    snr: Some(1.0),
                    carrier: Carrier::L1,
                }],
                vec![],
                false,
            ),
            (
                vec![
                    PseudoRange {
                        value: 1.0,
                        snr: Some(1.0),
                        carrier: Carrier::L1,
                    },
                    PseudoRange {
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
            );
            assert_eq!(cd.cpp_compatible(), cpp_compatible);
        }
    }
}
