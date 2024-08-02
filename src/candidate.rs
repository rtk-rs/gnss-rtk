//! Position solving candidate
use crate::prelude::{Carrier, Config, Duration, Epoch, Error, OrbitalState, Vector3, SV};
use hifitime::Unit;
use itertools::Itertools;
use log::debug;
use nyx::cosmic::SPEED_OF_LIGHT_M_S;
use std::cmp::Ordering;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Observation {
    /// [Carrier]
    pub carrier: Carrier,
    /// Pseudo Range observation in [m]
    pub pseudo: Option<f64>,
    /// Phase Range observation in [m]
    pub phase: Option<f64>,
    /// Possible doppler observation
    pub doppler: Option<f64>,
    /// SNR
    pub snr: Option<f64>,
    /// For navigation methods that use phase range like [Method::PPP], phase range ambiguities
    /// need to be fixed at some point, otherwise, you end up with similar performances as
    /// [PseudoRange] based navigation methods.
    /// If you resolved the ambiguities yourself, set this value ahead of time, otherwise we will take care of it.
    pub ambiguity: Option<f64>,
}

impl Observation {
    /// Set [Carrier]
    pub fn set_carrier(&mut self, c: Carrier) {
        self.carrier = c;
    }
    /// Creates [Self] with given phase range [m] observation
    pub fn with_phase_range(&self, ph: f64) -> Self {
        let mut s = self.clone();
        s.phase = Some(ph);
        s
    }
    /// Creates [Self] with given pseudo range [m] observation
    pub fn with_pseudo_range(&self, pr: f64) -> Self {
        let mut s = self.clone();
        s.pseudo = Some(pr);
        s
    }
    /// Creates [Self] with doppler observation
    pub fn with_doppler(&self, dop: f64) -> Self {
        let mut s = self.clone();
        s.doppler = Some(dop);
        s
    }
}

/// Signal combination
#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct Combination {
    /// Lhs signal
    pub lhs: Carrier,
    /// Rhs reference signal
    pub rhs: Carrier,
    /// Value
    pub value: f64,
    /// Phase combination ambiguity
    pub ambiguity: Option<f64>,
}

impl Combination {
    pub fn new(lhs: Carrier, rhs: Carrier, value: f64) -> Self {
        Self {
            lhs,
            rhs,
            value,
            ambiguity: None,
        }
    }
    pub fn with_ambiguity(&self, ambiguity: f64) -> Self {
        let mut s = self.clone();
        s.ambiguity = Some(ambiguity);
        s
    }
}

/// Position solving candidate
#[derive(Debug, Clone)]
pub struct Candidate {
    /// [SV]
    pub sv: SV,
    /// Sampling [Epoch]
    pub t: Epoch,
    /// State that needs to be resolved
    pub state: Option<OrbitalState>,
    /// t_tx Epoch
    pub(crate) t_tx: Epoch,
    // SV group delay expressed as a [Duration]
    pub(crate) tgd: Option<Duration>,
    // Windup term in cycles
    pub(crate) wind_up: f64,
    // SV clock correction
    pub(crate) clock_corr: Duration,
    // Observations
    pub(crate) remote: Vec<Observation>,
    // Observations
    pub(crate) observations: Vec<Observation>,
}

// public
impl Candidate {
    /// Creates a new candidate, to inject in the solver pool.
    /// ## Inputs
    /// - sv: [SV] Identity
    /// - t: sampling [Epoch]
    /// - clock_corr: SV onboard clock correction (mandatory)
    /// - tgd: possible onboard group delay
    /// - observations: provide signals observations.
    ///   You have to provide observations that match your navigation method.
    pub fn new(
        sv: SV,
        t: Epoch,
        clock_corr: Duration,
        tgd: Option<Duration>,
        observations: Vec<Observation>,
    ) -> Self {
        Self {
            sv,
            t,
            t_tx: t,
            clock_corr,
            tgd,
            state: None,
            observations,
            wind_up: 0.0_f64,
            remote: Vec::new(),
        }
    }
    /// Provide remote [Observation]s observed by [BaseStation]
    pub(crate) fn set_remote_observations(&mut self, remote: Vec<Observation>) {
        self.remote = remote.clone();
    }
    /// Add one [Observation] observed on [BaseStation]
    pub(crate) fn add_remote(&mut self, remote: Observation) {
        self.remote.push(remote);
    }
}

// private
impl Candidate {
    // Pseudo range iterator
    fn pseudo_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let pseudo = ob.pseudo?;
            Some((ob.carrier, pseudo))
        }))
    }
    // Phase Range iterator
    fn phase_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let phase = ob.phase?;
            Some((ob.carrier, phase))
        }))
    }
    /*
     * Returns best observed SNR, whatever the signal
     */
    pub(crate) fn pseudorange_best_snr(&self) -> Option<f64> {
        self.observations
            .iter()
            .max_by(|ob_a, ob_b| {
                if let Some(snr_a) = ob_a.snr {
                    if let Some(snr_b) = ob_b.snr {
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
    /// Returns one pseudo range observation [m], whatever the frequency.
    pub(crate) fn prefered_pseudorange(&self) -> Option<Observation> {
        if let Some(c1) = self
            .observations
            .iter()
            .filter(|ob| {
                matches!(
                    ob.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) && ob.pseudo.is_some()
            })
            .reduce(|k, _| k)
        {
            Some(c1.clone())
        } else {
            self.observations
                .iter()
                .filter(|ob| {
                    ob.pseudo.is_some()
                        && !matches!(
                            ob.carrier,
                            Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                        )
                })
                .reduce(|k, _| k)
                .cloned()
        }
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
        self.pseudo_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }
    // True if dual phase is present
    pub(crate) fn dual_phase(&self) -> bool {
        self.phase_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }
    // Returns the L1 Pseudo Range observation [m] if it exists
    pub(crate) fn l1_pseudorange(&self) -> Option<(Carrier, f64)> {
        self.pseudo_range_iter()
            .filter(|(signal, _)| {
                matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the L1 Phase Range observation [m] if it exists
    pub(crate) fn l1_phaserange(&self) -> Option<(Carrier, f64)> {
        self.phase_range_iter()
            .filter(|(signal, _)| {
                matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the Lj Pseudo Range observation [m] if it exists
    pub(crate) fn lj_pseudorange(&self) -> Option<(Carrier, f64)> {
        self.pseudo_range_iter()
            .filter(|(signal, _)| {
                !matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    // Returns the Lj Phase Range observation [m] if it exists
    pub(crate) fn lj_phaserange(&self) -> Option<(Carrier, f64)> {
        self.phase_range_iter()
            .filter(|(signal, _)| {
                !matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                )
            })
            .reduce(|k, _| k)
    }
    /// Returns IF code range combination
    pub(crate) fn code_if_combination(&self) -> Option<Combination> {
        let (c_l1, l1_pr) = self.l1_pseudorange()?;
        let freq_l1 = c_l1.frequency();

        let (c_lx, lx_pr) = self
            .pseudo_range_iter()
            .filter(|(c, _)| *c != c_l1)
            .reduce(|k, _| k)?;

        let freq_lx = c_lx.frequency();

        let alpha = 1.0 / (freq_l1.powi(2) - freq_lx.powi(2));
        let beta = freq_l1.powi(2);
        let gamma = freq_lx.powi(2);
        Some(Combination::new(
            c_lx,
            c_l1,
            alpha * (beta * l1_pr - gamma * lx_pr),
        ))
    }
    /// Returns IF phase range combination
    pub(crate) fn phase_if_combination(&self) -> Option<Combination> {
        let (c_1, l1_ph) = self.l1_phaserange()?;
        let f_l1 = c_1.frequency();

        let (c_lx, lx_ph) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        let f_lx = c_lx.frequency();

        let alpha = 1.0 / (f_l1.powi(2) - f_lx.powi(2));
        let beta = f_l1.powi(2);
        let gamma = f_lx.powi(2);
        Some(Combination::new(
            c_lx,
            c_1,
            alpha * (beta * l1_ph - gamma * lx_ph),
        ))
    }
    /// Returns phase wide lane combination
    pub(crate) fn phase_wl_combination(&self) -> Option<Combination> {
        let (c_1, l_1) = self.l1_phaserange()?;
        let (c_j, l_j) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        let (f_1, f_j) = (c_1.frequency(), c_j.frequency());
        Some(Combination::new(
            c_j,
            c_1,
            (f_1 * l_1 - f_j * l_j) / (f_1 - f_j),
        ))
    }
    /// Returns code narrow lane combination
    pub(crate) fn code_nl_combination(&self) -> Option<Combination> {
        let (c_1, l_1) = self.l1_pseudorange()?;
        let (c_j, l_j) = self
            .pseudo_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        let (f_1, f_j) = (c_1.frequency(), c_j.frequency());

        Some(Combination::new(
            c_j,
            c_1,
            (f_1 * l_1 + f_j * l_j) / (f_1 + f_j),
        ))
    }
    pub(crate) fn mw_combination(&self) -> Option<Combination> {
        let ph_w = self.phase_wl_combination()?;
        let pr_n = self.code_nl_combination()?;
        Some(Combination::new(
            ph_w.lhs,
            ph_w.rhs,
            ph_w.value - pr_n.value,
        ))
    }
    // Form GF combination
    pub(crate) fn phase_gf_combination(&self) -> Option<Combination> {
        let (c_1, l_1) = self
            .phase_range_iter()
            .filter(|(c, _)| matches!(c, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let (c_j, l_j) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        Some(Combination::new(c_j, c_1, l_1 - l_j))
    }
    // Form GF combination
    pub(crate) fn code_gf_combination(&self) -> Option<Combination> {
        let (c_1, pr_1) = self
            .pseudo_range_iter()
            .filter(|(c, _)| matches!(c, Carrier::L1 | Carrier::E1 | Carrier::B1aB1c))
            .reduce(|k, _| k)?;

        let (c_j, pr_j) = self
            .phase_range_iter()
            .filter(|(c, _)| *c != c_1)
            .reduce(|k, _| k)?;

        Some(Combination::new(c_j, c_1, pr_j - pr_1))
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
        self.observations.retain(|ob| {
            if let Some(snr) = ob.snr {
                snr >= min_snr
            } else {
                // no SNR information: we decide to still retain
                // because old or exotic software might not provide SNR information
                // and this would prohibit using the solver
                true
            }
        })
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
                .pseudo
                .unwrap()
                / SPEED_OF_LIGHT_M_S;

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
            dt_secs.is_positive(),
            "Physical non sense - RX {:?} prior TX {:?}",
            t,
            e_tx
        );
        assert!(
            dt_secs <= 0.2,
            "{}({}): {} Space/Earth propagation delay is unrealistic: invalid input",
            t,
            self.sv,
            dt
        );
        Ok((e_tx, dt))
    }
    #[cfg(test)]
    pub fn set_state(&mut self, state: OrbitalState) {
        self.state = Some(state);
    }
}

#[cfg(test)]
mod test {
    use super::Combination;
    use crate::prelude::{Candidate, Carrier, Duration, Epoch, Observation, SV};
    #[test]
    fn cpp_compatibility() {
        for (observations, cpp_compatible) in [(
            vec![
                Observation {
                    snr: Some(1.0),
                    pseudo: Some(1.0),
                    phase: Some(2.0),
                    ambiguity: None,
                    doppler: None,
                    carrier: Carrier::L1,
                },
                Observation {
                    snr: Some(1.0),
                    pseudo: Some(2.0),
                    phase: Some(2.0),
                    ambiguity: None,
                    doppler: None,
                    carrier: Carrier::L5,
                },
            ],
            true,
        )] {
            let cd = Candidate::new(
                SV::default(),
                Epoch::default(),
                Duration::default(),
                None,
                observations,
            );
            assert_eq!(cd.cpp_compatible(), cpp_compatible);
        }
    }
}
