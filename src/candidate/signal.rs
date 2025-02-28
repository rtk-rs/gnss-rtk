//! Position solving candidate
use itertools::Itertools;
use std::cmp::Ordering;

use crate::prelude::{Candidate, Carrier};

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
    /// Creates new Pseudo Range [Observation] from given
    /// raw measurement (in meters), and possible other information.
    pub fn pseudo_range(carrier: Carrier, range_m: f64, snr: Option<f64>) -> Self {
        Self {
            snr,
            carrier,
            phase: None,
            doppler: None,
            ambiguity: None,
            pseudo: Some(range_m),
        }
    }

    /// Creates new ambiguous Phase Range [Observation] from given
    /// raw measurement (in meters, not cycles), and possible other information.
    pub fn ambiguous_phase_range(carrier: Carrier, range_m: f64, snr: Option<f64>) -> Self {
        Self {
            snr,
            carrier,
            pseudo: None,
            doppler: None,
            ambiguity: None,
            phase: Some(range_m),
        }
    }

    /// Creates new (unambiguous) Phase Range [Observation] from given
    /// raw measurement (in meters, not cycles), ambiguity (as cycle fraction), and possible other information.
    pub fn phase_range(carrier: Carrier, range_m: f64, ambiguity: f64, snr: Option<f64>) -> Self {
        Self {
            snr,
            carrier,
            pseudo: None,
            doppler: None,
            phase: Some(range_m),
            ambiguity: Some(ambiguity),
        }
    }

    /// Creates new Doppler [Observation]
    pub fn doppler(carrier: Carrier, doppler: f64, snr: Option<f64>) -> Self {
        Self {
            snr,
            carrier,
            pseudo: None,
            phase: None,
            ambiguity: None,
            doppler: Some(doppler),
        }
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

impl Candidate {
    /// Pseudo range iterator
    pub(crate) fn pseudo_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let pseudo = ob.pseudo?;
            Some((ob.carrier, pseudo))
        }))
    }

    /// Phase Range iterator
    pub(crate) fn phase_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let phase = ob.phase?;
            Some((ob.carrier, phase))
        }))
    }

    /// Returns best SNR value, amongst all observations
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

    /// Returns the L1 pseudo range [Observation].
    pub(crate) fn l1_pseudorange_observation(&self) -> Option<&Observation> {
        self.observations
            .iter()
            .filter(|ob| {
                matches!(
                    ob.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) && ob.pseudo.is_some()
            })
            .reduce(|k, _| k)
    }

    /// True if Self is Method::CPP compatible
    pub(crate) fn cpp_compatible(&self) -> bool {
        self.has_dual_pseudorange()
    }

    /// True if Self is Method::PPP compatible
    pub(crate) fn ppp_compatible(&self) -> bool {
        self.has_dual_pseudorange() && self.has_dual_phase()
    }

    /// True if dual PR is present
    pub(crate) fn has_dual_pseudorange(&self) -> bool {
        self.pseudo_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }

    /// True if dual phase is present
    pub(crate) fn has_dual_phase(&self) -> bool {
        self.phase_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }

    /// Returns the L1 Pseudo Range observation [m] if it exists
    pub(crate) fn l1_pseudorange_m_freq_hz(&self) -> Option<(f64, f64)> {
        self.pseudo_range_iter()
            .filter_map(|(signal, pr)| {
                if matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) {
                    Some((pr, signal.frequency()))
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
    }

    /// Returns the L1 Phase Range observation [m] if it exists
    pub(crate) fn l1_phase_range_m(&self) -> Option<f64> {
        let (pr, _) = self.l1_pseudorange_m_freq_hz()?;
        Some(pr)
    }

    /// Returns the Lj Pseudo Range observation [m] if it exists
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

    /// Returns the Lj Pseudo Range observation if it exists
    pub(crate) fn lj_pseudorange_m_freq_hz(&self) -> Option<(f64, f64)> {
        self.pseudo_range_iter()
            .filter_map(|(signal, pr)| {
                if !matches!(
                    signal,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) {
                    Some((pr, signal.frequency()))
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
    }

    /// Returns the L1 Phase Range observation [m] if it exists
    pub(crate) fn lj_phase_range_m(&self) -> Option<f64> {
        let (pr, _) = self.lj_pseudorange_m_freq_hz()?;
        Some(pr)
    }

    /// Discards all observations below given SNR mask (>)
    pub(crate) fn min_snr_mask(&mut self, min_snr: f64) {
        self.observations.retain(|ob| {
            if let Some(snr) = ob.snr {
                snr > min_snr
            } else {
                // no SNR information: we decide to still retain
                // because old or exotic software might not provide SNR information
                // and this would prohibit using the solver
                true
            }
        })
    }
}
