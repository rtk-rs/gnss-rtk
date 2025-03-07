//! Position solving candidate
use itertools::Itertools;
use std::cmp::Ordering;

use crate::prelude::{Candidate, Carrier};

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Observation {
    /// [Carrier] frequency.
    pub carrier: Carrier,
    /// Pseudo range observation, expressed in meters.
    pub pseudo_range_m: Option<f64>,
    /// Phase range observatio, expressed in meters.
    pub phase_range_m: Option<f64>,
    /// Possible doppler observation (in Hz/Hz).
    pub doppler: Option<f64>,
    /// Possible SNR indication (in dB/Hz).
    pub snr_dbhz: Option<f64>,
    /// Ambiguity over that phase range.
    /// If ambiguity is already solved (externally), define it here.
    /// Otherwise, ambiguities will be fixed during the navigation process,
    /// if the technique is set to [Method::PPP].
    pub ambiguity: Option<f64>,
}

impl Observation {
    /// Creates new pseudo range [Observation] (in meters), with possible
    /// SNR in dB/Hz.
    pub fn pseudo_range(carrier: Carrier, range_m: f64, snr_dbhz: Option<f64>) -> Self {
        Self {
            snr_dbhz,
            carrier,
            doppler: None,
            ambiguity: None,
            phase_range_m: None,
            pseudo_range_m: Some(range_m),
        }
    }

    /// Creates new ambiguous phase range [Observation] (in meters), with possible
    /// SNR in dB/Hz.
    pub fn ambiguous_phase_range(carrier: Carrier, range_m: f64, snr_dbhz: Option<f64>) -> Self {
        Self {
            snr_dbhz,
            carrier,
            doppler: None,
            ambiguity: None,
            pseudo_range_m: None,
            phase_range_m: Some(range_m),
        }
    }

    /// Creates new (unambiguous) phase range [Observation] (in meters), with possible
    /// SNR in dB/Hz, and ambiguity as fraction of signal propagation cycles.
    pub fn phase_range(
        carrier: Carrier,
        range_m: f64,
        ambiguity: f64,
        snr_dbhz: Option<f64>,
    ) -> Self {
        Self {
            snr_dbhz,
            carrier,
            doppler: None,
            pseudo_range_m: None,
            ambiguity: Some(ambiguity),
            phase_range_m: Some(range_m),
        }
    }

    /// Creates new Doppler [Observation], with possible SNR in dB/Hz.
    pub fn doppler(carrier: Carrier, doppler: f64, snr_dbhz: Option<f64>) -> Self {
        Self {
            snr_dbhz,
            carrier,
            ambiguity: None,
            doppler: Some(doppler),
            pseudo_range_m: None,
            phase_range_m: None,
        }
    }

    /// Copies and returns new [Observation] with defined ambiguous phase range (in meters)
    /// for that frequency.
    pub fn with_ambiguous_phase_range_m(&self, phase_range_m: f64) -> Self {
        let mut s = self.clone();
        s.ambiguity = None;
        s.phase_range_m = Some(phase_range_m);
        s
    }

    /// Copies and returns new [Observation] with defined pseudo range (in meters)
    /// for that frequency.
    pub fn with_pseudo_range_m(&self, pseudo_range_m: f64) -> Self {
        let mut s = self.clone();
        s.pseudo_range_m = Some(pseudo_range_m);
        s
    }

    /// Copies and returns new [Observation] with defined doppler shift (in Hz/Hz),
    /// for that frequency.
    pub fn with_doppler(&self, doppler_hz_hz: f64) -> Self {
        let mut s = self.clone();
        s.doppler = Some(doppler_hz_hz);
        s
    }
}

impl Candidate {
    /// Pseudo range iterator
    pub(crate) fn pseudo_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let pseudo = ob.pseudo_range_m?;
            Some((ob.carrier, pseudo))
        }))
    }

    /// Phase Range iterator
    pub(crate) fn phase_range_iter(&self) -> Box<dyn Iterator<Item = (Carrier, f64)> + '_> {
        Box::new(self.observations.iter().filter_map(|ob| {
            let phase = ob.phase_range_m?;
            Some((ob.carrier, phase))
        }))
    }

    /// Returns signal observation with best SNR value, amongst all frequencies.
    pub(crate) fn best_snr_observation(&self) -> Option<Observation> {
        self.observations
            .iter()
            .max_by(|ob_a, ob_b| {
                if let Some(snr_a) = ob_a.snr_dbhz {
                    if let Some(snr_b) = ob_b.snr_dbhz {
                        snr_a.partial_cmp(&snr_b).unwrap()
                    } else {
                        Ordering::Greater
                    }
                } else {
                    Ordering::Less
                }
            })
            .cloned()
    }

    /// Returns pseudo range (in meters) with best SNR value, among all observed frequencies.
    pub(crate) fn best_snr_pseudo_range_m(&self) -> Option<f64> {
        self.best_snr_observation().map(|ob| ob.pseudo_range_m)?
    }

    /// Returns pseudo range (in meters) with best SNR value, among all observed frequencies.
    pub(crate) fn best_snr_phase_range_m(&self) -> Option<f64> {
        self.best_snr_observation().map(|ob| ob.phase_range_m)?
    }

    /// True if Self is [Method::CPP] compatible
    pub(crate) fn cpp_compatible(&self) -> bool {
        self.has_dual_pseudorange()
    }

    /// True if Self is [Method::PPP] compatible
    pub(crate) fn ppp_compatible(&self) -> bool {
        self.has_dual_pseudorange() && self.has_dual_phase()
    }

    /// True if dual pseudo range measurement is present
    pub(crate) fn has_dual_pseudorange(&self) -> bool {
        self.pseudo_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }

    /// True if dual phase range measurement exist.
    pub(crate) fn has_dual_phase(&self) -> bool {
        self.phase_range_iter()
            .map(|(signal, _)| signal)
            .unique()
            .count()
            > 1
    }

    /// Returns the L1 Pseudo Range observation [m] if it exists
    pub(crate) fn l1_pseudo_range(&self) -> Option<(Carrier, f64)> {
        let l1 = self
            .observations
            .iter()
            .filter(|ob| {
                matches!(
                    ob.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) && ob.pseudo_range_m.is_some()
            })
            .reduce(|k, _| k)?;

        Some((l1.carrier, l1.pseudo_range_m.unwrap()))
    }

    /// Returns the L1 Phase Range observation [m] if it exists
    pub(crate) fn l1_phase_range(&self) -> Option<(Carrier, f64)> {
        let l1 = self
            .observations
            .iter()
            .filter(|ob| {
                matches!(
                    ob.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) && ob.phase_range_m.is_some()
            })
            .reduce(|k, _| k)?;

        Some((l1.carrier, l1.phase_range_m.unwrap()))
    }

    /// Returns the Lj Pseudo Range observation [m] if it exists
    pub(crate) fn lj_pseudo_range(&self) -> Option<(Carrier, f64)> {
        let lj = self
            .observations
            .iter()
            .filter(|ob| {
                !matches!(
                    ob.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) && ob.pseudo_range_m.is_some()
            })
            .reduce(|k, _| k)?;

        Some((lj.carrier, lj.pseudo_range_m.unwrap()))
    }

    /// Returns the Lj Phase Range observation [m] if it exists
    pub(crate) fn lj_phase_range(&self) -> Option<(Carrier, f64)> {
        let lj = self
            .observations
            .iter()
            .filter(|ob| {
                !matches!(
                    ob.carrier,
                    Carrier::L1 | Carrier::E1 | Carrier::B1aB1c | Carrier::B1I
                ) && ob.phase_range_m.is_some()
            })
            .reduce(|k, _| k)?;

        Some((lj.carrier, lj.phase_range_m.unwrap()))
    }

    /// Discards all observations below given SNR mask (>)
    pub(crate) fn min_snr_mask(&mut self, min_snr_dbhz: f64) {
        self.observations.retain(|ob| {
            if let Some(snr_dbhz) = ob.snr_dbhz {
                snr_dbhz > min_snr_dbhz
            } else {
                // no SNR information: we decide to still retain
                // because old or exotic software might not provide SNR information
                // and this would prohibit using the solver
                true
            }
        })
    }
}

#[cfg(test)]
mod test {
    use crate::prelude::{Candidate, Carrier, Epoch, Observation, SV};
    use std::str::FromStr;

    #[test]
    fn gps_l1_observation() {
        let t0 = Epoch::from_str("2000-01-01T00:00:00 UTC").unwrap();

        let g01 = SV::from_str("G01").unwrap();

        let (l1, l2, l5) = (Carrier::L1, Carrier::L2, Carrier::L5);

        let cd = Candidate::new(
            g01,
            t0,
            vec![Observation::pseudo_range(Carrier::L1, 0.1, None)],
        );

        assert_eq!(cd.l1_pseudo_range(), Some((l1, 0.1)));
        assert!(cd.l1_phase_range().is_none());
        assert!(cd.lj_phase_range().is_none());

        let cd = Candidate::new(
            g01,
            t0,
            vec![
                Observation::pseudo_range(l1, 0.1, None),
                Observation::pseudo_range(l2, 0.2, None),
            ],
        );

        assert_eq!(cd.l1_pseudo_range(), Some((l1, 0.1)));
        assert_eq!(cd.lj_pseudo_range(), Some((l2, 0.2)));
        assert!(cd.code_if_combination().is_some());

        assert!(cd.l1_phase_range().is_none());
        assert!(cd.lj_phase_range().is_none());

        let cd = Candidate::new(
            g01,
            t0,
            vec![
                Observation::pseudo_range(l1, 0.1, None),
                Observation::ambiguous_phase_range(l1, 0.2, None),
            ],
        );

        assert_eq!(cd.l1_pseudo_range(), Some((l1, 0.1)));
        assert_eq!(cd.l1_phase_range(), Some((l1, 0.2)));

        assert!(cd.lj_pseudo_range().is_none());
        assert!(cd.lj_phase_range().is_none());

        let cd = Candidate::new(
            g01,
            t0,
            vec![
                Observation::ambiguous_phase_range(l1, 0.1, None),
                Observation::ambiguous_phase_range(l5, 0.5, None),
            ],
        );

        assert_eq!(cd.l1_phase_range(), Some((l1, 0.1)));
        assert_eq!(cd.lj_phase_range(), Some((l5, 0.5)));

        assert!(cd.l1_pseudo_range().is_none());
        assert!(cd.lj_pseudo_range().is_none());

        assert!(cd.phase_if_combination().is_some());
        assert!(cd.code_if_combination().is_none());
    }
}
