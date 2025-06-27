use crate::prelude::{Candidate, Carrier, SPEED_OF_LIGHT_M_S};

/// Signal [Combination]
#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct Combination {
    /// Lhs [Carrier] signal
    pub lhs: Carrier,

    /// Rhs reference [Carrier] signal
    pub rhs: Carrier,

    /// Value
    pub value: f64,
}

impl Combination {
    pub fn new(lhs: Carrier, rhs: Carrier, value: f64) -> Self {
        Self { lhs, rhs, value }
    }
}

impl Candidate {
    /// Returns an IF code range [Combination] using prefered available frequencies
    pub(crate) fn code_if_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_pseudo_range()?;
        let f1_hz = c1.frequency_hz();

        let (cj, lj_pr) = self.subsidary_pseudo_range()?;
        let fj_hz = cj.frequency_hz();

        let alpha = 1.0 / (f1_hz.powi(2) - fj_hz.powi(2));
        let (beta, gamma) = (f1_hz.powi(2), fj_hz.powi(2));

        Some(Combination::new(
            cj,
            c1,
            alpha * (beta * l1_pr - gamma * lj_pr),
        ))
    }

    /// Returns an IF phase range [Combination] using prefered available frequencies
    pub(crate) fn phase_if_combination(&self) -> Option<Combination> {
        let (c1, l1) = self.l1_phase_range()?;
        let f1_hz = c1.frequency_hz();
        let lamb1 = SPEED_OF_LIGHT_M_S / f1_hz;

        let (cj, lj) = self.subsidary_phase_range()?;
        let fj_hz = cj.frequency_hz();
        let lambj = SPEED_OF_LIGHT_M_S / fj_hz;

        let (f1pow, fjpow) = (f1_hz.powi(2), fj_hz.powi(2));

        Some(Combination::new(
            cj,
            c1,
            (f1pow * l1 - fjpow * lj) / (f1pow - fjpow),
        ))
    }

    /// Returns phase wide lane [Combination]
    #[cfg(test)]
    pub(crate) fn phase_wl_combination(&self) -> Option<Combination> {
        let (c1, l1_cp) = self.l1_phase_range()?;
        let f1 = c1.frequency_hz();

        let (cj, lj_cp) = self.subsidary_phase_range()?;
        let fj = cj.frequency_hz();

        Some(Combination::new(
            cj,
            c1,
            (f1 * l1_cp - fj * lj_cp) / (f1 - fj),
        ))
    }

    /// Returns code narrow lane [Combination]
    #[cfg(test)]
    pub(crate) fn code_nl_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_pseudo_range()?;
        let f1 = c1.frequency_hz();

        let (cj, lj_pr) = self.subsidary_pseudo_range()?;
        let fj = cj.frequency_hz();

        Some(Combination::new(
            cj,
            c1,
            (f1 * l1_pr + fj * lj_pr) / (f1 + fj),
        ))
    }

    /// Returns MW [Combination]
    #[cfg(test)]
    pub(crate) fn mw_combination(&self) -> Option<Combination> {
        let ph_w = self.phase_wl_combination()?;
        let pr_n = self.code_nl_combination()?;
        Some(Combination::new(
            ph_w.lhs,
            ph_w.rhs,
            ph_w.value - pr_n.value,
        ))
    }

    /// Form GF [Combination]
    #[cfg(test)]
    pub(crate) fn code_gf_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_pseudo_range()?;
        let (c2, l2_pr) = self.subsidary_pseudo_range()?;
        Some(Combination::new(c2, c1, l2_pr - l1_pr))
    }

    /// Returns GF [Combination]
    #[cfg(test)]
    pub(crate) fn phase_gf_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_phase_range()?;
        let (c2, l2_pr) = self.subsidary_phase_range()?;
        Some(Combination::new(c2, c1, l1_pr - l2_pr))
    }
}
