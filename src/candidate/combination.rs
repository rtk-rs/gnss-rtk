use crate::prelude::{Candidate, Carrier};

/// Signal [Combination]
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

impl Candidate {
    /// Returns IF code range [Combination]
    pub(crate) fn code_if_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_pseudo_range()?;
        let f1 = c1.frequency();

        let (cj, lj_pr) = self.lj_pseudo_range()?;
        let fj = cj.frequency();

        let alpha = 1.0 / (f1.powi(2) - fj.powi(2));
        let beta = f1.powi(2);
        let gamma = fj.powi(2);

        Some(Combination::new(
            cj,
            c1,
            alpha * (beta * l1_pr - gamma * lj_pr),
        ))
    }

    /// Returns IF phase range [Combination]
    pub(crate) fn phase_if_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_phase_range()?;
        let f1 = c1.frequency();

        let (cj, lj_pr) = self.lj_phase_range()?;
        let fj = cj.frequency();

        let alpha = 1.0 / (f1.powi(2) - fj.powi(2));
        let beta = f1.powi(2);
        let gamma = fj.powi(2);

        Some(Combination::new(
            cj,
            c1,
            alpha * (beta * l1_pr - gamma * lj_pr),
        ))
    }

    /// Returns phase wide lane [Combination]
    pub(crate) fn phase_wl_combination(&self) -> Option<Combination> {
        let (c1, l1_cp) = self.l1_phase_range()?;
        let f1 = c1.frequency();

        let (cj, lj_cp) = self.lj_phase_range()?;
        let fj = cj.frequency();

        Some(Combination::new(
            cj,
            c1,
            (f1 * l1_cp - fj * lj_cp) / (f1 - fj),
        ))
    }

    /// Returns code narrow lane [Combination]
    pub(crate) fn code_nl_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_pseudo_range()?;
        let f1 = c1.frequency();

        let (cj, lj_pr) = self.lj_pseudo_range()?;
        let fj = cj.frequency();

        Some(Combination::new(
            cj,
            c1,
            (f1 * l1_pr + fj * lj_pr) / (f1 + fj),
        ))
    }

    /// Returns MW [Combination]
    pub(crate) fn mw_combination(&self) -> Option<Combination> {
        let ph_w = self.phase_wl_combination()?;
        let pr_n = self.code_nl_combination()?;
        Some(Combination::new(
            ph_w.lhs,
            ph_w.rhs,
            ph_w.value - pr_n.value,
        ))
    }

    /// Returns GF [Combination]
    pub(crate) fn phase_gf_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_phase_range()?;
        let (cj, lj_pr) = self.lj_phase_range()?;
        Some(Combination::new(cj, c1, l1_pr - lj_pr))
    }

    /// Form GF combination
    pub(crate) fn code_gf_combination(&self) -> Option<Combination> {
        let (c1, l1_pr) = self.l1_pseudo_range()?;
        let (cj, lj_pr) = self.lj_pseudo_range()?;
        Some(Combination::new(cj, c1, lj_pr - l1_pr))
    }
}
