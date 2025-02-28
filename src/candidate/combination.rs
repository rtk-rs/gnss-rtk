use crate::prelude::{Candidate, Carrier};

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

impl Candidate {
    /// Returns IF code range combination
    pub(crate) fn code_if_combination(&self) -> Option<Combination> {
        let (f1, c_l1) = self.l1_pseudorange_m_freq_hz()?;
        let (fx, c_lx) = self.lj_pseudorange_m_freq_hz()?;
        
        let alpha = 1.0 / (f1.powi(2) - fx.powi(2));
        let beta = f1.powi(2);
        let gamma = fx.powi(2);
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

    /// Returns MW combination
    pub(crate) fn mw_combination(&self) -> Option<Combination> {
        let ph_w = self.phase_wl_combination()?;
        let pr_n = self.code_nl_combination()?;
        Some(Combination::new(
            ph_w.lhs,
            ph_w.rhs,
            ph_w.value - pr_n.value,
        ))
    }

    /// Returns GF combination
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

    /// Form GF combination
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
}
