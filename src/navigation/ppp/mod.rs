use log::{debug, error};

#[cfg(doc)]
use crate::prelude::TimeScale;

mod kalman;
mod lambda;
pub mod state;

use nalgebra::{allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, U1, U4, U8};

use crate::{
    navigation::{
        dop::DilutionOfPrecision,
        ppp::{
            kalman::{Kalman, KfEstimate},
            lambda::LambdaAR,
            state::PPPState,
        },
        state::State,
        Navigation, SVContribution,
    },
    prelude::{Bias, Candidate, Config, Duration, Epoch, Error, Frame, Method, SV},
    rtk::RTKBase,
    user::UserParameters,
};

use std::collections::HashMap;

/// Specific prefit solver
pub struct PrefitSolver {
    /// [Config] preset
    cfg: Config,

    frame: Frame,

    /// Y allocation
    y_vec: Vec<f64>,

    /// W_diag
    w_diag: Vec<f64>,

    /// W_mat
    w_mat: DMatrix<f64>,

    /// F_diag
    f_diag: Vec<f64>,

    /// F_mat
    f_mat: DMatrix<f64>,

    /// G_mat
    g_mat: DMatrix<f64>,

    /// X
    x_vec: DVector<f64>,

    /// P
    p_mat: DMatrix<f64>,

    /// indexes storage
    sv_indexes: Vec<(SV, usize)>,

    /// [Kalman]
    kalman: Kalman,

    /// Lambda X
    lambda_x: DMatrix<f64>,

    /// Lambda Q
    lambda_q: DMatrix<f64>,

    /// [Lambda]
    lambda: LambdaAR,

    /// n_amb
    n_amb: HashMap<SV, u64>,

    /// True if this filter has been initialized
    pub initialized: bool,

    /// [DilutionOfPrecision]
    pub dop: DilutionOfPrecision,

    /// current [PPPState]
    pub state: PPPState,

    /// Null of first iter
    prev_epoch: Option<Epoch>,

    /// SV contributions
    sv: Vec<SVContribution>,
}

impl PrefitSolver {
    /// Creates new [PrefitSolver].
    /// ## Input
    /// - cfg: [Config] preset
    /// - apriori: [Apriori] input
    /// - candidates: selected [Candidate]s
    /// - size: number of proposal
    /// - bias: [Bias] model implementation
    /// ## Returns
    /// - [PPPPrefitSolver]
    pub fn new<D: DimName>(cfg: &Config, initial_state: &State<D>, frame: Frame) -> Self
    where
        DefaultAllocator: Allocator<D> + Allocator<D, D>,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
        <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
    {
        Self {
            frame,
            cfg: cfg.clone(),
            prev_epoch: None,
            initialized: false,
            kalman: Kalman::new(U8::USIZE),
            lambda: LambdaAR::default(),
            y_vec: Vec::with_capacity(U8::USIZE),
            w_diag: Vec::with_capacity(U8::USIZE),
            f_diag: Vec::with_capacity(U8::USIZE),
            sv_indexes: Vec::with_capacity(U8::USIZE),
            x_vec: DVector::<f64>::zeros(U8::USIZE),
            sv: Vec::with_capacity(4),
            dop: DilutionOfPrecision::default(),
            state: PPPState::from_initial_state(initial_state),
            lambda_x: DMatrix::<f64>::zeros(U4::USIZE, U1::USIZE),
            lambda_q: DMatrix::<f64>::zeros(U4::USIZE, U4::USIZE),
            w_mat: DMatrix::<f64>::zeros(U4::USIZE, U4::USIZE),
            g_mat: DMatrix::<f64>::zeros(U4::USIZE, U4::USIZE),
            f_mat: DMatrix::<f64>::zeros(U4::USIZE, U4::USIZE),
            p_mat: DMatrix::<f64>::zeros(U4::USIZE, U4::USIZE),
            n_amb: HashMap::with_capacity(U4::USIZE),
        }
    }

    /// Reset filter.
    pub fn reset(&mut self) {
        self.clear();
        self.n_amb.clear();

        self.sv.clear();
        self.kalman.reset();
        self.prev_epoch = None;
        self.initialized = false;
    }

    /// Iterates mutable [Navigation] filter.
    /// ## Input
    /// - t: sampling [Epoch]
    /// - params: [UserParameters]
    /// - candidates: proposed [Candidate]s
    /// - size: number of proposed [Cadndidate]s
    /// - rtk_base: possible [RTKBase] implementation]
    /// - bias: external [Bias] implementation
    pub fn run<B: Bias, R: RTKBase>(
        &mut self,
        t: Epoch,
        params: UserParameters,
        candidates: &[Candidate],
        size: usize,
        rtk_base: &R,
        bias: &B,
    ) -> Result<(), Error> {
        self.clear();

        if !self.kalman.initialized {
            self.kf_initialization(t, candidates, params, size, rtk_base, bias)?;
        } else {
            self.kf_run(t, candidates, params, size, rtk_base, bias)?;
        }

        self.prev_epoch = Some(t);
        self.initialized = true;

        Ok(())
    }

    /// Resets [Self] before a new run
    fn clear(&mut self) {
        self.sv.clear();
        self.sv_indexes.clear();

        self.y_vec.clear();
        self.w_diag.clear();
        self.f_diag.clear();

        self.w_mat = DMatrix::<f64>::zeros(U8::USIZE, U8::USIZE);
        self.f_mat = DMatrix::<f64>::zeros(U8::USIZE, U8::USIZE);
    }

    /// Arming internal core.
    pub fn kf_initialization<B: Bias, RTK: RTKBase>(
        &mut self,
        t: Epoch,
        candidates: &[Candidate],
        params: UserParameters,
        size: usize,
        _: &RTK,
        bias: &B,
    ) -> Result<(), Error> {
        const NB_ITER: usize = 10;

        let mut pending = self.state.clone();
        let mut dop = DilutionOfPrecision::default();

        // measurements
        for i in 0..size {
            let mut contrib = SVContribution::default();

            let position_m = pending.position_ecef_m();

            match candidates[i].vector_contribution(
                t,
                &self.cfg,
                true,
                None,
                position_m,
                pending.lat_long_alt_deg_deg_km,
                &mut contrib,
                bias,
            ) {
                Ok(vec) => {
                    self.y_vec.push(vec.row1);
                    self.y_vec.push(vec.row2);

                    self.w_diag.push(1.0); // TODO: improve model
                    self.sv.push(contrib);
                    self.sv_indexes.push((candidates[i].sv, i));

                    if self.cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativistic path range: {:.3}m",
                            t, candidates[i].sv, vec.dr
                        );
                    }
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let nrows = self.y_vec.len();

        // verifications prior moving forward
        if nrows < U8::USIZE {
            // maths limitations: do not propose
            return Err(Error::MatrixMinimalDimension);
        }

        // run
        for ith in 0..NB_ITER {
            let nrows = self.y_vec.len();
            let lambda_ndf = nrows / 2;
            let ndf = U4::USIZE + lambda_ndf;

            // form W
            self.w_mat.resize_mut(nrows, nrows, 0.0);

            for i in 0..nrows {
                self.w_mat[(i, i)] = 1.0; // TODO: improve model
            }

            // form G
            self.g_mat.resize_mut(nrows, ndf, 0.0);

            for (i, (_sv, index)) in self.sv_indexes.iter().enumerate() {
                let dr_i = self.sv[i].relativistic_path_range_m;

                let position_m = pending.position_ecef_m();

                let (dx, dy, dz) =
                    candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

                self.g_mat[(2 * i, 0)] = dx;
                self.g_mat[(2 * i, 1)] = dy;
                self.g_mat[(2 * i, 2)] = dz;
                self.g_mat[(2 * i, Navigation::<U4>::clock_index())] = 1.0;

                self.g_mat[(2 * i + 1, 0)] = dx;
                self.g_mat[(2 * i + 1, 1)] = dy;
                self.g_mat[(2 * i + 1, 2)] = dz;
                self.g_mat[(2 * i + 1, Navigation::<U4>::clock_index())] = 1.0;
                self.g_mat[(2 * i + 1, Navigation::<U4>::clock_index() + 1 + i)] = 1.0;
            }

            debug!("(i={}) G(ppp): {} W(ppp): {}", ith, self.g_mat, self.w_mat);

            // run
            let gt = self.g_mat.transpose();
            let gt_g = gt.clone() * self.g_mat.clone();
            let gt_w = gt.clone() * self.w_mat.clone();

            let gt_w_g = gt_w * self.g_mat.clone();
            let gt_w_g_inv = gt_w_g.try_inverse().ok_or(Error::MatrixInversion)?;
            let gt_w_g_inv_gt = gt_w_g_inv.clone() * gt.clone();
            let gt_w_g_inv_gt_w = gt_w_g_inv_gt * self.w_mat.clone();

            // TODO: improve malloc
            let y = DVector::<f64>::from_row_slice(&self.y_vec);
            debug!("(i={}) Y(ppp): {}", ith, y);

            self.x_vec = gt_w_g_inv_gt_w * y;
            self.p_mat = gt_w_g_inv.clone();

            debug!("(i={}) dx(ppp)={}", ith, self.x_vec);

            pending
                .correct_mut(self.frame, t, &self.x_vec, ndf)
                .map_err(|e| {
                    error!("{} - state update failed with physical error: {}", t, e);
                    Error::StateUpdate
                })?;

            let gt_g_inv = gt_g.try_inverse().ok_or(Error::MatrixInversion)?;

            // DoP update
            dop = DilutionOfPrecision::from_ppp(&pending, gt_g_inv);

            debug!("(i={}) {} - PPP pending state {}", ith, t, pending);

            // update models
            self.y_vec.clear();
            self.w_diag.clear();

            self.sv_indexes.retain(|(_, i)| {
                let mut unused = SVContribution::default();

                let position_m = pending.position_ecef_m();

                match candidates[*i].vector_contribution(
                    t,
                    &self.cfg,
                    true,
                    None,
                    position_m,
                    pending.lat_long_alt_deg_deg_km,
                    &mut unused,
                    bias,
                ) {
                    Ok(vec) => {
                        self.y_vec.push(vec.row1);
                        self.y_vec.push(vec.row2);

                        self.w_diag.push(1.0); // TODO: improve model
                        true
                    },
                    Err(e) => {
                        error!("{}({}) - cannot contribute: {}", t, candidates[*i].sv, e);
                        false
                    },
                }
            });
        }

        // validation
        self.state_validation(&dop)?;

        debug!("dx(ppp)={}", self.x_vec);

        let initial_estimate = KfEstimate::new(&self.x_vec, &self.p_mat);

        let ndf = U4::USIZE;
        let nrows = self.x_vec.nrows();
        let lambda_ndf = nrows - ndf;

        let mut q_mat = params.q_matrix(ndf + lambda_ndf, Duration::ZERO);

        for i in ndf..ndf+lambda_ndf {
            q_mat[(i, i)] = 100.0_f64.powi(2);
        }

        debug!("ndf(ppp)={} Q(ppp)={}", ndf, q_mat);

        // build F
        self.f_mat = DMatrix::identity(ndf + lambda_ndf, ndf + lambda_ndf);

        self.kalman.initialize(&self.f_mat, q_mat, initial_estimate);

        self.state = pending;
        debug!("{} - new PPP state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        self.lambda_x.resize_mut(lambda_ndf, 1, 0.0);
        self.lambda_q.resize_mut(lambda_ndf, lambda_ndf, 0.0);

        for i in 0..lambda_ndf {
            for j in 0..lambda_ndf {
                self.lambda_q[(i, j)] = self.p_mat[(
                    i + Navigation::<U4>::clock_index() + 1,
                    j + Navigation::<U4>::clock_index() + 1,
                )];
            }
            self.lambda_x[i] = self.x_vec[i + Navigation::<U4>::clock_index() + 1];
        }

        match self
            .lambda
            .run(lambda_ndf, lambda_ndf, &self.lambda_x, &self.lambda_q)
        {
            Ok((f_mat, s_vec)) => {
                // TODO fix confirmation / validation

                for (i, (sv, _)) in self.sv_indexes.iter().enumerate() {
                    self.n_amb.insert(*sv, f_mat[(i, 0)].round() as u64);
                }
            },
            Err(e) => {
                error!("lambda search failed with {}", e);
                return Err(e);
            },
        }

        Ok(())
    }

    /// KF run
    pub fn kf_run<B: Bias, RTK: RTKBase>(
        &mut self,
        t: Epoch,
        candidates: &[Candidate],
        params: UserParameters,
        size: usize,
        _: &RTK,
        bias: &B,
    ) -> Result<(), Error> {
        let mut pending = self.state.clone();

        debug!("ppp pending state= {:?}", pending);

        // measurements
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            let position_m = pending.position_ecef_m();

            match candidates[i].vector_contribution(
                t,
                &self.cfg,
                true,
                None,
                position_m,
                pending.lat_long_alt_deg_deg_km,
                &mut contrib,
                bias,
            ) {
                Ok(vec) => {
                    self.y_vec.push(vec.row1);
                    self.y_vec.push(vec.row2);

                    self.w_diag.push(1.0); // TODO

                    self.sv.push(contrib);
                    self.sv_indexes.push((candidates[i].sv, i));

                    if self.cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativistic path range: {:.3}m",
                            t, candidates[i].sv, vec.dr
                        );
                    }
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let nrows = self.y_vec.len();
        let lambda_ndf = nrows / 2;
        let ndf = U4::USIZE + lambda_ndf;

        // verifications prior moving forward
        if nrows < U8::USIZE {
            // maths limitations: do not propose
            return Err(Error::MatrixMinimalDimension);
        }

        // form W
        self.w_mat.resize_mut(nrows, nrows, 0.0);

        for i in 0..nrows {
            self.w_mat[(i, i)] = 1.0; // TODO: improve model
        }

        // form G
        self.g_mat.resize_mut(nrows, ndf, 0.0);

        for (i, (_sv, index)) in self.sv_indexes.iter().enumerate() {
            let dr_i = self.sv[i].relativistic_path_range_m;

            let position_m = pending.position_ecef_m();

            let (dx, dy, dz) = candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

            self.g_mat[(2 * i, 0)] = dx;
            self.g_mat[(2 * i, 1)] = dy;
            self.g_mat[(2 * i, 2)] = dz;
            self.g_mat[(2 * i, Navigation::<U4>::clock_index())] = 1.0;

            self.g_mat[(2 * i + 1, 0)] = dx;
            self.g_mat[(2 * i + 1, 1)] = dy;
            self.g_mat[(2 * i + 1, 2)] = dz;
            self.g_mat[(2 * i + 1, Navigation::<U4>::clock_index())] = 1.0;
            self.g_mat[(2 * i + 1, Navigation::<U4>::clock_index() + 1 + i)] = 1.0;
        }

        debug!("G(ppp): {} W(ppp): {}", self.g_mat, self.w_mat);

        // form Y
        let y = DVector::<f64>::from_row_slice(self.y_vec.as_slice());

        debug!("Y(ppp): {}", y);

        // build F
        self.f_mat = DMatrix::identity(ndf, ndf);

        // build Q
        let mut q_mat = params.q_matrix(ndf, Duration::ZERO);

        for i in ndf..ndf+lambda_ndf {
            q_mat[(i, i)] = 100.0_f64.powi(2);
        }

        debug!("ndf(ppp)={} F(ppp)={} Q(ppp)={}", ndf, self.f_mat, q_mat);

        let estimate = self
            .kalman
            .run(&self.f_mat, &self.g_mat, &self.w_mat, &q_mat, &y)?;

        let ndf = estimate.x.nrows();

        for i in 0..ndf {
            self.x_vec[i] = estimate.x[i];
        }

        debug!("dx(ppp)={}", self.x_vec);

        pending
            .correct_mut(self.frame, t, &self.x_vec, ndf)
            .map_err(|e| {
                error!("{} - state update failed with physical error: {}", t, e);
                Error::StateUpdate
            })?;

        let gt_g_inv = (self.g_mat.transpose() * self.g_mat.clone())
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        // DoP update
        let dop = DilutionOfPrecision::from_ppp(&pending, gt_g_inv);

        // validation
        self.state_validation(&dop)?;

        self.state = pending;
        self.dop = dop;

        debug!("{} - new PPP state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        if self.cfg.method == Method::PPP {
            self.lambda_q.resize_mut(lambda_ndf, lambda_ndf, 0.0);
            self.lambda_x.resize_mut(lambda_ndf, 1, 0.0);

            for i in 0..lambda_ndf {
                for j in 0..lambda_ndf {
                    self.lambda_q[(i, j)] = self.p_mat[(
                        i + Navigation::<U4>::clock_index() + 1,
                        j + Navigation::<U4>::clock_index() + 1,
                    )];
                }

                self.lambda_x[i] = self.x_vec[i + Navigation::<U4>::clock_index() + 1];
            }

            match self
                .lambda
                .run(lambda_ndf, lambda_ndf, &self.lambda_x, &self.lambda_q)
            {
                Ok((f_mat, s_vec)) => {
                    // TODO fix confirmation / validation

                    for (i, (sv, _)) in self.sv_indexes.iter().enumerate() {
                        self.n_amb.insert(*sv, f_mat[(i, 0)].round() as u64);
                    }
                },
                Err(e) => {
                    error!("lambda search failed with {}", e);
                    return Err(e);
                },
            }
        }

        Ok(())
    }

    /// Validate pending [State]
    fn state_validation(&self, dop: &DilutionOfPrecision) -> Result<(), Error> {
        if dop.gdop > self.cfg.solver.max_gdop {
            return Err(Error::MaxGdopExceeded);
        }

        Ok(())
    }

    /// Returns resolved ambiguity (if any)
    pub fn fixed_ambiguity(&self, sv: &SV) -> Option<u64> {
        let n_fixed = self.n_amb.get(&sv)?;
        Some(*n_fixed)
    }
}

#[cfg(test)]
mod test {
    use super::PPPState;
    use crate::navigation::DilutionOfPrecision;
    use nalgebra::{DMatrix, DVector};

    #[test]
    fn ppp_nav_dop() {
        let state = PPPState::default();
        let matrix = DMatrix::from_diagonal(&DVector::from_row_slice(&[1.0, 2.0, 3.0, 4.0]));
        let dop = DilutionOfPrecision::from_ppp(&state, matrix);
        assert_eq!(dop.gdop, (1.0_f64 + 2.0_f64 + 3.0_f64 + 4.0_f64).sqrt());
        assert_eq!(dop.tdop, 2.0);
    }
}
