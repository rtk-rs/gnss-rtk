use log::{debug, error};

#[cfg(doc)]
use crate::prelude::TimeScale;

pub(crate) mod apriori;
pub(crate) mod solutions;

mod dop;
mod kalman;
mod postfit;
mod ppp;
mod sv;

pub(crate) mod state;

use nalgebra::{allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, OMatrix};

use crate::{
    navigation::{
        apriori::Apriori,
        dop::DilutionOfPrecision,
        kalman::{Kalman, KfEstimate},
        postfit::PostfitKf,
        ppp::PrefitSolver as PPPPrefitSolver,
        state::State,
    },
    prelude::{
        Bias, Candidate, Config, Epoch, Error, Frame, Method, UserParameters, SPEED_OF_LIGHT_M_S,
    },
    rtk::RTKBase,
};

pub use solutions::PVTSolution;
pub use sv::SVContribution;

/// [Navigation] Solver
pub(crate) struct Navigation<D: DimName>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// [Config] preset
    cfg: Config,

    /// [Frame]
    frame: Frame,

    /// Y allocation
    y_k_vec: Vec<f64>,

    /// W allocation
    w_k_vec: Vec<f64>,

    /// W
    w_k: DMatrix<f64>,

    /// G
    g_k: DMatrix<f64>,

    /// F
    f_k: OMatrix<f64, D, D>,

    /// Q
    q_k: OMatrix<f64, D, D>,

    /// X
    x_k: DVector<f64>,

    /// P
    p_k: DMatrix<f64>,

    /// [Kalman]
    kalman: Kalman<D>,

    /// contribution allocation
    indexes: Vec<usize>,

    /// [SVContribution]s
    pub sv: Vec<SVContribution>,

    /// Postfit [Kalman]
    postfit: Option<PostfitKf>,

    /// PPP specific prefit
    ppp_prefit: Option<PPPPrefitSolver>,

    /// True if this filter has been initialized
    pub initialized: bool,

    /// Current [State]
    pub state: State<D>,

    /// [DilutionOfPrecision]
    pub dop: DilutionOfPrecision,

    /// Null of first iter
    prev_epoch: Option<Epoch>,
}

impl<D: DimName> Navigation<D>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// Creates new [Navigation] solver.
    /// ## Input
    /// - cfg: [Config] preset
    /// - apriori: [Apriori] input
    /// - candidates: selected [Candidate]s
    /// - size: number of proposal
    /// - bias: [Bias] model implementation
    /// ## Returns
    /// - [Navigation], [Error]
    pub fn new(cfg: &Config, frame: Frame) -> Self {
        match D::USIZE {
            4 | 7 => {},
            u => panic!("non supported dimensions: {}", u),
        }

        let mut f_k = OMatrix::<f64, D, D>::zeros();
        let mut q_k = OMatrix::<f64, D, D>::zeros();

        if D::USIZE == 4 {
            for i in 0..=2 {
                f_k[(i, i)] = 1.0;
            }
        }

        // TODO: improve this model
        q_k[(0, 0)] = 1.0;
        q_k[(1, 1)] = 1.0;
        q_k[(2, 2)] = 1.0;

        Self {
            f_k,
            q_k,
            frame,
            postfit: None,
            prev_epoch: None,
            initialized: false,
            ppp_prefit: None,
            cfg: cfg.clone(),
            state: State::default(),
            sv: Vec::with_capacity(8),
            kalman: Kalman::<D>::new(),
            y_k_vec: Vec::with_capacity(8),
            w_k_vec: Vec::with_capacity(8),
            indexes: Vec::with_capacity(8),
            x_k: DVector::zeros(D::USIZE),
            dop: DilutionOfPrecision::default(),
            g_k: DMatrix::<f64>::zeros(4, 4),
            w_k: DMatrix::<f64>::zeros(4, 4),
            p_k: DMatrix::<f64>::zeros(D::USIZE, D::USIZE),
        }
    }

    /// Reset [Navigation] filter.
    ///
    /// To this day, it has not been demonstrated that this
    /// operation is needed by any real-world application, even
    /// real-time processing. But anyways.. this exists.
    ///
    /// After reset, the [Navigation] solver is in the same state as deployment,
    /// ready to consume a first measurement.
    pub fn reset(&mut self) {
        self.clear();
        self.kalman.reset();

        if let Some(prefit) = &mut self.ppp_prefit {
            prefit.reset();
        }

        if let Some(postfit) = &mut self.postfit {
            postfit.reset();
        }
        self.prev_epoch = None;
        self.initialized = false;
        self.dop = DilutionOfPrecision::default();
    }

    /// Returns clock index
    pub(crate) fn clock_index() -> usize {
        D::USIZE - 1
    }

    /// Iterates mutable [Navigation] filter.
    ///
    /// ## Input
    /// - epoch: sampling [Epoch]
    /// - user: [User] profile
    /// - past_state: past [State]
    /// - candidates: proposed [Candidate]s
    /// - size: number of proposed [Cadndidate]s
    /// - rtk_base: possible [RTKBase] implementation]
    /// - bias: external [Bias] implementation
    pub fn solving<B: Bias, R: RTKBase>(
        &mut self,
        epoch: Epoch,
        params: UserParameters,
        initial_state: &State<D>,
        candidates: &[Candidate],
        size: usize,
        rtk_base: &R,
        bias: &B,
    ) -> Result<(), Error> {
        self.clear();

        let mut initial_state = initial_state.clone();

        if self.cfg.method == Method::PPP {
            if self.ppp_prefit.is_none() {
                self.ppp_prefit = Some(PPPPrefitSolver::new(&self.cfg, &initial_state, self.frame));
            }
        }

        // TODO: verif la relation état interne ppp_prefit et nav_state
        if let Some(ppp_prefit) = &mut self.ppp_prefit {
            debug!("**** ppp prefit *****");
            ppp_prefit.run(epoch, params, candidates, size, rtk_base, bias)?;
            debug!("**** end of ppp prefit *****");

            initial_state = ppp_prefit.state.to_initial_state();
        }

        // TODO Q model

        // self.q_k[(Self::clock_index(), Self::clock_index())] =
        //     (user.clock_sigma_s * SPEED_OF_LIGHT_M_S).powi(2);

        self.q_k[(Self::clock_index(), Self::clock_index())] =
            (100e-3 * SPEED_OF_LIGHT_M_S).powi(2);

        if !self.kalman.initialized {
            self.kf_initialization(epoch, &initial_state, candidates, size, rtk_base, bias)?;
        } else {
            self.kf_run(epoch, candidates, size, rtk_base, bias)?;
        }

        if self.cfg.solver.postfit_denoising > 0.0 {
            if let Some(postfit) = &mut self.postfit {
                let prev_epoch = self
                    .prev_epoch
                    .expect("internal error: undetermined past epoch");

                let dt = epoch - prev_epoch;
                let dx = postfit.run(&self.state, dt)?;

                // update state
                self.state
                    .postfit_update_mut(self.frame, dx.x)
                    .map_err(|e| {
                        error!(
                            "{} - postfit state update failed with physical error: {}",
                            epoch, e
                        );
                        Error::StateUpdate
                    })?;
            } else {
                self.postfit = Some(PostfitKf::new(
                    &self.state,
                    1.0 / self.cfg.solver.postfit_denoising,
                    1.0 / self.cfg.solver.postfit_denoising,
                    1.0,
                    1.0,
                ));
            }
        }

        self.prev_epoch = Some(epoch);
        self.initialized = true;

        Ok(())
    }

    fn clear(&mut self) {
        self.sv.clear();
        self.indexes.clear();
        self.y_k_vec.clear();
        self.w_k_vec.clear();
    }

    /// Filter first iteration.
    pub fn kf_initialization<B: Bias, RTK: RTKBase>(
        &mut self,
        t: Epoch,
        state: &State<D>,
        candidates: &[Candidate],
        size: usize,
        _: &RTK,
        bias: &B,
    ) -> Result<(), Error> {
        let nb_iter = 10;
        let mut pending = state.clone();
        let mut dop = DilutionOfPrecision::default();

        // measurement
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            let position_m = pending.position_ecef_m();

            let amb = match self.cfg.method {
                Method::PPP => {
                    if let Some(prefit) = &self.ppp_prefit {
                        if let Some(n_amb) = prefit.fixed_ambiguity(&candidates[i].sv) {
                            Some(n_amb)
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                },
                Method::SPP | Method::CPP => None,
            };

            match candidates[i].vector_contribution(
                t,
                &self.cfg,
                false,
                amb,
                position_m,
                pending.lat_long_alt_deg_deg_km,
                &mut contrib,
                bias,
            ) {
                Ok(vec) => {
                    self.y_k_vec.push(vec.row1);

                    self.w_k_vec.push(1.0); // TODO

                    self.indexes.push(i);
                    self.sv.push(contrib);

                    if self.cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativistic path range: {:.3}m",
                            t, candidates[i].sv, vec.dr,
                        );
                    }
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let y_len = self.indexes.len();

        if y_len < D::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        self.g_k.resize_mut(y_len, D::USIZE, 0.0);

        // run
        for ith in 0..nb_iter {
            let y_len = self.y_k_vec.len();

            // Form W
            self.w_k.resize_mut(y_len, y_len, 0.0);

            for i in 0..y_len {
                self.w_k[(i, i)] = 1.0 / self.w_k_vec[i];
            }

            let y_k = DVector::from_row_slice(&self.y_k_vec); // TODO malloc
            debug!("(i={}) Y: {}", ith, y_k);

            // Form G
            for (i, index) in self.indexes.iter().enumerate() {
                let dr_i = self.sv[i].relativistic_path_range_m;

                let position_m = pending.position_ecef_m();

                let (dx, dy, dz) =
                    candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

                self.g_k[(i, 0)] = dx;
                self.g_k[(i, 1)] = dy;
                self.g_k[(i, 2)] = dz;

                self.g_k[(i, Self::clock_index())] = 1.0;
            }

            // run
            let gt = self.g_k.transpose();
            let gt_g = gt.clone() * self.g_k.clone();
            let gt_w = gt.clone() * self.w_k.clone();
            let gt_w_g = gt_w * self.g_k.clone();
            let gt_w_g_inv = gt_w_g.try_inverse().ok_or(Error::MatrixInversion)?;
            let gt_w_g_inv_gt = gt_w_g_inv.clone() * gt.clone();
            let gt_w_g_inv_gt_w = gt_w_g_inv_gt * self.w_k.clone();

            self.x_k = gt_w_g_inv_gt_w * y_k.clone();
            self.p_k = gt_w_g_inv.clone();

            let ndf = self.x_k.nrows();
            debug!("(i={}) dx={}", ith, self.x_k);

            pending
                .correct_mut(self.frame, t, &self.x_k, ndf)
                .map_err(|e| {
                    error!("{} - state update failed with physical error: {}", t, e);
                    Error::StateUpdate
                })?;

            let gt_g_inv = gt_g.try_inverse().ok_or(Error::MatrixInversion)?;

            // update latest DoP
            dop = DilutionOfPrecision::new(&pending, gt_g_inv);

            debug!("(i={}) {} - pending state {}", ith, t, pending);

            // models update
            self.y_k_vec.clear();
            self.w_k_vec.clear();

            self.indexes.retain(|i| {
                let mut unused = SVContribution::default();

                let position_m = pending.position_ecef_m();

                let amb = match self.cfg.method {
                    Method::PPP => {
                        if let Some(prefit) = &self.ppp_prefit {
                            if let Some(n_amb) = prefit.fixed_ambiguity(&candidates[*i].sv) {
                                Some(n_amb)
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    },
                    Method::SPP | Method::CPP => None,
                };

                match candidates[*i].vector_contribution(
                    t,
                    &self.cfg,
                    false,
                    amb,
                    position_m,
                    pending.lat_long_alt_deg_deg_km,
                    &mut unused,
                    bias,
                ) {
                    Ok(vec) => {
                        self.y_k_vec.push(vec.row1);
                        self.w_k_vec.push(1.0); // TODO
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

        let initial_estimate = KfEstimate::from_dynamic(self.x_k.clone(), self.p_k.clone());

        self.kalman.initialize(self.f_k, self.q_k, initial_estimate);

        self.state = pending;
        self.dop = dop;

        debug!("{} - new state {}", t, self.state);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        Ok(())
    }

    /// [Kalman] filter run
    pub fn kf_run<B: Bias, RTK: RTKBase>(
        &mut self,
        t: Epoch,
        candidates: &[Candidate],
        size: usize,
        _: &RTK,
        bias: &B,
    ) -> Result<(), Error> {
        let mut pending = self.state.clone();

        // measurement
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            let pos_m = pending.position_ecef_m();

            let amb = match self.cfg.method {
                Method::PPP => {
                    if let Some(prefit) = &self.ppp_prefit {
                        if let Some(n_amb) = prefit.fixed_ambiguity(&candidates[i].sv) {
                            Some(n_amb)
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                },
                Method::SPP | Method::CPP => None,
            };

            match candidates[i].vector_contribution(
                t,
                &self.cfg,
                false,
                amb,
                pos_m,
                pending.lat_long_alt_deg_deg_km,
                &mut contrib,
                bias,
            ) {
                Ok(vec) => {
                    self.y_k_vec.push(vec.row1);
                    self.w_k_vec.push(1.0); // TODO
                    self.indexes.push(i);
                    self.sv.push(contrib);

                    if self.cfg.modeling.relativistic_path_range {
                        debug!(
                            "{}({}) - relativistic path range: {:.3}m",
                            t, candidates[i].sv, vec.dr,
                        );
                    }
                },
                Err(e) => {
                    error!("{}({}) - cannot contribute: {}", t, candidates[i].sv, e);
                },
            }
        }

        let y_len = self.y_k_vec.len();

        if y_len < D::USIZE {
            return Err(Error::MatrixMinimalDimension);
        }

        let y_k = DVector::from_row_slice(&self.y_k_vec); // TODO malloc
        debug!("Y: {}", y_k);

        self.w_k.resize_mut(y_len, y_len, 0.0);
        self.g_k.resize_mut(y_len, D::USIZE, 0.0);

        for i in 0..y_len {
            self.w_k[(i, i)] = 1.0 / self.w_k_vec[i];
        }

        // form g_k
        for (i, index) in self.indexes.iter().enumerate() {
            let dr_i = self.sv[i].relativistic_path_range_m;

            let position_m = pending.position_ecef_m();

            let (dx, dy, dz) = candidates[*index].matrix_contribution(&self.cfg, dr_i, position_m);

            self.g_k[(i, 0)] = dx;
            self.g_k[(i, 1)] = dy;
            self.g_k[(i, 2)] = dz;

            self.g_k[(i, Self::clock_index())] = 1.0;
        }

        let estimate = self
            .kalman
            .run(&self.f_k, &self.g_k, &self.w_k, &self.q_k, &y_k)?;

        debug!("state correction: dx={}", estimate.x);

        let ndf = estimate.x.nrows();

        for i in 0..ndf {
            self.x_k[i] = estimate.x[i];
        }

        pending
            .correct_mut(self.frame, t, &self.x_k, ndf)
            .map_err(|e| {
                error!("{} - state update failed with physical error: {}", t, e);
                Error::StateUpdate
            })?;

        let gt_g_inv = (self.g_k.transpose() * self.g_k.clone())
            .try_inverse()
            .ok_or(Error::MatrixInversion)?;

        // update
        let dop = DilutionOfPrecision::new(&pending, gt_g_inv);

        self.state_validation(&dop)?;

        self.dop = dop;
        self.state = pending;

        debug!("{} - new state {}", t, pending);
        debug!("{} - gdop={} tdop={}", t, self.dop.gdop, self.dop.tdop);

        Ok(())
    }

    /// Validate pending [State]
    fn state_validation(&self, dop: &DilutionOfPrecision) -> Result<(), Error> {
        if dop.gdop > self.cfg.solver.max_gdop {
            return Err(Error::MaxGdopExceeded);
        }
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use crate::navigation::{DilutionOfPrecision, Navigation, State};
    use nalgebra::{DMatrix, DVector, U4, U7, U9};

    #[test]
    fn navigation_dimensions_clock_index() {
        assert_eq!(
            Navigation::<U4>::clock_index(),
            3,
            "invalid clock index for U4"
        );
        assert_eq!(
            Navigation::<U7>::clock_index(),
            6,
            "invalid clock index for U7"
        );
        assert_eq!(
            Navigation::<U9>::clock_index(),
            8,
            "invalid clock index for U9"
        );
    }

    #[test]
    fn dilution_of_navigation_precision() {
        let state = State::<U4>::default();
        let matrix = DMatrix::from_diagonal(&DVector::from_row_slice(&[1.0, 2.0, 3.0, 4.0]));
        let dop = DilutionOfPrecision::new(&state, matrix);
        assert_eq!(dop.gdop, (1.0_f64 + 2.0_f64 + 3.0_f64 + 4.0_f64).sqrt());
        assert_eq!(dop.tdop, 2.0);
    }
}
