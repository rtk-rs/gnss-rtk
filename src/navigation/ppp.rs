use log::{debug, error};

#[cfg(doc)]
use crate::prelude::TimeScale;

use nalgebra::{allocator::Allocator, DMatrix, DVector, DefaultAllocator, DimName, OMatrix};

use crate::{
    navigation::{
        apriori::Apriori,
        dop::DilutionOfPrecision,
        kalman::{Kalman, KfEstimate},
        postfit::PostfitKf,
        ppp_ar::PrefitSolver as PPPPrefitSolver,
        state::State,
        sv::SVContribution,
        Navigation,
    },
    prelude::{Candidate, Config, Epoch, Error, Frame, Method, UserParameters, SPEED_OF_LIGHT_M_S},
};

impl<D: DimName> Navigation<D>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// Absolute mutable iteration of the [Navigation] filter.
    ///
    /// ## Input
    /// - epoch: sampling [Epoch]
    /// - user: [User] profile
    /// - past_state: past [State]
    /// - candidates: [Candidate]s proposal
    /// - size: number of proposed [Cadndidate]s
    pub fn absolute_solving(
        &mut self,
        epoch: Epoch,
        params: UserParameters,
        initial_state: &State<D>,
        candidates: &[Candidate],
        size: usize,
    ) -> Result<(), Error> {
        self.clear();

        let mut initial_state = initial_state.clone();

        if self.cfg.method == Method::PPP_AR {
            if self.ppp_prefit.is_none() {
                self.ppp_prefit = Some(PPPPrefitSolver::new(&self.cfg, &initial_state, self.frame));
            }
        }

        if let Some(ppp_prefit) = &mut self.ppp_prefit {
            // run PPP-AR prefit
            ppp_prefit.run(epoch, params, candidates, size)?;
            initial_state = ppp_prefit.state.to_initial_state();
        }

        // TODO Q model

        // self.q_k[(Self::clock_index(), Self::clock_index())] =
        //     (user.clock_sigma_s * SPEED_OF_LIGHT_M_S).powi(2);

        self.q_k[(Self::clock_index(), Self::clock_index())] =
            (100e-3 * SPEED_OF_LIGHT_M_S).powi(2);

        if !self.kalman.initialized {
            self.kf_initialization(epoch, &initial_state, candidates, size)?;
        } else {
            self.kf_run(epoch, candidates, size)?;
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
    pub fn kf_initialization(
        &mut self,
        t: Epoch,
        state: &State<D>,
        candidates: &[Candidate],
        size: usize,
    ) -> Result<(), Error> {
        let nb_iter = 10;
        let mut pending = state.clone();
        let mut dop = DilutionOfPrecision::default();

        // measurement
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            let position_m = pending.to_position_ecef_m();

            let amb = match self.cfg.method {
                Method::PPP_AR => {
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
                Method::SPP | Method::CPP | Method::PPP => None,
            };

            match candidates[i].ppp_vector_contribution(
                t,
                &self.cfg,
                false,
                amb,
                position_m,
                pending.lat_long_alt_deg_deg_km,
                &mut contrib,
            ) {
                Ok(vec) => {
                    self.y_k_vec.push(vec.row_1);
                    self.w_k_vec.push(1.0); // TODO improve model
                    self.indexes.push(i);
                    self.sv.push(contrib);
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
                let position_m = pending.to_position_ecef_m();

                let (dx, dy, dz) =
                    candidates[*index].ppp_matrix_contribution(&self.cfg, position_m);

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

                let position_m = pending.to_position_ecef_m();

                let amb = match self.cfg.method {
                    Method::PPP_AR => {
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
                    Method::SPP | Method::CPP | Method::PPP => None,
                };

                match candidates[*i].ppp_vector_contribution(
                    t,
                    &self.cfg,
                    false,
                    amb,
                    position_m,
                    pending.lat_long_alt_deg_deg_km,
                    &mut unused,
                ) {
                    Ok(vec) => {
                        self.y_k_vec.push(vec.row_1);
                        self.w_k_vec.push(1.0); // TODO improve model
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
    pub fn kf_run(&mut self, t: Epoch, candidates: &[Candidate], size: usize) -> Result<(), Error> {
        let mut pending = self.state.clone();

        // measurement
        for i in 0..size {
            let mut contrib = SVContribution::default();

            contrib.sv = candidates[i].sv;

            let pos_m = pending.to_position_ecef_m();

            let amb = match self.cfg.method {
                Method::PPP_AR => {
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
                Method::SPP | Method::CPP | Method::PPP => None,
            };

            match candidates[i].ppp_vector_contribution(
                t,
                &self.cfg,
                false,
                amb,
                pos_m,
                pending.lat_long_alt_deg_deg_km,
                &mut contrib,
            ) {
                Ok(vec) => {
                    self.y_k_vec.push(vec.row_1);
                    self.w_k_vec.push(1.0); // TODO improve model
                    self.indexes.push(i);
                    self.sv.push(contrib);
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
            let position_m = pending.to_position_ecef_m();
            let (dx, dy, dz) = candidates[*index].ppp_matrix_contribution(&self.cfg, position_m);

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
