//! PVT solver
use std::collections::HashMap;

use hifitime::Epoch;
use log::{debug, error, warn};
use thiserror::Error;

use nyx::cosmic::eclipse::{eclipse_state, EclipseState};
use nyx::cosmic::Orbit;
use nyx::md::prelude::{Arc, Cosm};
use nyx::md::prelude::{Bodies, Frame, LightTimeCalc};

use gnss::prelude::SV;

use nalgebra::{DVector, Matrix3, MatrixXx4, Vector3};

use crate::{
    apriori::AprioriPosition,
    bias::{IonosphericBias, TroposphericBias},
    candidate::Candidate,
    cfg::Config,
    solutions::{PVTSVData, PVTSolution, PVTSolutionType},
};

/// Solving mode
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    /// Single Point Positioning (SPP).
    /// Combine this to advanced configurations (refined compensations),
    /// and you may achieve metric precision in the best case.
    #[default]
    SPP,
    /// Precise Point Positioning (PPP).
    /// Combine this to advanced configurations (refined compensations)
    /// and you may achieve centimetric precision.
    PPP,
}

impl std::fmt::Display for Mode {
    fn fmt(&self, fmt: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::SPP => write!(fmt, "SPP"),
            Self::PPP => write!(fmt, "PPP"),
        }
    }
}

#[derive(Debug, Clone, Error)]
pub enum Error {
    #[error("need more candidates to resolve a {0} a solution")]
    NotEnoughInputCandidates(PVTSolutionType),
    #[error("not enough candidates fit criteria")]
    NotEnoughFittingCandidates,
    #[error("failed to invert navigation matrix")]
    MatrixInversionError,
    #[error("reolved NaN: invalid input matrix")]
    TimeIsNan,
    #[error("undefined apriori position")]
    UndefinedAprioriPosition,
    #[error("at least one pseudo range observation is mandatory")]
    NeedsAtLeastOnePseudoRange,
    #[error("failed to model or measure ionospheric delay")]
    MissingIonosphericDelayValue,
    #[error("unresolved state: interpolation should have passed")]
    UnresolvedState,
    #[error("unable to form signal combination")]
    SignalRecombination,
    #[error("physical non sense: rx prior tx")]
    PhysicalNonSenseRxPriorTx,
    #[error("physical non sense: t_rx is too late")]
    PhysicalNonSenseRxTooLate,
}

/// Interpolation result (state vector) that needs to be
/// resolved for every single candidate.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct InterpolationResult {
    /// Position vector in [m] ECEF
    pub position: Vector3<f64>,
    /// Velocity vector in [m/s] ECEF
    pub velocity: Option<Vector3<f64>>,
    /// Elevation compared to reference position and horizon in [°]
    pub elevation: f64,
    /// Azimuth compared to reference position and magnetic North in [°]
    pub azimuth: f64,
}

impl InterpolationResult {
    pub(crate) fn position(&self) -> (f64, f64, f64) {
        (self.position[0], self.position[1], self.position[2])
    }
    pub(crate) fn velocity(&self) -> Option<(f64, f64, f64)> {
        let velocity = self.velocity?;
        Some((velocity[0], velocity[1], velocity[2]))
    }
    pub(crate) fn orbit(&self, dt: Epoch, frame: Frame) -> Orbit {
        let (x, y, z) = self.position();
        let (v_x, v_y, v_z) = self.velocity().unwrap_or((0.0_f64, 0.0_f64, 0.0_f64));
        Orbit::cartesian(x, y, z, v_x / 1000.0, v_y / 1000.0, v_z / 1000.0, dt, frame)
    }
}

/// PVT Solver
#[derive(Debug, Clone)]
pub struct Solver<I>
where
    I: Fn(Epoch, SV, usize) -> Option<InterpolationResult>,
{
    /// Solver parametrization
    pub cfg: Config,
    /// Type of solver implemented
    pub mode: Mode,
    /// apriori position
    pub apriori: AprioriPosition,
    /// SV state interpolation method. It is mandatory
    /// to resolve the SV state at the requested Epoch otherwise the solver
    /// will not proceed further. User should provide the interpolation method.
    /// Other parameters are SV: Space Vehicle identity we want to resolve, and "usize" interpolation order.
    pub interpolator: I,
    /* Cosmic model */
    cosmic: Arc<Cosm>,
    /* Earth frame */
    earth_frame: Frame,
    /* Sun frame */
    sun_frame: Frame,
    /* prev. solution for internal logic */
    prev_pvt: Option<(Epoch, PVTSolution)>,
}

impl<I: std::ops::Fn(Epoch, SV, usize) -> Option<InterpolationResult>> Solver<I> {
    pub fn new(
        mode: Mode,
        apriori: AprioriPosition,
        cfg: &Config,
        interpolator: I,
    ) -> Result<Self, Error> {
        let cosmic = Cosm::de438();
        let sun_frame = cosmic.frame("Sun J2000");
        let earth_frame = cosmic.frame("EME2000");

        /*
         * print some infos on latched config
         */
        if cfg.modeling.relativistic_clock_corr {
            warn!("relativistic clock corr. is not feasible at the moment");
        }

        if mode == Mode::SPP && cfg.min_sv_sunlight_rate.is_some() {
            warn!("eclipse filter is not meaningful when using spp strategy");
        }

        Ok(Self {
            mode,
            cosmic,
            sun_frame,
            earth_frame,
            apriori,
            interpolator,
            cfg: cfg.clone(),
            prev_pvt: Option::<(Epoch, PVTSolution)>::None,
        })
    }
    /// Try to resolve a PVTSolution at desired "t".
    /// "t": sampling instant.
    /// "solution": desired PVTSolutionType.
    /// "pool": List of candidates.
    /// iono_bias: possible IonosphericBias if you can provide such info.
    /// tropo_bias: possible TroposphericBias if you can provide such info.
    pub fn resolve(
        &mut self,
        t: Epoch,
        solution: PVTSolutionType,
        pool: Vec<Candidate>,
        iono_bias: &IonosphericBias,
        tropo_bias: &TroposphericBias,
    ) -> Result<(Epoch, PVTSolution), Error> {
        let min_required = Self::min_required(solution, &self.cfg);

        if pool.len() < min_required {
            return Err(Error::NotEnoughInputCandidates(solution));
        }

        let (x0, y0, z0) = (
            self.apriori.ecef.x,
            self.apriori.ecef.y,
            self.apriori.ecef.z,
        );

        let (lat_ddeg, lon_ddeg, altitude_above_sea_m) = (
            self.apriori.geodetic.x,
            self.apriori.geodetic.y,
            self.apriori.geodetic.z,
        );

        let modeling = self.cfg.modeling;
        let interp_order = self.cfg.interp_order;

        /* interpolate positions */
        let mut pool: Vec<Candidate> = pool
            .iter()
            .filter_map(|c| {
                let (t_tx, dt_ttx) = c.transmission_time(&self.cfg).ok()?;

                if let Some(mut interpolated) = (self.interpolator)(t_tx, c.sv, interp_order) {
                    let mut c = c.clone();

                    if modeling.earth_rotation {
                        const EARTH_OMEGA_E_WGS84: f64 = 7.2921151467E-5;

                        let we = EARTH_OMEGA_E_WGS84 * dt_ttx;
                        let (we_cos, we_sin) = (we.cos(), we.sin());

                        let r = Matrix3::<f64>::new(
                            we_cos, we_sin, 0.0_f64, -we_sin, we_cos, 0.0_f64, 0.0_f64, 0.0_f64,
                            1.0_f64,
                        );

                        interpolated.position = r * interpolated.position;
                    }

                    debug!(
                        "{:?} ({}) : interpolated state: {:?}",
                        t_tx, c.sv, interpolated.position
                    );

                    c.state = Some(interpolated);
                    Some(c)
                } else {
                    warn!("{:?} ({}) : interpolation failed", t_tx, c.sv);
                    None
                }
            })
            .collect();

        /* apply elevation filter (if any) */
        if let Some(min_elev) = self.cfg.min_sv_elev {
            let mut idx: usize = 0;
            let mut nb_removed: usize = 0;
            while idx < pool.len() {
                if let Some(state) = pool[idx - nb_removed].state {
                    if state.elevation < min_elev {
                        debug!(
                            "{:?} ({}) : below elevation mask",
                            pool[idx - nb_removed].t,
                            pool[idx - nb_removed].sv
                        );
                        let _ = pool.swap_remove(idx - nb_removed);
                        nb_removed += 1;
                    }
                }
                idx += 1;
            }
        }

        /* remove observed signals above snr mask (if any) */
        if let Some(min_snr) = self.cfg.min_snr {
            for idx in 0..pool.len() {
                let (init_code, init_phase) = (pool[idx].code.len(), pool[idx].phase.len());
                pool[idx].min_snr_mask(min_snr);
                let delta_code = init_code - pool[idx].code.len();
                let delta_phase = init_phase - pool[idx].phase.len();
                if delta_code > 0 || delta_phase > 0 {
                    debug!(
                        "{:?} ({}) : {} code | {} phase below snr mask",
                        pool[idx].t, pool[idx].sv, delta_code, delta_phase
                    );
                }
            }
        }

        /* apply eclipse filter (if need be) */
        if let Some(min_rate) = self.cfg.min_sv_sunlight_rate {
            for idx in 0..pool.len() - 1 {
                let state = pool[idx].state.unwrap(); // infaillible
                let orbit = state.orbit(pool[idx].t, self.earth_frame);
                let state = eclipse_state(&orbit, self.sun_frame, self.earth_frame, &self.cosmic);
                let eclipsed = match state {
                    EclipseState::Umbra => true,
                    EclipseState::Visibilis => false,
                    EclipseState::Penumbra(r) => r < min_rate,
                };
                if eclipsed {
                    debug!(
                        "{:?} ({}): earth eclipsed, dropping",
                        pool[idx].t, pool[idx].sv
                    );
                    let _ = pool.swap_remove(idx);
                }
            }
        }

        /* make sure we still have enough SV */
        let nb_candidates = pool.len();
        if nb_candidates < min_required {
            return Err(Error::NotEnoughFittingCandidates);
        } else {
            debug!("{:?}: {} elected sv", t, nb_candidates);
        }

        /* form matrix */
        let mut y = DVector::<f64>::zeros(nb_candidates);
        let mut g = MatrixXx4::<f64>::zeros(nb_candidates);
        let mut pvt_sv_data = HashMap::<SV, PVTSVData>::with_capacity(nb_candidates);

        for (row_index, cd) in pool.iter().enumerate() {
            if let Ok(sv_data) = cd.resolve(
                t,
                &self.cfg,
                self.mode,
                (x0, y0, z0),
                (lat_ddeg, lon_ddeg, altitude_above_sea_m),
                iono_bias,
                tropo_bias,
                row_index,
                &mut y,
                &mut g,
            ) {
                pvt_sv_data.insert(cd.sv, sv_data);
            }
        }

        let w = self.cfg.lsq_weight_matrix(
            nb_candidates,
            pvt_sv_data
                .iter()
                .map(|(sv, sv_data)| sv_data.elevation)
                .collect(),
        );

        let mut pvt_solution = PVTSolution::new(g.clone(), w, y, pvt_sv_data)?;

        if let Some((prev_t, prev_pvt)) = &self.prev_pvt {
            pvt_solution.v = (pvt_solution.p - prev_pvt.p) / (t - *prev_t).to_seconds();
        }

        /*
         * slightly rework the solution so it ""physically"" (/ looks like)
         * what we expect based on the predefined setup.
         */
        if let Some(alt) = self.cfg.fixed_altitude {
            pvt_solution.p.z = self.apriori.ecef.z - alt;
            pvt_solution.v.z = 0.0_f64;
            pvt_solution.covar[(2, 2)] = 0.0_f64;
        }

        match solution {
            PVTSolutionType::TimeOnly => {
                pvt_solution.p = Vector3::<f64>::default();
                pvt_solution.v = Vector3::<f64>::default();
                pvt_solution.covar[(0, 0)] = 0.0_f64;
                pvt_solution.covar[(1, 1)] = 0.0_f64;
                pvt_solution.covar[(2, 2)] = 0.0_f64;
            },
            _ => {},
        }

        self.prev_pvt = Some((t, pvt_solution.clone()));
        Ok((t, pvt_solution))
    }
    /*
     * Evaluates Sun/Earth vector, <!> expressed in Km <!>
     * for all SV NAV Epochs in provided context
     */
    fn sun_earth_vector(&mut self, t: Epoch) -> Vector3<f64> {
        let sun_body = Bodies::Sun;
        let orbit = self.cosmic.celestial_state(
            sun_body.ephem_path(),
            t,
            self.earth_frame,
            LightTimeCalc::None,
        );
        Vector3::new(
            orbit.x_km * 1000.0,
            orbit.y_km * 1000.0,
            orbit.z_km * 1000.0,
        )
    }
    /*
     * Returns nb of vehicles we need to gather
     */
    fn min_required(solution: PVTSolutionType, cfg: &Config) -> usize {
        match solution {
            PVTSolutionType::TimeOnly => 1,
            _ => {
                let mut n = 4;
                if cfg.fixed_altitude.is_some() {
                    n -= 1;
                }
                n
            },
        }
    }
}
