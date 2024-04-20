//! PVT solver
use std::collections::HashMap;

use gnss::prelude::SV;
use hifitime::Unit;
use log::{debug, error, warn};
use map_3d::deg2rad;
use nalgebra::{DVector, Matrix3, Matrix4, Matrix4x1, MatrixXx4, Vector3};
use thiserror::Error;

use nyx::{
    cosmic::{
        eclipse::{eclipse_state, EclipseState},
        Orbit, SPEED_OF_LIGHT,
    },
    md::prelude::{Arc, Cosm, Frame},
};

use crate::{
    apriori::AprioriPosition,
    bias::{IonosphericBias, TroposphericBias},
    candidate::Candidate,
    cfg::{Config, Filter, Method},
    prelude::{Duration, Epoch},
    solutions::{
        validator::{SolutionInvalidation, SolutionValidator},
        PVTSVData, PVTSolution, PVTSolutionType,
    },
};

#[derive(Debug, Clone, Error)]
pub enum Error {
    #[error("need more candidates to resolve a {0} a solution")]
    NotEnoughInputCandidates(PVTSolutionType),
    #[error("not enough candidates fit criteria")]
    NotEnoughFittingCandidates,
    #[error("failed to invert navigation matrix")]
    MatrixInversionError,
    #[error("failed to invert covar matrix")]
    CovarMatrixInversionError,
    #[error("reolved NaN: invalid input matrix")]
    TimeIsNan,
    #[error("undefined apriori position")]
    UndefinedAprioriPosition,
    #[error("missing pseudo range observation")]
    MissingPseudoRange,
    #[error("cannot form signal combination: missing dual freq signals")]
    PseudoRangeCombination,
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
    #[error("invalidated solution: {0}")]
    InvalidatedSolution(SolutionInvalidation),
    // Kalman filter bad op: should never happen
    #[error("uninitialized kalman filter!")]
    UninitializedKalmanFilter,
}

#[derive(Debug, Clone)]
pub(crate) struct LSQState {
    /* p matrix */
    pub(crate) p: Matrix4<f64>,
    /* x estimate */
    pub(crate) x: Matrix4x1<f64>,
}

#[derive(Debug, Clone)]
pub(crate) struct KfState {
    /* p matrix */
    pub(crate) p: Matrix4<f64>,
    /* x estimate */
    pub(crate) x: Matrix4x1<f64>,
}

// impl KfState {
//     pub fn eval(&mut self,
//         phi: &Matrix4<f64>,
//         q: &Matrix4<f64>,
//         x: &Vector4<f64>,
//         p: &Matrix4x1<f64>,
//     ) -> Result<Self, Error> {
//         let x = phi * x;
//         let p = (phi * p * phi.transpose()) + q;
//         Ok(Self {
//             x,
//             p,
//         })
//     }
// }

// Filter state
#[derive(Debug, Clone)]
pub(crate) enum FilterState {
    /// LSQ state
    LSQState(LSQState),
    /// KF state
    KfState(KfState),
}

/// Interpolation result (state vector) that needs to be
/// resolved for every single candidate.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct InterpolationResult {
    /// Elevation compared to reference position and horizon in [°]
    pub elevation: f64,
    /// Azimuth compared to reference position and magnetic North in [°]
    pub azimuth: f64,
    /// APC Position vector in [m] ECEF
    pub position: Vector3<f64>,
    // Velocity vector in [m/s] ECEF that we calculated ourselves
    velocity: Option<Vector3<f64>>,
}

impl InterpolationResult {
    /// Builds InterpolationResults from an Antenna Phase Center (APC) position
    /// as ECEF [m] coordinates
    pub fn from_apc_position(pos: (f64, f64, f64)) -> Self {
        let mut s = Self::default();
        s.position = Vector3::<f64>::new(pos.0, pos.1, pos.2);
        s
    }
    /// Builds Self with given SV (elevation, azimuth) attitude
    pub fn with_elevation_azimuth(&self, elev_azim: (f64, f64)) -> Self {
        let mut s = *self;
        s.elevation = elev_azim.0;
        s.azimuth = elev_azim.1;
        s
    }
    pub(crate) fn velocity(&self) -> Option<Vector3<f64>> {
        self.velocity
    }
    pub(crate) fn orbit(&self, dt: Epoch, frame: Frame) -> Orbit {
        let p = self.position;
        let v = self.velocity().unwrap_or_default();
        Orbit::cartesian(
            p[0] / 1000.0,
            p[1] / 1000.0,
            p[2] / 1000.0,
            v[0] / 1000.0,
            v[1] / 1000.0,
            v[2] / 1000.0,
            dt,
            frame,
        )
    }
}

/// PVT Solver.
/// I: Interpolated SV APC coordinates interface.
/// You are required to provide APC coordinates at requested ("t", "sv"),
/// expressed in meters [ECEF], for this to proceed.
#[derive(Debug, Clone)]
pub struct Solver<I>
where
    I: Fn(Epoch, SV, usize) -> Option<InterpolationResult>,
{
    /// Solver parametrization
    pub cfg: Config,
    /// apriori position
    pub apriori: AprioriPosition,
    /// Interpolated SV state.
    pub interpolator: I,
    /* Cosmic model */
    cosmic: Arc<Cosm>,
    /*
     * (Reference) Earth frame.
     * Could be relevant to rename this the day we want to
     * resolve solutions on other Planets...  ; )
     */
    earth_frame: Frame,
    /*
     * Sun Body frame
     * Could be relevant to rename this the day we want to resolve
     * solutions in other Star systems....   o___O
     */
    sun_frame: Frame,
    /* prev. solution for internal logic */
    prev_pvt: Option<(Epoch, PVTSolution)>,
    /* current filter state */
    filter_state: Option<FilterState>,
    /* prev. state vector for internal velocity determination */
    prev_sv_state: HashMap<SV, (Epoch, Vector3<f64>)>,
}

impl<I: std::ops::Fn(Epoch, SV, usize) -> Option<InterpolationResult>> Solver<I> {
    pub fn new(cfg: &Config, apriori: AprioriPosition, interpolator: I) -> Result<Self, Error> {
        let cosmic = Cosm::de438();
        let sun_frame = cosmic.frame("Sun J2000");
        let earth_frame = cosmic.frame("EME2000");
        /*
         * print some infos on latched config
         */
        if cfg.method == Method::SPP && cfg.min_sv_sunlight_rate.is_some() {
            warn!("Eclipse filter is not meaningful in SPP mode");
        }
        if cfg.modeling.relativistic_path_range {
            warn!("Relativistic path range cannot be modeled at the moment");
        }
        Ok(Self {
            cosmic,
            sun_frame,
            earth_frame,
            apriori,
            interpolator,
            cfg: cfg.clone(),
            prev_sv_state: HashMap::new(),
            filter_state: Option::<FilterState>::None,
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

        let method = self.cfg.method;
        let modeling = self.cfg.modeling;
        let solver_opts = &self.cfg.solver;
        let filter = solver_opts.filter;
        let interp_order = self.cfg.interp_order;

        /* interpolate positions */
        let mut pool: Vec<Candidate> = pool
            .iter()
            .filter_map(|cd| match cd.transmission_time(&self.cfg) {
                Ok((t_tx, dt_tx)) => {
                    debug!("{:?} ({}) : signal propagation {}", t_tx, cd.sv, dt_tx);
                    let interpolated = (self.interpolator)(t_tx, cd.sv, interp_order)?;

                    let min_elev = match self.cfg.min_sv_elev {
                        Some(el) => el,
                        None => 0.0_f64,
                    };

                    let min_azim = match self.cfg.min_sv_azim {
                        Some(az) => az,
                        None => 0.0_f64,
                    };

                    let max_azim = match self.cfg.max_sv_azim {
                        Some(az) => az,
                        None => 360.0_f64,
                    };

                    if interpolated.elevation < min_elev {
                        debug!(
                            "{:?} ({}) - {:?} rejected : below elevation mask",
                            cd.t, cd.sv, interpolated
                        );
                        None
                    } else if interpolated.azimuth < min_azim {
                        debug!(
                            "{:?} ({}) - {:?} rejected : below azimuth mask",
                            cd.t, cd.sv, interpolated
                        );
                        None
                    } else if interpolated.azimuth > max_azim {
                        debug!(
                            "{:?} ({}) - {:?} rejected : above azimuth mask",
                            cd.t, cd.sv, interpolated
                        );
                        None
                    } else {
                        let mut cd = cd.clone();
                        let interpolated =
                            Self::rotate_position(modeling.earth_rotation, interpolated, dt_tx);
                        let interpolated = self.velocities(t_tx, cd.sv, interpolated);
                        cd.t_tx = t_tx;
                        debug!("{:?} ({}) : {:?}", cd.t, cd.sv, interpolated);
                        cd.state = Some(interpolated);
                        Some(cd)
                    }
                },
                Err(e) => {
                    error!("{} - transmision time error: {:?}", cd.sv, e);
                    None
                },
            })
            .collect();
        /*
         * Update internal state
         */
        for cd in pool.iter_mut() {
            if modeling.relativistic_clock_bias {
                /*
                 * following calculations need inst. velocity
                 */
                let state = cd.state.unwrap();
                if state.velocity.is_some() {
                    const EARTH_SEMI_MAJOR_AXIS_WGS84: f64 = 6378137.0_f64;
                    const EARTH_GRAVITATIONAL_CONST: f64 = 3986004.418 * 10.0E8;
                    let orbit = state.orbit(cd.t_tx, self.earth_frame);
                    let ea_rad = deg2rad(orbit.ea_deg());
                    let gm = (EARTH_SEMI_MAJOR_AXIS_WGS84 * EARTH_GRAVITATIONAL_CONST).sqrt();
                    let bias = -2.0_f64 * orbit.ecc() * ea_rad.sin() * gm
                        / SPEED_OF_LIGHT
                        / SPEED_OF_LIGHT
                        * Unit::Second;
                    debug!("{:?} ({}) : relativistic clock bias: {}", cd.t, cd.sv, bias);
                    cd.clock_corr += bias;
                }
            }

            self.prev_sv_state
                .insert(cd.sv, (cd.t_tx, cd.state.unwrap().position));
        }

        /* remove observed signals above snr mask (if any) */
        if let Some(min_snr) = self.cfg.min_snr {
            let mut nb_removed: usize = 0;
            for idx in 0..pool.len() {
                let (init_code, init_phase) = (
                    pool[idx - nb_removed].code.len(),
                    pool[idx - nb_removed].phase.len(),
                );
                pool[idx - nb_removed].min_snr_mask(min_snr);
                let delta_code = init_code - pool[idx - nb_removed].code.len();
                let delta_phase = init_phase - pool[idx - nb_removed].phase.len();
                if delta_code > 0 || delta_phase > 0 {
                    debug!(
                        "{:?} ({}) : {} code | {} phase below snr mask",
                        pool[idx - nb_removed].t,
                        pool[idx - nb_removed].sv,
                        delta_code,
                        delta_phase
                    );
                }
                /* make sure we're still compliant */
                match method {
                    Method::SPP => {
                        if pool[idx - nb_removed].code.is_empty() {
                            debug!(
                                "{:?} ({}) dropped on bad snr",
                                pool[idx - nb_removed].t,
                                pool[idx - nb_removed].sv
                            );
                            let _ = pool.swap_remove(idx - nb_removed);
                            nb_removed += 1;
                        }
                    },
                    Method::CODE_PPP => {
                        let mut drop = false;
                        if !pool[idx - nb_removed].dual_pseudorange() {
                            error!(
                                "{} ({}) PPP missing one code observation",
                                pool[idx - nb_removed].t,
                                pool[idx - nb_removed].sv
                            );
                            drop = true;
                        }
                        if drop {
                            let _ = pool.swap_remove(idx - nb_removed);
                            nb_removed += 1;
                        }
                    },
                }
            }
        }

        /* apply eclipse filter (if need be) */
        if let Some(min_rate) = self.cfg.min_sv_sunlight_rate {
            let mut nb_removed: usize = 0;
            for idx in 0..pool.len() {
                let state = pool[idx - nb_removed].state.unwrap(); // infaillible
                let orbit = state.orbit(pool[idx - nb_removed].t, self.earth_frame);
                let state = eclipse_state(&orbit, self.sun_frame, self.earth_frame, &self.cosmic);
                let eclipsed = match state {
                    EclipseState::Umbra => true,
                    EclipseState::Visibilis => false,
                    EclipseState::Penumbra(r) => r < min_rate,
                };
                if eclipsed {
                    debug!(
                        "{:?} ({}): dropped - eclipsed by Earth",
                        pool[idx - nb_removed].t,
                        pool[idx - nb_removed].sv
                    );
                    let _ = pool.swap_remove(idx - nb_removed);
                    nb_removed += 1;
                }
            }
        }

        // adapt to navigation
        pool.sort_by(|cd_a, cd_b| {
            let state_a = cd_a.state.unwrap();
            let state_b = cd_b.state.unwrap();
            state_b.elevation.partial_cmp(&state_a.elevation).unwrap()
        });

        let mut index = 0;
        pool.retain(|_| {
            index += 1;
            index < min_required + 1
        });

        if pool.len() != min_required {
            return Err(Error::NotEnoughFittingCandidates);
        }

        /* form matrix */
        let mut row_index = 0;
        let mut y = DVector::<f64>::zeros(min_required);
        let mut g = MatrixXx4::<f64>::zeros(min_required);
        let mut pvt_sv_data = HashMap::<SV, PVTSVData>::with_capacity(min_required);

        for cd in pool.iter() {
            match cd.resolve(
                t,
                &self.cfg,
                (x0, y0, z0),
                (lat_ddeg, lon_ddeg, altitude_above_sea_m),
                iono_bias,
                tropo_bias,
                row_index,
                &mut y,
                &mut g,
            ) {
                Ok(sv_data) => {
                    pvt_sv_data.insert(cd.sv, sv_data);
                    row_index += 1;
                },
                Err(e) => {
                    error!("{:?} - {} failed to form candidate : {}", t, cd.sv, e);
                },
            }
        }

        if row_index != min_required {
            return Err(Error::NotEnoughFittingCandidates);
        }

        let w = self.cfg.solver.weight_matrix(
            min_required,
            pvt_sv_data
                .values()
                .map(|sv_data| sv_data.elevation)
                .collect(),
        );

        let (mut pvt_solution, new_state) = PVTSolution::new(
            g.clone(),
            w.clone(),
            y.clone(),
            pvt_sv_data.clone(),
            filter,
            self.filter_state.clone(),
        )?;

        let validator = SolutionValidator::new(&self.apriori.ecef, &pool, &w, &pvt_solution);

        let valid = validator.valid(solver_opts);
        if valid.is_err() {
            return Err(Error::InvalidatedSolution(valid.err().unwrap()));
        }

        if let Some((prev_t, prev_pvt)) = &self.prev_pvt {
            pvt_solution.vel = (pvt_solution.pos - prev_pvt.pos) / (t - *prev_t).to_seconds();
        }

        self.prev_pvt = Some((t, pvt_solution.clone()));

        if filter != Filter::None {
            self.filter_state = new_state;
        }

        /*
         * slightly rework the solution so it ""looks"" like
         * what we expect based on the defined setup.
         */
        if let Some(alt) = self.cfg.fixed_altitude {
            pvt_solution.pos.z = self.apriori.ecef.z - alt;
            pvt_solution.vel.z = 0.0_f64;
        }

        match solution {
            PVTSolutionType::TimeOnly => {
                pvt_solution.pos = Vector3::<f64>::default();
                pvt_solution.vel = Vector3::<f64>::default();
            },
            _ => {},
        }

        Ok((t, pvt_solution))
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
    /*
     * Apply appropriate adjustments
     */
    fn rotate_position(
        rotate: bool,
        interpolated: InterpolationResult,
        dt_tx: Duration,
    ) -> InterpolationResult {
        let mut reworked = interpolated.clone();
        let rot = if rotate {
            const EARTH_OMEGA_E_WGS84: f64 = 7.2921151467E-5;
            let dt_tx = dt_tx.to_seconds();
            let we = EARTH_OMEGA_E_WGS84 * dt_tx;
            let (we_cos, we_sin) = (we.cos(), we.sin());
            Matrix3::<f64>::new(
                we_cos, we_sin, 0.0_f64, -we_sin, we_cos, 0.0_f64, 0.0_f64, 0.0_f64, 1.0_f64,
            )
        } else {
            Matrix3::<f64>::new(
                1.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 1.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 1.0_f64,
            )
        };
        reworked.position = rot * interpolated.position;
        reworked
    }
    /*
     * Determine velocities
     */
    fn velocities(
        &self,
        t_tx: Epoch,
        sv: SV,
        interpolated: InterpolationResult,
    ) -> InterpolationResult {
        let mut reworked = interpolated.clone();
        if let Some((p_ttx, p_pos)) = self.prev_sv_state.get(&sv) {
            let dt = (t_tx - *p_ttx).to_seconds();
            reworked.velocity = Some((interpolated.position - p_pos) / dt);
        }
        reworked
    }
}
