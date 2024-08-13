//! PVT solver

use std::{cmp::Ordering, collections::HashMap};

use hifitime::Unit;
use thiserror::Error;

use log::{debug, error, info, warn};
use nalgebra::{Matrix3, Vector3};

use nyx::cosmic::{
    eclipse::{eclipse_state, EclipseState},
    SPEED_OF_LIGHT_M_S,
};

use anise::{
    constants::frames::{EARTH_ITRF93, EARTH_J2000, SUN_J2000},
    errors::{AlmanacError, PhysicsError},
    prelude::{Almanac, Frame},
};

use crate::{
    ambiguity::AmbiguitySolver,
    bancroft::Bancroft,
    candidate::Candidate,
    cfg::{Config, Method},
    constants::Constants,
    navigation::{
        solutions::validator::{InvalidationCause, Validator as SolutionValidator},
        Input as NavigationInput, Navigation, PVTSolution, PVTSolutionType,
    },
    orbit::{OrbitalState, OrbitalStateProvider},
    position::Position,
    prelude::{Duration, Epoch, Observation, SV},
    rtk::BaseStation,
};

#[derive(Debug, PartialEq, Error)]
pub enum Error {
    /// Not enough candidates were proposed: we do not attempt resolution
    #[error("not enough candidates provided")]
    NotEnoughCandidates,
    /// PreFit (signal quality, other..) criterias
    /// have been applied but we're left with not enough vehicles that match
    /// the navigation technique: no attempt.
    #[error("not enough candidates match pre-fit criteria")]
    NotEnoughPreFitCandidates,
    /// PostFit (state solver and other) have been resolved,
    /// but we're left with not enough vehicles that match
    /// the navigation technique: no attempt.
    #[error("not enough candidates match post-fit criteria")]
    NotEnoughPostFitCandidates,
    /// Failed to parse navigation method
    #[error("non supported/invalid strategy")]
    InvalidStrategy,
    #[error("failed to form matrix (invalid input or not enough data)")]
    MatrixFormationError,
    /// Invalid orbital states or bad signal data may cause the algebric calculations
    /// to wind up here.
    #[error("failed to invert matrix")]
    MatrixInversionError,
    /// Invalid orbital states or bad signal data may cause the algebric calculations
    /// to wind up here.
    #[error("resolved time is `nan` (invalid value(s))")]
    TimeIsNan,
    /// Invalid orbital states or bad signal data may cause the algebric calculations
    /// to abort.
    #[error("internal navigation error")]
    NavigationError,
    #[error("missing pseudo range observation")]
    MissingPseudoRange,
    /// [Method::CPP] requires the special signal combination to exist.
    /// This require the user to sample PR on two separate frequencies.
    #[error("failed to form pseudo range combination")]
    PseudoRangeCombination,
    /// [Method::PPP] requires the special signal combination to exist.
    /// This require the user to sample PR + PH on two separate frequencies.
    #[error("failed to form phase range combination")]
    PhaseRangeCombination,
    /// Each [Candidate] state needs to be resolved to contribute to any PPP resolution attempt.
    #[error("unresolved candidate state")]
    UnresolvedState,
    /// When [Modeling.sv_clock_bias] is turned on and we attempt PPP resolution,
    /// it is mandatory for the user to provide [ClockCorrection].
    #[error("missing clock correction")]
    UnknownClockCorrection,
    /// Physical non sense due to bad signal data or invalid orbital state, will cause us
    /// abort with this message.
    #[error("physical non sense: rx prior tx")]
    PhysicalNonSenseRxPriorTx,
    /// Physical non sense due to bad signal data or invalid orbital state, will cause us
    /// abort with this message.
    #[error("physical non sense: t_rx is too late")]
    PhysicalNonSenseRxTooLate,
    /// Solutions may be invalidated and are rejected with [InvalidationCause].
    #[error("invalidated solution, cause: {0}")]
    InvalidatedSolution(InvalidationCause),
    /// In pure PPP survey (no RTK, no position apriori knowledge = worst case scenario),
    /// [Solver] is initiliazed by [Bancroft] algorithm, which requires
    /// temporary 4x4 navigation and pseudo range sampling (whatever your navigation technique),
    /// until at least initialization is achieved.
    #[error("bancroft solver error: invalid input ?")]
    BancroftError,
    /// [Bancroft] initialization process (see [BancroftError]) will wind up here
    /// in case unrealistic or bad signal observation or orbital states were forwarded.
    #[error("bancroft solver error: invalid input (imaginary solution)")]
    BancroftImaginarySolution,
    /// PPP navigation technique requires phase ambiguity to be solved prior any attempt.
    /// It is Okay to wind up here for a few iterations, until the ambiguities are fixed
    /// and we may proceed to precise navigation. We will reject solving attempt until then.
    /// Hardware and external events may reset the ambiguity fixes and it is okay to need to
    /// rerun through this phase for a short period of time. Normally not too often, when good
    /// equipment is properly operated.
    #[error("unresolved signal ambiguity")]
    UnresolvedAmbiguity,
    /// [Solver] requires [Almanac] determination at build up and may wind-up here this step is in failure.
    #[error("issue with Almanac: {0}")]
    Almanac(AlmanacError),
    /// [Solver] requires to determine a [Frame] from [Almanac] and we wind-up here if this step is in failure.
    #[error("failed to retrieve Earth reference Frame")]
    EarthFrame,
    /// Any physical non sense detected by ANISE will cause us to abort with this error.
    #[error("physics issue: {0}")]
    Physics(PhysicsError),
    /// Remote observation is required for a [Candidate] to contribute in RTK solving attempt.
    /// You need up to four of them to resolve. We may print this internal message and still
    /// proceed to resolve, as [SV] may go out of sight of rover or reference site.
    #[error("missing observation on remote site {0}({1})")]
    MissingRemoteRTKObservation(Epoch, SV),
    /// In RTK resolution attempt, you need to observe all pending [SV] on reference site as well.
    /// If that is not the case, we abort with this error.
    #[error("missing observations on remote site")]
    MissingRemoteRTKObservations,
}

/// [Solver] to resolve [PVTSolution]s.
pub struct Solver<O: OrbitalStateProvider> {
    /// [OrbitalStateProvider]
    orbit: O,
    /// Solver parametrization
    pub cfg: Config,
    /// Initial [Position]
    initial: Option<Position>,
    /// [Almanac]
    almanac: Almanac,
    /// [Frame]
    earth_cef: Frame,
    /// [Navigation]
    nav: Navigation,
    /// [AmbiguitySolver]
    ambiguity: AmbiguitySolver,
    // Post fit KF
    // postfit_kf: Option<KF<State3D, U3, U3>>,
    /* prev. solution for internal logic */
    /// Previous solution (internal logic)
    prev_solution: Option<(Epoch, PVTSolution)>,
    /// Previous VDOP (internal logic)
    prev_vdop: Option<f64>,
    /// Previously used (internal logic)
    prev_used: Vec<SV>,
    /// Stored previous SV state (internal logic)
    prev_sv_state: HashMap<SV, (Epoch, Vector3<f64>)>,
}

/// Apply signal condition criteria
fn signal_condition_filter(method: Method, pool: &mut Vec<Candidate>) {
    pool.retain(|cd| match method {
        Method::SPP => {
            if cd.prefered_pseudorange().is_some() {
                true
            } else {
                error!("{} ({}) missing pseudo range observation", cd.t, cd.sv);
                false
            }
        },
        Method::CPP => {
            if cd.cpp_compatible() {
                true
            } else {
                debug!("{} ({}) missing secondary frequency", cd.t, cd.sv);
                false
            }
        },
        Method::PPP => {
            if cd.ppp_compatible() {
                true
            } else {
                debug!("{} ({}) missing phase or phase combination", cd.t, cd.sv);
                false
            }
        },
    })
}

/// Apply signal quality criteria
fn signal_quality_filter(method: Method, min_snr: f64, pool: &mut Vec<Candidate>) {
    pool.retain_mut(|cd| {
        cd.min_snr_mask(min_snr);
        !cd.observations.is_empty()
    })
}

impl<O: OrbitalStateProvider> Solver<O> {
    /// Create a new Position [Solver] that may support any positioning technique..
    /// ## Inputs
    /// - cfg: Solver [Config]
    /// - initial: possible initial [Position] knowledge, can then be used
    ///   to initialize [Self]. When not provided (None), the solver will initialize itself
    ///   autonomously by consuming at least one [Epoch].
    ///   Note that we need at least 4 valid SV observations to initiliaze the [Solver].
    ///   You have to take that into account, especially when operating in Fixed Altitude
    ///   or Time Only modes.
    /// - orbit: [OrbitalStateProvider] must be provided for Direct (1D) PPP
    pub fn new(cfg: &Config, initial: Option<Position>, orbit: O) -> Result<Self, Error> {
        // Default Almanac, valid until 2035
        let almanac = Almanac::until_2035().map_err(Error::Almanac)?;

        let earth_cef = almanac
            //.frame_from_uid(EARTH_J2000)
            .frame_from_uid(EARTH_ITRF93)
            .map_err(|_| Error::EarthFrame)?;

        // Print more information
        if cfg.method == Method::SPP && cfg.min_sv_sunlight_rate.is_some() {
            warn!("Eclipse filter is not meaningful in SPP mode");
        }
        if cfg.externalref_delay.is_some() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is either incomplete or not entirely enabled");
        }
        if !cfg.int_delay.is_empty() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is either incomplete or not entirely enabled");
        }
        Ok(Self {
            orbit,
            almanac,
            earth_cef,
            initial: {
                if let Some(ref initial) = initial {
                    let geo = initial.geodetic();
                    let (lat, lon) = (geo[0].to_degrees(), geo[1].to_degrees());
                    info!("initial position lat={:.3E}°, lon={:.3E}°", lat, lon);
                }
                initial
            },
            prev_vdop: None,
            prev_used: vec![],
            cfg: cfg.clone(),
            prev_solution: None,
            // TODO
            ambiguity: AmbiguitySolver::new(Duration::from_seconds(120.0)),
            // postfit_kf: None,
            prev_sv_state: HashMap::new(),
            nav: Navigation::new(cfg.solver.filter),
        })
    }
    /// Create new Position [Solver] without knowledge of apriori position (full survey)
    pub fn survey(cfg: &Config, orbit: O) -> Result<Self, Error> {
        Self::new(cfg, None, orbit)
    }
    /// [PVTSolution] resolution attempt.
    /// ## Inputs
    /// - t: desired [Epoch]
    /// - pool: list of [Candidate]
    pub fn resolve(&mut self, t: Epoch, pool: &[Candidate]) -> Result<(Epoch, PVTSolution), Error> {
        let min_required = self.min_sv_required();
        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughCandidates);
        }

        let mut pool = pool.to_vec();

        let method = self.cfg.method;
        let modeling = self.cfg.modeling;
        let interp_order = self.cfg.interp_order;
        let max_iono_bias = self.cfg.max_iono_bias;
        let max_tropo_bias = self.cfg.max_tropo_bias;
        let iono_modeling = self.cfg.modeling.iono_delay;
        let tropo_modeling = self.cfg.modeling.tropo_delay;

        // signal condition filter
        signal_condition_filter(method, &mut pool);

        // signal quality filter
        if let Some(min_snr) = self.cfg.min_snr {
            signal_quality_filter(method, min_snr, &mut pool);
        }

        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughPreFitCandidates);
        }

        // orbital state solver
        let mut pool: Vec<Candidate> = pool
            .iter()
            .filter_map(|cd| match cd.transmission_time(&self.cfg) {
                Ok((t_tx, dt_tx)) => {
                    let orbits = &mut self.orbit;
                    debug!("{} ({}) : signal propagation {}", cd.t, cd.sv, dt_tx);
                    // determine orbital state
                    if let Some(mut orbit) = orbits.next_at(t_tx, cd.sv, interp_order) {
                        let mut min_elev = self.cfg.min_sv_elev.unwrap_or(0.0_f64);
                        let mut min_azim = self.cfg.min_sv_azim.unwrap_or(0.0_f64);
                        let mut max_azim = self.cfg.max_sv_azim.unwrap_or(360.0_f64);

                        // fix orbital state after first iteration
                        if let Some(initial) = &self.initial {
                            let (x0, y0, z0) = (initial.ecef[0], initial.ecef[1], initial.ecef[2]);
                            orbit = orbit.with_elevation_azimuth((x0, y0, z0));
                        } else {
                            // not apply criterias yet
                            min_elev = 0.0_f64;
                            min_azim = 0.0_f64;
                            max_azim = 360.0_f64;
                        }

                        if orbit.elevation < min_elev {
                            debug!(
                                "{} ({}) - {:?} rejected (below elevation mask)",
                                cd.t, cd.sv, orbit
                            );
                            None
                        } else if orbit.azimuth < min_azim {
                            debug!(
                                "{} ({}) - {:?} rejected (below azimuth mask)",
                                cd.t, cd.sv, orbit
                            );
                            None
                        } else if orbit.azimuth > max_azim {
                            debug!(
                                "{} ({}) - {:?} rejected (above azimuth mask)",
                                cd.t, cd.sv, orbit
                            );
                            None
                        } else {
                            let mut cd = cd.clone();
                            let orbit =
                                Self::rotate_position(modeling.earth_rotation, orbit, dt_tx);
                            let orbit = self.velocities(t_tx, cd.sv, orbit);
                            cd.t_tx = t_tx;
                            debug!("{} ({}) : {:?}", cd.t, cd.sv, orbit);
                            cd.state = Some(orbit);
                            Some(cd)
                        }
                    } else {
                        if cd.is_rtk_compatible() {
                            // preserve and permit pure RTK scenario
                            Some(cd.clone())
                        } else {
                            debug!("{}({}) to be dropped", cd.t, cd.sv);
                            None
                        }
                    }
                },
                Err(e) => {
                    error!("{} - transmision time error: {}", cd.sv, e);
                    None
                },
            })
            .collect();

        // relativistic clock bias
        for cd in pool.iter_mut() {
            if modeling.relativistic_clock_bias {
                if let Some(ref mut state) = cd.state {
                    if let Some(ref mut correction) = cd.clock_corr {
                        if state.velocity.is_some() && correction.needs_relativistic_correction {
                            let w_e = Constants::EARTH_SEMI_MAJOR_AXIS_WGS84;
                            let mu = Constants::EARTH_GRAVITATION;

                            let orbit = state.orbit(cd.t_tx, self.earth_cef);
                            let ea_deg = orbit.ea_deg().map_err(Error::Physics)?;
                            let ea_rad = ea_deg.to_radians();
                            let gm = (w_e * mu).sqrt();
                            let bias =
                                -2.0_f64 * orbit.ecc().map_err(Error::Physics)? * ea_rad.sin() * gm
                                    / SPEED_OF_LIGHT_M_S
                                    / SPEED_OF_LIGHT_M_S
                                    * Unit::Second;
                            debug!("{} ({}) : relativistic clock bias: {}", cd.t, cd.sv, bias);
                            correction.duration += bias;
                        }
                    }
                    // update for next time
                    self.prev_sv_state.insert(cd.sv, (cd.t_tx, state.position));
                }
            }
        }

        // apply eclipse filter (if need be)
        if let Some(min_rate) = self.cfg.min_sv_sunlight_rate {
            pool.retain(|cd| {
                if let Some(state) = cd.state {
                    let orbit = state.orbit(cd.t, self.earth_cef);
                    let state =
                        eclipse_state(orbit, SUN_J2000, EARTH_J2000, &self.almanac).unwrap();
                    let eclipsed = match state {
                        EclipseState::Umbra => true,
                        EclipseState::Visibilis => false,
                        EclipseState::Penumbra(r) => r < min_rate,
                    };
                    if eclipsed {
                        debug!("{} ({}): eclipsed", cd.t, cd.sv);
                    }
                    !eclipsed
                } else {
                    true // preserve, for pure RTK
                }
            });
        }

        //if pool.len() < min_required {
        //    return Err(Error::NotEnoughPostFitCandidates);
        //}

        if self.initial.is_none() {
            // TODO:
            //  - this is not needed in pure RTK scenario
            //  - and will not work in pure RTK scenario
            let solver = Bancroft::new(&pool)?;
            let output = solver.resolve()?;
            let (x0, y0, z0) = (output[0], output[1], output[2]);
            let position = Position::from_ecef(Vector3::<f64>::new(x0, y0, z0));
            let geo = position.geodetic();
            let (lat, lon) = (geo[0].to_degrees(), geo[1].to_degrees());
            info!(
                "{} - estimated initial position lat={:.3E}°, lon={:.3E}°",
                pool[0].t, lat, lon
            );
            // update attitudes
            for cd in pool.iter_mut() {
                if let Some(state) = &mut cd.state {
                    *state = state.with_elevation_azimuth((x0, y0, z0));
                }
            }
            // store
            self.initial = Some(Position::from_ecef(Vector3::new(
                output[0], output[1], output[2],
            )));
        }

        let initial = self.initial.as_ref().unwrap();
        let (x0, y0, z0) = (initial.ecef()[0], initial.ecef()[1], initial.ecef()[2]);
        let (lat_rad, lon_rad, altitude_above_sea_m) = (
            initial.geodetic()[0],
            initial.geodetic()[1],
            initial.geodetic()[2],
        );
        let (lat_ddeg, lon_ddeg) = (lat_rad.to_degrees(), lon_rad.to_degrees());

        // Apply models
        for cd in &mut pool {
            cd.apply_models(
                method,
                tropo_modeling,
                iono_modeling,
                (lat_ddeg, lon_ddeg, altitude_above_sea_m),
            );
        }

        // Resolve ambiguities
        let ambiguities = if method == Method::PPP {
            self.ambiguity.resolve(&pool)
        } else {
            Default::default()
        };

        // Prepare for NAV
        //  select best candidates, sort (coherent matrix), propose
        pool.retain(|cd| {
            let retained = cd.tropo_bias < max_tropo_bias;
            if retained {
                debug!("{}({}): tropo delay {:.3E}[m]", cd.t, cd.sv, cd.tropo_bias);
            } else {
                debug!("{}({}) rejected (extreme tropo delay)", cd.t, cd.sv);
            }
            retained
        });

        pool.retain(|cd| {
            let retained = cd.iono_bias < max_iono_bias;
            if retained {
                debug!("{}({}): iono delay {:.3E}[m]", cd.t, cd.sv, cd.iono_bias);
            } else {
                debug!("{}({}) rejected (extreme iono delay)", cd.t, cd.sv);
            }
            retained
        });

        pool.retain(|cd| {
            let retained = cd.is_navi_compatible();
            if !retained {
                debug!("{}({}): not proposed - missing data", cd.t, cd.sv);
            }
            retained
        });

        if pool.len() < min_required {
            return Err(Error::NotEnoughPostFitCandidates);
        }

        Self::retain_best_elevation(&mut pool, min_required);
        pool.sort_by(|cd_a, cd_b| cd_a.sv.prn.partial_cmp(&cd_b.sv.prn).unwrap());

        let w = self.cfg.solver.weight_matrix(); //sv.values().map(|sv| sv.elevation).collect());
                                                 // // Reduce contribution of newer (rising) vehicles (rising)
                                                 // for (i, cd) in pool.iter().enumerate() {
                                                 //     if !self.prev_used.contains(&cd.sv) {
                                                 //         w[(i, i)] = 0.05;
                                                 //         w[(2 * i, 2 * i)] = 0.05;
                                                 //     }
                                                 // }

        let input = match NavigationInput::new((x0, y0, z0), &self.cfg, &pool, w, &ambiguities) {
            Ok(input) => input,
            Err(e) => {
                error!("Failed to form navigation matrix: {}", e);
                return Err(Error::MatrixFormationError);
            },
        };

        self.prev_used = pool.iter().map(|cd| cd.sv).collect::<Vec<_>>();

        // Regular Iteration
        let output = match self.nav.resolve(&input) {
            Ok(output) => output,
            Err(e) => {
                error!("Failed to resolve: {}", e);
                return Err(Error::NavigationError);
            },
        };

        let x = output.state.estimate();
        debug!("x: {}", x);

        let position = match method {
            // Method::PPP => Vector3::new(x[4] + x0, x[5] + y0, x[6] + z0),
            Method::PPP => Vector3::new(x[0] + x0, x[1] + y0, x[2] + z0),
            Method::SPP | Method::CPP => Vector3::new(x[0] + x0, x[1] + y0, x[2] + z0),
        };

        // Bias
        // let mut bias = InstrumentBias::new();
        //if method == Method::PPP {
        //    for i in 0..x.ncols() - 4 {
        //        let b_i = x[i + 4];
        //        let cd = &pool[i];
        //        if let Some(l_c) = cd.phase_if_combination() {
        //            if let Some(amb) = ambiguities.get(&(cd.sv, l_c.reference)) {
        //                //TODO: c'est n_c pas n_1, puisque b_i est lié à la combinaison LC
        //                bias.insert((cd.sv, l_c.reference), b_i - amb.n_1);
        //            }
        //        }
        //    }
        //}

        // Form Solution
        let mut solution = PVTSolution {
            // bias,
            position,
            ambiguities,
            gdop: output.gdop,
            tdop: output.tdop,
            pdop: output.pdop,
            sv: input.sv.clone(),
            q: output.q_covar4x4(),
            timescale: self.cfg.timescale,
            velocity: Vector3::<f64>::default(),
            dt: Duration::from_seconds(x[3] / SPEED_OF_LIGHT_M_S),
            d_dt: 0.0_f64,
        };

        // First solution
        if self.prev_solution.is_none() {
            self.prev_vdop = Some(solution.vdop(lat_rad, lon_rad));
            self.prev_solution = Some((t, solution.clone()));
            // always discard 1st solution
            return Err(Error::InvalidatedSolution(InvalidationCause::FirstSolution));
        }

        let validator =
            SolutionValidator::new(Vector3::<f64>::new(x0, y0, z0), &pool, &input, &output);

        match validator.validate(&self.cfg) {
            Ok(_) => {
                self.nav.validate();
            },
            Err(cause) => {
                error!("solution invalidated - {}", cause);
                return Err(Error::InvalidatedSolution(cause));
            },
        };

        /*
         * Post-fit KF
         */
        if self.cfg.solver.postfit_kf {
            //if let Some(kf) = &mut self.postfit_kf {
            //} else {
            //    let kf_estim = KfEstimate::from_diag(
            //        State3D {
            //            t: Epoch::from_gpst_seconds(x[3] / SPEED_OF_LIGHT_KM_S),
            //            inner: Vector3::new(x[0], x[1], x[2]),
            //        },
            //        OVector::<f64, U3>::new(1.0, 1.0, 1.0),
            //    );
            //    let noise =
            //        OMatrix::<f64, U3, U3>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
            //    self.postfit_kf = Some(KF::no_snc(kf_estim, noise));
            //}
        }

        if let Some((prev_t, prev_solution)) = &self.prev_solution {
            let dt_s = (t - *prev_t).to_seconds();
            solution.velocity = (solution.position - prev_solution.position) / dt_s;
            solution.d_dt = (prev_solution.dt - solution.dt).to_seconds() / dt_s;
        }

        self.prev_solution = Some((t, solution.clone()));

        Self::rework_solution(&mut solution, &self.cfg);
        Ok((t, solution))
    }
    /* returns minimal number of SV */
    fn min_sv_required(&self) -> usize {
        if self.initial.is_none() {
            4
        } else {
            match self.cfg.sol_type {
                PVTSolutionType::TimeOnly => 1,
                _ => {
                    if self.cfg.fixed_altitude.is_some() {
                        3
                    } else {
                        4
                    }
                },
            }
        }
    }
    /* rotate interpolated position */
    fn rotate_position(rotate: bool, interpolated: OrbitalState, dt_tx: Duration) -> OrbitalState {
        let mut reworked = interpolated;
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
    fn velocities(&self, t_tx: Epoch, sv: SV, interpolated: OrbitalState) -> OrbitalState {
        let mut reworked = interpolated;
        if let Some((p_ttx, p_pos)) = self.prev_sv_state.get(&sv) {
            let dt = (t_tx - *p_ttx).to_seconds();
            reworked.velocity = Some((interpolated.position - p_pos) / dt);
        }
        reworked
    }
    /*
     * Reworks solution
     */
    fn rework_solution(pvt: &mut PVTSolution, cfg: &Config) {
        if let Some(alt) = cfg.fixed_altitude {
            pvt.position.z = alt;
            pvt.velocity.z = 0.0_f64;
        }
        if cfg.sol_type == PVTSolutionType::TimeOnly {
            pvt.position = Default::default();
            pvt.velocity = Default::default();
        }
    }
    fn retain_best_elevation(pool: &mut Vec<Candidate>, min_required: usize) {
        pool.sort_by(|cd_a, cd_b| {
            if let Some(state_a) = cd_a.state {
                if let Some(state_b) = cd_b.state {
                    state_a.elevation.partial_cmp(&state_b.elevation).unwrap()
                } else {
                    Ordering::Greater
                }
            } else {
                if cd_b.state.is_some() {
                    Ordering::Less
                } else {
                    Ordering::Greater
                }
            }
        });

        let mut index = 0;
        let total = pool.len();

        if min_required == 1 {
            pool.retain(|_| {
                index += 1;
                index == total
            });
        } else {
            pool.retain(|_| {
                index += 1;
                index > total - min_required
            });
        }
    }
}

// #[cfg(test)]
// mod test {
//     use crate::prelude::{Solver, Candidate, Duration, Epoch, Observation, SV, OrbitalState};
//     #[test]
//     fn retain_best_elev() {
//         let mut pool = Vec::<Candidate>::new();
//         for elev in [0.0, 3.0, 8.0, 16.0, 16.5, 21.0, 45.0] {
//             let cd = Candidate::new(
//                 SV::default(),
//                 Epoch::default(),
//                 Duration::default(),
//                 None,
//                 vec![],
//                 vec![],
//             );
//             let mut state = OrbitalState::from_position((0.0, 0.0, 0.0));
//             state.set_elevation(elev);
//             cd.set_state(state);
//             pool.push(cd);
//         }
//
//         for min_required in [1, 3, 4, 5] {
//             let mut tested = pool.clone();
//             Solver::retain_best_elevation(&mut tested, min_required);
//             if min_required == 1 {
//                 assert_eq!(tested.len(), 1);
//                 assert_eq!(tested[0].state.unwrap().elevation, 45.0);
//             } else if min_required == 3 {
//             } else if min_required == 4 {
//             } else if min_required == 5 {
//             }
//         }
//     }
// }
