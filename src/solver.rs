//! PVT solver
use hifitime::Unit;
use std::{cmp::Ordering, collections::HashMap};

use thiserror::Error;

use log::{debug, error, info, warn};
use nalgebra::Vector3;
//use nalgebra::Matrix3;

use nyx::cosmic::{
    eclipse::{eclipse_state, EclipseState},
    SPEED_OF_LIGHT_M_S,
};

use anise::{
    almanac::metaload::MetaFile,
    constants::frames::{EARTH_ITRF93, EARTH_J2000, IAU_EARTH_FRAME, SUN_J2000},
    errors::{AlmanacError, PhysicsError},
    math::Matrix3,
    prelude::{Almanac, Frame, MetaAlmanac},
};

use crate::{
    ambiguity::AmbiguitySolver,
    bancroft::Bancroft,
    candidate::Candidate,
    cfg::{Config, Method},
    constants::{Constants, Url},
    navigation::{
        solutions::validator::{InvalidationCause, Validator as SolutionValidator},
        Input as NavigationInput, Navigation, PVTSolution, PVTSolutionType,
    },
    orbit::OrbitSource,
    prelude::{Duration, Epoch, Orbit, SV},
};

#[derive(Debug, PartialEq, Error)]
pub enum Error {
    /// Not enough candidates were proposed: we do not attempt resolution
    #[error("not enough candidates provided")]
    NotEnoughCandidates,
    /// Survey initialization (no apriori = internal guess)
    /// requires at least 4 SV in sight temporarily, whatever
    /// your navigation technique.
    #[error("needs 4 SV in sight at least temporarily")]
    NotEnoughCandidatesBancroft,
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
    /// Each [Candidate] presented to the Bancroft solver needs a resolved state.
    #[error("bancroft requires 4 fully resolved candidates")]
    UnresolvedStateBancroft,
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
pub struct Solver<O: OrbitSource> {
    /// [OrbitSource]
    orbit: O,
    /// Solver parametrization
    pub cfg: Config,
    /// Initial [Orbit] either forwarded by User
    /// or guess by a first Iteration. The latest [Orbit]
    /// that we have resolved is contained in [prev_solution].
    initial: Option<Orbit>,
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
    /// Stored previous SV state (internal logic)
    sv_orbits: HashMap<SV, Orbit>,
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
fn signal_quality_filter(min_snr: f64, pool: &mut Vec<Candidate>) {
    pool.retain_mut(|cd| {
        cd.min_snr_mask(min_snr);
        !cd.observations.is_empty()
    })
}

impl<O: OrbitSource> Solver<O> {
    /// Infaillible method to either download, retrieve or create
    /// a basic [Almanac] and reference [Frame] to work with.
    /// We always prefer the highest precision scenario.
    /// On first deployment, it will require internet access.
    /// We can only rely on lower precision kernels if we cannot access the cloud.
    fn build_almanac() -> Result<(Almanac, Frame), Error> {
        let almanac = Almanac::until_2035().map_err(|e| Error::Almanac(e))?;
        match almanac.load_from_metafile(Url::nyx_anise_de440s_bsp()) {
            Ok(almanac) => {
                info!("ANISE DE440S BSP has been loaded");
                match almanac.load_from_metafile(Url::nyx_anise_pck11_pca()) {
                    Ok(almanac) => {
                        info!("ANISE PCK11 PCA has been loaded");
                        match almanac.load_from_metafile(Url::jpl_latest_high_prec_bpc()) {
                            Ok(almanac) => {
                                info!("JPL high precision (daily) kernels loaded.");
                                if let Ok(itrf93) = almanac.frame_from_uid(EARTH_ITRF93) {
                                    info!("High precision context initiated.");
                                    Ok((almanac, itrf93))
                                } else {
                                    let iau_earth = almanac
                                        .frame_from_uid(IAU_EARTH_FRAME)
                                        .map_err(|_| Error::EarthFrame)?;
                                    warn!("Failed to build ITRF93: relying on IAU model");
                                    Ok((almanac, iau_earth))
                                }
                            },
                            Err(e) => {
                                let iau_earth = almanac
                                    .frame_from_uid(IAU_EARTH_FRAME)
                                    .map_err(|_| Error::EarthFrame)?;
                                error!("Failed to dowload JPL High precision kernels: relying on IAU model");
                                Ok((almanac, iau_earth))
                            },
                        }
                    },
                    Err(e) => {
                        let iau_earth = almanac
                            .frame_from_uid(IAU_EARTH_FRAME)
                            .map_err(|_| Error::EarthFrame)?;
                        error!(
                            "Failed to dowload JPL High precision kernels: relying on IAU model"
                        );
                        Ok((almanac, iau_earth))
                    },
                }
            },
            Err(e) => {
                error!("Failed to load DE440S BSP: {}", e);
                //let almanac = Almanac::until_2035().map_err(Error::Almanac)?;
                let iau_earth = almanac
                    .frame_from_uid(IAU_EARTH_FRAME)
                    .map_err(|_| Error::EarthFrame)?;
                warn!("Relying on lowest precision kernels");
                Ok((almanac, iau_earth))
            },
        }
    }
    /// Create a new Position [Solver] that may support any positioning technique..
    /// ## Inputs
    /// - cfg: Solver [Config]
    /// - initial: possible initial position expressed as [Orbit] in ECEF.
    ///   When not provided, the solver will initialize itself autonomously by consuming at least one [Epoch].
    ///   Note that we need at least 4 valid SV observations to initiliaze the [Solver].
    ///   You have to take that into account, especially when operating in Fixed Altitude
    ///   or Time Only modes.
    /// - orbit: [OrbitSource] must be provided for Direct (1D) PPP
    pub fn new(cfg: &Config, initial: Option<Orbit>, orbit: O) -> Result<Self, Error> {
        let (almanac, earth_cef) = Self::build_almanac()?;

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
            initial,
            cfg: cfg.clone(),
            prev_solution: None,
            // TODO
            ambiguity: AmbiguitySolver::new(Duration::from_seconds(120.0)),
            // postfit_kf: None,
            sv_orbits: HashMap::new(),
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
            signal_quality_filter(min_snr, &mut pool);
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
                    if let Some(mut tx_orbit) =
                        orbits.next_at(t_tx, cd.sv, self.earth_cef, interp_order)
                    {
                        let tx_orbit = if modeling.earth_rotation {
                            // TODO: missing rotate_by
                            // tx_orbit.with_position_rotated_by(Self::direct_cosine_rot3(dt_tx))
                            tx_orbit
                        } else {
                            tx_orbit
                        };
                        Some(cd.with_orbit(tx_orbit).with_propagation_delay(dt_tx))
                    } else {
                        // preserve: may still apply to RTK
                        Some(cd.clone())
                    }
                },
                Err(e) => {
                    error!("{} - transmision time error: {}", cd.sv, e);
                    None
                },
            })
            .collect();

        // initialize (if need be)
        if self.initial.is_none() {
            let solver = Bancroft::new(&pool)?;
            let output = solver.resolve()?;
            let (x0, y0, z0) = (output[0], output[1], output[2]);
            let orbit = Orbit::from_position(
                x0 / 1.0E3,
                y0 / 1.0E3,
                z0 / 1.0E3,
                pool[0].t,
                self.earth_cef,
            );
            let (lat_deg, long_deg, alt_km) = orbit.latlongalt().map_err(|e| Error::Physics(e))?;
            info!(
                "{} estimated initial position lat={:.5}°, lon={:.5}°, alt={:.3}m",
                pool[0].t,
                lat_deg,
                long_deg,
                alt_km * 1.0E3,
            );
            self.initial = Some(orbit); // store
        }

        // (local) state
        let rx_orbit = if let Some((_, prev_sol)) = &self.prev_solution {
            // TODO: we may have problems here in kinematics apps
            //       if latest solution is too old
            prev_sol.state
        } else {
            self.initial.unwrap()
        };

        let rx_geo = rx_orbit.latlongalt().map_err(|e| Error::Physics(e))?;

        let rx_rad = (rx_geo.0.to_radians(), rx_geo.1.to_radians());

        let rx_pos_vel = rx_orbit.to_cartesian_pos_vel() * 1.0E3;
        let (x0, y0, z0) = (rx_pos_vel[0], rx_pos_vel[1], rx_pos_vel[2]);

        // apply eclipse filter (if need be)
        if let Some(min_rate) = self.cfg.min_sv_sunlight_rate {
            pool.retain(|cd| {
                if let Some(orbit) = cd.orbit {
                    let eclipse_state =
                        eclipse_state(orbit, SUN_J2000, EARTH_J2000, &self.almanac).unwrap();
                    let eclipsed = match eclipse_state {
                        EclipseState::Umbra => true,
                        EclipseState::Visibilis => false,
                        EclipseState::Penumbra(r) => r < min_rate,
                    };
                    if eclipsed {
                        debug!("{} ({}): eclipsed", cd.t, cd.sv);
                    }
                    !eclipsed
                } else {
                    true // preserve for possible RTK
                }
            });
        }

        // sv fixup
        self.fix_sv_states(rx_orbit, &mut pool)?;
        Self::sv_state_filter(&self.cfg, &mut pool);

        // Apply models
        for cd in &mut pool {
            if let Some((el_deg, az_deg)) = cd.attitude() {
                cd.apply_models(
                    method,
                    tropo_modeling,
                    iono_modeling,
                    az_deg,
                    el_deg,
                    rx_geo,
                    rx_rad,
                )?;
            }
        }

        // Resolve ambiguities
        let ambiguities = if method == Method::PPP {
            self.ambiguity.resolve(&pool)
        } else {
            Default::default()
        };

        // Prepare for NAV
        pool.retain(|cd| {
            let retained = cd.is_navi_compatible();
            if !retained {
                debug!("{}({}): not proposed - missing data", cd.t, cd.sv);
            }
            retained
        });

        //  select best candidates, sort (coherent matrix), propose
        pool.retain(|cd| {
            let retained = cd.tropo_bias < max_tropo_bias;
            if retained {
                debug!("{}({}) - tropo delay {:.3E}[m]", cd.t, cd.sv, cd.tropo_bias);
            } else {
                debug!("{}({}) - rejected (extreme tropo delay)", cd.t, cd.sv);
            }
            retained
        });

        pool.retain(|cd| {
            let retained = cd.iono_bias < max_iono_bias;
            if retained {
                debug!("{}({}) - iono delay {:.3E}[m]", cd.t, cd.sv, cd.iono_bias);
            } else {
                debug!("{}({}) - rejected (extreme iono delay)", cd.t, cd.sv);
            }
            retained
        });

        if pool.len() < min_required {
            return Err(Error::NotEnoughPostFitCandidates);
        }

        let rx_orbit = if let Some((_, prev_sol)) = &self.prev_solution {
            prev_sol.state
        } else {
            self.initial.unwrap()
        };

        Self::retain_best_elevation(rx_orbit, &self.almanac, &mut pool, min_required);

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

        // self.prev_used = pool.iter().map(|cd| cd.sv).collect::<Vec<_>>();

        // Regular Iteration
        let output = match self.nav.resolve(&input) {
            Ok(output) => output,
            Err(e) => {
                error!("Failed to resolve: {}", e);
                return Err(Error::NavigationError);
            },
        };

        let sol_x = output.state.estimate();
        let sol_dt = sol_x[3] / SPEED_OF_LIGHT_M_S;
        let (sol_x, sol_y, sol_z) = (sol_x[0] + x0, sol_x[1] + y0, sol_x[2] + z0);

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
            state: Orbit::from_position(
                sol_x / 1.0E3,
                sol_y / 1.0E3,
                sol_z / 1.0E3,
                t,
                self.earth_cef,
            ),
            ambiguities,
            gdop: output.gdop,
            tdop: output.tdop,
            pdop: output.pdop,
            sv: input.sv.clone(),
            q: output.q_covar4x4(),
            timescale: self.cfg.timescale,
            dt: Duration::from_seconds(sol_dt),
            d_dt: 0.0_f64,
        };

        let (lat, long, alt_km) = solution.state.latlongalt().map_err(|e| Error::Physics(e))?;

        debug!(
            "{} new solution lat={:.5}°, long={:.5}°, alt={:.5}m",
            t,
            lat,
            long,
            alt_km * 1.0E3,
        );

        // First solution
        if self.prev_solution.is_none() {
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

        self.update_solution(t, &mut solution);
        // store for next time
        self.prev_solution = Some((t, solution.clone()));

        Self::rework_solution(t, self.earth_cef, &self.cfg, &mut solution);
        Ok((t, solution))
    }
    fn fix_sv_states(&mut self, rx_orbit: Orbit, pool: &mut Vec<Candidate>) -> Result<(), Error> {
        // clear loss of sight
        let svnn = pool.iter().map(|cd| cd.sv).collect::<Vec<_>>();
        self.sv_orbits.retain(|sv, _| {
            let retain = svnn.contains(&sv);
            if !retain {
                debug!("{} loss of sight", sv);
            }
            retain
        });

        for cd in pool.iter_mut() {
            if let Some(orbit) = &mut cd.orbit {
                // clock
                if orbit.vmag_km_s() > 0.0 {
                    if self.cfg.modeling.relativistic_clock_bias {
                        if let Some(clock_corr) = &mut cd.clock_corr {
                            if clock_corr.needs_relativistic_correction {
                                let w_e = Constants::EARTH_SEMI_MAJOR_AXIS_WGS84;
                                let mu = Constants::EARTH_GRAVITATION;
                                let ea_deg = orbit.ea_deg().map_err(Error::Physics)?;
                                let ea_rad = ea_deg.to_radians();
                                let gm = (w_e * mu).sqrt();
                                let bias = -2.0_f64
                                    * orbit.ecc().map_err(Error::Physics)?
                                    * ea_rad.sin()
                                    * gm
                                    / SPEED_OF_LIGHT_M_S
                                    / SPEED_OF_LIGHT_M_S
                                    * Unit::Second;
                                debug!("{} ({}) : relativistic clock bias: {}", cd.t, cd.sv, bias);
                                clock_corr.duration += bias;
                            }
                        }
                    } //clockbias
                } //velocity

                // orbit..
                let rx_orbit = Orbit::from_cartesian_pos_vel(
                    rx_orbit.to_cartesian_pos_vel(),
                    cd.t - cd.dt_tx,
                    self.earth_cef,
                );

                let elazrg = self
                    .almanac
                    .azimuth_elevation_range_sez(*orbit, rx_orbit)
                    .map_err(|e| Error::Almanac(e))?;

                cd.azimuth_deg = Some(elazrg.azimuth_deg);
                cd.elevation_deg = Some(elazrg.elevation_deg);
                self.sv_orbits.insert(cd.sv, *orbit);
            } //has_orbit
        }
        Ok(())
    }
    fn sv_state_filter(cfg: &Config, pool: &mut Vec<Candidate>) {
        let min_elev_deg = cfg.min_sv_elev.unwrap_or(0.0);
        let min_azim_deg = cfg.min_sv_azim.unwrap_or(0.0);
        let max_azim_deg = cfg.max_sv_azim.unwrap_or(360.0);
        pool.retain(|cd| {
            if let Some((elev, azim)) = cd.attitude() {
                if elev < min_elev_deg {
                    debug!("{}({}) - rejected (below elevation mask)", cd.t, cd.sv);
                    false
                } else if azim < min_azim_deg {
                    debug!("{}({}) - rejected (below azimuth mask)", cd.t, cd.sv);
                    false
                } else if azim > max_azim_deg {
                    debug!("{}({}) - rejected (above azimuth mask)", cd.t, cd.sv);
                    false
                } else {
                    debug!("{}({}) - elev={:.3}° azim={:.3}°", cd.t, cd.sv, elev, azim);
                    true
                }
            } else {
                true
            }
        });
    }
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
    fn direct_cosine_rot3(dt: Duration) -> Matrix3 {
        let we = Constants::EARTH_ANGULAR_VEL_RAD * dt.to_seconds();
        let (we_sin, we_cos) = we.sin_cos();
        Matrix3::new(we_cos, we_sin, 0.0, -we_sin, we_cos, 0.0, 0.0, 0.0, 1.0)
    }
    fn update_solution(&self, t: Epoch, sol: &mut PVTSolution) {
        if let Some((prev_t, prev_sol)) = &self.prev_solution {
            let dt_s = (t - *prev_t).to_seconds();
            // update velocity
            sol.state = Self::update_velocity(sol.state, prev_sol.state, dt_s);
            // update clock drift
            sol.d_dt = dt_s;
        }
    }
    fn update_velocity(orbit: Orbit, p_orbit: Orbit, dt_sec: f64) -> Orbit {
        let state = orbit.to_cartesian_pos_vel();
        let p_state = p_orbit.to_cartesian_pos_vel();
        let (x_km, y_km, z_km) = (state[0], state[1], state[2]);
        let (p_x_km, p_y_km, p_z_km) = (p_state[0], p_state[1], p_state[2]);
        let (vel_x_km_s, vel_y_km_s, vel_z_km_s) = (
            x_km - p_x_km / dt_sec,
            y_km - p_y_km / dt_sec,
            z_km - p_z_km / dt_sec,
        );
        orbit.with_velocity_km_s(Vector3::new(vel_x_km_s, vel_y_km_s, vel_z_km_s))
    }
    fn rework_solution(t: Epoch, frame: Frame, cfg: &Config, pvt: &mut PVTSolution) {
        // emphazise we only resolve dt by setting null attitude
        if cfg.sol_type == PVTSolutionType::TimeOnly {
            pvt.state = Orbit::zero_at_epoch(t, frame);
        }
        // TODO:
        //  1. replace height component with user input
        //  2. static in altitude: needs to reflect on velocity
        // to emphasize that it is being used
        if let Some(_alt_m) = cfg.fixed_altitude {}
    }
    fn retain_best_elevation(
        rx_orbit: Orbit,
        almanac: &Almanac,
        pool: &mut Vec<Candidate>,
        min_required: usize,
    ) {
        pool.sort_by(|cd_a, cd_b| {
            if let Some(orbit_a) = cd_a.orbit {
                let elev_a_deg =
                    if let Ok(elazrg) = almanac.azimuth_elevation_range_sez(rx_orbit, orbit_a) {
                        elazrg.elevation_deg
                    } else {
                        0.0
                    };
                if let Some(orbit_b) = cd_b.orbit {
                    let elev_b_deg = if let Ok(elazrg) =
                        almanac.azimuth_elevation_range_sez(rx_orbit, orbit_b)
                    {
                        elazrg.elevation_deg
                    } else {
                        0.0
                    };
                    elev_a_deg.partial_cmp(&elev_b_deg).unwrap()
                } else {
                    Ordering::Greater
                }
            } else {
                if cd_b.orbit.is_some() {
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
