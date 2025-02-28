//! PVT solver
use nyx::cosmic::{
    // eclipse::EclipseLocator,
    SPEED_OF_LIGHT_M_S,
};

use std::{
    collections::HashMap,
    fs::{create_dir_all, File},
    io::Write,
};

use itertools::Itertools;
use log::{debug, error, info, warn};

use anise::{
    almanac::{
        metaload::{MetaAlmanac, MetaAlmanacError, MetaFile},
        planetary::PlanetaryDataError,
    },
    astro::PhysicsResult,
    constants::frames::{EARTH_ITRF93, IAU_EARTH_FRAME, SUN_J2000},
    errors::{AlmanacError, PhysicsError},
    math::{Matrix3, Vector3, Vector6},
    prelude::{Almanac, Frame, Unit},
};

use crate::{
    //ambiguity::Solver as AmbiguitySolver,
    bancroft::Bancroft,
    candidate::Candidate,
    cfg::{Config, Method},
    constants::Constants,
    navigation::{
        //solutions::validator::{InvalidationCause, Validator as SolutionValidator},
        //Input as NavigationInput,
        Navigation,
        PVTSolution,
        PVTSolutionType,
        State,
    },
    orbit::OrbitSource,
    prelude::{Duration, Epoch, Error, Orbit, SV},
};

/// [Solver] to resolve [PVTSolution]s.
pub struct Solver<O: OrbitSource> {
    /// Solver [Config]uration preset
    pub cfg: Config,
    /// [OrbitSource]
    orbit: O,
    /// [Almanac]
    almanac: Almanac,
    /// [Frame]
    earth_cef: Frame,
    /// Latest [State].
    state: Option<State>,
    /// Possible initial (very first) preset
    initial_ecef_m: Option<Vector3>,
    // /// [AmbiguitySolver]
    // ambiguity: AmbiguitySolver,
    // Post fit KF
    // postfit_kf: Option<KF<State3D, U3, U3>>,
    /* prev. solution for internal logic */
    /// Previous solution (internal logic)
    prev_solution: Option<(Epoch, PVTSolution)>,
    /// Stored sky view, for internal logic.
    sv_orbits: HashMap<SV, Orbit>,
}

impl<O: OrbitSource> Solver<O> {
    const ALMANAC_LOCAL_STORAGE: &str = ".cache";

    fn nyx_anise_de440s_bsp() -> MetaFile {
        MetaFile {
            crc32: Some(1921414410),
            uri: String::from("http://public-data.nyxspace.com/anise/de440s.bsp"),
        }
    }

    fn nyx_anise_pck11_pca() -> MetaFile {
        MetaFile {
            crc32: Some(0x8213b6e9),
            uri: String::from("http://public-data.nyxspace.com/anise/v0.5/pck11.pca"),
        }
    }

    fn nyx_anise_jpl_bpc() -> MetaFile {
        MetaFile {
            crc32: None,
            uri:
                "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc"
                    .to_string(),
        }
    }

    /// Builds an [Almanac] and [Frame] model, with prefered
    /// high precision models. We only use Earth centered [Frame],
    /// so this library is currently limited Earth ground navigation.
    /// We always prefer the highest precision model, which requires daily internet access.
    /// If internet access is in failure, the [Almanac] relies on an offline model.
    fn build_almanac_frame_model() -> Result<(Almanac, Frame), Error> {
        let mut initial_setup = false;

        // Meta almanac for local storage management
        let local_storage = format!(
            "{}/{}/anise.dhall",
            env!("CARGO_MANIFEST_DIR"),
            Self::ALMANAC_LOCAL_STORAGE
        );

        let mut meta_almanac = match MetaAlmanac::new(local_storage.clone()) {
            Ok(meta) => {
                debug!("(anise) from local storage");
                meta
            },
            Err(_) => {
                debug!("(anise) local storage setup");
                initial_setup = true;
                MetaAlmanac {
                    files: vec![
                        Self::nyx_anise_de440s_bsp(),
                        Self::nyx_anise_pck11_pca(),
                        Self::nyx_anise_jpl_bpc(),
                    ],
                }
            },
        };

        // download (if need be)
        let almanac = meta_almanac.process(true).map_err(|e| Error::Almanac(e))?;

        if initial_setup {
            let updated = meta_almanac.dumps().map_err(|e| Error::MetaAlmanac(e))?;

            let _ = create_dir_all(&format!(
                "{}/{}",
                env!("CARGO_MANIFEST_DIR"),
                Self::ALMANAC_LOCAL_STORAGE
            ));

            let mut fd = File::create(&local_storage)
                .unwrap_or_else(|e| panic!("almanac storage setup error: {}", e));

            fd.write_all(updated.as_bytes())
                .unwrap_or_else(|e| panic!("almanac storage setup error: {}", e));
        }

        match almanac.frame_from_uid(EARTH_ITRF93) {
            Ok(itrf93) => {
                info!("highest precision context setup");
                return Ok((almanac, itrf93));
            },
            Err(e) => {
                error!("(anise) jpl_bpc: {}", e);
            },
        }

        let earth_cef = almanac
            .frame_from_uid(IAU_EARTH_FRAME)
            .map_err(|e| Error::EarthFrame(e))?;

        warn!("deployed with offline model");
        Ok((almanac, earth_cef))
    }

    /// Create a new Position [Solver] that may support any positioning technique
    /// and uses internal [Almanac] and [Frame] model definition.
    /// ## Input
    /// - cfg: Solver [Config]
    /// - orbit: [OrbitSource] must be provided for Direct (1D) PPP
    /// - state_ecef_m: possible initial state expressed expressed as meters in ECEF.
    /// When not provided, the solver deploys in survey mode with stringent requirements
    /// on first iteration.
    pub fn new(
        cfg: &Config,
        orbit: O,
        state_ecef_m: Option<(f64, f64, f64)>,
    ) -> Result<Self, Error> {
        let (almanac, earth_cef) = Self::build_almanac_frame_model()?;

        // Analyze preset
        if cfg.method == Method::SPP && cfg.max_sv_occultation_percent.is_some() {
            warn!("occultation filter is not meaningful in SPP navigation");
        }

        if cfg.externalref_delay.is_some() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is either incomplete or not entirely enabled");
        }

        if !cfg.int_delay.is_empty() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is not fully supported yet.");
        }

        // let eclipse = EclipseLocator::cislunar(Arc::new(almanac.clone()));
        // let almanac = Arc::new(almanac);

        let initial_ecef_m = match state_ecef_m {
            Some((x0, y0, z0)) => Some(Vector3::new(x0, y0, z0)),
            _ => None,
        };

        Ok(Self {
            orbit,
            almanac,
            earth_cef,
            state: None,
            cfg: cfg.clone(),
            initial_ecef_m,
            prev_solution: None,
            sv_orbits: HashMap::new(),
        })
    }

    /// Create new Position [Solver] without knowledge of apriori position (full survey)
    pub fn new_survey(cfg: &Config, orbit: O) -> Result<Self, Error> {
        Self::new(cfg, orbit, None)
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
        let max_iono_bias = self.cfg.max_iono_bias;
        let max_tropo_bias = self.cfg.max_tropo_bias;
        let iono_modeling = self.cfg.modeling.iono_delay;
        let tropo_modeling = self.cfg.modeling.tropo_delay;

        Self::signal_condition_filter(t, method, &mut pool);

        if let Some(min_snr) = self.cfg.min_snr {
            Self::signal_quality_filter(min_snr, &mut pool);
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
                    if let Some(tx_orbit) = orbits.next_at(t_tx, cd.sv, self.earth_cef) {
                        let orbit = Self::rotate_orbit_dcm3x3(
                            cd.t,
                            dt_tx,
                            tx_orbit,
                            modeling.earth_rotation,
                            self.earth_cef,
                        );
                        Some(cd.with_orbit(orbit))
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

        // First iteration (special case)
        if self.state.is_none() {
            match self.initial_ecef_m {
                Some(initial_ecef_m) => {
                    // Define initial state
                    debug!("{}: initialized with {}", t, initial_ecef_m);
                    self.state = Some(State::from_ecef_m(initial_ecef_m, t, self.earth_cef));
                },
                None => {
                    // Special solver
                    let solver = Bancroft::new(&pool)?;
                    let solution = solver.resolve()?;

                    debug!(
                        "{}: initial solution: {:?} [COMPARE LOCAL TIME HERE PLEASE]",
                        t, solution
                    );

                    let coords = Vector3::new(solution[0], solution[1], solution[2]);

                    // Define initial state
                    self.state = Some(State::from_ecef_m(coords, t, self.earth_cef));
                },
            }
        }

        let state = self
            .state
            .as_mut()
            .expect("internal error: bad initialization logic!");

        // Real time navigation

        // Eclipse filter (if need be)
        if let Some(max_occultation_rate) = self.cfg.max_sv_occultation_percent {
            pool.retain(|cd| {
                if let Some(sv_orbit) = cd.orbit {
                    match self
                        .almanac
                        .occultation(SUN_J2000, self.earth_cef, sv_orbit, None)
                    {
                        Ok(occultation) => {
                            if occultation.percentage > max_occultation_rate {
                                false // filter out
                            } else {
                                true // preserve
                            }
                        },
                        Err(e) => {
                            error!("(anise) eclipse error: {}", e);
                            // discard in this case
                            false
                        },
                    }
                } else {
                    // Undefined orbital state: can't apply.
                    true
                }
            });
        }

        self.fix_sv_states(&mut pool)?;

        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughPreFitCandidates);
        }

        Self::sv_attitude_filters(&self.cfg, &mut pool);

        // tropo + ionosphere modeling, if need be
        for cd in &mut pool {
            if let Some(rtm) = cd.to_bias_runtime(t, &state) {
                if self.cfg.modeling.tropo_delay {}

                if self.cfg.modeling.iono_delay {}
            }
        }

        // // Resolve ambiguities
        // let ambiguities = if method == Method::PPP {
        //     self.ambiguity.resolve(&pool)
        // } else {
        //     Default::default()
        // };

        // Prepare for NAV
        pool.retain(|cd| {
            let retained = cd.is_navi_compatible();
            if !retained {
                debug!("{}({}): not proposed - missing data", cd.t, cd.sv);
            }
            retained
        });

        // last verification (tropo)
        pool.retain(|cd| {
            let retained = cd.tropo_bias_m < max_tropo_bias;
            if retained {
                debug!(
                    "{}({}) - tropo delay {:.3E}[m]",
                    cd.t, cd.sv, cd.tropo_bias_m
                );
            } else {
                debug!("{}({}) - rejected (extreme tropo delay)", cd.t, cd.sv);
            }
            retained
        });

        // last verification (tropo)
        // TODO: when cancelled from signal, fill cd.iono_bias_m
        pool.retain(|cd| {
            let retained = cd.iono_bias_m < max_iono_bias;
            if retained {
                debug!("{}({}) - iono delay {:.3E}[m]", cd.t, cd.sv, cd.iono_bias_m);
            } else {
                debug!("{}({}) - rejected (extreme iono delay)", cd.t, cd.sv);
            }
            retained
        });

        if pool.len() < min_required {
            return Err(Error::NotEnoughPostFitCandidates);
        }

        Self::update_sky_view(&pool, &mut self.sv_orbits);

        // TODO ?
        // pool.sort_by(|cd_a, cd_b| cd_a.sv.prn.partial_cmp(&cd_b.sv.prn).unwrap());

        let w = self.cfg.solver.weight_matrix(); //sv.values().map(|sv| sv.elevation).collect());
                                                 // // Reduce contribution of newer (rising) vehicles (rising)
                                                 // for (i, cd) in pool.iter().enumerate() {
                                                 //     if !self.prev_used.contains(&cd.sv) {
                                                 //         w[(i, i)] = 0.05;
                                                 //         w[(2 * i, 2 * i)] = 0.05;
                                                 //     }
                                                 // }

        // Build nav Matrix
        let mut nav = match Navigation::new(&self.cfg, &state, &pool) {
            Ok(nav) => nav,
            Err(e) => {
                error!("matrix formation error: {}", e);
                return Err(e);
            },
        };

        let mut i = 0;
        loop {
            match nav.iter() {
                Ok(_) => {
                    debug!("iter={} {:?}", i, nav.dx);

                    state.update(nav.dx);

                    let norm = (nav.dx[0].powi(2) + nav.dx[1].powi(2) + nav.dx[3].powi(2)).sqrt();
                    if norm < 1E-6 {
                        // reached unrealistic correction
                        break;
                    }

                    i += 1;
                },
                Err(e) => {
                    error!("iter={}, FAILURE: {}", i, e);
                },
            }
        }

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

        // // Form Solution
        // let mut solution = PVTSolution {
        //     state: Orbit::from_position(
        //         sol_x / 1.0E3,
        //         sol_y / 1.0E3,
        //         sol_z / 1.0E3,
        //         t,
        //         self.earth_cef,
        //     ),
        //     ambiguities,
        //     gdop: output.gdop,
        //     tdop: output.tdop,
        //     pdop: output.pdop,
        //     sv: input.sv.clone(),
        //     q: output.q_covar4x4(),
        //     timescale: self.cfg.timescale,
        //     dt: Duration::from_seconds(sol_dt),
        //     d_dt: 0.0_f64,
        // };

        //let (lat, long, alt_km) = solution.state.latlongalt().map_err(|e| Error::Physics(e))?;

        // debug!(
        //     "{} new solution lat={:.5}°, long={:.5}°, alt={:.5}m",
        //     t,
        //     lat,
        //     long,
        //     alt_km * 1.0E3,
        // );

        // // First solution
        // if self.prev_solution.is_none() {
        //     self.prev_solution = Some((t, solution.clone()));
        //     // always discard 1st solution
        //     return Err(Error::InvalidatedSolution(InvalidationCause::FirstSolution));
        // }

        // let validator =
        //     SolutionValidator::new(Vector3::<f64>::new(x0, y0, z0), &pool, &input, &output);

        // match validator.validate(&self.cfg) {
        //     Ok(_) => {
        //         self.nav.validate();
        //     },
        //     Err(cause) => {
        //         error!("solution invalidated - {}", cause);
        //         return Err(Error::InvalidatedSolution(cause));
        //     },
        // };

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

        // update & store for next time
        //self.update_solution(t, &mut solution);

        // Self::rework_solution(t, self.earth_cef, &self.cfg, &mut solution);
        //Ok((t, solution))
        Err(Error::MissingRemoteRTKObservation(
            Default::default(),
            Default::default(),
        ))
    }

    /// Apply signal quality criteria
    fn signal_quality_filter(min_snr: f64, pool: &mut Vec<Candidate>) {
        pool.retain_mut(|cd| {
            cd.min_snr_mask(min_snr);
            !cd.observations.is_empty()
        })
    }

    /// Apply signal condition criteria
    fn signal_condition_filter(t: Epoch, method: Method, pool: &mut Vec<Candidate>) {
        pool.retain(|cd| match method {
            Method::SPP => {
                if cd.l1_phase_range_m().is_some() {
                    true
                } else {
                    error!("{} ({}) missing pseudo range observation", t, cd.sv);
                    false
                }
            },
            Method::CPP => {
                if cd.cpp_compatible() {
                    true
                } else {
                    debug!("{} ({}) missing secondary frequency", t, cd.sv);
                    false
                }
            },
            Method::PPP => {
                if cd.ppp_compatible() {
                    true
                } else {
                    debug!("{} ({}) missing phase or phase combination", t, cd.sv);
                    false
                }
            },
        })
    }

    /// - Define attitude [elev, azim]
    /// - Define velocities when feasible
    /// - Account for relativistic effects
    fn fix_sv_states(&self, pool: &mut Vec<Candidate>) -> Result<(), Error> {
        let mu = Constants::EARTH_GRAVITATION;
        let w_e = Constants::EARTH_SEMI_MAJOR_AXIS_WGS84;

        let state = self
            .state
            .as_ref()
            .expect("internal error: undefined state");

        let rx_orbit = state.to_orbit(self.earth_cef);

        for cd in pool.iter_mut() {
            if let Some(orbit) = &mut cd.orbit {
                // Define velocities when feasible
                if let Some(past_orbit) = self.sv_orbits.get(&cd.sv) {
                    let dt_s = (orbit.epoch - past_orbit.epoch).to_seconds();
                    let pos_vel_km_s = orbit.to_cartesian_pos_vel();
                    let pos_vel_z1_km_s = past_orbit.to_cartesian_pos_vel();

                    let vel_km_s = (
                        (pos_vel_km_s[0] - pos_vel_z1_km_s[0]) / dt_s,
                        (pos_vel_km_s[1] - pos_vel_z1_km_s[1]) / dt_s,
                        (pos_vel_km_s[2] - pos_vel_z1_km_s[2]) / dt_s,
                    );

                    *orbit =
                        orbit.with_velocity_km_s(Vector3::new(vel_km_s.0, vel_km_s.1, vel_km_s.2));
                }

                // relativistic clock
                if orbit.vmag_km_s() > 0.0 {
                    if self.cfg.modeling.relativistic_clock_bias {
                        if let Some(clock_corr) = &mut cd.clock_corr {
                            if clock_corr.needs_relativistic_correction {
                                let ea_deg = orbit.ea_deg().map_err(Error::Physics)?;

                                let ea_rad = ea_deg.to_radians();
                                let gm = (w_e * mu).sqrt();
                                let ecc = orbit.ecc().map_err(Error::Physics)?;

                                let bias = -2.0_f64 * ecc * ea_rad.sin() * gm
                                    / SPEED_OF_LIGHT_M_S
                                    / SPEED_OF_LIGHT_M_S
                                    * Unit::Second;

                                debug!("{} ({}) : relativistic clock bias: {}", cd.t, cd.sv, bias);
                                clock_corr.duration += bias;
                            }
                        }
                    } //clockbias
                } //velocity

                let elazrg = self
                    .almanac
                    .azimuth_elevation_range_sez(*orbit, rx_orbit, None, None)
                    .map_err(|e| Error::Almanac(e))?;

                cd.azimuth_deg = Some(elazrg.azimuth_deg);
                cd.elevation_deg = Some(elazrg.elevation_deg);
            }
        }
        Ok(())
    }

    // Apply attitude filters
    fn sv_attitude_filters(cfg: &Config, pool: &mut Vec<Candidate>) {
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

    fn update_sky_view(pool: &[Candidate], sv_orbits: &mut HashMap<SV, Orbit>) {
        // Update local sky buffer
        let svnn = pool.iter().map(|cd| cd.sv).unique().collect::<Vec<_>>();

        sv_orbits.retain(|sv, _| {
            let retain = svnn.contains(&sv);
            if !retain {
                debug!("{} loss of sight", sv);
            }
            retain
        });
    }

    fn min_sv_required(&self) -> usize {
        if self.state.is_none() {
            4
        } else {
            match self.cfg.solution {
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

    fn rotate_orbit_dcm3x3(
        t: Epoch,
        dt: Duration,
        orbit: Orbit,
        modeling: bool,
        frame: Frame,
    ) -> Orbit {
        let we = Constants::EARTH_ANGULAR_VEL_RAD * dt.to_seconds();
        let (we_sin, we_cos) = we.sin_cos();
        let dcm3 = if modeling {
            Matrix3::new(we_cos, we_sin, 0.0, -we_sin, we_cos, 0.0, 0.0, 0.0, 1.0)
        } else {
            Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        };
        let state = orbit.to_cartesian_pos_vel() * 1.0E3;
        let position = Vector3::new(state[0], state[1], state[2]);
        let position = dcm3 * position;
        Orbit::from_position(
            position[0] / 1.0E3,
            position[1] / 1.0E3,
            position[2] / 1.0E3,
            t,
            frame,
        )
    }

    fn update_solution(&self, t: Epoch, sol: &mut PVTSolution) {
        if let Some((prev_t, prev_sol)) = &self.prev_solution {
            let dt_s = (t - *prev_t).to_seconds();
            // update clock drift
            sol.d_dt = dt_s;
            // update velocity
            sol.state = Self::update_velocity(sol.state, prev_sol.state, dt_s);
        }
    }

    fn update_velocity(orbit: Orbit, p_orbit: Orbit, dt_sec: f64) -> Orbit {
        let state = orbit.to_cartesian_pos_vel();
        let p_state = p_orbit.to_cartesian_pos_vel();
        let (x, y, z) = (state[0], state[1], state[2]);
        let (p_x, p_y, p_z) = (p_state[0], p_state[1], p_state[2]);
        orbit.with_velocity_km_s(Vector3::new(
            (x - p_x) / dt_sec,
            (y - p_y) / dt_sec,
            (z - p_z) / dt_sec,
        ))
    }

    // fn rework_solution(t: Epoch, frame: Frame, cfg: &Config, pvt: &mut PVTSolution) {
    //     // emphazise we only resolve dt by setting null attitude
    //     if cfg.sol_type == PVTSolutionType::TimeOnly {
    //         pvt.state = Orbit::zero_at_epoch(t, frame);
    //     }
    //     // TODO:
    //     //  1. replace height component with user input
    //     //  2. static in altitude: needs to reflect on velocity
    //     // to emphasize that it is being used
    //     if let Some(_alt_m) = cfg.fixed_altitude {}
    // }

    fn retain_best_elevation(pool: &mut Vec<Candidate>, min_required: usize) {
        pool.sort_by(|cd_a, cd_b| {
            let elev_a_deg = cd_a.elevation_deg.unwrap_or_default();
            let elev_b_deg = cd_b.elevation_deg.unwrap_or_default();
            elev_a_deg.partial_cmp(&elev_b_deg).unwrap()
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
