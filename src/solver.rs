//! PVT solver
use std::collections::HashMap;

use gnss::prelude::SV;
use hifitime::Unit;
use log::{debug, error, warn};
use map_3d::{deg2rad, ecef2geodetic, Ellipsoid};
use nalgebra::{Matrix3, OMatrix, OVector, Vector3, U3};
use thiserror::Error;

use nyx::{
    cosmic::{
        eclipse::{eclipse_state, EclipseState},
        Orbit, SPEED_OF_LIGHT,
    },
    md::prelude::{Arc, Cosm, Frame},
    od::{filter::kalman::KF, process::KfEstimate},
};

use crate::{
    bancroft::Bancroft,
    bias::{IonosphereBias, TroposphereBias},
    candidate::Candidate,
    cfg::{Config, Method},
    navigation::{
        solutions::validator::Validator as SolutionValidator, Input as NavigationInput, Navigation,
        PVTSolution, PVTSolutionType, State3D,
    },
    position::Position,
    prelude::{Duration, Epoch},
    tracker::Tracker,
};

#[derive(Debug, Clone, PartialEq, Error)]
pub enum Error {
    #[error("not enough candidates provided")]
    NotEnoughCandidates,
    #[error("not enough candidates match pre-fit criteria")]
    NotEnoughMatchingCandidates,
    #[error("failed to form matrix (invalid input?)")]
    MatrixError,
    #[error("first guess failure")]
    FirstGuess,
    #[error("failed to invert matrix")]
    MatrixInversionError,
    #[error("resolved time is `nan` (invalid value(s))")]
    TimeIsNan,
    #[error("internal navigation error")]
    NavigationError,
    #[error("missing pseudo range observation")]
    MissingPseudoRange,
    #[error("failed to form pseudo range combination")]
    PseudoRangeCombination,
    #[error("failed to form phase range combination")]
    PhaseCombination,
    #[error("unresolved candidate state")]
    UnresolvedState,
    #[error("physical non sense: rx prior tx")]
    PhysicalNonSenseRxPriorTx,
    #[error("physical non sense: t_rx is too late")]
    PhysicalNonSenseRxTooLate,
    #[error("invalidated solution")]
    InvalidatedSolution,
    #[error("bancroft solver error: invalid input ?")]
    BancroftError,
    #[error("bancroft solver error: invalid input (imaginary solution)")]
    BancroftImaginarySolution,
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
    /// Builds InterpolationResults from Antenna Phase Center (APC) position coordinates,
    /// expressed in ECEF [m].
    pub fn from_position(position: (f64, f64, f64)) -> Self {
        Self {
            velocity: None,
            azimuth: 0.0_f64,
            elevation: 0.0_f64,
            position: Vector3::<f64>::new(position.0, position.1, position.2),
        }
    }
    /// Builds Self with given SV (elevation, azimuth) attitude
    pub fn with_elevation_azimuth(&self, elev_azim: (f64, f64)) -> Self {
        Self {
            azimuth: elev_azim.1,
            elevation: elev_azim.0,
            position: self.position,
            velocity: self.velocity,
        }
    }
    pub(crate) fn velocity(&self) -> Option<Vector3<f64>> {
        self.velocity
    }
    pub(crate) fn orbit(&self, dt: Epoch, frame: Frame) -> Orbit {
        let p = self.position;
        let v = self.velocity().unwrap_or_default();
        Orbit::cartesian(
            p[0] / 1.0E3,
            p[1] / 1.0E3,
            p[2] / 1.0E3,
            v[0] / 1.0E3,
            v[1] / 1.0E3,
            v[2] / 1.0E3,
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
    /// Interpolated SV state.
    interpolator: I,
    /// Initial [Position]
    initial: Option<Position>,
    /* Cosmic model */
    cosmic: Arc<Cosm>,
    // Solid / Earth body frame.
    earth_frame: Frame,
    // Sun / Star body frame
    sun_frame: Frame,
    // Navigator
    nav: Navigation,
    // Tracker
    tracker: Tracker,
    // Post fit KF
    // postfit_kf: Option<KF<State3D, U3, U3>>,
    /* prev. solution for internal logic */
    /// Previous solution
    prev_solution: Option<(Epoch, PVTSolution)>,
    /// Stored previous SV state
    prev_sv_state: HashMap<SV, (Epoch, Vector3<f64>)>,
}

impl<I: std::ops::Fn(Epoch, SV, usize) -> Option<InterpolationResult>> Solver<I> {
    /// Create a new Position [Solver].
    /// - cfg: Solver [Config]
    /// - initial: Possible initial [Position], used to initialize the Solver
    /// - interpolator: function pointer to external method to provide 3D interpolation results.
    pub fn new(cfg: &Config, initial: Option<Position>, interpolator: I) -> Result<Self, Error> {
        let cosmic = Cosm::de438();
        let sun_frame = cosmic.frame("Sun J2000");
        let earth_frame = cosmic.frame("EME2000");

        /*
         * print more infos
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
            initial,
            interpolator,
            cfg: cfg.clone(),
            tracker: Tracker::new(),
            // postfit_kf: None,
            prev_sv_state: HashMap::new(),
            nav: Navigation::new(cfg.solver.filter),
            prev_solution: Option::<(Epoch, PVTSolution)>::None,
        })
    }
    /// [PVTSolution] resolution attempt.
    /// ## Inputs
    /// - t: desired [Epoch]
    /// - pool: list of [Candidate]
    /// - iono_bias: TODO/Rework
    /// - tropo_bias: TODO/Rework
    pub fn resolve(
        &mut self,
        t: Epoch,
        pool: &Vec<Candidate>,
        iono_bias: &IonosphereBias,
        tropo_bias: &TroposphereBias,
    ) -> Result<(Epoch, PVTSolution), Error> {
        let min_required = Self::min_required(&self.cfg);

        if pool.len() < min_required {
            return Err(Error::NotEnoughCandidates);
        }

        let (x0, y0, z0) = if let Some((_, pvt)) = &self.prev_solution {
            //(pvt.position[0], pvt.position[1], pvt.position[2])
            let ecef = match &self.initial {
                Some(initial) => initial.ecef(),
                None => Vector3::<f64>::default(),
            };
            (ecef[0], ecef[1], ecef[2])
        } else {
            let ecef = match &self.initial {
                Some(initial) => initial.ecef(),
                None => Vector3::<f64>::default(),
            };
            (ecef[0], ecef[1], ecef[2])
        };

        let method = self.cfg.method;
        let modeling = self.cfg.modeling;
        let solver_opts = &self.cfg.solver;
        let _filter = solver_opts.filter;
        let interp_order = self.cfg.interp_order;

        /* apply signal quality and condition filters */
        let pool: Vec<Candidate> = pool
            .iter()
            .filter_map(|cd| match method {
                Method::SPP => {
                    let pr = cd.prefered_pseudorange()?;
                    if let Some(min_snr) = self.cfg.min_snr {
                        let snr = pr.snr?;
                        if snr < min_snr {
                            None
                        } else {
                            Some(cd.clone())
                        }
                    } else {
                        Some(cd.clone())
                    }
                },
                Method::CPP => {
                    if cd.cpp_compatible() {
                        // TODO: apply MIN SNR too, (when desired)
                        Some(cd.clone())
                    } else {
                        debug!("{} ({}) missing secondary frequency", cd.t, cd.sv);
                        None
                    }
                },
                Method::PPP => {
                    if cd.ppp_compatible() {
                        // TODO: apply MIN SNR too, (when desired)
                        Some(cd.clone())
                    } else {
                        debug!("{} ({}) missing secondary phase", cd.t, cd.sv);
                        None
                    }
                },
            })
            .collect();

        /* interpolate positions */
        let mut pool: Vec<Candidate> = pool
            .iter()
            .filter_map(|cd| match cd.transmission_time(&self.cfg) {
                Ok((t_tx, dt_tx)) => {
                    debug!("{} ({}) : signal propagation {}", cd.t, cd.sv, dt_tx);
                    let interpolated = (self.interpolator)(t_tx, cd.sv, interp_order)?;

                    let min_elev = self.cfg.min_sv_elev.unwrap_or(0.0_f64);
                    let min_azim = self.cfg.min_sv_azim.unwrap_or(0.0_f64);
                    let max_azim = self.cfg.max_sv_azim.unwrap_or(360.0_f64);

                    if interpolated.elevation < min_elev {
                        debug!(
                            "{} ({}) - {:?} rejected : below elevation mask",
                            cd.t, cd.sv, interpolated
                        );
                        None
                    } else if interpolated.azimuth < min_azim {
                        debug!(
                            "{} ({}) - {:?} rejected : below azimuth mask",
                            cd.t, cd.sv, interpolated
                        );
                        None
                    } else if interpolated.azimuth > max_azim {
                        debug!(
                            "{} ({}) - {:?} rejected : above azimuth mask",
                            cd.t, cd.sv, interpolated
                        );
                        None
                    } else {
                        let mut cd = cd.clone();
                        let interpolated =
                            Self::rotate_position(modeling.earth_rotation, interpolated, dt_tx);
                        let interpolated = self.velocities(t_tx, cd.sv, interpolated);
                        cd.t_tx = t_tx;
                        debug!("{} ({}) : {:?}", cd.t, cd.sv, interpolated);
                        cd.state = Some(interpolated);
                        Some(cd)
                    }
                },
                Err(e) => {
                    error!("{} - transmision time error: {}", cd.sv, e);
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
                    debug!("{} ({}) : relativistic clock bias: {}", cd.t, cd.sv, bias);
                    cd.clock_corr += bias;
                }
            }

            self.prev_sv_state
                .insert(cd.sv, (cd.t_tx, cd.state.unwrap().position));
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
                        "{} ({}): dropped - eclipsed by Earth",
                        pool[idx - nb_removed].t,
                        pool[idx - nb_removed].sv
                    );
                    let _ = pool.swap_remove(idx - nb_removed);
                    nb_removed += 1;
                }
            }
        }

        // sort by quality
        pool.sort_by(|cd_a, cd_b| {
            let state_a = cd_a.state.unwrap();
            let state_b = cd_b.state.unwrap();
            state_b.elevation.partial_cmp(&state_a.elevation).unwrap()
        });

        // Retain required nb of SV
        if self.prev_solution.is_none() && self.initial.is_none() {
            // Bancroft needs 4 SV
            let mut index = 0;
            pool.retain(|_| {
                index += 1;
                index < 5
            });
        } else {
            // Regular iteration
            let mut index = 0;
            let min_required = match self.cfg.sol_type {
                PVTSolutionType::TimeOnly => 2,
                _ => min_required + 1,
            };
            pool.retain(|_| {
                index += 1;
                index < min_required
            });
        }

        if pool.len() != min_required {
            return Err(Error::NotEnoughMatchingCandidates);
        }

        // sort by PRN to form consistant matrix
        pool.sort_by(|cd_a, cd_b| cd_a.sv.prn.partial_cmp(&cd_b.sv.prn).unwrap());

        let (lat_ddeg, lon_ddeg, altitude_above_sea_m) = if let Some((_, pvt)) = &self.prev_solution
        {
            let geo = ecef2geodetic(
                pvt.position[0],
                pvt.position[1],
                pvt.position[2],
                Ellipsoid::WGS84,
            );
            (deg2rad(geo.0), deg2rad(geo.1), geo.2)
        } else {
            let geo = match &self.initial {
                Some(initial) => initial.geodetic(),
                None => Vector3::<f64>::default(),
            };
            (deg2rad(geo[0]), deg2rad(geo[1]), geo[2])
        };

        let input = match NavigationInput::new(
            (x0, y0, z0),
            (lat_ddeg, lon_ddeg, altitude_above_sea_m),
            &self.cfg,
            &pool,
            iono_bias,
            tropo_bias,
        ) {
            Ok(input) => input,
            Err(e) => {
                error!("Failed to form navigation matrix: {}", e);
                return Err(Error::MatrixError);
            },
        };

        // First guess
        if self.prev_solution.is_none() && self.initial.is_none() {
            let solver = match Bancroft::new(&pool) {
                Ok(solver) => solver,
                Err(e) => {
                    error!("failed to deploy bancroft solver: {}", e);
                    return Err(Error::BancroftError);
                },
            };
            let output = match solver.resolve() {
                Ok(output) => output,
                Err(e) => {
                    error!("initial guess failure: {}", e);
                    return Err(Error::FirstGuess);
                },
            };
            panic!("todo");
        }

        // Regular Iteration
        let output = match self.nav.resolve(&input) {
            Ok(output) => output,
            Err(e) => {
                error!("Failed to resolve: {}", e);
                return Err(Error::NavigationError);
            },
        };

        let x = output.state.estimate();

        let mut solution = PVTSolution {
            gdop: output.gdop,
            tdop: output.tdop,
            pdop: output.pdop,
            sv: input.sv.clone(),
            q: output.q_covar4x4(),
            timescale: self.cfg.timescale,
            velocity: Vector3::<f64>::default(),
            position: Vector3::new(x[0] + x0, x[1] + y0, x[2] + z0),
            dt: Duration::from_seconds(x[3] / SPEED_OF_LIGHT),
        };

        // First solution
        if self.prev_solution.is_none() {
            // always discard 1st solution
            self.prev_solution = Some((t, solution.clone()));
            return Err(Error::InvalidatedSolution);
        }

        let validator =
            SolutionValidator::new(Vector3::<f64>::new(x0, y0, z0), &pool, &input, &output);

        match validator.validate(solver_opts) {
            Ok(_) => {
                if method == Method::PPP {
                    let ambiguities = output.state.ambiguities();
                    for (i, sv) in input.sv.keys().enumerate() {
                        debug!("{} - {} amb: {}", t, sv, ambiguities[i]);
                        self.tracker.update(*sv, ambiguities[i]);
                    }
                }
                self.nav.validate();
            },
            Err(e) => {
                error!("solution invalidated - {}", e);
                return Err(Error::InvalidatedSolution);
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
            //            t: Epoch::from_gpst_seconds(x[3] / SPEED_OF_LIGHT),
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
            solution.velocity =
                (solution.position - prev_solution.position) / (t - *prev_t).to_seconds();
        }

        self.prev_solution = Some((t, solution.clone()));

        Self::rework_solution(&mut solution, &self.cfg);
        Ok((t, solution))
    }
    /*
     * Returns nb of vehicles we need to gather
     */
    fn min_required(cfg: &Config) -> usize {
        match cfg.sol_type {
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
    fn velocities(
        &self,
        t_tx: Epoch,
        sv: SV,
        interpolated: InterpolationResult,
    ) -> InterpolationResult {
        let mut reworked = interpolated;
        if let Some((p_ttx, p_pos)) = self.prev_sv_state.get(&sv) {
            let dt = (t_tx - *p_ttx).to_seconds();
            reworked.velocity = Some((interpolated.position - p_pos) / dt);
        }
        reworked
    }
    /*
     * Reworks solution to adapt to configuration setup and desired format.
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
}
