//! PVT solver
use std::collections::HashMap;

use hifitime::Unit;
// use itertools::Itertools;
use log::{debug, error, info, warn};
use map_3d::{deg2rad, ecef2geodetic, rad2deg};
use nalgebra::{Matrix3, Vector3};
// use std::cmp::Ordering;
use std::f64::consts::PI;
use thiserror::Error;

use nyx::cosmic::eclipse::{eclipse_state, EclipseState};

use anise::{
    constants::{
        frames::{EARTH_J2000, SUN_J2000},
        SPEED_OF_LIGHT_KM_S,
    },
    errors::{AlmanacError, PhysicsError},
    prelude::{Almanac, Frame, Orbit},
};

use crate::{
    ambiguity::AmbiguitySolver,
    bancroft::Bancroft,
    bias::{IonosphereBias, TroposphereBias},
    candidate::Candidate,
    cfg::{Config, Method},
    navigation::{
        solutions::validator::{InvalidationCause, Validator as SolutionValidator},
        Input as NavigationInput, Navigation, PVTSolution, PVTSolutionType,
    },
    position::Position,
    prelude::{Duration, Epoch, SV},
};

#[derive(Debug, PartialEq, Error)]
pub enum Error {
    #[error("not enough candidates provided")]
    NotEnoughCandidates,
    #[error("not enough candidates match pre-fit criteria")]
    NotEnoughMatchingCandidates,
    #[error("non supported/invalid strategy")]
    InvalidStrategy,
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
    PhaseRangeCombination,
    #[error("unresolved candidate state")]
    UnresolvedState,
    #[error("physical non sense: rx prior tx")]
    PhysicalNonSenseRxPriorTx,
    #[error("physical non sense: t_rx is too late")]
    PhysicalNonSenseRxTooLate,
    #[error("invalidated solution, cause: {0}")]
    InvalidatedSolution(InvalidationCause),
    #[error("bancroft solver error: invalid input ?")]
    BancroftError,
    #[error("bancroft solver error: invalid input (imaginary solution)")]
    BancroftImaginarySolution,
    #[error("unresolved signal ambiguity")]
    UnresolvedAmbiguity,
    #[error("issue with Almanac: {0}")]
    Almanac(AlmanacError),
    #[error("physics issue: {0}")]
    Physics(PhysicsError),
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
    /// Augment self to fully defined with both Elevation and Azimuth angles
    pub(crate) fn with_elevation_azimuth(&self, position: (f64, f64, f64)) -> Self {
        let (x, y, z) = position;
        let (lat_rad, lon_rad, _) = ecef2geodetic(x, y, z, map_3d::Ellipsoid::WGS84);

        let (sv_x, sv_y, sv_z) = (self.position[0], self.position[1], self.position[2]);
        let a_i = (sv_x - x, sv_y - y, sv_z - z);
        let norm = (a_i.0.powf(2.0) + a_i.1.powf(2.0) + a_i.2.powf(2.0)).sqrt();
        let a_i = (a_i.0 / norm, a_i.1 / norm, a_i.2 / norm);

        let ecef_to_ven = (
            (
                lat_rad.cos() * lon_rad.cos(),
                lat_rad.cos() * lon_rad.sin(),
                lat_rad.sin(),
            ),
            (-lon_rad.sin(), lon_rad.cos(), 0.0_f64),
            (
                -lat_rad.sin() * lon_rad.cos(),
                -lat_rad.sin() * lon_rad.sin(),
                lat_rad.cos(),
            ),
        );
        // ECEF to VEN transform
        let ven = (
            (ecef_to_ven.0 .0 * a_i.0 + ecef_to_ven.0 .1 * a_i.1 + ecef_to_ven.0 .2 * a_i.2),
            (ecef_to_ven.1 .0 * a_i.0 + ecef_to_ven.1 .1 * a_i.1 + ecef_to_ven.1 .2 * a_i.2),
            (ecef_to_ven.2 .0 * a_i.0 + ecef_to_ven.2 .1 * a_i.1 + ecef_to_ven.2 .2 * a_i.2),
        );
        let elevation = rad2deg(PI / 2.0 - ven.0.acos());
        let mut azimuth = rad2deg(ven.1.atan2(ven.2));
        if azimuth < 0.0 {
            azimuth += 360.0;
        }
        Self {
            azimuth,
            elevation,
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
    #[cfg(test)]
    fn set_elevation(&mut self, elev: f64) {
        self.elevation = elev;
    }
}

/// PVT Solver.
/// I: Interpolated SV APC coordinates interface.
/// You are required to provide APC coordinates at requested ("t", "sv"),
/// expressed in meters [ECEF], for this to proceed.
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
    // Almanac, is loaded when deploying.
    // Currently requires network access (at least on 1st deployment)
    almanac: Almanac,
    // Navigator
    nav: Navigation,
    // Solver
    ambiguity: AmbiguitySolver,
    // Post fit KF
    // postfit_kf: Option<KF<State3D, U3, U3>>,
    /* prev. solution for internal logic */
    /// Previous solution
    prev_solution: Option<(Epoch, PVTSolution)>,
    /// Previous VDOP
    prev_vdop: Option<f64>,
    /// Previously used
    prev_used: Vec<SV>,
    /// Stored previous SV state
    prev_sv_state: HashMap<SV, (Epoch, Vector3<f64>)>,
}

impl<I: std::ops::Fn(Epoch, SV, usize) -> Option<InterpolationResult>> Solver<I> {
    /// Create a new Position [Solver].
    /// - cfg: Solver [Config]
    /// - initial: possible initial [Position], used to initialize the Solver.
    ///   When not provided (no apriori knowledge), the solver will initialize itself autonomously.
    ///   Note that we need at least one set of 4 valid SV observations to initiliaze ourselves.
    ///   This means you need to have special initialization cycles when operating
    ///   in Fixed Altitude or Time Only modes.
    /// - interpolator: function pointer to external method to provide 3D interpolation results.
    pub fn new(cfg: &Config, initial: Option<Position>, interpolator: I) -> Result<Self, Error> {
        // Regularly refer to https://github.com/nyx-space/anise/blob/master/data/ci_config.dhall for the latest CRC, although it should not change between minor versions!
        // NB: a default almanac will soon be provided by ANISE directly
        //     this triggers a network access at least once
        let almanac = Almanac::until_2035().map_err(Error::Almanac)?;

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
            almanac,
            initial: {
                if let Some(ref initial) = initial {
                    let geo = initial.geodetic();
                    let (lat, lon) = (rad2deg(geo[0]), rad2deg(geo[1]));
                    info!("initial position lat={:.3E}°, lon={:.3E}°", lat, lon);
                }
                initial
            },
            interpolator,
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
    /// [PVTSolution] resolution attempt.
    /// ## Inputs
    /// - t: desired [Epoch]
    /// - pool: list of [Candidate]
    /// - iono_bias: TODO/Rework
    /// - tropo_bias: TODO/Rework
    pub fn resolve(
        &mut self,
        t: Epoch,
        pool: &[Candidate],
        iono_bias: &IonosphereBias,
        tropo_bias: &TroposphereBias,
    ) -> Result<(Epoch, PVTSolution), Error> {
        let min_required = self.min_sv_required();
        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughCandidates);
        }

        let method = self.cfg.method;
        let modeling = self.cfg.modeling;
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

        let earth_j2000 = self.almanac.frame_from_uid(EARTH_J2000).unwrap();

        /* interpolate positions */
        let mut pool: Vec<Candidate> = pool
            .iter()
            .filter_map(|cd| match cd.transmission_time(&self.cfg) {
                Ok((t_tx, dt_tx)) => {
                    debug!("{} ({}) : signal propagation {}", cd.t, cd.sv, dt_tx);
                    let mut interpolated = (self.interpolator)(t_tx, cd.sv, interp_order)?;
                    let mut min_elev = self.cfg.min_sv_elev.unwrap_or(0.0_f64);
                    let mut min_azim = self.cfg.min_sv_azim.unwrap_or(0.0_f64);
                    let mut max_azim = self.cfg.max_sv_azim.unwrap_or(360.0_f64);

                    // provide more information after first iteration
                    if let Some(initial) = &self.initial {
                        let (x0, y0, z0) = (initial.ecef[0], initial.ecef[1], initial.ecef[2]);
                        interpolated = interpolated.with_elevation_azimuth((x0, y0, z0));
                    } else {
                        min_elev = 0.0_f64; // cannot apply yet
                        min_azim = 0.0_f64; // cannot apply yet
                        max_azim = 360.0_f64; // cannot apply yet
                    }

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
                    let orbit = state.orbit(cd.t_tx, earth_j2000);
                    let ea_rad = deg2rad(orbit.ea_deg().map_err(Error::Physics)?);
                    let gm = (EARTH_SEMI_MAJOR_AXIS_WGS84 * EARTH_GRAVITATIONAL_CONST).sqrt();
                    let bias = -2.0_f64 * orbit.ecc().map_err(Error::Physics)? * ea_rad.sin() * gm
                        / SPEED_OF_LIGHT_KM_S
                        / SPEED_OF_LIGHT_KM_S
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
            pool.retain(|cd| {
                let state = cd.state.unwrap(); // infaillible
                let orbit = state.orbit(cd.t, earth_j2000);
                let state = eclipse_state(orbit, SUN_J2000, EARTH_J2000, &self.almanac).unwrap();
                let eclipsed = match state {
                    EclipseState::Umbra => true,
                    EclipseState::Visibilis => false,
                    EclipseState::Penumbra(r) => r < min_rate,
                };
                if eclipsed {
                    debug!("{} ({}): eclipsed", cd.t, cd.sv);
                }
                !eclipsed
            });
        }

        if pool.len() < min_required {
            return Err(Error::NotEnoughMatchingCandidates);
        }

        if self.initial.is_none() {
            let solver = Bancroft::new(&pool)?;
            let output = solver.resolve()?;
            let (x0, y0, z0) = (output[0], output[1], output[2]);
            let position = Position::from_ecef(Vector3::<f64>::new(x0, y0, z0));
            let geo = position.geodetic();
            let (lat, lon) = (rad2deg(geo[0]), rad2deg(geo[1]));
            info!(
                "{} - estimated initial position lat={:.3E}°, lon={:.3E}°",
                pool[0].t, lat, lon
            );
            // update attitudes
            for cd in pool.iter_mut() {
                let state = cd.state.unwrap();
                state.with_elevation_azimuth((x0, y0, z0));
            }
            // store
            self.initial = Some(Position::from_ecef(Vector3::new(
                output[0], output[1], output[2],
            )));
        }

        // Resolve ambiguities
        let ambiguities = if method == Method::PPP {
            self.ambiguity.resolve(&pool)
        } else {
            Default::default()
        };

        // Prepare for NAV:
        Self::retain_best_elevation(&mut pool, min_required);
        // Sort by PRN to form consistant matrix
        pool.sort_by(|cd_a, cd_b| cd_a.sv.prn.partial_cmp(&cd_b.sv.prn).unwrap());

        let initial = self.initial.as_ref().unwrap();
        let (x0, y0, z0) = (initial.ecef()[0], initial.ecef()[1], initial.ecef()[2]);
        let (lat_rad, lon_rad, altitude_above_sea_m) = (
            initial.geodetic()[0],
            initial.geodetic()[1],
            initial.geodetic()[2],
        );
        let (lat_ddeg, lon_ddeg) = (deg2rad(lat_rad), deg2rad(lon_rad));

        let w = self.cfg.solver.weight_matrix(); //sv.values().map(|sv| sv.elevation).collect());
                                                 // // Reduce contribution of newer (rising) vehicles (rising)
                                                 // for (i, cd) in pool.iter().enumerate() {
                                                 //     if !self.prev_used.contains(&cd.sv) {
                                                 //         w[(i, i)] = 0.05;
                                                 //         w[(2 * i, 2 * i)] = 0.05;
                                                 //     }
                                                 // }

        let input = match NavigationInput::new(
            (x0, y0, z0),
            (lat_ddeg, lon_ddeg, altitude_above_sea_m),
            &self.cfg,
            &pool,
            w,
            &ambiguities,
            iono_bias,
            tropo_bias,
        ) {
            Ok(input) => input,
            Err(e) => {
                error!("Failed to form navigation matrix: {}", e);
                return Err(Error::MatrixError);
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
            dt: Duration::from_seconds(x[3] / SPEED_OF_LIGHT_KM_S),
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
            solution.velocity =
                (solution.position - prev_solution.position) / (t - *prev_t).to_seconds();
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
        //   1. Retain best elevation
        pool.sort_by(|cd_a, cd_b| {
            let state_a = cd_a.state.unwrap();
            let state_b = cd_b.state.unwrap();
            state_a.elevation.partial_cmp(&state_b.elevation).unwrap()
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
//     use crate::prelude::{Solver, Candidate, Duration, Epoch, Observation, SV, InterpolationResult};
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
//             let mut state = InterpolationResult::from_position((0.0, 0.0, 0.0));
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
