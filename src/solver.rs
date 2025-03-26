use itertools::Itertools;
use std::collections::HashMap;

use log::{debug, error, warn};

use anise::{
    constants::frames::SUN_J2000,
    math::{Matrix3, Vector3},
    prelude::{Almanac, Frame, Unit},
};

use crate::{
    ambiguity::{Input as AmbiguityInput, Solver as AmbiguitySolver},
    bancroft::Bancroft,
    bias::Bias,
    candidate::Candidate,
    cfg::{Config, Method},
    constants::Constants,
    navigation::{apriori::Apriori, state::State, Navigation, PVTSolution},
    orbit::OrbitSource,
    postfit::PostfitKf,
    prelude::{Duration, Epoch, Error, Orbit, SPEED_OF_LIGHT_M_S, SV},
    smoothing::Smoother,
};

/// [Solver] to resolve [PVTSolution]s.
/// ## Generics:
/// - O: [OrbitSource], custom [Orbit] provider.
/// - B: custom [Bias] model.
pub struct Solver<O: OrbitSource, B: Bias> {
    /// Solver [Config]uration preset
    pub cfg: Config,
    /// [Almanac]
    almanac: Almanac,
    /// [Frame]
    earth_cef: Frame,
    /// [OrbitSource]
    orbit_source: O,
    /// [Bias] model implementation
    bias: B,
    /// Previous resolved [State].
    past_state: Option<State>,
    /// Possible initial (very first) preset
    initial_ecef_m: Option<Vector3>,
    /// Past elected
    past_elected: Vec<Candidate>,
    /// Smoother
    smoother: Smoother,
    /// [AmbiguitySolver]
    ambiguities: HashMap<SV, AmbiguitySolver>,
    /// Possible [PostfitKf]
    postfit_kf: Option<PostfitKf>,
}

pub(crate) fn sv_orbital_attitude_fixup(
    almanac: &Almanac,
    apriori: &Apriori,
    pool: &mut Vec<Candidate>,
) {
    let rx_orbit = apriori.to_orbit();

    pool.retain_mut(|cd| match cd.orbital_attitude_fixup(almanac, rx_orbit) {
        Ok(_) => true,
        Err(e) => {
            debug!("{}({}) - orbital fixup: {}", apriori.t, cd.sv, e);
            false
        },
    });
}

// Apply attitude filters
pub(crate) fn sv_attitude_filters(cfg: &Config, pool: &mut Vec<Candidate>) {
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

/// - Define velocities when feasible
/// - Account for relativistic effects
fn sv_velocities_fixup(
    relativistic_clock_bias: bool,
    past_elected: &Vec<Candidate>,
    pool: &mut Vec<Candidate>,
) -> Result<(), Error> {
    let mu = Constants::EARTH_GRAVITATION;
    let w_e = Constants::EARTH_SEMI_MAJOR_AXIS_WGS84;

    for cd in pool.iter_mut() {
        if let Some(sv_orbit) = &mut cd.orbit {
            if let Some(past_elected) = past_elected.iter().find(|elected| elected.sv == cd.sv) {
                let pos_vel_km_s = sv_orbit.to_cartesian_pos_vel();

                let dt_s = (cd.t_tx - past_elected.t_tx).to_seconds();
                let past_orbit = past_elected.orbit.unwrap();
                let pos_vel_z1_km_s = past_orbit.to_cartesian_pos_vel();

                let vel_km_s = (
                    (pos_vel_km_s[0] - pos_vel_z1_km_s[0]) / dt_s,
                    (pos_vel_km_s[1] - pos_vel_z1_km_s[1]) / dt_s,
                    (pos_vel_km_s[2] - pos_vel_z1_km_s[2]) / dt_s,
                );

                debug!("{} ({}) : vel {:?} km/s", cd.t, cd.sv, vel_km_s);

                *sv_orbit =
                    sv_orbit.with_velocity_km_s(Vector3::new(vel_km_s.0, vel_km_s.1, vel_km_s.2));

                if let Some(clock_corr) = &mut cd.clock_corr {
                    if relativistic_clock_bias && clock_corr.needs_relativistic_correction {
                        let pos_m = (
                            pos_vel_km_s[0] * 1.0E3,
                            pos_vel_km_s[1] * 1.0E3,
                            pos_vel_km_s[2] * 1.0E3,
                        );

                        let vel_m_s = (vel_km_s.0 * 1.0E3, vel_km_s.1 * 1.0E3, vel_km_s.2 * 1.0E3);

                        let r_v_sat =
                            pos_m.0 * vel_m_s.0 + pos_m.1 * vel_m_s.1 + pos_m.2 * vel_m_s.2;

                        let bias =
                            -2.0 * r_v_sat / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S * Unit::Second;

                        // let ea_deg = sv_orbit.ea_deg().map_err(Error::Physics)?;

                        // let ea_rad = ea_deg.to_radians();
                        // let gm = (w_e * mu).sqrt();
                        // let ecc = sv_orbit.ecc().map_err(Error::Physics)?;

                        // let bias = -2.0_f64 * ecc * ea_rad.sin() * gm
                        //     / SPEED_OF_LIGHT_M_S
                        //     / SPEED_OF_LIGHT_M_S
                        //     * Unit::Second;

                        debug!("{} ({}) : relativistic clock bias: {}", cd.t, cd.sv, bias);
                        // clock_corr.duration += bias;
                    }
                } //clockbias
            } //velocity
        }
    }
    Ok(())
}

fn update_sky_view(t: Epoch, pool: &[Candidate], elected: &mut Vec<Candidate>) {
    for cd in pool.iter() {
        if elected.iter().find(|elected| elected.sv == cd.sv).is_none() {
            elected.push(cd.clone());
        }
    }

    let current_sv = pool.iter().map(|cd| cd.sv).unique().collect::<Vec<_>>();

    elected.retain(|elected| {
        let retained = current_sv.contains(&elected.sv);
        if !retained {
            debug!("{} ({}) - loss of sight", t, elected.sv);
        }
        retained
    });
}

impl<O: OrbitSource, B: Bias> Solver<O, B> {
    /// Creates a new Position, Velocity, Time [Solver] without
    /// apriori knowledge of the initial position.
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - orbit_source: custom [OrbitSource] implementation.
    /// - bias: [Bias] model implementation
    /// - state_ecef_m: if you have accurate knowledge of the initial position,
    /// you may define it here. Otherwise, we recommend you tie this to None.
    pub fn new(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        orbit_source: O,
        bias: B,
        state_ecef_m: Option<(f64, f64, f64)>,
    ) -> Result<Self, Error> {
        Ok(Self::new_almanac_frame(
            cfg,
            almanac,
            earth_cef,
            orbit_source,
            bias,
            state_ecef_m,
        ))
    }

    /// Creates a new Position [Solver] without apriori knowledge.
    /// The solver will have to initiliaze iteself.
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - orbit_source: custom [OrbitSource] implementation.
    /// - bias: [Bias] model implementation
    pub fn new_survey(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        orbit_source: O,
        bias: B,
    ) -> Result<Self, Error> {
        Self::new(almanac, earth_cef, cfg, orbit_source, bias, None)
    }

    /// Creates a new Position, Velocity, Time [Solver] with your own [Almanac] and [Frame] definitions.
    /// ## Input
    /// - cfg: solver [Config]uration
    /// - almanac: [Almanac] definition
    /// - frame: [Frame] which must be an ECEF for valid results
    /// - orbit_source: custom [OrbitSource] implementation.
    /// - bias: [Bias] model implementation
    /// - state_ecef_m: if you have accurate knowledge of the initial position,
    /// you may define it here. Otherwise, we recommend you tie this to None.
    pub fn new_almanac_frame(
        cfg: Config,
        almanac: Almanac,
        earth_cef: Frame,
        orbit_source: O,
        bias: B,
        state_ecef_m: Option<(f64, f64, f64)>,
    ) -> Self {
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

        Self {
            almanac,
            earth_cef,
            bias,
            orbit_source,
            initial_ecef_m,
            past_state: None,
            postfit_kf: None,
            smoother: Smoother::new(cfg.code_smoothing),
            cfg: cfg.clone(),
            past_elected: Vec::with_capacity(8),
            ambiguities: HashMap::with_capacity(8),
        }
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
        let modeling = self.cfg.modeling;

        // TODO
        // Self::signal_condition_filter(t, method, &mut pool);

        if let Some(min_snr) = self.cfg.min_snr {
            // TODO
            // Self::signal_quality_filter(min_snr, &mut pool);
        }

        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughPreFitCandidates);
        }

        pool.retain_mut(|cd| {
            match cd.tx_epoch(&self.cfg) {
                Ok(_) => {
                    // fix orbital state
                    let source = &mut self.orbit_source;

                    if let Some(orbit) = source.next_at(cd.t_tx, cd.sv, self.earth_cef) {
                        debug!("{} - sv dt_tx={} sv t={}", t, cd.t_tx, cd.t);
                        let orbit = Self::rotate_orbit_dcm3x3(
                            cd.t,
                            cd.dt_tx,
                            orbit,
                            modeling.earth_rotation,
                            self.earth_cef,
                        );

                        cd.orbit = Some(orbit);
                        true
                    } else {
                        false
                    }
                },
                Err(e) => {
                    error!("{} ({}) - transmision time error: {}", t, cd.sv, e);
                    false
                },
            }
        });

        // Obtain apriori
        let apriori = match self.past_state {
            Some(state) => Apriori::from_state(t, &state).unwrap_or_else(|e| {
                panic!(
                    "Internal error. Solver reinit procedure failed due to physical error: {}",
                    e
                )
            }),
            None => match self.initial_ecef_m {
                Some(x0_y0_z0_m) => {
                    debug!("{}: initializing with external preset {}", t, x0_y0_z0_m);

                    Apriori::from_ecef_m(x0_y0_z0_m, t, self.earth_cef).unwrap_or_else(|e| {
                        panic!(
                            "Solver initialization failure - {}. Invalid user preset?",
                            e
                        )
                    })
                },
                None => {
                    let solver = Bancroft::new(&pool)?;
                    let solution = solver.resolve()?;
                    let x0_y0_z0_m = Vector3::new(solution[0], solution[1], solution[2]);

                    debug!("{}: initial solution: {}", t, x0_y0_z0_m);

                    Apriori::from_ecef_m(x0_y0_z0_m, t, self.earth_cef).unwrap_or_else(|e| {
                        panic!("Solver failed to initialize itself. Phyiscal error :{}", e)
                    })
                },
            },
        };

        sv_orbital_attitude_fixup(&self.almanac, &apriori, &mut pool);

        sv_velocities_fixup(
            self.cfg.modeling.relativistic_clock_bias,
            &self.past_elected,
            &mut pool,
        )?;

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

        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughPreFitCandidates);
        }

        sv_attitude_filters(&self.cfg, &mut pool);

        // ambiguity solving
        if self.cfg.code_smoothing > 0 || self.cfg.method == Method::PPP {
            pool.retain_mut(|cd| {
                if let Some(l1) = cd.l1_phase_range() {
                    if let Some(c1) = cd.l1_pseudo_range() {
                        if let Some(l2) = cd.lj_phase_range() {
                            if let Some(c2) = cd.lj_pseudo_range() {
                                let input = AmbiguityInput {
                                    f1: l1.0.frequency(),
                                    c1: c1.1,
                                    l1: l1.1,
                                    f2: l2.0.frequency(),
                                    c2: c2.1,
                                    l2: l2.1,
                                };

                                let output = if let Some(solver) = self.ambiguities.get_mut(&cd.sv)
                                {
                                    solver.solve(input)
                                } else {
                                    let mut solver = AmbiguitySolver::new();
                                    solver.solve(input)
                                };

                                debug!(
                                    "{} ({}) - n_1={}(\u{03c3}={}) n_2={}(\u{03c3}w)={})",
                                    cd.t,
                                    cd.sv,
                                    output.n1,
                                    0.0, // output.sigma_n1,
                                    output.n2,
                                    0.0, // output.sigma_nw,
                                );

                                cd.update_ambiguities(output);
                                true
                            } else {
                                debug!("{} ({}) - missing lj pseudo range", cd.t, cd.sv);
                                false
                            }
                        } else {
                            debug!("{} ({}) - missing lj phase range", cd.t, cd.sv);
                            false
                        }
                    } else {
                        debug!("{} ({}) - missing l1 code", cd.t, cd.sv);
                        false
                    }
                } else {
                    debug!("{} ({}) - missing l1 phase range", cd.t, cd.sv);
                    false
                }
            });
        }

        // smoothing
        if self.cfg.code_smoothing > 0 {
            for cd in pool.iter_mut() {
                for sv_observ in cd.observations.iter_mut() {
                    if sv_observ.carrier.is_l1_pivot() {
                        let lambda_1 = sv_observ.carrier.wavelength();
                        let n_1 = sv_observ.ambiguity.unwrap_or_default();
                        if n_1 != 0.0 {
                            if let Some(c_n) = &mut sv_observ.pseudo_range_m {
                                if let Some(l_n) = sv_observ.phase_range_m {
                                    *c_n = self.smoother.smoothing(
                                        sv_observ.carrier,
                                        cd.sv,
                                        *c_n,
                                        n_1,
                                        lambda_1,
                                        l_n,
                                    );
                                }
                            }
                        }
                    } else {
                        let lambda_j = sv_observ.carrier.wavelength();
                        let n_j = sv_observ.ambiguity.unwrap_or_default();
                        if n_j != 0.0 {
                            if let Some(c_n) = &mut sv_observ.pseudo_range_m {
                                if let Some(l_n) = sv_observ.phase_range_m {
                                    *c_n = self.smoother.smoothing(
                                        sv_observ.carrier,
                                        cd.sv,
                                        *c_n,
                                        n_j,
                                        lambda_j,
                                        l_n,
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }

        // Prepare for NAV
        pool.retain(|cd| {
            let retained = cd.is_navi_compatible();
            if !retained {
                debug!("{}({}): not proposed - missing data", cd.t, cd.sv);
            }
            retained
        });

        let pool_size = pool.len();

        if pool_size < min_required {
            return Err(Error::NotEnoughPostFitCandidates);
        }

        update_sky_view(t, &pool, &mut self.past_elected);

        // Build nav filter
        let mut nav = match Navigation::new(t, &self.cfg, apriori, &pool, pool_size, &self.bias) {
            Ok(nav) => nav,
            Err(e) => {
                error!("matrix formation error: {}", e);
                return Err(e);
            },
        };

        // discard non contributing data
        let contributing = nav.sv.iter().map(|contrib| contrib.sv).collect::<Vec<_>>();
        pool.retain(|cd| contributing.contains(&cd.sv));

        let pool_size = pool.len();

        // iterate nav filter
        loop {
            match nav.iterate(t, &self.cfg, &pool, pool_size, &self.bias) {
                Ok(converged) => {
                    if converged {
                        break;
                    } else {
                        debug!(
                            "iter={} | {:?} dt={}",
                            nav.iter, nav.state.pos_m, nav.state.clock_dt
                        );
                    }
                },
                Err(e) => {
                    error!("{} - filter iter={}: {}", t, nav.iter, e);
                    return Err(e);
                },
            }
        }

        if self.cfg.solver.postfit_kf {
            if self.postfit_kf.is_none() {
                self.postfit_kf = Some(PostfitKf::new(self.earth_cef, &nav.state));
            }

            let kf = self.postfit_kf.as_mut().unwrap();

            let new_state = kf
                .run(&nav.state)
                .unwrap_or_else(|e| panic!("kf error: {}", e));

            let residual = new_state.residual(&nav.state);
            debug!("{} - postfit(kf) residual: {}", t, residual);

            nav.state = new_state;
        }

        if let Some(past_state) = self.past_state {
            nav.state
                .velocity_update(past_state.t, past_state.pos_m, past_state.clock_dt);
        }

        // Validated solution
        let solution = PVTSolution::new(&nav.state, &nav.dop, &nav.sv);
        self.past_state = Some(nav.state.clone());

        Ok((t, solution))
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
                if cd.l1_pseudo_range().is_some() {
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

    fn min_sv_required(&self) -> usize {
        if self.past_state.is_none() {
            if self.initial_ecef_m.is_none() {
                4
            } else {
                if self.cfg.fixed_altitude.is_some() {
                    3
                } else {
                    4
                }
            }
        } else {
            if self.cfg.fixed_altitude.is_some() {
                3
            } else {
                4
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

        let state_m = orbit.to_cartesian_pos_vel() * 1.0E3;
        let position_m = Vector3::new(state_m[0], state_m[1], state_m[2]);
        let position = dcm3 * position_m;

        Orbit::from_position(
            position[0] / 1.0E3,
            position[1] / 1.0E3,
            position[2] / 1.0E3,
            t,
            frame,
        )
    }
}

#[cfg(test)]
mod test {
    use itertools::Itertools;
    use std::str::FromStr;

    use crate::{
        prelude::{Almanac, Candidate, Config, Epoch, OrbitSource, Solver, EARTH_J2000},
        solver::sv_orbital_attitude_fixup,
        tests::{
            bias::NullBias,
            gps::{G02, G05, G07, G08, G09, G13, G15},
            orbits::{GpsOrbits, NullOrbits},
            reference_orbit,
        },
    };

    use super::sv_attitude_filters;

    // #[test]
    // fn test_min_sv_required() {
    //     for (preset, expected) in [
    //         (Config::default(), 4),
    //         (
    //             Config::default().with_pvt_solutions_type(PVTSolutionType::PositionVelocityTime),
    //             4,
    //         ),
    //     ] {
    //         let null_bias = NullBias {};
    //         let solver = Solver::new(preset, NullOrbits {}, null_bias, None).unwrap();

    //         assert_eq!(solver.min_sv_required(), expected);
    //     }

    //     let mut preset = Config::default();
    //     preset.fixed_altitude = Some(1.0);

    //     let solver = Solver::new(preset.clone(), NullOrbits {}, NullBias {}, None).unwrap();
    //     assert_eq!(solver.min_sv_required(), 4);

    //     let solver = Solver::new(
    //         preset.clone(),
    //         NullOrbits {},
    //         NullBias {},
    //         Some((1.0, 2.0, 3.0)),
    //     )
    //     .unwrap();
    //     assert_eq!(solver.min_sv_required(), 3);

    //     let preset = Config::default().with_pvt_solutions_type(PVTSolutionType::TimeOnly);

    //     let solver = Solver::new(preset.clone(), NullOrbits {}, NullBias {}, None).unwrap();
    //     assert_eq!(solver.min_sv_required(), 4);

    //     let solver = Solver::new(
    //         preset.clone(),
    //         NullOrbits {},
    //         NullBias {},
    //         Some((1.0, 2.0, 3.0)),
    //     )
    //     .unwrap();
    //     assert_eq!(solver.min_sv_required(), 1);
    // }

    // #[test]
    // fn test_attitude_filters() {
    //     let mut preset = Config::default();
    //     preset.min_sv_elev = Some(14.0);

    //     let almanac = Almanac::until_2035().unwrap();
    //     let frame = almanac.frame_from_uid(EARTH_J2000).unwrap();

    //     let t0_gpst: Epoch = Epoch::from_str("2020-06-25T00:00:00 GPST").unwrap();

    //     let rx_orbit = reference_orbit(frame);

    //     let mut gpst_orbits = GpsOrbits::build();

    //     let mut candidates = vec![
    //         Candidate::new(G02, t0_gpst, vec![]),
    //         Candidate::new(G05, t0_gpst, vec![]),
    //         Candidate::new(G07, t0_gpst, vec![]),
    //         Candidate::new(G08, t0_gpst, vec![]),
    //         Candidate::new(G09, t0_gpst, vec![]),
    //         Candidate::new(G13, t0_gpst, vec![]),
    //         Candidate::new(G15, t0_gpst, vec![]),
    //     ];

    //     for cd in candidates.iter_mut() {
    //         let orbit = gpst_orbits.next_at(t0_gpst, cd.sv, frame).unwrap();
    //         cd.set_orbit(orbit);
    //     }

    //     sv_orbital_attitude_fixup(&almanac, t0_gpst, rx_orbit, &mut candidates);
    //     assert_eq!(candidates.len(), 7, "invalid test initialization");

    //     sv_attitude_filters(&preset, &mut candidates);

    //     let remainder = candidates
    //         .iter()
    //         .map(|cd| cd.sv)
    //         .sorted()
    //         .collect::<Vec<_>>();

    //     assert_eq!(remainder, vec![G05, G07, G13, G15]);

    //     preset.min_sv_elev = Some(16.0);

    //     sv_attitude_filters(&preset, &mut candidates);

    //     let remainder = candidates
    //         .iter()
    //         .map(|cd| cd.sv)
    //         .sorted()
    //         .collect::<Vec<_>>();

    //     assert_eq!(remainder, vec![G05, G07, G13]);
    // }
}
