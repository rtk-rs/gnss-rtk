use itertools::Itertools;

use log::{debug, error, warn};

use anise::{
    math::Vector3,
    prelude::{Almanac, Frame},
};

use crate::{
    bancroft::Bancroft,
    bias::Bias,
    candidate::Candidate,
    cfg::{Config, Method},
    navigation::{apriori::Apriori, state::State, Navigation, PVTSolution},
    orbit::OrbitSource,
    pool::Pool,
    postfit::PostfitKf,
    prelude::{Epoch, Error},
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
    /// Pool
    pool: Pool,
    /// Possible [PostfitKf]
    postfit_kf: Option<PostfitKf>,
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
            cfg: cfg.clone(),
            pool: Pool::allocate(cfg.code_smoothing, earth_cef),
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

        self.pool.new_epoch(pool);
        self.pool.pre_fit(&self.cfg);

        if self.pool.len() < min_required {
            // TODO: catch and reset self
            return Err(Error::NotEnoughPreFitCandidates);
        }

        let orbit_source = &mut self.orbit_source;
        self.pool.orbital_states(&self.cfg, orbit_source);

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

        self.pool.post_fit(&self.almanac, &self.cfg, &apriori);

        let pool_size = self.pool.len();

        if pool_size < min_required {
            // TODO: catch & reset self
            return Err(Error::NotEnoughPostFitCandidates);
        }

        // Build nav filter
        let mut nav = match Navigation::new(
            t,
            &self.cfg,
            apriori,
            &self.pool.candidates(),
            pool_size,
            &self.bias,
        ) {
            Ok(nav) => nav,
            Err(e) => {
                error!("matrix formation error: {}", e);
                return Err(e);
            },
        };

        // discard non contributing data
        let contributing = nav.sv.iter().map(|contrib| contrib.sv).collect::<Vec<_>>();
        self.pool.retain(|cd| contributing.contains(&cd.sv));

        let pool_size = self.pool.len();

        // iterate nav filter
        loop {
            match nav.iterate(t, &self.cfg, &self.pool.candidates(), pool_size, &self.bias) {
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
}

#[cfg(test)]
mod test {
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
