use log::{debug, error};

use anise::{
    math::Vector3,
    prelude::{Almanac, Frame},
};

use crate::{
    bancroft::Bancroft,
    bias::Bias,
    candidate::Candidate,
    cfg::Config,
    navigation::{apriori::Apriori, state::State, Navigation, PVTSolution},
    orbit::OrbitSource,
    pool::Pool,
    postfit::PostfitKf,
    prelude::{Epoch, Error},
    rtk::RTKBase,
};

/// Generic [Solver] to resolve [PVTSolution]s with or without
/// apriori knowledge and using possible remote [RTKBase]
///
/// ## Generics:
/// - O: [OrbitSource], custom [Orbit] provider.
/// - B: external [Bias] model(s).
/// - RTK: remote [RTKBase]
pub(crate) struct Solver<O: OrbitSource, RTK: RTKBase, B: Bias> {
    /// Solver [Config]uration preset
    pub cfg: Config,
    /// [Almanac]
    pub almanac: Almanac,
    /// [Frame]
    pub earth_cef: Frame,
    /// [OrbitSource]
    pub orbit_source: O,
    /// [Bias] model implementation
    pub bias: B,
    /// [RTKBase] for differential (RTK) navigation
    pub rtk_base: RTK,
    /// Previous resolved [State].
    pub past_state: Option<State>,
    /// Possible initial (very first) preset
    pub initial_ecef_m: Option<Vector3>,
    /// Pool
    pub pool: Pool,
    /// Possible [PostfitKf]
    pub postfit_kf: Option<PostfitKf>,
}

impl<O: OrbitSource, RTK: RTKBase, B: Bias> Solver<O, RTK, B> {
    /// [PVTSolution] solving attempt, using either absolute (1D) or differential (2D) navigation technique.
    ///
    /// ## Inputs
    /// - t: desired [Epoch]
    /// - rover: local list of [Candidate]s
    pub fn resolve(
        &mut self,
        t: Epoch,
        rover: &[Candidate],
    ) -> Result<(Epoch, PVTSolution), Error> {
        let min_required = self.min_sv_required();

        if rover.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughCandidates);
        }

        self.pool.new_epoch(rover);

        // Try to augment this [Pool] with remote observations
        let rtk_base = &mut self.rtk_base;
        self.pool.remote_observation_enhancing(rtk_base);

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
                    let solver = Bancroft::new(&rover)?;
                    let solution = solver.resolve()?;
                    let x0_y0_z0_m = Vector3::new(solution[0], solution[1], solution[2]);

                    debug!("{}: initial solution: {}", t, x0_y0_z0_m);

                    Apriori::from_ecef_m(x0_y0_z0_m, t, self.earth_cef).unwrap_or_else(|e| {
                        panic!("Solver failed to initialize itself. Physical error :{}", e)
                    })
                },
            },
        };

        self.pool.post_fit(&self.almanac, &self.cfg, &apriori);

        let pool_size = self.pool.len();

        if pool_size < min_required {
            // TODO: catch & reset self
            // self.pool.end_of_epoch();
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

    /// Reset navigation filter and phase trackers. Call this on any external abnormal event.
    pub fn reset(&mut self) {
        self.pool.reset();
        self.past_state = None;
    }

    /// Reset internal phase tracker. Call this on any external abnormal perturbation
    /// of the phase lock loop. Does not reset the navigation filter.
    pub fn phase_tracking_reset(&mut self) {
        self.pool.reset();
    }

    /// Returns minimal SV requirement.
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
