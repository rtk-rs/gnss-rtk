use log::{debug, error, info, warn};

use anise::{
    math::Vector3,
    prelude::{Almanac, Frame},
};

use nalgebra::U4;

use crate::{
    bancroft::Bancroft,
    bias::Bias,
    candidate::Candidate,
    cfg::{Config, Method},
    navigation::{apriori::Apriori, state::State, Navigation, PVTSolution},
    orbit::OrbitSource,
    pool::Pool,
    prelude::{Epoch, Error},
    time::{AbsoluteTime, Time},
};

/// [Solver] to resolve [PVTSolution]s.
/// ## Generics:
/// - O: [OrbitSource], custom [Orbit] provider.
/// - B: custom [Bias] model.
/// - T: [Time] source for correct absolute time
pub struct Solver<O: OrbitSource, B: Bias, T: Time> {
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

    /// Pool
    pool: Pool,

    /// [Navigation] solver
    navigation: Navigation<U4>,

    /// Possible initial position
    initial_ecef_m: Option<Vector3>,

    /// [AbsoluteTime] source
    absolute_time: AbsoluteTime<T>,
}

impl<O: OrbitSource, B: Bias, T: Time> Solver<O, B, T> {
    /// Creates a new Position, Velocity, Time (PVT) [Solver] with possible
    /// apriori knowledge. When set to None, the [Solver] will have to
    /// determine a first initial guess itself.
    ///
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - orbit_source: custom [OrbitSource] implementation.
    /// - bias: [Bias] model implementation
    /// - state_ecef_m: provide initial state as ECEF 3D coordinates,
    /// otherwise we will have to figure them.
    pub fn new(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        orbit_source: O,
        time_source: T,
        bias: B,
        state_ecef_m: Option<(f64, f64, f64)>,
    ) -> Result<Self, Error> {
        Ok(Self::new_almanac_frame(
            cfg,
            almanac,
            earth_cef,
            orbit_source,
            time_source,
            bias,
            state_ecef_m,
        ))
    }

    /// Creates a new Position [Solver] without apriori knowledge.
    /// The solver will have to initiliaze iteself.
    ///
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
        time_source: T,
        bias: B,
    ) -> Result<Self, Error> {
        Self::new(
            almanac,
            earth_cef,
            cfg,
            orbit_source,
            time_source,
            bias,
            None,
        )
    }

    /// Creates a new Position, Velocity, Time (PVT) [Solver] with your own [Almanac] and [Frame] definitions.
    ///
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
        time_source: T,
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

        let initial_ecef_m = match state_ecef_m {
            Some((x0, y0, z0)) => Some(Vector3::new(x0, y0, z0)),
            _ => None,
        };

        let navigation = Navigation::new(&cfg, earth_cef);

        Self {
            bias,
            almanac,
            earth_cef,
            navigation,
            orbit_source,
            initial_ecef_m,
            cfg: cfg.clone(),
            absolute_time: AbsoluteTime::new(time_source),
            pool: Pool::allocate(cfg.code_smoothing, earth_cef),
        }
    }

    /// PVT (Position, Velocity, Time) solution solving attempt.
    ///
    /// ## Input
    /// - t: [Epoch] of observation
    /// - pool: proposed [Candidate]s
    ///
    /// ## Output
    /// - ([Epoch], [PVTSolution]): epoch is simply copied, and resolved solution.
    pub fn resolve(&mut self, t: Epoch, pool: &[Candidate]) -> Result<(Epoch, PVTSolution), Error> {
        let ts = self.cfg.timescale;
        let min_required = self.min_sv_required();

        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughCandidates);
        }

        self.absolute_time.update(t);

        self.pool.new_epoch(pool);
        self.pool.pre_fit(&self.cfg, &self.absolute_time);

        if self.pool.len() < min_required {
            return Err(Error::NotEnoughPreFitCandidates);
        }

        let t = if t.time_scale == self.cfg.timescale {
            t
        } else {
            match self
                .absolute_time
                .epoch_time_correction(t, self.cfg.timescale)
            {
                Ok(new_t) => {
                    let correction = t - new_t;
                    debug!(
                        "{} - |{} - {}| {} sampling instant correction",
                        t, t.time_scale, ts, correction
                    );
                    new_t
                },
                Err(e) => {
                    error!(
                        "{} - failed to apply |{} - {}| correction: {}",
                        t, t.time_scale, ts, e
                    );
                    return Err(e);
                },
            }
        };

        let orbit_source = &mut self.orbit_source;
        self.pool.orbital_states(&self.cfg, orbit_source);

        // current state
        let state = if self.navigation.initialized {
            self.navigation.state
        } else {
            match self.initial_ecef_m {
                Some(x0_y0_z0_m) => {
                    let apriori = Apriori::from_ecef_m(x0_y0_z0_m, t, self.earth_cef);

                    let state = State::from_apriori(&apriori).unwrap_or_else(|e| {
                        panic!("Solver failed to initialize itself. Physical error: {}", e);
                    });

                    debug!("{} - initial state: {}", t, state);
                    state
                },
                None => {
                    let solver = Bancroft::new(&pool)?;
                    let solution = solver.resolve()?;
                    let x0_y0_z0_m = Vector3::new(solution[0], solution[1], solution[2]);

                    let apriori = Apriori::from_ecef_m(x0_y0_z0_m, t, self.earth_cef);

                    let state = State::from_apriori(&apriori).unwrap_or_else(|e| {
                        panic!("Solver failed to initialize itself. Physical error: {}", e);
                    });

                    debug!("{} - initial state: {}", t, state);
                    state
                },
            }
        };

        self.pool
            .post_fit(&self.almanac, self.earth_cef, &self.cfg, &state);

        let pool_size = self.pool.len();

        if pool_size < min_required {
            return Err(Error::NotEnoughPostFitCandidates);
        }

        // Solving attempt
        match self.navigation.solving(
            t,
            &state,
            &self.pool.candidates(),
            pool_size,
            &self.bias,
            &self.absolute_time,
        ) {
            Ok((_)) => {
                info!("{} - navigation iteration completed", t);
            },
            Err(e) => {
                error!("{} - navigation iteration failure: {}", t, e);
                return Err(e);
            },
        }

        // Publish solution
        let solution = PVTSolution::new(
            &self.navigation.state,
            &self.navigation.dop,
            &self.navigation.sv,
        );

        Ok((t, solution))
    }

    /// Reset this [Solver]
    pub fn reset(&mut self) {
        self.navigation.reset();
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
        let mut min_sv = 4;

        if self.navigation.initialized && self.cfg.fixed_altitude.is_some() {
            min_sv -= 1;
        }

        min_sv
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
