use log::{debug, error, info};

use anise::{
    math::Vector3,
    prelude::{Almanac, Frame},
};

use crate::{
    bancroft::Bancroft,
    candidate::Candidate,
    cfg::Config,
    ephemeris::EphemerisSource,
    navigation::{apriori::Apriori, solutions::PVTSolution, state::State, Navigation},
    orbit::OrbitSource,
    pool::Pool,
    prelude::{EnvironmentalBias, Epoch, Error, Rc, SpacebornBias, UserParameters},
    rtk::{NullRTK, RTKBase},
    time::AbsoluteTime,
};

use nalgebra::U4;

/// [Solver] to resolve [PVTSolution]s.
///
/// ## Generics:
/// - EPH: [EphemerisSource] custom data source. Curreuntly unused: limited to [ORB] only!
/// - ORB: [OrbitSource], custom Orbit data source.
/// - EB: [EnvironmentalBias] implementation using either intenral or custom external model.
/// - SB: [SpacebornBias] external information provider.
/// - TIM: [AbsoluteTime] source for correct absolute time management.
pub struct Solver<
    EPH: EphemerisSource,
    ORB: OrbitSource,
    EB: EnvironmentalBias,
    SB: SpacebornBias,
    TIM: AbsoluteTime,
> {
    /// Solver [Config]uration
    pub cfg: Config,

    /// [Almanac]
    almanac: Almanac,

    /// [Frame]
    earth_cef: Frame,

    /// Rover pool
    rover_pool: Pool<EPH, ORB, EB, SB>,

    /// Base pool
    base_pool: Pool<EPH, ORB, EB, SB>,

    /// [Navigation] solver
    navigation: Navigation<U4>,

    /// [AbsoluteTime] implementation
    absolute_time: TIM,

    /// Possible initial position
    initial_ecef_m: Option<Vector3>,
}

impl<
        EPH: EphemerisSource,
        ORB: OrbitSource,
        EB: EnvironmentalBias,
        SB: SpacebornBias,
        TIM: AbsoluteTime,
    > Solver<EPH, ORB, EB, SB, TIM>
{
    /// Creates a new [Solver] for either direct or differential navigation,
    /// with possible apriori knowledge.
    ///
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - eph_source: custom [EphemerisSource], unavailable right now,
    /// tie to null.
    /// - orbit_source: custom [OrbitSource] implementation,
    /// wrapped in a Rc<RefCell<>> which allows the solver
    /// and the orbital provider to live in the same thread.
    /// - absolute_time: external [AbsoluteTime] implementation.
    /// - bias: [Bias] model implementation
    /// - state_ecef_m: provide initial state as ECEF 3D coordinates,
    /// otherwise we will have to figure them.
    pub fn new(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        eph_source: Rc<EPH>,
        orbit_source: Rc<ORB>,
        spaceborn_biases: Rc<SB>,
        environmental_biases: Rc<EB>,
        absolute_time: TIM,
        state_ecef_m: Option<(f64, f64, f64)>,
    ) -> Self {
        let initial_ecef_m = match state_ecef_m {
            Some((x0, y0, z0)) => Some(Vector3::new(x0, y0, z0)),
            _ => None,
        };

        let navigation = Navigation::new(&cfg, earth_cef);

        let rover_pool = Pool::allocate(
            almanac.clone(),
            cfg.clone(),
            earth_cef,
            eph_source.clone(),
            orbit_source.clone(),
            environmental_biases.clone(),
            spaceborn_biases.clone(),
        );

        let base_pool = Pool::allocate(
            almanac.clone(),
            cfg.clone(),
            earth_cef,
            eph_source,
            orbit_source,
            environmental_biases,
            spaceborn_biases,
        );

        Self {
            almanac,
            earth_cef,
            navigation,
            rover_pool,
            base_pool,
            absolute_time,
            initial_ecef_m,
            cfg: cfg.clone(),
        }
    }

    /// Creates a new [Solver] to operate without a priori knowledge
    /// and perform a complete survey from scratch. Refer to [Self::new]
    /// for more information.
    pub fn new_survey(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        eph_source: Rc<EPH>,
        orbit_source: Rc<ORB>,
        spaceborn_biases: Rc<SB>,
        environmental_biases: Rc<EB>,
        absolute_time: TIM,
    ) -> Self {
        Self::new(
            almanac,
            earth_cef,
            cfg,
            eph_source,
            orbit_source,
            spaceborn_biases,
            environmental_biases,
            absolute_time,
            None,
        )
    }

    /// [PVTSolution] solving attempt using PPP technique (no reference).
    /// Unlike RTK resolution attempt, PPP solving will resolve both
    /// the position and the local clock state, but it is
    /// a less accurate and much more complex process.
    ///
    /// ## Input
    /// - epoch: [Epoch] of measurement
    /// - params: [UserParameters]
    /// - candidates: proposed [Candidate]s (= measurements)
    /// - rtk_base: possible [RTKBase] we will connect to
    ///
    /// ## Output
    /// - success: [PVTSolution]
    /// - failure: [Error]
    pub fn ppp_solving(
        &mut self,
        epoch: Epoch,
        params: UserParameters,
        candidates: &[Candidate],
    ) -> Result<PVTSolution, Error> {
        let null_rtk = NullRTK {};
        let solution = self.solving(epoch, params, candidates, &null_rtk, false)?;
        Ok(solution)
    }

    /// [PVTSolution] solving attempt using RTK technique and a single remote
    /// [RTKBase] reference site. This library is currently limited to a single
    /// reference, although there should be limitations to support any amount of
    /// reference stations in the future. You may catch solving errors that are
    /// related to the RTK network to try-again but using [Self::ppp_solving]
    /// when the network is down. For this function to converge, the remote
    /// site and rover proposals must agree in their content (enough matches)
    /// and signals must fulfill the navigation [Method] being used.
    ///
    /// NB: unlike PPP solving, RTK solving will only resolve the spatial
    /// state, the clock state can only remain undetermined. If you are
    /// interested by both, you will either have to restrict to [Self::ppp_solving],
    /// or do a PPP (time only) run after the RTK run.
    ///
    // ## Input
    /// - epoch: [Epoch] of measurement from the rover reported by the rover.
    /// The remote site must match the sampling instant closely, reducing
    /// the time-difference (RTK aging).
    /// - profile: rover [UserProfile]
    /// - candidates: rover measurements, wrapped as [Candidate]s.
    /// - rtk_base: remote reference site that implements [RTKBase]reference.
    ///
    /// ## Output
    /// - [PVTSolution].
    pub fn rtk_solving<RTK: RTKBase>(
        &mut self,
        epoch: Epoch,
        params: UserParameters,
        candidates: &[Candidate],
        rtk_base: &RTK,
    ) -> Result<PVTSolution, Error> {
        self.solving(epoch, params, candidates, rtk_base, true)
    }

    /// [PVTSolution] solving attempt.
    fn solving<RTK: RTKBase>(
        &mut self,
        epoch: Epoch,
        params: UserParameters,
        pool: &[Candidate],
        rtk_base: &RTK,
        uses_rtk: bool,
    ) -> Result<PVTSolution, Error> {
        let rtk_base_name = rtk_base.name();
        let min_required = self.min_sv_required();

        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughCandidates);
        }

        self.rover_pool.new_epoch(pool);

        if uses_rtk {
            let observations = rtk_base.observe(epoch);
            info!("{} - using remote {} reference", epoch, rtk_base_name);
            self.base_pool.new_epoch(&observations);
        }

        self.rover_pool.pre_fit("rover", &self.absolute_time);

        if uses_rtk {
            self.base_pool.pre_fit(&rtk_base_name, &self.absolute_time);
        }

        if self.rover_pool.len() < min_required {
            return Err(Error::NotEnoughPreFitCandidates);
        }

        let epoch = if epoch.time_scale == self.cfg.timescale {
            epoch
        } else {
            let corrected = self
                .absolute_time
                .epoch_correction(epoch, self.cfg.timescale);

            debug!(
                "{} - |{}-{}| corrected sampling: {}",
                epoch, epoch.time_scale, self.cfg.timescale, corrected
            );

            corrected
        };

        self.rover_pool.orbital_states_fit("rover");

        if uses_rtk {
            self.base_pool.orbital_states_fit(&rtk_base_name);
        }

        // current state
        let state = if self.navigation.initialized {
            self.navigation.state.with_epoch(epoch)
        } else {
            match self.initial_ecef_m {
                Some(x0_y0_z0_m) => {
                    let apriori = Apriori::from_ecef_m(x0_y0_z0_m, epoch, self.earth_cef);

                    let state = State::from_apriori(&apriori).unwrap_or_else(|e| {
                        panic!("Solver initial preset is physically incorrect: {}", e);
                    });

                    debug!("{} - initial state: {}", epoch, state);
                    state
                },
                None => {
                    let solver = Bancroft::new(self.rover_pool.candidates())?;
                    let solution = solver.resolve()?;
                    let x0_y0_z0_m = Vector3::new(solution[0], solution[1], solution[2]);

                    let apriori = Apriori::from_ecef_m(x0_y0_z0_m, epoch, self.earth_cef);

                    let state = State::from_apriori(&apriori).unwrap_or_else(|e| {
                        panic!("Solver failed to initialize itself. Physical error: {}", e);
                    });

                    debug!("{} - initial state: {}", epoch, state);
                    state
                },
            }
        };

        // rover post-fit
        self.rover_pool.post_fit("rover", &state).map_err(|e| {
            error!("{} rover postfit error {}", epoch, e);
            Error::PostfitPrenav
        })?;

        let double_differences = if uses_rtk {
            // base post-fit
            self.base_pool
                .post_fit(&rtk_base_name, &state)
                .map_err(|e| {
                    error!("{} {} postfit error: {}", epoch, rtk_base_name, e);
                    Error::PostfitPrenav
                })?;

            // special RTK post-fit
            match self.rover_pool.rtk_post_fit(&mut self.base_pool) {
                Ok(dd_observations) => Some(dd_observations),
                Err(e) => {
                    error!("{} - rtk post-fit error: {}", epoch, e);
                    return Err(e);
                },
            }
        } else {
            None
        };

        if let Some(double_differences) = &double_differences {
            for ((sat, carrier), ddiff) in double_differences.inner.iter() {
                debug!("{}({}) - DD({})={}", epoch, sat, carrier, ddiff);
            }
        }

        let pool_size = self.rover_pool.len();

        if pool_size < min_required {
            return Err(Error::NotEnoughPostFitCandidates);
        }

        // Solving attempt
        match self.navigation.solving(
            epoch,
            params,
            &state,
            &self.rover_pool.candidates(),
            pool_size,
        ) {
            Ok(_) => {
                info!("{} - iteration completed", epoch);
            },
            Err(e) => {
                error!("{} - iteration failure: {}", epoch, e);
                return Err(e);
            },
        }

        // Publish solution
        let solution = PVTSolution::new(
            epoch,
            &self.navigation.state,
            &self.navigation.dop,
            &self.navigation.sv,
        );

        if self.cfg.solver.open_loop {
            self.navigation.state = state;
        }

        Ok(solution)
    }

    /// Reset this [Solver].
    pub fn reset(&mut self) {
        self.navigation.reset();
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
