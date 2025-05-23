use log::{debug, error, info};

use anise::{
    math::Vector3,
    prelude::{Almanac, Frame},
};

use crate::{
    bancroft::Bancroft,
    bias::Bias,
    candidate::Candidate,
    cfg::{Config, User},
    navigation::{apriori::Apriori, state::State, Navigation, PVTSolution},
    orbit::OrbitSource,
    pool::Pool,
    prelude::{Epoch, Error, Rc},
    rtk::RTKBase,
    time::AbsoluteTime,
};

use nalgebra::{allocator::Allocator, DefaultAllocator, DimName};

/// [Solver] to resolve [PVTSolution]s.
/// ## Generics:
/// - O: [OrbitSource], custom Orbit provider.
/// - B: custom [Bias] model.
/// - T: [AbsoluteTime] source for correct absolute time
pub struct Solver<D: DimName, O: OrbitSource, B: Bias, T: AbsoluteTime>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// Solver [Config]uration preset
    pub cfg: Config,

    /// [Almanac]
    almanac: Almanac,

    /// [Frame]
    earth_cef: Frame,

    /// [OrbitSource]
    orbit_source: Rc<O>,

    /// [Bias] model implementation
    bias: B,

    /// Pool
    pool: Pool,

    /// To invalidate first solution
    first_solution: bool,

    /// [Navigation] solver
    navigation: Navigation<D>,

    /// Possible initial position
    initial_ecef_m: Option<Vector3>,

    /// [AbsoluteTime] implementation
    absolute_time: T,
}

impl<D: DimName, O: OrbitSource, B: Bias, T: AbsoluteTime> Solver<D, O, B, T>
where
    DefaultAllocator: Allocator<D> + Allocator<D, D>,
    <DefaultAllocator as Allocator<D>>::Buffer<f64>: Copy,
    <DefaultAllocator as Allocator<D, D>>::Buffer<f64>: Copy,
{
    /// Creates a new [Solver] for either direct or differential navigation,
    /// with possible apriori knowledge.
    ///
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
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
        orbit_source: Rc<O>,
        absolute_time: T,
        bias: B,
        state_ecef_m: Option<(f64, f64, f64)>,
    ) -> Self {
        // Analyze preset
        if cfg.externalref_delay.is_some() && !cfg.modeling.cable_delay {
            panic!("RF cable delay compensation is incorrectly defined!");
        }

        if !cfg.int_delay.is_empty() && !cfg.modeling.cable_delay {
            panic!("RF cable delay compensation is not fully supported yet.");
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
            absolute_time,
            initial_ecef_m,
            cfg: cfg.clone(),
            first_solution: true,
            pool: Pool::allocate(cfg.code_smoothing, earth_cef),
        }
    }

    /// [PVTSolution] solving attempt.
    /// ## Input
    /// - t: [Epoch] of observation
    /// - user: [User] parameters that we will adapt to
    /// - pool: proposed [Candidate]s
    /// - rtk_base: possible [RTKBase] we will connect to
    ///
    /// ## Output
    /// - [PVTSolution].
    pub fn resolve<RTK: RTKBase>(
        &mut self,
        t: Epoch,
        user: User,
        pool: &[Candidate],
        rtk_base: &RTK,
    ) -> Result<PVTSolution, Error> {
        let ts = self.cfg.timescale;

        let min_required = self.min_sv_required();

        if pool.len() < min_required {
            // no need to proceed further
            return Err(Error::NotEnoughCandidates);
        }

        self.pool.new_epoch(pool);
        self.pool.pre_fit(&self.cfg, &self.absolute_time);

        if self.pool.len() < min_required {
            return Err(Error::NotEnoughPreFitCandidates);
        }

        let t = if t.time_scale == self.cfg.timescale {
            t
        } else {
            let corrected = self.absolute_time.epoch_correction(t, self.cfg.timescale);

            debug!(
                "{} - |{}-{}| corrected sampling: {}",
                t, t.time_scale, ts, corrected
            );

            corrected
        };

        let orbit_source = &self.orbit_source;

        self.pool.orbital_states(&self.cfg, orbit_source);

        // current state
        let state = if self.navigation.initialized {
            self.navigation.state.with_epoch(t)
        } else {
            match self.initial_ecef_m {
                Some(x0_y0_z0_m) => {
                    let apriori = Apriori::from_ecef_m(x0_y0_z0_m, t, self.earth_cef);

                    let state = State::from_apriori(&apriori).unwrap_or_else(|e| {
                        panic!("Solver initial preset is physically incorrect: {}", e);
                    });

                    debug!("{} - initial state: {}", t, state);
                    self.navigation.state = state;
                    state
                },
                None => {
                    let solver = Bancroft::new(self.pool.candidates())?;
                    let solution = solver.resolve()?;
                    let x0_y0_z0_m = Vector3::new(solution[0], solution[1], solution[2]);

                    let apriori = Apriori::from_ecef_m(x0_y0_z0_m, t, self.earth_cef);

                    let state = State::from_apriori(&apriori).unwrap_or_else(|e| {
                        panic!("Solver failed to initialize itself. Physical error: {}", e);
                    });

                    debug!("{} - initial state: {}", t, state);
                    self.navigation.state = state;
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
            user,
            &state,
            &self.pool.candidates(),
            pool_size,
            rtk_base,
            &self.bias,
        ) {
            Ok(_) => {
                info!("{} - iteration completed", t);
            },
            Err(e) => {
                error!("{} - iteration failure: {}", t, e);
                return Err(e);
            },
        }

        // Publish solution
        let solution = PVTSolution::new(
            t,
            &self.navigation.state,
            &self.navigation.dop,
            &self.navigation.sv,
        );

        if self.cfg.solver.open_loop {
            self.navigation.state = state;
        }

        if self.first_solution {
            self.first_solution = false;
            Err(Error::InvalidatedFirstSolution)
        } else {
            Ok(solution)
        }
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
