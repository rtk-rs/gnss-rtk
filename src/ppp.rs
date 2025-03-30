use crate::{
    bias::Bias,
    cfg::Config,
    orbit::OrbitSource,
    pool::Pool,
    prelude::{
        Almanac, Candidate, Epoch, Error, Frame, Method, Observation, PVTSolution, Vector3, SV,
    },
    rtk::RTKBase,
    solver::Solver,
};

use log::warn;

struct AbsoluteNavigation {}

impl RTKBase for AbsoluteNavigation {
    fn remote_observation(&mut self, _: Epoch, _: SV) -> Option<Observation> {
        None
    }
}

/// [PPPSolver] to resolve [PVTSolution]s with or without
/// apriori knowledge
///
/// ## Generics:
/// - O: [OrbitSource], custom [Orbit] provider.
/// - B: external [Bias] model(s).
/// - RTK: remote [RTKBase]
pub struct PPPSolver<O: OrbitSource, B: Bias> {
    solver: Solver<O, AbsoluteNavigation, B>,
}

impl<O: OrbitSource, B: Bias> PPPSolver<O, B> {
    /// Creates a new Position, Velocity, Time [Solver] for absolute Navigation (1D)
    /// with apriori knowledge (initial position preset).
    ///
    /// ## Input
    /// - cfg: solver [Config]uration
    /// - almanac: [Almanac] definition
    /// - frame: [Frame] which must be an ECEF for valid results
    /// - orbit_source: custom [OrbitSource] implementation.
    /// - bias: [Bias] model implementation
    /// - apriori_ecef_m: known initial coordinates, expressed in [Frame] ECEF meters.  
    /// Whether rover remains static or moves from this initial position needs to be defined
    /// in [Solver] [Config]uration.
    pub fn new_apriori(
        cfg: Config,
        almanac: Almanac,
        earth_cef: Frame,
        orbit_source: O,
        bias: B,
        apriori_ecef_m: (f64, f64, f64),
    ) -> Self {
        if cfg.method == Method::SPP && cfg.max_sv_occultation_percent.is_some() {
            warn!("SV eclipse / occultation criteria is not meaningful in SPP navigation");
        }

        if cfg.externalref_delay.is_some() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is either incomplete or not entirely enabled");
        }

        if !cfg.int_delay.is_empty() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is not fully supported yet.");
        }

        let (x0_m, y0_m, z0_m) = apriori_ecef_m;

        Self {
            solver: Solver {
                almanac,
                earth_cef,
                bias,
                orbit_source,
                past_state: None,
                postfit_kf: None,
                cfg: cfg.clone(),
                rtk_base: AbsoluteNavigation {},
                initial_ecef_m: Some(Vector3::new(x0_m, y0_m, z0_m)),
                pool: Pool::allocate(cfg.code_smoothing, earth_cef),
            },
        }
    }

    /// Creates a new Position, Velocity, Time [Solver] for absolute (1D) survey
    /// without any apriori knowledge (initial preset). [PPPSolver] will have
    /// initialize itself, which is more complex than the [Self::new_apriori] scenario.
    ///
    /// ## Input
    /// - cfg: solver [Config]uration
    /// - almanac: [Almanac] definition
    /// - frame: [Frame] which must be an ECEF for valid results
    /// - orbit_source: custom [OrbitSource] implementation.
    /// - bias: [Bias] model implementation
    /// - apriori_ecef_m: known initial coordinates, expressed in [Frame] ECEF meters.  
    /// Whether target remains static or moves from this initial position needs to be defined
    /// using the [Solver] [Config]uration.
    pub fn new_survey(
        cfg: Config,
        almanac: Almanac,
        earth_cef: Frame,
        orbit_source: O,
        bias: B,
    ) -> Self {
        if cfg.method == Method::SPP && cfg.max_sv_occultation_percent.is_some() {
            warn!("SV eclipse / occultation criteria is not meaningful in SPP navigation");
        }

        if cfg.externalref_delay.is_some() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is either incomplete or not entirely enabled");
        }

        if !cfg.int_delay.is_empty() && !cfg.modeling.cable_delay {
            warn!("RF cable delay compensation is not fully supported yet.");
        }

        Self {
            solver: Solver {
                almanac,
                earth_cef,
                bias,
                orbit_source,
                past_state: None,
                postfit_kf: None,
                cfg: cfg.clone(),
                initial_ecef_m: None,
                rtk_base: AbsoluteNavigation {},
                pool: Pool::allocate(cfg.code_smoothing, earth_cef),
            },
        }
    }

    /// [PVTSolution] solving attempt.
    ///
    /// ## Inputs
    /// - t: desired [Epoch]
    /// - pool: list of navigation [Candidate]s
    pub fn resolve(&mut self, t: Epoch, pool: &[Candidate]) -> Result<(Epoch, PVTSolution), Error> {
        self.solver.resolve(t, pool)
    }

    /// Reset navigation filter and phase trackers. Call this on any external abnormal event.
    pub fn reset(&mut self) {
        self.solver.reset();
    }

    /// Reset internal phase tracker. Call this on any external abnormal perturbation
    /// of the phase lock loop. Does not reset the navigation filter.
    pub fn phase_tracking_reset(&mut self) {
        self.solver.phase_tracking_reset();
    }
}
