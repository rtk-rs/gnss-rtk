use crate::{
    prelude::{
        AbsoluteTime, Almanac, Bias, Candidate, Config, Epoch, Error, Frame, Observation,
        OrbitSource, PVTSolution, SV,
    },
    rtk::RTKBase,
    solver::Solver,
};

struct NullRTK {}

impl RTKBase for NullRTK {
    fn name(&self) -> String {
        "UNUSED".to_string()
    }

    fn observe(&mut self, _: Epoch, _: SV) -> Option<Observation> {
        None
    }

    fn reference_position_ecef_m(&self, _: Epoch) -> Option<(f64, f64, f64)> {
        None
    }
}

/// [PPPSolver] is used for direct absolute navigation, without
/// access to any remote reference sites. The objective is to resolve [PVTSolution]s
/// with high accuracy.
pub struct PPPSolver<O: OrbitSource, B: Bias, T: AbsoluteTime> {
    /// Internal [Solver]
    solver: Solver<O, B, T>,
}

impl<O: OrbitSource, B: Bias, T: AbsoluteTime> PPPSolver<O, B, T> {
    /// Creates a new [PPPSolver] for direct absolute navigation,
    /// with possible apriori knowledge. If you know the initial position (a rough estimate will do),
    /// it simplifies the solver deployment. Otherwise, the solver will have to initialize itself.
    /// When targetting high accuracy and quality of the solutions, we recommend letting the solver
    /// figure the initial guess itself if you are not confident about the initial position.
    ///
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - orbit_source: external [OrbitSource] implementation, oftentimes referred to
    /// as "orbit provider".
    /// - time_source: external [Time] implementation, for applications that require
    /// correct temporal solutions at all times. If you cannot fulffil its requirements
    /// or do not care about the accuracy of the absolute temporal solution, you can simply
    /// tie our [NullTime] structure here.
    /// - bias: external [Bias] model implementation, to improve overall accuracy.
    /// - initial_position_ecef_m: possible initial position, as ECEF coordinates in meters.
    pub fn new(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        orbit_source: O,
        time_source: T,
        bias: B,
        initial_position_ecef_m: Option<(f64, f64, f64)>,
    ) -> Self {
        let solver = Solver::new(
            almanac,
            earth_cef,
            cfg,
            orbit_source,
            time_source,
            bias,
            initial_position_ecef_m,
        );

        Self { solver }
    }

    /// Creates a new [PPPSolver] for direct absolute navigation,
    /// without apriori knowledge. In this case, the solver will
    /// have to initialize itself.
    ///
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - orbit_source: external [OrbitSource] implementation, oftentimes referred to
    /// as "orbit provider".
    /// - bias: external [Bias] model implementation, to improve overall accuracy.
    pub fn new_survey(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        orbit_source: O,
        time_source: T,
        bias: B,
    ) -> Self {
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

    /// [PVTSolution] solving attempt, at specified [Epoch] and using proposed [Candidate]s.
    pub fn resolve(
        &mut self,
        epoch: Epoch,
        candidates: &[Candidate],
    ) -> Result<PVTSolution, Error> {
        let solution = self.solver.resolve::<NullRTK>(epoch, candidates, &[], 0)?;
        Ok(solution)
    }

    /// Reset [PPPSolver]. This is usually not needed, even on data gaps.
    /// For the simple reason that a correctly tuned filter will correctly adapt.
    pub fn reset(&mut self) {
        self.solver.reset();
    }
}
