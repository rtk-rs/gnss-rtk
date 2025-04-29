use crate::{
    prelude::{
        AbsoluteTime, Almanac, Bias, Candidate, Config, Epoch, Error, Frame, Observation,
        OrbitSource, PVTSolution, SV,
    },
    solver::Solver,
};

// pub(crate) mod star;
// pub use star::StarNRTK;

/// Any [RTKBase] provides remote data by implementing the [RemoteSource] trait.
pub trait RTKBase {
    /// Provide a meaningful name of individual reference stations.
    /// This is useful when connecting to several at once.
    fn name(&self) -> String;

    /// Provide remote synchronous [Observation] of this [SV] if you can.
    /// If you fail to provide and fulfill the navigation technique requirements,
    /// the [SV] will be dropped for this [Epoch]. In other words, it will not contribute to the process.
    /// The [Observation] should be synchronous, ideally you should use an interpolation scheme.
    /// For high sampling rates, taking the closest neighbouring point in time will do, but that only stands
    /// if your [RTKBase] remained static in the meantime !
    fn observe(&mut self, t: Epoch, sv: SV) -> Option<Observation>;

    /// Any [RTKBase] should be able to desribe its absolute position, at all times,
    /// with highest accuracy. These coordinates (in meters, ECEF), should correspond to the precise
    /// position where all remote [Observation]s were made.
    /// Any moving [RTKBase] should keep the position up to date and it should reflect through this method.
    fn reference_position_ecef_m(&self, t: Epoch) -> Option<(f64, f64, f64)>;
}

/// [RTKSolver] is used for differential navigation, to resolve
/// the position of a single rover connected to a single reference site.
/// The objective is to resolve [PVTSolution]s with high accuracy.
pub struct RTKSolver<O: OrbitSource, B: Bias, T: AbsoluteTime> {
    /// Internal [Solver]
    solver: Solver<O, B, T>,
}

impl<O: OrbitSource, B: Bias, T: AbsoluteTime> RTKSolver<O, B, T> {
    /// Creates a new [RTKSolver] for direct differential navigation,
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
    /// - time_source: external [AbsoluteTime] implementation, for applications that require
    /// correct temporal solutions at all times.
    /// - rtk_base: single reference that implements the [RTKBase] trait.
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
    /// - time_source: external [Time] implementation, for applications that require
    /// correct temporal solutions at all times. If you cannot fulffil its requirements
    /// or do not care about the accuracy of the absolute temporal solution, you can simply
    /// tie our [NullTime] structure here.
    /// - rtk_base: single reference that implements the [RTKBase] trait.
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

    /// [PVTSolution] solving attempt, at specified [Epoch] and using local [Candidate]s (rover's proposal).
    /// ## Input
    /// - epoch: sampling [Epoch], all forwarded [Observation]s should be synchronous
    /// - candidates: proposed [Candidate]s
    /// - rtk_bases: [RTKBase]s we will try to connect to
    /// - num_bases: number of proposed [RTKBase]s.
    pub fn resolve<RTK: RTKBase>(
        &mut self,
        epoch: Epoch,
        candidates: &[Candidate],
        rtk_bases: &[RTK],
        num_bases: usize,
    ) -> Result<PVTSolution, Error> {
        let solution = self
            .solver
            .resolve(epoch, candidates, rtk_bases, num_bases)?;
        Ok(solution)
    }
}
