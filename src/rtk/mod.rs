use crate::{
    prelude::{
        Almanac, Bias, Candidate, Config, Epoch, Error, Frame, Observation, OrbitSource,
        PVTSolution, Time, SV,
    },
    solver::Solver,
};

#[cfg(doc)]
use crate::prelude::NullTime;

// pub(crate) mod star;
// pub use star::StarNRTK;

/// Any RTK reference site needs to implement the [RTKBase] trait.
pub trait RTKBase {
    /// Provide remote synchronous [Observation] of this [SV] if you can.
    /// If you fail to provide and fulfill the navigation technique requirements,
    /// the [SV] will be dropped for this [Epoch]. In other words, it will not contribute to the process.
    /// The [Observation] should be synchronous, ideally you should use an interpolation scheme.
    /// For high sampling rates, taking the closest neighbouring point in time will do, but that only stands
    /// if your [RTKBase] remained static in the meantime !
    fn remote_observation(&mut self, t: Epoch, sv: SV) -> Option<Observation>;

    /// Any [RTKBase] should be able to desribe its absolute position, at all times,
    /// with highest accuracy. These coordinates (in meters, ECEF), should correspond to the precise
    /// position where all remote [Observation]s were made.
    /// Any moving [RTKBase] should keep the position up to date and it should reflect through this method.
    fn reference_position_ecef_m(&self, t: Epoch) -> Option<(f64, f64, f64)>;
}

/// [NullBase] is simply used to deploy the Navigation [Solver] in absolute direct navigation.
pub struct NullBase {}

impl RTKBase for NullBase {
    fn remote_observation(&mut self, t: Epoch, sv: SV) -> Option<Observation> {
        None
    }

    fn reference_position_ecef_m(&self, t: Epoch) -> Option<(f64, f64, f64)> {
        None
    }
}

/// [RTKSolver] is used in differential absolute navigation scenarios, to resolve
/// the position of a single rover, connected to a single reference site.
/// The objective is to resolve [PVTSolution]s with high accuracy.
pub struct RTKSolver<O: OrbitSource, B: Bias, T: Time, RTK: RTKBase> {
    solver: Solver<O, B, T, RTK>,
}

impl<O: OrbitSource, B: Bias, T: Time, RTK: RTKBase> RTKSolver<O, B, T, RTK> {
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
    /// - time_source: external [Time] implementation, for applications that require
    /// correct temporal solutions at all times. If you cannot fulffil its requirements
    /// or do not care about the accuracy of the absolute temporal solution, you can simply
    /// tie our [NullTime] structure here.
    /// - rtk_base: single reference that implements the [RTKBase] trait.
    /// - bias: external [Bias] model implementation, to improve overall accuracy.
    /// - initial_position_ecef_m: possible initial position, as ECEF coordinates in meters.
    pub fn new(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        orbit_source: O,
        time_source: T,
        rtk_base: RTK,
        bias: B,
        initial_position_ecef_m: Option<(f64, f64, f64)>,
    ) -> Self {
        let solver = Solver::new(
            almanac,
            earth_cef,
            cfg,
            orbit_source,
            time_source,
            true,
            rtk_base,
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
        rtk_base: RTK,
        bias: B,
    ) -> Self {
        Self::new(
            almanac,
            earth_cef,
            cfg,
            orbit_source,
            time_source,
            rtk_base,
            bias,
            None,
        )
    }

    /// [PVTSolution] solving attempt, at specified [Epoch] and using local [Candidate]s (rover's proposal).
    pub fn resolve(
        &mut self,
        epoch: Epoch,
        candidates: &[Candidate],
    ) -> Result<PVTSolution, Error> {
        let solution = self.solver.resolve(epoch, candidates)?;
        Ok(solution)
    }
}
