use crate::{
    prelude::{
        AbsoluteTime, Almanac, Bias, Candidate, Config, Epoch, Error, Frame, OrbitSource,
        PVTSolution, User,
    },
    rtk::RTKBase,
    solver::Solver,
};

use nalgebra::U4;

/// [StaticRTK] is dedicated to static sites (receivers) surveying with help
/// of at least 1 reference station, using differential navigation technique.
/// For roaming (moving) targets, you should use the [RTk] solver instead.
/// The objective is to resolve the state with ultra high accuracy.
/// IT is the most performant scenario we propose here.
pub struct StaticRTK<O: OrbitSource, B: Bias, T: AbsoluteTime> {
    /// Internal [Solver]
    solver: Solver<U4, O, B, T>,
}

impl<O: OrbitSource, B: Bias, T: AbsoluteTime> StaticRTK<O, B, T> {
    /// Creates a new [StaticRTK] for direct differential navigation
    /// of a static receiver, with external help of at least one reference site.
    /// If you know the initial position (a rough estimate will do),
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
    /// - rtk_base: [RTKBase] implementation (reference site)
    pub fn resolve<R: RTKBase>(
        &mut self,
        epoch: Epoch,
        user: User,
        candidates: &[Candidate],
        rtk_base: &R,
    ) -> Result<PVTSolution, Error> {
        let solution = self.solver.resolve(epoch, user, candidates, rtk_base)?;

        Ok(solution)
    }
}
