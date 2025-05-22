use crate::{
    cfg::User,
    prelude::{
        AbsoluteTime, Almanac, Bias, Candidate, Config, Epoch, Error, Frame, OrbitSource,
        PVTSolution, Rc,
    },
    rtk::RTKBase,
    solver::Solver,
};

use nalgebra::U7;

/// [RTK] is dedicated to resolve the state of a roaming (moving) target
/// with help of at least 1 external reference sitee.
/// It is the most accurate scenario we propose for moving targets.
/// For static site surveying, you should prefer the [StaticRTK] solver.
pub struct RTK<O: OrbitSource, B: Bias, T: AbsoluteTime> {
    /// Internal [Solver]
    solver: Solver<U7, O, B, T>,
}

impl<O: OrbitSource, B: Bias, T: AbsoluteTime> RTK<O, B, T> {
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
        orbit_source: Rc<O>,
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
        orbit_source: Rc<O>,
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
    /// - user: latest most suited [User] profile
    /// - candidates: proposed [Candidate]s
    /// - rtk_base: [RTKBase] implementation
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
