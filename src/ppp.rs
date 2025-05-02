use nalgebra::U4;

use crate::{
    ppp::NullRTK,
    prelude::{
        AbsoluteTime, Almanac, Bias, Candidate, Config, Epoch, Error, Frame, OrbitSource,
        PVTSolution, User,
    },
    solver::Solver,
};

/// The [PPP] solver is used for absolute navigation, without access to an RTK network.
/// It achieves the complex task of obtaining a [PVTSolution], possibly from scratch
/// without any initial apriori.
pub struct PPP<O: OrbitSource, B: Bias, T: AbsoluteTime> {
    /// Internal [Solver]
    solver: Solver<U4, O, B, T>,
}

impl<O: OrbitSource, B: Bias, T: AbsoluteTime> PPP<O, B, T> {
    /// Creates a new [PPP] solver for direct absolute navigation,
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

    /// Creates a new [PPP] solver for direct absolute navigation,
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
    /// ## Input
    /// - user: latest [User] profile so we can adapt.   
    /// In static applications, only the measurement system profile counts here, anything else is disregarded.
    /// - epoch: sampling [Epoch]
    /// - candidates: proposed [Candidate]s
    /// ## Output
    /// - solution: as [PVTSolution]
    pub fn resolve(
        &mut self,
        user: User,
        epoch: Epoch,
        candidates: &[Candidate],
    ) -> Result<PVTSolution, Error> {
        let null_base = NullRTK {};
        let solution = self.solver.resolve(epoch, user, candidates, &null_base)?;

        Ok(solution)
    }

    /// Reset [PPP] solver. This is usually not needed, even on data gaps.
    /// For the simple reason that a correctly tuned filter will correctly adapt.
    pub fn reset(&mut self) {
        self.solver.reset();
    }
}
