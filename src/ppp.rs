use nalgebra::U4;

use crate::{
    prelude::{
        AbsoluteTime, Almanac, Bias, Candidate, Config, Epoch, Error, Frame, OrbitSource,
        PVTSolution, Rc, User, SV,
    },
    rtk::{RTKBase, NullRTK},
    solver::Solver,
};

/// [PPP] is an advanced navigation solver, capable of solving precise navigation
/// [PVTSolution]s from either real-time or post-processed data. [PPP] supports
/// both absolute navigation with [PPP::ppp_solving], differential navigation with
/// [PPP::rtk_solving]. It can operate with or without apriori knowledge and offers
/// three navigation strategies, expressed as [Method], that define what you must provide
/// and the accuracy you can hope for.
pub struct PPP<EPH: EphemerisSource, ORB: OrbitSource, B: Bias, TIM: AbsoluteTime> {
    /// Internal [Solver]
    solver: Solver<EPH, ORB, B, TIM>,
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

    /// [PVTSolution] solving attempt, at specified [Epoch] and using proposed [Candidate]s.
    /// ## Input
    /// - user: latest [User] profile so we can adapt.   
    /// Keep the [User] profile up to date with the rover behavior, in dynamic applications.  
    /// The measurement system profile is also contained in the profile, and this may apply to static applications as well.
    /// - epoch: sampling [Epoch]
    /// - candidates: proposed [Candidate]s
    /// ## Output
    /// - solution: as [PVTSolution]
    pub fn ppp_solving(
        &mut self,
        user: User,
        epoch: Epoch,
        candidates: &[Candidate],
    ) -> Result<PVTSolution, Error> {
        let null_base = NullRTK {};
        let solution = self.solver.resolve(epoch, user, candidates, &null_base)?;

        Ok(solution)
    }

    // pub fn rtk_solving(
    //     &mut self,
    //     user: User,
    //     epoch: Epoch,
    //     candidates: &[Candidate],
    //     rtk: RTK,
    // ) -> Result<PVTSolution, Error> {
    //     let null_base = NullRTK {};
    //     let solution = self.solver.resolve(epoch, user, candidates, &null_base)?;

    //     Ok(solution)
    // }

    /// Reset [PPP] solver. This is usually not needed, even on data gaps.
    /// For the simple reason that a correctly tuned filter will correctly adapt.
    pub fn reset(&mut self) {
        self.solver.reset();
    }
}
