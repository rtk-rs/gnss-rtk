use anise::{
    prelude::{Almanac, Frame},
};

use crate::{
    bias::Bias,
    candidate::Candidate,
    cfg::{Config},
    navigation::PVTSolution,
    orbit::OrbitSource,
    prelude::{Epoch, Error, Rc},
    rtk::{RTKBase},
    time::AbsoluteTime,
    user::UserProfile,
    ephemeris::EphemerisSource,
    solver::Solver,
};

use nalgebra::{U7};

/// [KinematicSolver] to resolve [PVTSolution]s with dynamics.
/// Dedicated to moving targets, the dynamics are modeled and predicted,
/// as opposed to the [StaticSolver].
///
/// ## Generics:
/// - EPH: [EphemerisSource], possible Ephemeris provider.
/// - ORB: [OrbitSource], possible Orbit state provider.
/// - B: [Bias] model.
/// - TIM: [AbsoluteTime] source for correct absolute time
pub struct KinematicSolver<EPH: EphemerisSource, ORB: OrbitSource, B: Bias, TIM: AbsoluteTime> {
    solver: Solver<U7, EPH, ORB, B, TIM>,
}

impl<EPH: EphemerisSource, ORB: OrbitSource, B: Bias, TIM: AbsoluteTime> KinematicSolver<EPH, ORB, B, TIM>
{
    /// Creates a new [KinematicSolver] with possible initial position.
    ///
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - eph_source: [EphemerisSource] implementation, serves as "raw" / indirect
    /// orbit provider.
    /// - orbit_source: [OrbitSource] implementation to provide [Orbit]al states directly
    /// - time_source: [AbsoluteTime] implementation
    /// - bias: external [Bias] model implementation, to improve overall accuracy.
    /// - initial_ecef_m: possible initial state (ECEF, meters)
    pub fn new(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        eph_source: Rc<EPH>,
        orbit_source: Rc<ORB>,
        time_source: TIM,
        bias: B,
        initial_ecef_m: Option<(f64, f64, f64)>,
    ) -> Self {
        Self {
            solver: Solver::<U7, EPH, ORB, B, TIM>::new(
                almanac,
                earth_cef,
                cfg,
                eph_source,
                orbit_source,
                time_source,
                bias,
                initial_ecef_m,
            ),
        }
    }

    /// Creates a new [KinematicSolver] with no a apriori knowledge.
    /// In this case, the solver will have to initialize itself.
    ///
    /// ## Input
    /// - almanac: provided valid [Almanac]
    /// - earth_cef: [Frame] that must be an ECEF
    /// - cfg: solver [Config]uration
    /// - eph_source: [EphemerisSource] implementation, serves as "raw" / indirect
    /// orbit provider.
    /// - orbit_source: [OrbitSource] implementation to provide [Orbit]al states directly
    /// - time_source: [AbsoluteTime] implementation
    /// - bias: external [Bias] model implementation, to improve overall accuracy.
    pub fn new_survey(
        almanac: Almanac,
        earth_cef: Frame,
        cfg: Config,
        eph_source: Rc<EPH>,
        orbit_source: Rc<ORB>,
        time_source: TIM,
        bias: B,
    ) -> Self {
        Self {
            solver: Solver::<U7, EPH, ORB, B, TIM>::new(
                almanac,
                earth_cef,
                cfg,
                eph_source,
                orbit_source,
                time_source,
                bias,
                None,
            ),
        }
    }

    /// [PVTSolution] solving attempt using PPP technique (no reference).
    /// Use this when no [RTKBase] may be accessed. 
    /// Switch to RTK at any point in your session, when at least one [RTKBase] becomes
    /// accessible.
    ///
    /// ## Input
    /// - epoch: [Epoch] of measurement
    /// - profile: [UserProfile]
    /// - candidates: proposed [Candidate]s (= measurements)
    /// - rtk_base: possible [RTKBase] we will connect to
    ///
    /// ## Output
    /// - [PVTSolution].
    pub fn ppp_solving(
        &mut self,
        epoch: Epoch,
        profile: UserProfile,
        candidates: &[Candidate],
    ) -> Result<PVTSolution, Error> {
        self.solver.ppp_solving(epoch, profile, candidates)
    }

    /// [PVTSolution] solving attempt using RTK technique and a single reference
    /// site. Switch to PPP at any point in your session, when access to remote
    /// site is lost.
    ///
    /// ## Input
    /// - epoch: [Epoch] of measurement
    /// - profile: [UserProfile]
    /// - candidates: proposed [Candidate]s (= measurements)
    /// - base: [RTKBase] implementation, that must provide enough information
    /// for this to proceed. You may catch RTK related issues and
    /// retry using PPP technique.
    ///
    /// ## Output
    /// - [PVTSolution].
    pub fn rtk_solving<RTK: RTKBase>(
        &mut self,
        epoch: Epoch,
        profile: UserProfile,
        candidates: &[Candidate],
        base: &RTK,
    ) -> Result<PVTSolution, Error> {
        self.solver.rtk_solving(
            epoch,
            profile,
            candidates,
            base,
        )
    }

    /// Reset this [Solver].
    pub fn reset(&mut self) {
        self.solver.reset();
    }
}
