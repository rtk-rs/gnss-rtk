use crate::{
    navigation::state::State,
    prelude::{Almanac, Bias, Frame, OrbitSource, PVTSolution, RTKBase, RTKSolver, Time},
};

use nalgebra::U4;

/// [StarNRTK] (NRTK*) allows solving the position of a single rover
/// while connecting to N- Base stations (maximal connection number).
/// Unlike basic [RTKsolver], connection to one or several bases is now tolerated,
/// as long as a minimum of one base remains connected and within baseline range limitation, at all times.
pub struct StarNRTK<const N: usize, O: OrbitSource, B: Bias, T: Time, RTK: RTKBase> {
    /// [Almanac]
    almanac: Almanac,
    /// [Frame]
    earth_cef: Frame,
    /// Current state
    state: State<U4>,
    /// [R]TK
    rtk_bases: Vec<RTK>,
    /// Internal solvers, one per rover-base couple
    solvers: Vec<RTKSolver<O, B, T>>,
}

impl<const N: usize, O: OrbitSource, B: Bias, T: Time, RTK: RTKBase> StarNRTK<N, O, B, T, RTK> {
    pub fn new() -> Self {
        Self {}
    }

    pub fn new_survey() -> Self {
        Self {}
    }

    pub fn resolve(&mut self, t: Epoch, candidates: &[Candidate]) -> Result<PVTSolution, Error> {}
}
