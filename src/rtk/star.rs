
/// StarNRTK (NRTK*) allows solving the position of a single rover
/// while connecting to N- Base stations (maximal connection number). 
/// Unlike basic [RTKsolver], connection to one or several bases is now tolerated,
/// as long as a minimum of one base remains connected and within baseline range, at all times.
pub struct StarNRTK<const N: usize, O: OrbitSource, B: Bias, T: Time, RTK: RTKBase> {
    /// [Almanac]
    almanac: Almanac,
    /// [Frame]
    earth_cef: Frame,
    /// Current state
    state: State,
    /// [R]TK
    rtk_bases: Vec<RTK>,
    /// Internal solvers, one per base
    solvers: Vec<RTKSolver<O, B, T>>,
    /// [AbsoluteTime] source
    absolute_time: AbsoluteTime<T>,
}