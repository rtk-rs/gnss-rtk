use thiserror::Error;

use anise::{
    almanac::{metaload::MetaAlmanacError, planetary::PlanetaryDataError},
    errors::{AlmanacError, PhysicsError},
};

use crate::prelude::{Epoch, SV};

#[derive(Debug, PartialEq, Error)]
pub enum Error {
    /// Not enough candidates were proposed, with respect to navigation parameters.
    #[error("not enough candidates provided")]
    NotEnoughCandidates,

    /// Survey initialization (no apriori = internal guess)
    /// requires at least 4 SV in sight temporarily, whatever
    /// your navigation technique.
    #[error("survey initialization requires at least 4 SV temporarily")]
    NotEnoughInitializationCandidates,

    /// PreFit (signal quality, other..) criterias
    /// have been applied but we're left with not enough vehicles that match
    /// the navigation technique: no attempt.
    #[error("not enough candidates match pre-fit criteria")]
    NotEnoughPreFitCandidates,

    /// PostFit (state solver and other) have been resolved,
    /// but we're left with not enough vehicles that match
    /// the navigation technique: no attempt.
    #[error("not enough candidates match post-fit criteria")]
    NotEnoughPostFitCandidates,

    /// Failed to parse navigation method
    #[error("non supported/invalid strategy")]
    InvalidStrategy,

    #[error("not enough post-fit candidates to form a matrix")]
    MatrixMinimalDimension,

    #[error("internal error: invalid matrix setup")]
    MatrixDimension,

    #[error("failed to form matrix (invalid input or not enough data)")]
    MatrixFormationError,

    /// Invalid orbital states or bad signal data may cause the algebric calculations
    /// to wind up here.
    #[error("failed to invert matrix")]
    MatrixInversion,

    /// Invalid orbital states or bad signal data may cause the algebric calculations
    /// to abort.
    #[error("internal navigation error")]
    NavigationError,

    /// Failed to initialize navigation filter
    #[error("nav filter initialization error")]
    NavigationFilterInitError,

    #[error("missing pseudo range observation")]
    MissingPseudoRange,

    /// [Method::CPP] requires the special signal combination to exist.
    /// This require the user to sample PR on two separate frequencies.
    #[error("failed to form pseudo range combination")]
    PseudoRangeCombination,

    /// [Method::PPP] requires the special signal combination to exist.
    /// This require the user to sample PR + PH on two separate frequencies.
    #[error("failed to form phase range combination")]
    PhaseRangeCombination,

    /// Each [Candidate] state needs to be resolved to contribute to any PPP resolution attempt.
    #[error("unresolved candidate state")]
    UnresolvedState,

    /// Each [Candidate] presented to the Bancroft solver needs a resolved state.
    #[error("bancroft requires 4 fully resolved candidates")]
    UnresolvedStateBancroft,

    /// When [Modeling.sv_clock_bias] is turned on and we attempt PPP resolution,
    /// it is mandatory for the user to provide [ClockCorrection].
    #[error("missing clock correction")]
    UnknownClockCorrection,

    /// Physical non sense due to bad signal data or invalid orbital state, will cause us
    /// abort with this message.
    #[error("physical non sense: rx prior tx")]
    PhysicalNonSenseRxPriorTx,

    /// Physical non sense due to bad signal data or invalid orbital state, will cause us
    /// abort with this message.
    #[error("physical non sense: t_rx is too late")]
    PhysicalNonSenseRxTooLate,

    /// Error during surveying initialization, without apriori knowledge.
    /// The solver initialization requires a minimum of 4 SV in sight temporarily,
    /// whatever the navigation technique being used.
    #[error("survey initialization error: invalid input ?")]
    BancroftError,

    /// [Bancroft] initialization process (see [BancroftError]) will wind up here
    /// in case unrealistic or bad signal observation or orbital states were forwarded.
    #[error("bancroft solver error: invalid input (imaginary solution)")]
    BancroftImaginarySolution,

    /// Ambiguity factorization failed
    #[error("ambiguity factorization error")]
    AmbiguityFactorization,

    /// Matrix inversion error during ambiguity solving process
    #[error("ambiguity inverse error")]
    AmbiguityInverse,

    /// PPP navigation technique requires phase ambiguity to be solved prior any attempt.
    /// It is Okay to wind up here for a few iterations, until the ambiguities are fixed
    /// and we may proceed to precise navigation. We will reject solving attempt until then.
    /// Hardware and external events may reset the ambiguity fixes and it is okay to need to
    /// rerun through this phase for a short period of time. Normally not too often, when good
    /// equipment is properly operated.
    #[error("unresolved signal ambiguity")]
    UnresolvedAmbiguity,

    /// [Solver] requires [Almanac] determination at build up and may wind-up here this step is in failure.
    #[error("issue with Almanac: {0}")]
    Almanac(AlmanacError),

    /// [Solver] uses local [Almanac] storage for efficient deployments
    #[error("almanac setup issue: {0}")]
    MetaAlmanac(MetaAlmanacError),

    /// [Solver] requires to determine a [Frame] from [Almanac] and we wind-up here if this step is in failure.
    #[error("frame model error: {0}")]
    EarthFrame(PlanetaryDataError),

    /// Any physical non sense detected by ANISE will cause us to abort with this error.
    #[error("physics issue: {0}")]
    Physics(PhysicsError),

    /// Post fit (pre nav) error
    #[error("post-fit error")]
    PostfitPrenav,

    /// Remote observation is required for a [Candidate] to contribute in RTK solving attempt.
    /// You need up to four of them to resolve. We may print this internal message and still
    /// proceed to resolve, as [SV] may go out of sight of rover or reference site.
    #[error("missing observation on remote site {0}({1})")]
    MissingRemoteRTKObservation(Epoch, SV),

    /// In RTK resolution attempt, you need to observe all pending [SV] on reference site as well.
    /// If that is not the case, we abort with this error.
    #[error("missing observations on remote site")]
    MissingRemoteRTKObservations,

    #[error("invalid frequency")]
    InvalidFrequency,

    #[error("unknown carrier frequency")]
    UnknownCarrierFrequency,

    #[error("rejected troposhere delay: model is diverging.")]
    RejectedTropoDelay,

    #[error("rejected ionosphere delay: model diverging.")]
    RejectedIonoDelay,

    #[error("converged to physically invalid state")]
    StateUpdate,

    #[error("bad operation: negative time")]
    TimeUnderflow,

    #[error("cannot resolve absolute time: unknown time correction")]
    UnknownTimeCorection,

    #[error("outdated time correction (need at least weekly update!)")]
    OutdatedTimeCorrection,

    #[error("unknown SV timescale: cannot proceed")]
    UnknownTimescale,

    #[error("postfit filter converged to physically invalid state")]
    PostFitUpdate,

    #[error("internal error: filter is not initialized (bad op)")]
    UninitializedFilter,

    #[error("rejected solution: GDOP limit exceeded")]
    MaxGdopExceeded,

    /// [Error::MissingPhaseRangeMeasurements] is returned when using PPP
    /// strategy and CP measurements were not associated to PR measurements
    /// (which is mandatory).
    #[error("ppp issue: missing phase range measurements along cp measurements")]
    MissingPhaseRangeMeasurements,
}
