use crate::{
    error::Error,
    prelude::{Constellation, Epoch, Polynomial, TimeScale},
};

use hifitime::Unit;

/// [TimeOffset]s as provided by the [Time] trait.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TimeOffset {
    /// Reference [Epoch]]
    pub ref_epoch: Epoch,
    /// LHS [TimeScale]
    pub(crate) lhs: TimeScale,
    /// RHS [TimeScale]
    pub(crate) rhs: TimeScale,
    /// Polynomial terms, for interpolation
    pub polynomial: Polynomial,
}

impl TimeOffset {
    /// Define a new |GPST-UTC| [TimeOffset] with reference [Epoch] that must be expressed in [TimeScale::GPST].
    pub fn from_gpst_utc_epoch(t_ref: Epoch, polynomial: Polynomial) -> Self {
        assert_eq!(t_ref.time_scale, TimeScale::GPST);

        Self::from_epoch(t_ref, polynomial).with_rhs(TimeScale::UTC)
    }

    /// Define a new |GPST-UTC| [TimeOffset] from week counter
    pub fn from_gpst_utc_time_of_week(t_ref: (u32, u64), polynomial: Polynomial) -> Self {
        Self::from_time_of_week(t_ref, TimeScale::GPST, polynomial).with_rhs(TimeScale::UTC)
    }

    /// Define a new |GST-GPST| [TimeOffset] with reference [Epoch] that must be expressed in [TimeScale::GST].
    pub fn from_gst_gpst_epoch(t_ref: Epoch, polynomial: Polynomial) -> Self {
        assert_eq!(t_ref.time_scale, TimeScale::GST);

        Self::from_epoch(t_ref, polynomial).with_rhs(TimeScale::GPST)
    }

    /// Define a new |GST-GPST| [TimeOffset] from week counter
    pub fn from_gst_gpst_time_of_week(t_ref: (u32, u64), polynomial: Polynomial) -> Self {
        Self::from_time_of_week(t_ref, TimeScale::GST, polynomial).with_rhs(TimeScale::GPST)
    }

    /// Define a new |GST-UTC| [TimeOffset] with reference [Epoch] that must be expressed in [TimeScale::GST].
    pub fn from_gst_utc_epoch(t_ref: Epoch, polynomial: Polynomial) -> Self {
        assert_eq!(t_ref.time_scale, TimeScale::GST);

        Self::from_epoch(t_ref, polynomial).with_rhs(TimeScale::UTC)
    }

    /// Define a new |GST-UTC| [TimeOffset] from week counter
    pub fn from_gst_utc_time_of_week(t_ref: (u32, u64), polynomial: Polynomial) -> Self {
        Self::from_time_of_week(t_ref, TimeScale::GST, polynomial).with_rhs(TimeScale::UTC)
    }

    /// Define a new |BDT-GPST| [TimeOffset] with reference [Epoch] that must be expressed in [TimeScale::BDT].
    pub fn from_bdt_gpst_epoch(t_ref: Epoch, polynomial: Polynomial) -> Self {
        assert_eq!(t_ref.time_scale, TimeScale::BDT);

        Self::from_epoch(t_ref, polynomial).with_rhs(TimeScale::GPST)
    }

    /// Define a new |BDT-GPST| [TimeOffset] from week counter
    pub fn from_bdt_gpst_time_of_week(t_ref: (u32, u64), polynomial: Polynomial) -> Self {
        Self::from_time_of_week(t_ref, TimeScale::BDT, polynomial).with_rhs(TimeScale::GPST)
    }

    /// Define a new |BDT-GST| [TimeOffset] with reference [Epoch] that must be expressed in [TimeScale::BDT].
    pub fn from_bdt_gst_epoch(t_ref: Epoch, polynomial: Polynomial) -> Self {
        assert_eq!(t_ref.time_scale, TimeScale::BDT);

        Self::from_epoch(t_ref, polynomial).with_rhs(TimeScale::GST)
    }

    /// Define a new |BDT-GST| [TimeOffset] from week counter
    pub fn from_bdt_gst_time_of_week(t_ref: (u32, u64), polynomial: Polynomial) -> Self {
        Self::from_time_of_week(t_ref, TimeScale::BDT, polynomial).with_rhs(TimeScale::GST)
    }

    /// Define a new |BDT-UTC| [TimeOffset] with reference [Epoch] that must be expressed in [TimeScale::BDT].
    pub fn from_bdt_utc_epoch(t_ref: Epoch, polynomial: Polynomial) -> Self {
        assert_eq!(t_ref.time_scale, TimeScale::BDT);

        Self::from_epoch(t_ref, polynomial).with_rhs(TimeScale::UTC)
    }

    /// Define a new |BDT-UTC| [TimeOffset] from week counter
    pub fn from_bdt_utc_time_of_week(t_ref: (u32, u64), polynomial: Polynomial) -> Self {
        Self::from_time_of_week(t_ref, TimeScale::BDT, polynomial).with_rhs(TimeScale::UTC)
    }

    /// Define a new [TimeOffset].
    fn from_epoch(ref_epoch: Epoch, polynomial: Polynomial) -> Self {
        Self {
            ref_epoch,
            polynomial,
            lhs: ref_epoch.time_scale,
            rhs: Default::default(),
        }
    }

    /// Define a new [TimeOffset] from reference time expressed as Time of Week
    /// ## Input
    /// - week counter (u32)
    /// - seconds within week (u64)
    /// - timescale: [TimeScale]
    /// - published [Polynomial]
    fn from_time_of_week(t_ref: (u32, u64), timescale: TimeScale, polynomial: Polynomial) -> Self {
        Self {
            polynomial,
            ref_epoch: Epoch::from_time_of_week(t_ref.0, t_ref.1 * 1_000_000_000, timescale),
            lhs: Default::default(),
            rhs: Default::default(),
        }
    }

    /// Define [TimeOffset] with desired LHS (TimeScale]
    pub(crate) fn with_rhs(&self, ts: TimeScale) -> Self {
        let mut s = self.clone();
        s.rhs = ts;
        s
    }
}

/// [Time] is implemented by applications where precise absolute [Time] needs to be resolved at
/// all times, and in all scenarios. Correct implementation of this trait allows
/// - to express temporal solution in the desired [TimeScale] very precisely (following the [TimeScale] behavior)
/// - measurements can be expressed in any supported [TimeScale] and the temporal solution will remain correct.
pub trait Time {
    /// Update the |[TimeScale::GPST] - [TimeScale::UTC]| [TimeOffset]
    fn gpst_utc_time_offset(&mut self, now: Epoch) -> Option<TimeOffset>;

    /// Update the |[TimeScale::GST] - [TimeScale::GPST]| [TimeOffset]
    fn gst_gpst_time_offset(&mut self, now: Epoch) -> Option<TimeOffset>;

    /// Update the |[TimeScale::GST] - [TimeScale::UTC]| [TimeOffset]
    fn gst_utc_time_offset(&mut self, now: Epoch) -> Option<TimeOffset>;

    /// Update the |[TimeScale::BDT] - [TimeScale::GPST]| [TimeOffset]
    fn bdt_gpst_time_offset(&mut self, now: Epoch) -> Option<TimeOffset>;

    /// Update the |[TimeScale::BDT] - [TimeScale::GST]| [TimeOffset]
    fn bdt_gst_time_offset(&mut self, now: Epoch) -> Option<TimeOffset>;

    /// Update the |[TimeScale::BDT] - [TimeScale::UTC]| [TimeOffset]
    fn bdt_utc_time_offset(&mut self, now: Epoch) -> Option<TimeOffset>;
}

/// [NullTime] is used by applications that do not require precise absolute temporal solutions,
/// or cannot fullfil its requirements.
pub struct NullTime {}

pub(crate) struct AbsoluteTime<T: Time> {
    /// Updater
    updater: T,
    /// Applicable [TimeOffset]s
    time_offsets: Vec<TimeOffset>,
}

impl<T: Time> AbsoluteTime<T> {
    /// Creates a new [AbsoluteTime] source
    pub fn new(updater: T) -> Self {
        Self {
            updater,
            time_offsets: Default::default(),
        }
    }

    /// Update [AbsoluteTime] reference
    pub fn update(&mut self, now: Epoch) {
        if let Some(t_offset) = self.updater.gpst_utc_time_offset(now) {
            self.time_offsets
                .retain(|t| !(t.lhs == TimeScale::GPST && t.rhs == TimeScale::UTC));
            self.time_offsets.push(t_offset);
        }

        if let Some(t_offset) = self.updater.gst_gpst_time_offset(now) {
            self.time_offsets
                .retain(|t| !(t.lhs == TimeScale::GST && t.rhs == TimeScale::GPST));
            self.time_offsets.push(t_offset);
        }

        if let Some(t_offset) = self.updater.gst_utc_time_offset(now) {
            self.time_offsets
                .retain(|t| !(t.lhs == TimeScale::GST && t.rhs == TimeScale::UTC));
            self.time_offsets.push(t_offset);
        }

        if let Some(t_offset) = self.updater.bdt_gpst_time_offset(now) {
            self.time_offsets
                .retain(|t| !(t.lhs == TimeScale::BDT && t.rhs == TimeScale::GPST));
            self.time_offsets.push(t_offset);
        }

        if let Some(t_offset) = self.updater.bdt_gst_time_offset(now) {
            self.time_offsets
                .retain(|t| !(t.lhs == TimeScale::BDT && t.rhs == TimeScale::GST));
            self.time_offsets.push(t_offset);
        }

        if let Some(t_offset) = self.updater.bdt_utc_time_offset(now) {
            self.time_offsets
                .retain(|t| !(t.lhs == TimeScale::BDT && t.rhs == TimeScale::UTC));
            self.time_offsets.push(t_offset);
        }
    }

    /// Returns temporal correction for this [Constellation] to prefered [TimeScale]
    pub fn constellation_correction_seconds(
        &self,
        t: Epoch,
        lhs: Constellation,
        prefered: TimeScale,
    ) -> Result<f64, Error> {
        let sv_ts = lhs.timescale().ok_or(Error::UnknownTimeCorection)?;

        self.time_correction_seconds(t, sv_ts, prefered)
    }

    /// Returns the correction for this [TimeScale] to RHS [TimeScale]
    pub fn time_correction_seconds(
        &self,
        t: Epoch,
        lhs: TimeScale,
        rhs: TimeScale,
    ) -> Result<f64, Error> {
        if let Some(time_offset) = self
            .time_offsets
            .iter()
            .filter(|t| t.lhs == lhs && t.rhs == rhs)
            .reduce(|k, _| k)
        {
            // Forward correction
            let dt_s = (t.to_time_scale(time_offset.lhs) - time_offset.ref_epoch).to_seconds();
            Ok(dt_s)
        } else if let Some(time_offset) = self
            .time_offsets
            .iter()
            .filter(|t| t.lhs == rhs && t.rhs == lhs)
            .reduce(|k, _| k)
        {
            // Forward correction
            let dt_s = (t.to_time_scale(time_offset.lhs) - time_offset.ref_epoch).to_seconds();
            Ok(dt_s)
        } else {
            Err(Error::UnknownTimeCorection)
        }
    }

    /// Applies a time correction, using current data base, but limited to 1 nanosecond precision.
    pub fn epoch_time_correction(&self, t: Epoch, target: TimeScale) -> Result<Epoch, Error> {
        let lhs = t.time_scale;

        let time_correction_nanos = self.time_correction_seconds(t, lhs, target)?;
        let t = t.to_time_scale(target) + time_correction_nanos * Unit::Second;

        Ok(t)
    }
}
