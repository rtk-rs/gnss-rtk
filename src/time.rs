use crate::{
    error::Error,
    prelude::{Constellation, Epoch, TimeScale},
};

use hifitime::Unit;

/// [TimeOffset]s as provided by the [Time] trait.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TimeOffset {
    /// Reference time
    pub t_ref: (u32, u64),
    /// LHS [TimeScale]
    pub(crate) lhs: TimeScale,
    /// RHS [TimeScale]
    pub(crate) rhs: TimeScale,
    /// Polynomial terms, for interpolation
    pub polynomials: (f64, f64, f64),
}

impl TimeOffset {
    /// Returns time correction in seconds
    pub(crate) fn time_correction_seconds(&self, t: Epoch) -> Result<f64, Error> {
        let (t_week, t_nanos) = t.to_time_of_week();

        // if t_week > self.t_ref.0 +1 || self.t_ref.0 > t_week +1 {
        //     panic!("t_week : {} t_ref: {}", t_week, self.t_ref.0);
        //     return Err(Error::OutdatedTimeCorrection);
        // }

        let dt_s = (t_nanos as f64 - self.t_ref.1 as f64) / 1.0E9;

        let (a0, a1, a2) = self.polynomials;
        let dt_s = a0 + a1 * dt_s + a2 * dt_s.powi(2);
        Ok(dt_s)
    }

    /// Define a new |GPST-UTC| [TimeOffset] from [Epoch]
    pub fn from_gpst_utc_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        Self::from_epoch(t_ref, polynomials)
            .with_lhs(TimeScale::GPST)
            .with_rhs(TimeScale::UTC)
    }

    /// Define a new |GPST-UTC| [TimeOffset] from week counter
    pub fn from_gpst_utc_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
        Self::from_time_of_week(t_ref, polynomials)
            .with_lhs(TimeScale::GPST)
            .with_rhs(TimeScale::UTC)
    }

    /// Define a new |GST-GPST| [TimeOffset] from [Epoch]
    pub fn from_gst_gpst_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        Self::from_epoch(t_ref, polynomials)
            .with_lhs(TimeScale::GST)
            .with_rhs(TimeScale::GPST)
    }

    /// Define a new |GST-GPST| [TimeOffset] from week counter
    pub fn from_gst_gpst_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
        Self::from_time_of_week(t_ref, polynomials)
            .with_lhs(TimeScale::GST)
            .with_rhs(TimeScale::GPST)
    }

    /// Define a new |GST-UTC| [TimeOffset] from [Epoch]
    pub fn from_gst_utc_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        Self::from_epoch(t_ref, polynomials)
            .with_lhs(TimeScale::GST)
            .with_rhs(TimeScale::UTC)
    }

    /// Define a new |GST-UTC| [TimeOffset] from week counter
    pub fn from_gst_utc_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
        Self::from_time_of_week(t_ref, polynomials)
            .with_lhs(TimeScale::GST)
            .with_rhs(TimeScale::UTC)
    }

    /// Define a new |BDT-GPST| [TimeOffset] from [Epoch]
    pub fn from_bdt_gpst_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        Self::from_epoch(t_ref, polynomials)
            .with_lhs(TimeScale::BDT)
            .with_rhs(TimeScale::GPST)
    }

    /// Define a new |BDT-GPST| [TimeOffset] from week counter
    pub fn from_bdt_gpst_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
        Self::from_time_of_week(t_ref, polynomials)
            .with_lhs(TimeScale::BDT)
            .with_rhs(TimeScale::GPST)
    }

    /// Define a new |BDT-GST| [TimeOffset] from [Epoch]
    pub fn from_bdt_gst_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        Self::from_epoch(t_ref, polynomials)
            .with_lhs(TimeScale::BDT)
            .with_rhs(TimeScale::GST)
    }

    /// Define a new |BDT-GST| [TimeOffset] from week counter
    pub fn from_bdt_gst_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
        Self::from_time_of_week(t_ref, polynomials)
            .with_lhs(TimeScale::BDT)
            .with_rhs(TimeScale::GST)
    }

    /// Define a new |BDT-UTC| [TimeOffset] from [Epoch]
    pub fn from_bdt_utc_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        Self::from_epoch(t_ref, polynomials)
            .with_lhs(TimeScale::BDT)
            .with_rhs(TimeScale::UTC)
    }

    /// Define a new |BDT-UTC| [TimeOffset] from week counter
    pub fn from_bdt_utc_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
        Self::from_time_of_week(t_ref, polynomials)
            .with_lhs(TimeScale::BDT)
            .with_rhs(TimeScale::UTC)
    }

    /// Define a new [TimeOffset].
    fn from_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        let t_ref = t_ref.to_time_of_week();
        Self {
            t_ref,
            polynomials,
            lhs: Default::default(),
            rhs: Default::default(),
        }
    }

    /// Define a new [TimeOffset].
    fn from_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
        Self {
            t_ref,
            polynomials,
            lhs: Default::default(),
            rhs: Default::default(),
        }
    }

    /// Define [TimeOffset] with desired LHS (TimeScale]
    pub(crate) fn with_lhs(&self, ts: TimeScale) -> Self {
        let mut s = self.clone();
        s.lhs = ts;
        s
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
/// correction of the temporal solution to match the desired [TimeScale] behavior accurately.
/// It also allows to provide measurements in all described [TimeScale]s and maintain temporal and geometric accuracy.
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
            // Correction is directly available
            time_offset.time_correction_seconds(t)
        } else if let Some(time_offset) = self
            .time_offsets
            .iter()
            .filter(|t| t.lhs == rhs && t.rhs == lhs)
            .reduce(|k, _| k)
        {
            // Swapped correction is directly available
            let nanos = time_offset.time_correction_seconds(t)?;
            Ok(-nanos)
        } else {
            // // Cross-mixed corrections
            // for t1_offset in self.time_offsets.iter() {
            //     if t1_offset.lhs == lhs {
            //         for t2_offset in self.time_offsets.iter() {
            //             if t1_offset.rhs == t2_offset.rhs && t2_offset.lhs == rhs {
            //                 // |GST-BDT| = |GST-GPST| ; |BDT-GPST|
            //                 let mut correction = t1_offset.time_correction_nanos(t)?;
            //                 correction -= t2_offset.time_correction_nanos(t)?;
            //                 return Ok(correction);
            //             }
            //         }
            //     }
            // }
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
