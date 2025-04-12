use crate::{
    error::Error,
    prelude::{Constellation, Epoch, TimeScale},
};

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
    /// Returns time correction in nanoseconds
    pub(crate) fn time_correction_nanos(&self, t: Epoch) -> Result<f64, Error> {
        let (t_week, t_nanos) = t.to_time_of_week();

        if t_week != self.t_ref.0 {
            return Err(Error::OutdatedTimeCorrection);
        }

        let dt = (t_nanos as f64 - self.t_ref.1 as f64) / 1.0E9;

        let (a0, a1, a2) = self.polynomials;
        let dt_s = a0 + a1 * dt + a2 * dt.powi(2);
        Ok(dt_s * 1.0E9)
    }

    /// Define a new [TimeOffset].
    pub fn from_epoch(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        let t_ref = t_ref.to_time_of_week();
        Self {
            t_ref,
            polynomials,
            lhs: Default::default(),
            rhs: Default::default(),
        }
    }

    /// Define a new [TimeOffset].
    pub fn from_time_of_week(t_ref: (u32, u64), polynomials: (f64, f64, f64)) -> Self {
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

/// The [Time] trait is required to obtain valid absolute temporal
/// solutions, in complex (multi constellation) sccenarios.
/// If you don't implement it, you can only obtain valid GPST solutions.
pub trait Time {
    /// Provide an update of the  time offset between LHS and RHS (reference) [TimeScale)
    fn time_offset_update(
        &mut self,
        t: Epoch,
        lhs: TimeScale,
        rhs: TimeScale,
    ) -> Option<TimeOffset>;
}

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
    pub fn update(&mut self, t: Epoch) {
        for (lhs, rhs) in [
            (TimeScale::GST, TimeScale::GPST),
            (TimeScale::GST, TimeScale::UTC),
            (TimeScale::BDT, TimeScale::GPST),
            (TimeScale::BDT, TimeScale::UTC),
            (TimeScale::BDT, TimeScale::GST),
            (TimeScale::GPST, TimeScale::UTC),
            (TimeScale::GPST, TimeScale::GST),
            (TimeScale::GPST, TimeScale::BDT),
        ] {
            if let Some(t_offset) = self.updater.time_offset_update(t, lhs, rhs) {
                self.time_offsets
                    .retain(|t| !(t.lhs == lhs && t.rhs == rhs));
                self.time_offsets.push(t_offset.with_lhs(lhs).with_rhs(rhs));
            }
        }
    }

    /// Returns temporal correction for this [Constellation] to prefered [TimeScale]
    pub fn constellation_correction_nanos(
        &self,
        t: Epoch,
        lhs: Constellation,
        prefered: TimeScale,
    ) -> Result<f64, Error> {
        let sv_ts = lhs.timescale().ok_or(Error::UnknownTimeCorection)?;

        self.time_correction_nanos(t, sv_ts, prefered)
    }

    /// Returns the correction for this [TimeScale] to RHS [TimeScale]
    pub fn time_correction_nanos(
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
            time_offset.time_correction_nanos(t)
        } else if let Some(time_offset) = self
            .time_offsets
            .iter()
            .filter(|t| t.lhs == rhs && t.rhs == lhs)
            .reduce(|k, _| k)
        {
            // Swapped correction is directly available
            let nanos = time_offset.time_correction_nanos(t)?;
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
}
