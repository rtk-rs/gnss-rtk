use crate::prelude::{Constellation, Duration, Epoch, TimeScale};
use hifitime::Unit;

/// [TimeOffset]s as provided by the [Time] trait.
#[derive(Copy, Clone, PartialEq)]
pub struct TimeOffset {
    /// Reference time in nanoseconds
    pub t_ref: u64,
    /// Polynomial terms, for interpolation
    pub polynomials: (f64, f64, f64),
    /// LHS [TimeScale]
    pub(crate) lhs: TimeScale,
    /// RHS [TimeScale]
    pub(crate) rhs: TimeScale,
}

impl TimeOffset {
    /// Returns time correction as [Duration]
    pub(crate) fn time_correction(&self, t: Epoch) -> Duration {
        let (_, t) = t.to_time_of_week();
        let dt = (t - self.t_ref) as f64;
        let (a0, a1, a2) = self.polynomials;
        Duration::from_nanoseconds(a0 + a1 * dt + a2 * dt.powi(2))
    }

    /// Define a new [TimeOffset].
    pub fn new(t_ref: Epoch, polynomials: (f64, f64, f64)) -> Self {
        let (_, t_ref) = t_ref.to_time_of_week();
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
    pub time_offsets: Vec<TimeOffset>,
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
            (TimeScale::BDT, TimeScale::GPST),
            (TimeScale::GPST, TimeScale::UTC),
            (TimeScale::GST, TimeScale::UTC),
            (TimeScale::BDT, TimeScale::UTC),
        ] {
            if let Some(t_offset) = self.updater.time_offset_update(t, lhs, rhs) {
                self.time_offsets.retain(|t| t.lhs != lhs && t.rhs != rhs);

                self.time_offsets.push(t_offset);
            }
        }
    }

    /// Returns temporal correction for this [Constellation] to prefered [TimeScale]
    pub fn constellation_correction(
        &self,
        t: Epoch,
        lhs: Constellation,
        prefered: TimeScale,
    ) -> Option<Duration> {
        let sv_ts = lhs.timescale()?;
        self.correction(t, sv_ts, prefered)
    }

    /// Returns the absolute correction for this [TimeScale] to prefered [TimeScale]
    pub fn correction(&self, t: Epoch, lhs: TimeScale, prefered: TimeScale) -> Option<Duration> {
        if let Some(time_offset) = self
            .time_offsets
            .iter()
            .filter(|t| t.lhs == lhs && t.rhs == prefered)
            .reduce(|k, _| k)
        {
            // verify correctness
            assert_eq!(t.time_scale, lhs, "internal error: timescale mismatch!");

            Some(time_offset.time_correction(t))
        } else {
            let pivots = match prefered {
                TimeScale::GPST => [TimeScale::UTC, TimeScale::GST, TimeScale::BDT],
                TimeScale::UTC => [TimeScale::GPST, TimeScale::GST, TimeScale::BDT],
                TimeScale::GST => [TimeScale::GPST, TimeScale::UTC, TimeScale::BDT],
                TimeScale::BDT => [TimeScale::GPST, TimeScale::GST, TimeScale::UTC],
                _ => {
                    return None;
                },
            };

            let mut ret = None;

            for pivot in pivots.iter() {}

            ret
        }
    }
}
