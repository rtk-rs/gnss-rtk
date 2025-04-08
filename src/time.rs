use crate::prelude::{Constellation, Duration, TimeScale};

/// The [Time] trait is required to obtain valid absolute temporal
/// solutions, in complex (multi constellation) sccenarios.
/// If you don't implement it, you can only obtain valid GPST solutions.
pub trait Time {
    /// Provide an update of the GST - GPST time offset
    fn gst_gpst_offset_update(&mut self) -> Option<Duration>;

    /// Provide an update of the BDT - GPST time offset
    fn bdt_gpst_offset_update(&mut self) -> Option<Duration>;

    /// Provide an update of the GST - BDT time offset
    fn bdt_gst_offset_update(&mut self) -> Option<Duration>;
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
    pub fn update(&mut self) {
        if let Some(gst_gpst) = self.updater.gst_gpst_offset_update() {
            self.time_offsets
                .retain(|t| t.lhs != TimeScale::GST && t.rhs != TimeScale::GPST);
            self.time_offsets.push(TimeOffset::new_gst_gpst(gst_gpst));
        }

        if let Some(bdt_gpst) = self.updater.bdt_gpst_offset_update() {
            self.time_offsets
                .retain(|t| t.lhs != TimeScale::BDT && t.rhs != TimeScale::GPST);
            self.time_offsets.push(TimeOffset::new_bdt_gpst(bdt_gpst));
        }

        if let Some(bdt_gst) = self.updater.bdt_gst_offset_update() {
            self.time_offsets
                .retain(|t| t.lhs != TimeScale::BDT && t.rhs != TimeScale::GST);
            self.time_offsets.push(TimeOffset::new_bdt_gst(bdt_gst));
        }
    }

    /// Returns temporal correction for this [Constellation] to prefered [TimeScale]
    pub fn constellation_correction(
        &self,
        lhs: Constellation,
        prefered: TimeScale,
    ) -> Option<Duration> {
        let sv_ts = lhs.timescale()?;
        self.timescale_correction(sv_ts, prefered)
    }

    /// Returns the absolute correction for this [TimeScale] to prefered [TimeScale]
    pub fn timescale_correction(&self, lhs: TimeScale, prefered: TimeScale) -> Option<Duration> {
        if let Some(offset) = self
            .time_offsets
            .iter()
            .filter(|t| t.lhs == lhs && t.rhs == prefered)
            .reduce(|k, _| k)
        {
            // direction offset is known
            Some(offset.dt)
        } else {
            // need cross correction
            None
        }
    }
}

/// [TimeOffset] represents the duration interval
/// that currently applies between two [TimeScale]s
pub(crate) struct TimeOffset {
    /// Reference [TimeScale]
    pub rhs: TimeScale,
    /// Left-hand side [TimeScale]
    pub lhs: TimeScale,
    /// Offset as [Duration]
    pub dt: Duration,
}

impl TimeOffset {
    /// Define a new [TimeOffset] defined as [Duration] dt=lhs-rhs,
    /// where `rhs` is considered the destionation [TimeScale] in the conversion
    pub fn new(lhs: TimeScale, rhs: TimeScale, dt: Duration) -> Self {
        Self { lhs, rhs, dt }
    }

    /// Define a new [TimeOffset] as [Duration] dt=lhs-rhs
    /// where `lhs` is the left hand side GNSS [TimeScale] and
    /// `rhs` is the reference (or destination) [TimeScale]
    pub fn from_constellations(
        lhs: Constellation,
        rhs: Constellation,
        dt: Duration,
    ) -> Option<Self> {
        let lhs = lhs.timescale()?;
        let rhs = rhs.timescale()?;
        Some(Self::new(lhs, rhs, dt))
    }

    /// Define a new [TimeScale::GST] - [TimeScale::GPST] [TimeOFfset]
    pub fn new_gst_gpst(dt: Duration) -> Self {
        Self {
            lhs: TimeScale::GST,
            rhs: TimeScale::GPST,
            dt,
        }
    }

    /// Define a new [TimeScale::BDT] - [TimeScale::GPST] [TimeOffset]
    pub fn new_bdt_gpst(dt: Duration) -> Self {
        Self {
            lhs: TimeScale::BDT,
            rhs: TimeScale::GPST,
            dt,
        }
    }

    /// Define a new [TimeScale::BDT] - [TimeScale::GST] [TimeOffset]
    pub fn new_bdt_gst(dt: Duration) -> Self {
        Self {
            lhs: TimeScale::BDT,
            rhs: TimeScale::GST,
            dt,
        }
    }
}
