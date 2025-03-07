use crate::prelude::Duration;

#[derive(Default, Debug, Copy, Clone)]
pub struct ClockCorrection {
    /// Correction to associated timescale, expressed as [Duration]
    pub duration: Duration,
    pub(crate) needs_relativistic_correction: bool,
}

impl ClockCorrection {
    /// Define a new [ClockCorrection] that already integrates relativistic corrections
    pub fn with_relativistic_correction(duration: Duration) -> Self {
        Self {
            duration,
            needs_relativistic_correction: false,
        }
    }
    /// Define a new [ClockCorrection] that does not integrate relativistic corrections
    pub fn without_relativistic_correction(duration: Duration) -> Self {
        Self {
            duration,
            needs_relativistic_correction: true,
        }
    }
}
