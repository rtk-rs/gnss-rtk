use crate::{
    navigation::apriori::Apriori,
    prelude::{Almanac, Config, Duration, Epoch, Frame, Method, TimeScale},
    tests::{fuzz::FuzzTest, reference_apriori_at_ref_epoch, TestNumber},
};

use anise::constants::frames::IAU_EARTH_FRAME;

use rstest::*;

#[fixture]
fn build_almanac() -> Almanac {
    use crate::tests::almanac;
    almanac()
}

#[fixture]
fn build_earth_frame() -> Frame {
    use crate::tests::earth_frame;
    earth_frame()
}

#[fixture]
fn build_initial_apriori() -> Apriori {
    use crate::tests::reference_apriori_at_ref_epoch;
    reference_apriori_at_ref_epoch()
}

#[derive(Debug, Clone)]
pub struct FuzzTestSettings {
    /// Name of this test
    pub name: String,

    /// Timescale
    pub timescale: TimeScale,

    /// Total number of epochs
    pub num_epochs: TestNumber<usize>,

    /// Number of SV per epoch
    pub num_sv: TestNumber<usize>,

    /// Number of signals
    pub num_signals: TestNumber<usize>,

    /// navi [Method] to deploy
    pub method: Method,

    /// Uses apriori preset or not
    pub apriori: bool,

    /// True if default signal is L1 signal
    pub default_signal_is_l1: bool,

    /// Offset in [Timescale] for first epoch
    pub timescale_t0_offset_nanos: TestNumber<i128>,

    /// Sampling period
    pub sampling_period_nanos: TestNumber<i128>,
}

impl Default for FuzzTestSettings {
    fn default() -> Self {
        Self {
            name: "unamed".to_string(),
            timescale: Default::default(),
            num_sv: TestNumber::fixed(4),
            num_signals: TestNumber::fixed(1),
            method: Default::default(),
            apriori: false,
            default_signal_is_l1: true,
            num_epochs: TestNumber::fixed(128),
            sampling_period_nanos: TestNumber::fixed(30_000_000_000),
            timescale_t0_offset_nanos: TestNumber::fixed(1_000_000_000_000),
        }
    }
}

impl FuzzTestSettings {
    pub fn name(&self, name: &str) -> Self {
        let mut s = self.clone();
        s.name = name.to_string();
        s
    }

    pub fn navigation_method(&self, method: Method) -> Self {
        let mut s = self.clone();
        s.method = method;
        s
    }

    pub fn timescale(&self, ts: TimeScale) -> Self {
        let mut s = self.clone();
        s.timescale = ts;
        s
    }

    pub fn t0_offset_nanos(&self, offset_nanos: i128) -> Self {
        let mut s = self.clone();
        s.timescale_t0_offset_nanos = TestNumber::fixed(offset_nanos);
        s
    }

    // pub fn random_epochs(&self, min: usize, max: usize) -> Self {
    //     let mut s = self.clone();
    //     s.num_epochs = TestNumber::range_random(min..max);
    //     s
    // }

    pub fn total_epochs(&self, size: usize) -> Self {
        let mut s = self.clone();
        s.num_epochs = TestNumber::fixed(size);
        s
    }

    pub fn constant_num_sv(&self, num: usize) -> Self {
        let mut s = self.clone();
        s.num_sv = TestNumber::fixed(num);
        s
    }

    // pub fn num_sv_range(&self, min: usize, max: usize) -> Self {
    //     let mut s = self.clone();
    //     s.num_sv = TestNumber::range_random(min..max);
    //     s
    // }

    pub fn constant_num_signals(&self, num: usize) -> Self {
        let mut s = self.clone();
        s.num_signals = TestNumber::fixed(num);
        s
    }

    // pub fn num_signals_range(&self, min: usize, max: usize) -> Self {
    //     let mut s = self.clone();
    //     s.num_signals = TestNumber::range_random(min..max);
    //     s
    // }

    pub fn sampling_period(&self, dt: Duration) -> Self {
        let mut s = self.clone();
        s.sampling_period_nanos = TestNumber::fixed(dt.total_nanoseconds());
        s
    }

    pub fn uses_apriori(&self) -> Self {
        let mut s = self.clone();
        s.apriori = true;
        s
    }

    pub fn build(&self) -> FuzzTest {
        let almanac = build_almanac();

        let frame = almanac.frame_from_uid(IAU_EARTH_FRAME).unwrap_or_else(|e| {
            panic!("failed to deploy fuzz-test: {}", e);
        });

        let cfg = Config::default().with_navigation_method(self.method);

        let t0 = Epoch::from_duration(
            Duration::from_nanoseconds(self.timescale_t0_offset_nanos.value() as f64),
            self.timescale,
        );

        let num_epochs = self.num_epochs.value();

        FuzzTest {
            almanac,
            frame,
            t0,
            cfg,
            num_epochs,
            name: self.name.clone(),
            num_sv: self.num_sv.clone(),
            sampling_period_nanos: self.sampling_period_nanos.clone(),
            apriori: if self.apriori {
                Some(reference_apriori_at_ref_epoch())
            } else {
                None
            },
        }
    }
}
