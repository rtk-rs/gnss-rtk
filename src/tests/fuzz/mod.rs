use hifitime::Unit;
use log::{debug, info};
use rstest::*;

use rand::{prelude::*, rngs::SmallRng, SeedableRng};

use std::str::FromStr;

use crate::{
    navigation::apriori::Apriori,
    prelude::{
        Almanac, Config, Duration, Epoch, Error, Frame, Method, Orbit, Solver, TimeScale,
        UserParameters,
    },
    tests::{
        bias::TestBias, ephemeris::NullEph, init_logger, time::NullTime, CandidatesBuilder,
        OrbitsData, TestNumber,
    },
};

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

mod epochs;
mod orbits;
mod settings;

use epochs::*;
use orbits::*;

pub struct FuzzTest {
    t0: Epoch,
    name: String,
    cfg: Config,
    frame: Frame,
    almanac: Almanac,
    num_epochs: usize,
    num_sv: TestNumber<usize>,
    apriori: Option<Apriori>,
    sampling_period_nanos: TestNumber<i128>,
}

impl FuzzTest {
    const NB_REFERENCE_ORBITS: usize = 4;

    const REFERENCE_ORBITS_KM: [(f64, f64, f64); Self::NB_REFERENCE_ORBITS] = [
        (-11562.163582, 14053.114306, 23345.128269),
        (4577.136069, -22995.974895, 18062.640686),
        (16577.017768, -4619.539763, 24092.494804),
        (18846.610510, 16144.830741, 16159.863309),
    ];

    const REFERENCE_PR_M: [(f64, f64); Self::NB_REFERENCE_ORBITS] = [
        (27616185.992, 27616184.819),
        (27055946.391, 27055945.532),
        (23730317.923, 23730316.788),
        (22756243.562, 22756242.295),
    ];

    const REFERENCE_CP_M: [(f64, f64); Self::NB_REFERENCE_ORBITS] = [
        (145124050.106, 108371872.760),
        (142179967.778, 106173364.686),
        (124703702.220, 93122915.921),
        (119584910.611, 89300442.791),
    ];

    fn generate_orbits(&self, rng: &mut SmallRng, frame: Frame) -> FuzzTestOrbits {
        let mut t = self.t0;

        let mut orbits = Vec::new();

        for i in 0..self.num_epochs {
            let num_sv = self.num_sv.value();

            for j in 0..num_sv {
                let sv_index = j % Self::NB_REFERENCE_ORBITS;

                let (x_km, y_km, z_km) = Self::REFERENCE_ORBITS_KM[sv_index];

                let orbit = Orbit::from_position(x_km, y_km, z_km, t, frame);

                orbits.push(orbit);
            }

            t = t + Duration::from_nanoseconds(self.sampling_period_nanos.value() as f64);
        }

        FuzzTestOrbits { database: orbits }
    }

    fn generate_epochs(&self, rng: &mut SmallRng) -> Vec<FuzzTestEpoch> {
        let mut t = self.t0;
        let mut epochs = Vec::new();

        for i in 0..self.num_epochs {}

        epochs
    }

    pub fn run(&mut self) {
        init_logger();

        let default_params = UserParameters::default();

        let bias = TestBias {};
        let null_time = NullTime {};
        let null_eph = NullEph {};

        let mut generator = rand::rng();
        let mut rng = SmallRng::from_rng(&mut generator);

        let orbits = self.generate_orbits(&mut rng, self.frame);

        let apriori = if let Some(apriori) = self.apriori {
            Some(apriori.pos_m)
        } else {
            None
        };

        let mut solver = Solver::new(
            self.almanac.clone(),
            self.frame,
            self.cfg.clone(),
            null_eph.into(),
            orbits.into(),
            null_time,
            bias,
            apriori,
        );

        let epochs = self.generate_epochs(&mut rng);

        for (nth_epoch, epochs) in epochs.iter().enumerate() {
            match solver.ppp_solving(epochs.time, default_params, &epochs.candidates) {
                Err(e) => {
                    panic!("Solver failed with {}", e);
                },
                Ok(pvt) => {
                    info!(
                        "fuzz-test {} - nth {} - position Fix {:#?}",
                        self.name, nth_epoch, pvt
                    );
                },
            }
        }
    }
}
