use crate::{averager::Averager, constants::SPEED_OF_LIGHT_M_S};

use log::warn;
// use polyfit_rs::polyfit_rs::polyfit;

// struct Buffer {
//     window: Duration,
//     gap_tolerance: Duration,
//     pub inner: Vec<(Epoch, f64)>,
// }
//
// impl Buffer {
//     /// Allocates a new Buffer
//     pub fn malloc(capacity: usize, window: Duration, gap_tolerance: Duration) -> Self {
//         Self {
//             window,
//             gap_tolerance,
//             inner: Vec::with_capacity(capacity),
//         }
//     }
//
//     /// Push data into self
//     pub fn push(&mut self, t: Epoch, y: f64) {
//         if let Some((t_last, _)) = self.inner.last() {
//             if (t - *t_last) > self.gap_tolerance {
//                 error!("{}: buffer reset on data gap", t);
//                 self.reset();
//             }
//         }
//         self.inner.push((t, y));
//
//         let t0 = self.inner[0].0;
//         if (t - t0) > self.window {
//             self.inner.remove(0);
//         }
//     }
//
//     /// Resets self
//     pub fn reset(&mut self) {
//         self.inner.clear();
//     }
//
//     /// Returns current number of symbols
//     pub fn len(&self) -> usize {
//         self.inner.len()
//     }
//
//     /// Performs polyfit (nth order) over self
//     pub fn polyfit(&self, order: usize) -> Option<Vec<f64>> {
//         if self.inner.len() > order {
//             let x_s = self
//                 .inner
//                 .iter()
//                 .map(|(k, _)| k.duration.to_seconds())
//                 .collect::<Vec<f64>>();
//             let y_s = self.inner.iter().map(|(_, v)| *v).collect::<Vec<f64>>();
//             match polyfit(&x_s, &y_s, order) {
//                 Ok(fit) => Some(fit),
//                 Err(e) => {
//                     warn!("polyfit error: {}", e);
//                     None
//                 },
//             }
//         } else {
//             None
//         }
//     }
// }

#[derive(Debug, Clone)]
pub struct Input {
    /// f1 frequency (Hz)
    pub f1_hz: f64,

    /// Code #1
    pub c1: f64,

    /// Phase #1
    pub l1: f64,

    /// f2 frequency (Hz)
    pub f2_hz: f64,

    /// Code #2
    pub c2: f64,

    /// Phase #2
    pub l2: f64,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Output {
    /// Estimated n_1
    pub n1: i32,

    /// Sigma (n_1)
    pub sigma_1: f64,

    /// Estimated n_2
    pub n2: i32,

    /// Sigma (n_w)
    pub sigma_w: f64,
}

#[derive(Debug, Clone)]
pub struct Solver {
    /// NW [Averager]
    nw_avg: Averager,

    /// N1 [Averager]
    n1_avg: Averager,
}

impl Solver {
    /// Builds a new [Solver].
    pub fn new() -> Self {
        Self {
            nw_avg: Averager::new(0.01),
            n1_avg: Averager::new(0.01),
        }
    }

    /// Reset this [Solver].
    pub fn reset(&mut self) {
        self.nw_avg.reset();
        self.n1_avg.reset();
    }

    /// Resolve [Output] from [Input]
    pub fn solve(&mut self, input: &Input) -> Option<Output> {
        let (lambda_1, lambda_2) = (
            SPEED_OF_LIGHT_M_S / input.f1_hz,
            SPEED_OF_LIGHT_M_S / input.f2_hz,
        );

        let lambda_wl = SPEED_OF_LIGHT_M_S / (input.f1_hz - input.f2_hz);

        let lw = (input.f1_hz * input.l1 - input.f2_hz * input.l2) / (input.f1_hz - input.f2_hz);
        let cn = (input.f1_hz * input.c1 + input.f2_hz * input.c2) / (input.f1_hz + input.f2_hz);

        self.nw_avg.add((lw - cn) / lambda_wl);

        let nw = self.nw_avg.mean.round();

        warn!("nw={nw}");

        let n1 = (input.l1 - input.l2 - lambda_2 * nw) / (lambda_1 - lambda_2);
        warn!("n1={n1}");

        self.n1_avg.add(n1);

        let n1 = self.n1_avg.mean.round() as i32;
        let n2 = n1 - nw as i32;
        warn!("n2={n2}");

        if self.n1_avg.count == 3 {
            Some(Output {
                n1,
                sigma_1: self.n1_avg.sigma(),
                n2,
                sigma_w: self.nw_avg.sigma(),
            })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{
        prelude::{Carrier, SPEED_OF_LIGHT_M_S},
        tests::init_logger,
    };

    use log::debug;

    use super::{Input, Solver};

    #[test]
    #[ignore]
    fn postfit_gps_amb_solver() {
        init_logger();

        let mut solver = Solver::new();

        let (sv, f1_hz, f2_hz) = (
            "G05",
            Carrier::L1.frequency_hz(),
            Carrier::L2.frequency_hz(),
        );

        let (lamb_1, lamb_2) = (SPEED_OF_LIGHT_M_S / f1_hz, SPEED_OF_LIGHT_M_S / f2_hz);

        for (t_str, c1, c2, l1, l2) in [
            (
                "2020-06-25T00:00:00 GPST",
                20947300.931,
                20947301.155,
                110078836.389,
                85775716.723,
            ),
            (
                "2020-06-25T00:00:30 GPST",
                20953278.537,
                20953278.871,
                110110249.716,
                85800194.640,
            ),
            (
                "2020-06-25T00:01:00 GPST",
                20959368.361,
                20959368.632,
                110142251.485,
                85825131.078,
            ),
            (
                "2020-06-25T00:01:30 GPST",
                20965569.284,
                20965569.440,
                110174836.965,
                85850522.363,
            ),
            (
                "2020-06-25T00:02:00 GPST",
                20971881.261,
                20971881.383,
                110208006.594,
                85876368.824,
            ),
            (
                "2020-06-25T00:02:30 GPST",
                20978303.919,
                20978304.178,
                110241757.569,
                85902668.288,
            ),
            (
                "2020-06-25T00:03:00 GPST",
                20984837.483,
                20984837.822,
                110276092.084,
                85929422.459,
            ),
            (
                "2020-06-25T00:03:30 GPST",
                20991482.603,
                20991482.873,
                110311012.321,
                85956633.031,
            ),
            (
                "2020-06-25T00:04:00 GPST",
                20998237.363,
                20998237.656,
                110346508.613,
                85984292.483,
            ),
            (
                "2020-06-25T00:04:30 GPST",
                21005103.246,
                21005103.458,
                110382588.560,
                86012406.724,
            ),
            (
                "2020-06-25T00:05:00 GPST",
                21012078.157,
                21012078.309,
                110419241.945,
                86040967.806,
            ),
            (
                "2020-06-25T00:05:30 GPST",
                21019163.813,
                21019163.853,
                110456477.249,
                86069982.327,
            ),
            (
                "2020-06-25T00:06:00 GPST",
                21026358.896,
                21026358.986,
                110494288.129,
                86099445.343,
            ),
            (
                "2020-06-25T00:06:30 GPST",
                21033664.096,
                21033664.270,
                110532677.671,
                86129359.273,
            ),
            (
                "2020-06-25T00:07:00 GPST",
                21041077.710,
                21041077.859,
                110571636.455,
                86159716.758,
            ),
            (
                "2020-06-25T00:07:30 GPST",
                21048601.584,
                21048601.582,
                110611173.893,
                86190525.151,
            ),
            (
                "2020-06-25T00:08:00 GPST",
                21056233.979,
                21056233.950,
                110651283.549,
                86221779.421,
            ),
            (
                "2020-06-25T00:08:30 GPST",
                21063974.565,
                21063974.620,
                110691960.272,
                86253475.568,
            ),
        ] {
            let input = Input {
                f1_hz,
                c1,
                l1: l1 / lamb_1,
                f2_hz,
                c2,
                l2: l2 / lamb_2,
            };

            let output = solver.solve(&input).unwrap_or_else(|| {
                panic!("G01 ambiguity solving failure");
            });

            let (lambda_1, lambda_2) = (SPEED_OF_LIGHT_M_S / f1_hz, SPEED_OF_LIGHT_M_S / f2_hz);

            debug!(
                "{}({}) - n_1={:.5}(\u{03c3}={:.5}) n_2={:.5}(\u{03c3}w={:.5}) P_1={:.5} L_1={:.5} P_2={:5} L_2={:.5}",
                t_str, sv, output.n1, output.sigma_1, output.n2, output.sigma_w,
                c1,
                c1 - l1 - output.n1 as f64 * lamb_1,
                c2,
                c2 - l2 - output.n2 as f64 * lamb_2,
            );
        }
        debug!("****************************");
    }
}
