use rand::rngs::SmallRng;

use crate::prelude::{Candidate, Constellation, Epoch, SV};

impl Candidate {
    /// Generates a randomized [Candidate]
    pub fn new_random(
        t: Epoch,
        sv_prn: u8,
        sv_constellation: Constellation,
        has_phase_range: bool,
        default_carrier_is_l1: bool,
        nominal_pseudo_range: f64,
        pseudo_range_max_noise_m: i32,
        nominal_phase_range: f64,
        phase_range_max_noise_m: i32,
        rand_num_signals: (usize, usize),
        rng: &mut SmallRng,
    ) -> Self {
        let freqz_table = [
            1575_420, // L1
            1227_600, // L2
            1176_450, // L5
        ];

        let freqz_table_len = freqz_table.len();

        let mut observations = Vec::new();

        let sv_constellation = Constellation::Galileo;

        // let num_signals = rng.random_range::<usize>(rand_num_signals.0..rand_num_signals.1);

        // for i in 0..num_signals {
        //     let mut observation = Observation::default();

        //     let carrier = if i == 0 {

        //         let default_carrier = if default_carrier_is_l1 {
        //             Carrier::L1
        //         } else {
        //
        //             let freq_id = rng.random_range::<usize>(0..freqz_table_len);
        //             let freqz = freqz_table[freq_id];

        //             Carrier::from_freqz_mega_hz(freqz)
        //                 .unwrap_or_else(|e| {
        //                     panic!("Signal generator failed: invalid frequency {}MHz - {}", freqz, e);
        //                 })
        //         };
        //
        //     } else {
        //
        //     };
        //
        //     observation.carrier = carrier;

        //     let pr_variation = rng.random_range::<i32>(0..pseudo_range_max_noise_m) as f64;
        //     let pseudo_range = nominal_pseudo_range + pr_variation;
        //     observation.pseudo_range_m = Some(pseudo_range);

        //     if has_phase_range {
        //
        //         let cp_variation = rng.random_range::<i32>(0..phase_range_max_noise_m) as f64;
        //         let phase = nominal_phase_range + cp_variation;
        //         observation.pseudo_range_m = Some(pseudo_range);
        //     }
        // }

        Self::new(SV::new(sv_constellation, sv_prn), t, observations)
    }

    #[cfg(test)]
    /// Returns true if all SD(code) measurements are zero
    pub fn sd_codes_are_null(&self) -> bool {
        for obs in self.sd.iter() {
            if let Some(code) = obs.pseudo_range_m {
                if code != 0.0 {
                    return false;
                }
            }
        }
        true
    }

    #[cfg(test)]
    /// Returns true if all SD(phase) measurements are zero
    pub fn sd_phases_are_null(&self) -> bool {
        for obs in self.sd.iter() {
            if let Some(lp) = obs.phase_range_m {
                if lp != 0.0 {
                    return false;
                }
            }
        }
        true
    }
}
