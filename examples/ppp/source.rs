use gnss_rtk::prelude::{Candidate, ClockCorrection, Duration, Epoch, Observation, SV};
use serde::Deserialize;

use flate2::read::GzDecoder;
use std::fs::File;
use std::io::Read;
use std::str::FromStr;

#[derive(Clone, Default, Deserialize)]
pub struct ObservationData {
    clk: f64,
    sv: String,
    epoch: Epoch,
    observation: Observation,
}

// Data Source Example
#[derive(Clone, Default)]
pub struct DataSource {
    pos: usize,
    len: usize,
    pending_epoch: Epoch,
    max_candidates: usize,
    pending: Vec<Candidate>,
    data: Vec<ObservationData>,
}

impl DataSource {
    pub fn new(max_candidates: usize) -> Self {
        // Data is compressed to reduce storage size
        let mut content = String::new();
        let f = format!("{}/examples/data/obs.json.gz", env!("CARGO_MANIFEST_DIR"));
        let fd =
            File::open(f).unwrap_or_else(|e| panic!("failed to read observations source: {}", e));
        let mut decoder = GzDecoder::new(fd);
        decoder
            .read_to_string(&mut content)
            .unwrap_or_else(|e| panic!("failed to read observations source: {}", e));
        let data: Vec<ObservationData> = serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("failed to parse observations: {}", e));
        let len = data.len();
        info!("Data source created: examples/data/obs.json");
        info!("Largest possible proposal: {}", max_candidates);
        Self {
            pos: 0,
            len,
            data,
            max_candidates,
            pending: Default::default(),
            pending_epoch: Default::default(),
        }
    }
}

impl Iterator for DataSource {
    type Item = (Epoch, Vec<Candidate>);
    fn next(&mut self) -> Option<Self::Item> {
        let mut proposal = Option::<Self::Item>::None;
        loop {
            if self.pos >= self.len {
                info!("consumed all data");
                return None;
            }
            let data = &self.data[self.pos];
            let sv = SV::from_str(&data.sv).expect("test data is not valid!");

            if data.epoch != self.pending_epoch {
                if !self.pending.is_empty() {
                    // returned snapshot
                    proposal = Some((self.pending_epoch, self.pending.clone()));
                }

                self.pending.clear();
                let dt = Duration::from_seconds(data.clk);
                let correction = ClockCorrection::without_relativistic_correction(dt);
                self.pending.push(Candidate::new(
                    sv,
                    data.epoch,
                    correction,
                    Default::default(),
                    [data.observation.clone()].to_vec(),
                    Default::default(),
                    Default::default(),
                ));

                self.pending_epoch = data.epoch;
            } else {
                if let Some(cd) = self
                    .pending
                    .iter_mut()
                    .filter(|p| p.sv == sv)
                    .reduce(|k, _| k)
                {
                    cd.add_observation_mut(data.observation.clone());
                } else {
                    self.pending.push(Candidate::new(
                        sv,
                        data.epoch,
                        Default::default(),
                        Default::default(),
                        [data.observation.clone()].to_vec(),
                        Default::default(),
                        Default::default(),
                    ));
                }

                if self.pending.len() == self.max_candidates {
                    // returned snapshot
                    proposal = Some((self.pending_epoch, self.pending.clone()));
                }
            }
            self.pos += 1;
            if let Some(proposal) = proposal {
                return Some(proposal);
            }
        }
    }
}
