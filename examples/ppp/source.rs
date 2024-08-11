use gnss_rtk::prelude::{Candidate, Epoch, Observation, SV};
use itertools::Itertools;
use serde::Deserialize;
use std::fs::read_to_string;
use std::str::FromStr;

#[derive(Clone, Default, Deserialize)]
pub struct ObservationData {
    sv: String,
    epoch: Epoch,
    observation: Observation,
}

// Data Source Example
#[derive(Clone, Default)]
pub struct DataSource {
    pos: usize,
    len: usize,
    max_candidates: usize,
    data: Vec<ObservationData>,
    pending_epoch: Epoch,
    pending: Vec<Candidate>,
}

impl DataSource {
    pub fn new(max_candidates: usize) -> Self {
        let content = read_to_string("examples/data/obs.json")
            .unwrap_or_else(|e| panic!("failed to read observations source: {}", e));
        let data: Vec<ObservationData> = serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("failed to parse observations: {}", e));
        let len = data.len();
        println!("Data source created: examples/data/obs.json");
        println!("Largest possible proposal: {}", max_candidates);
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
                println!("consumed all data");
                return None;
            }
            let data = &self.data[self.pos];
            let sv = SV::from_str(&data.sv).expect("test data is not valid!");

            if data.epoch != self.pending_epoch {
                println!("new epoch: {}", data.epoch);

                if !self.pending.is_empty() {
                    // returned snapshot
                    proposal = Some((self.pending_epoch, self.pending.clone()));
                }

                self.pending.clear();
                self.pending.push(Candidate::new(
                    sv,
                    data.epoch,
                    Default::default(),
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
            }
            self.pos += 1;
            if let Some(proposal) = proposal {
                return Some(proposal);
            }
        }
    }
}
