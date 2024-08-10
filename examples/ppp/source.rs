use gnss_rtk::prelude::{Candidate, Epoch, Observation, SV};
use serde::Deserialize;
use std::fs::read_to_string;

#[derive(Clone, Default, Deserialize)]
pub struct ObservationData {
    sv: SV,
    epoch: Epoch,
    observation: Observation,
}

// Data Source Example: we parse a local text file
#[derive(Clone, Default)]
pub struct DataSource {
    pos: usize,
    len: usize,
    epoch: Epoch,
    last_proposal: Epoch,
    pending: Vec<Candidate>,
    data: Vec<ObservationData>,
    // maximal candidates proposed at once.
    // We use this to constraint a given run
    // and verify we still can function, depending on
    // selected mode of navigation.
    max_candidates: usize,
}

impl DataSource {
    pub fn new() -> Self {
        let content = read_to_string("examples/data/obs.json")
            .unwrap_or_else(|e| panic!("failed to read observations source: {}", e));
        let data: Vec<ObservationData> = serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("failed to parse observations: {}", e));
        let len = data.len();
        Self {
            pos: 0,
            len,
            data,
            max_candidates: 16,
            epoch: Default::default(),
            pending: Default::default(),
            last_proposal: Default::default(),
        }
    }
}

impl Iterator for DataSource {
    type Item = (Epoch, Vec<Candidate>);
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.pos < self.len {
                println!("data: {}/{}", self.pos + 1, self.len);
                return None;
            } else {
                println!("consumed all data");
                return None;
            }
            let t = self.data[self.pos].epoch;
            let sv = self.data[self.pos].sv;
            if self.pos == 0 {
                // grab first
                self.epoch = t;
                self.pending.push(Candidate::new(
                    sv,
                    t,
                    Default::default(),
                    None,
                    Default::default(),
                    Default::default(),
                    Default::default(),
                ));
            } else {
                self.last_proposal = self.epoch;
                let pending_len = self.pending.len();
                if t > self.epoch || pending_len == self.max_candidates {
                    let proposal = self.pending.clone();
                    self.epoch = t;
                    self.pending.clear();
                    return Some((self.last_proposal, proposal));
                }
            }
            self.pos += 1;
        }
    }
}
