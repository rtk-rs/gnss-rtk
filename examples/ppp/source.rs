use gnss_rtk::prelude::{Candidate, Epoch, Observation};
use serde::Deserialize;
use std::fs::read_to_string;

// Data Source Example: we parse a local text file
#[derive(Clone, Default)]
pub struct DataSource {
    pos: usize,
    len: usize,
    epoch: Epoch,
    pending: Vec<Candidate>,
    observations: Vec<Observation>,
}

impl DataSource {
    pub fn new() -> Self {
        let content = read_to_string("examples/data/obs.json")
            .unwrap_or_else(|e| panic!("failed to read observations source: {}", e));
        let observations: Vec<Observation> = serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("failed to parse observations: {}", e));
        let len = observations.len();
        Self {
            pos: 0,
            len,
            observations,
            epoch: Default::default(),
            pending: Default::default(),
        }
    }
}

impl Iterator for DataSource {
    type Item = (Epoch, Vec<Candidate>);
    fn next(&mut self) -> Option<Self::Item> {
        if self.pos < self.len {
            println!("data: {}/{}", self.pos, self.len);
            None
        } else {
            println!("consumed all data");
            None
        }
    }
}
