use gnss_rtk::prelude::{Observation};
use std::fs::read_to_string;
use serde::Deserialize;

// Data Source Example: we parse a local text file
#[derive(Clone, Debug, Default)]
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
            .unwrap_or_else(|e| panic!("failed to read observations source: {}", e);
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
