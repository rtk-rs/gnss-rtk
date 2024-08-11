use gnss_rtk::prelude::{Epoch, OrbitalState, OrbitalStateProvider, SV};
use serde::Deserialize;
use std::fs::read_to_string;

// Orbit Source Example: we parse a local text file
#[derive(Clone, Debug, Default)]
pub struct Orbits {
    pos: usize,
    len: usize,
    orbits: Vec<OrbitalState>,
}

impl Orbits {
    pub fn new() -> Self {
        let content = read_to_string("examples/data/orbits.json")
            .unwrap_or_else(|e| panic!("failed to read orbit source: {}", e));
        let orbits: Vec<OrbitalState> = serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("failed to parse orbits: {}", e));
        let len = orbits.len();
        println!("Orbit source created: examples/data/orbits.json");
        Self {
            orbits,
            pos: 0,
            len,
        }
    }
}

impl OrbitalStateProvider for Orbits {
    // For each requested "t" and "sv",
    // if we can, we should resolve the SV [OrbitalState].
    // If interpolation is to be used (depending on your apps), you can
    // use the interpolation order that we recommend here, or decide to ignore it.
    // If you're not in position to determine [OrbitalState], simply return None.
    // If None is returned for too long, this [Epoch] will eventually be dropped out,
    // and we will move on to the next
    fn next_at(&mut self, t: Epoch, sv: SV, order: usize) -> Option<OrbitalState> {
        if self.pos < self.len {
            println!("orbit: {}/{}", self.pos + 1, self.len);
            let ret = self.orbits[self.pos];
            self.pos += 1;
            Some(ret)
        } else {
            println!("consumed all orbits");
            None
        }
    }
}
