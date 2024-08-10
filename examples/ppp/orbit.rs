use gnss_rtk::prelude::{Epoch, OrbitalState, OrbitalStateProvider, SV};
use serde::Deserialize;
use std::fs::read_to_string;

#[derive(Clone, Debug, Default, Deserialize)]
pub struct Orbits(Vec<OrbitalState>);

impl Orbits {
    pub fn new() -> Self {
        let content = read_to_string("examples/data/orbits.json")
            .unwrap_or_else(|e| panic!("failed to read orbit source: {}", e));
        let orbits: Orbits = serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("failed to parse orbits: {}", e));
        orbits
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
        let (x, y, z) = (0.0_f64, 0.0_f64, 0.0_f64);
        Some(OrbitalState::from_position((x, y, z)))
    }
}
