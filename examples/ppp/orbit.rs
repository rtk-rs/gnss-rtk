use flate2::read::GzDecoder;
use gnss_rtk::prelude::{Epoch, OrbitalState, OrbitalStateProvider, SV};
use serde::Deserialize;
use std::fs::File;
use std::io::Read;
use std::str::FromStr;

#[derive(Clone, Debug, Default, Deserialize)]
struct OrbitalData {
    sv: String,
    epoch: Epoch,
    state: OrbitalState,
}

// Orbit Source Example: we parse a local text file
#[derive(Clone, Debug, Default)]
pub struct Orbits {
    data: Vec<OrbitalData>,
}

impl Orbits {
    pub fn new() -> Self {
        // Data is compressed to reduce storage size
        let mut content = String::new();
        let f = format!(
            "{}/examples/data/orbits.json.gz",
            env!("CARGO_MANIFEST_DIR")
        );
        let fd = File::open(f).unwrap_or_else(|e| panic!("failed to read orbit source: {}", e));
        let mut decoder = GzDecoder::new(fd);
        decoder
            .read_to_string(&mut content)
            .unwrap_or_else(|e| panic!("failed to read orbit source: {}", e));
        let data: Vec<OrbitalData> = serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("failed to parse orbital data: {}", e));
        info!(
            "Orbit source created: examples/data/orbits.json [{}]",
            data.len()
        );
        Self { data }
    }
}

impl OrbitalStateProvider for Orbits {
    // For each requested "t" and "sv"
    // if we can, we should resolve the SV [OrbitalState].
    // If interpolation is to be used (depending on your apps), you can
    // use the interpolation order that we recommend here, or decide to ignore it.
    // If you're not in position to determine [OrbitalState], simply return None.
    // If None is returned for too long, this [Epoch] will eventually be dropped out,
    // and we will move on to the next
    fn next_at(&mut self, t: Epoch, sv: SV, _: usize) -> Option<OrbitalState> {
        if let Some(state) = self
            .data
            .iter()
            .filter_map(|orb| {
                if let Ok(sv_i) = SV::from_str(&orb.sv) {
                    if orb.epoch == t && sv_i == sv {
                        Some(orb.state)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
        {
            Some(state)
        } else {
            error!("{} ({}): not found", t, sv);
            None
        }
    }
}
