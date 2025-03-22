mod bancroft;
mod data;
mod phase_range;
mod pseudo_range;

pub mod bias;
mod navi;
mod spp;

pub use data::*;

use log::LevelFilter;
use std::sync::Once;

static INIT: Once = Once::new();

pub fn init_logger() {
    INIT.call_once(|| {
        env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .init();
    });
}
