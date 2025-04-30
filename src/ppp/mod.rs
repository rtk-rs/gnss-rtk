use crate::{
    prelude::{Candidate, Epoch, SV},
    rtk::RTKBase,
};

mod dynamic_ppp;
mod static_ppp;

pub use dynamic_ppp::PPP;
pub use static_ppp::StaticPPP;

struct NullRTK {}

impl RTKBase for NullRTK {
    fn name(&self) -> String {
        "UNUSED".to_string()
    }

    fn observe(&mut self, _: Epoch, _: SV) -> Option<Candidate> {
        None
    }

    fn reference_position_ecef_m(&self, _: Epoch) -> Option<(f64, f64, f64)> {
        None
    }
}
