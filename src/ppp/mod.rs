use crate::{
    prelude::{Candidate, Epoch, SV},
    rtk::RTKBase,
};

mod kinematics;
mod static_ppp;

pub use kinematics::PPP;
pub use static_ppp::StaticPPP;

pub(crate) struct NullRTK {}

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
