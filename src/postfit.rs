use nyx::{
    od::filter::kalman::Kf,
    cosmic::State as NyxState,
};

use crate::navigation::State;

impl std::fmt::Display for State {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        Ok(())
    }
}

impl std::fmt::LowerExp for State {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        Ok(())
    }
}

impl NyxState for State {
    type Size : U8;
    type VecLength : U8;

    fn epoch(&self) -> Epoch {
        self.t
    }

    fn set_epoch(&mut self, t: Epoch) {
        self.t = t;
    }

    fn to_vector(&self) -> OVector<f64, Self::VecLength> {
        OVector::new(self.pos_m.0, self.pos_m.1, self.pos_m.2, self.vel_m_s.0, self.vel_m_s.1, self.vel_m_s.2)
    }

    fn orbit(&self) -> Orbit {
        self.to_orbit()
    }
    
    fn unset_stm(&mut self) { todo!("unset_stm") }

    fn set(&mut self, _: Epoch, _: OVector<f64, Self::VecLength>) { todo!("set()") }

}

pub struct PostfitKf {
    kf: Kf<State, U3, U8>,
}
