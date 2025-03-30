use crate::{pool::Pool, prelude::RTKBase};

impl Pool {
    pub fn remote_observation_enhancing<R: RTKBase>(&mut self, rtk_base: &mut R) {
        for cd in self.inner.iter_mut() {
            if let Some(observation) = rtk_base.remote_observation(cd.t, cd.sv) {}
        }
    }
}
