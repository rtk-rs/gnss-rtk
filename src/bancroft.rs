//! Brancroft solver
use crate::{
    navigation::{Input, Output},
    solver::Error,
};

pub struct Bancroft {}

impl Bancroft {
    pub fn resolve(x: &Input) -> Result<Output, Error> {
        let mut out = Output::default();
        Ok(out)
    }
}

#[cfg(test)]
mod test {}
