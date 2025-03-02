use crate::prelude::Epoch;
use rinex::prelude::Rinex;
use sp3::prelude::SP3;

pub struct EpochDataSet<'a> {
    iter: Box<dyn Iterator<Item = Epoch> + 'a>,
}

impl<'a> Iterator for EpochDataSet<'a> {
    type Item = Epoch;
    /// Returns next [Epoch] in the dataset
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next()
    }
}

impl<'a> EpochDataSet<'a> {
    /// Builds new test [EpochDataSet] from a [Rinex].
    pub fn from_rinex(rinex: &'a Rinex) -> Self {
        Self {
            iter: rinex.epoch_iter(),
        }
    }

    /// Builds new test [EpochDataSet] from [SP3] data.
    pub fn from_sp3(sp3: &'a SP3) -> Self {
        Self {
            iter: Box::new(sp3.epochs_iter()),
        }
    }

    /// Builds new test [EpochDataSet] from RINEX Iterator
    pub fn from_iter(iter: Box<dyn Iterator<Item = Epoch>>) -> Self {
        Self { iter }
    }
}
