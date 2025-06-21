use std::fmt::Debug;

/// [TestNumber] used in tests to generate number,
/// ranged number or random numbers.
#[derive(Debug, Clone)]
pub enum TestNumber<T: Clone + Debug> {
    /// Pre-determined number
    Fixed(T),
    // /// Random number
    // Random,

    // /// Random ranged number
    // RangeRandom(RG),
}

impl<T: Clone + Debug> TestNumber<T> {
    pub fn fixed(value: T) -> Self {
        Self::Fixed(value)
    }

    // pub fn nominal(value: T) -> Self {
    //     Self::Nominal(value)
    // }

    // pub fn random() -> Self {
    //     Self::Randomized
    // }

    // pub fn range_random<R: SampleRange<T>>(range: R) -> Self {
    //     Self::RangeRandom(range)
    // }

    pub fn value(&self) -> T {
        match self {
            Self::Fixed(value) => value.clone(),
            // Self::Randomized => rng.random(),
            // Self::Nominal(value) => {
            //     let variations = rng.random();
            //     value + variations
            // },
            // Self::RangeRandomized(range) => {
            //     let random = rng.random_range::<T>(range);
            //     random
            // },
        }
    }
}

#[cfg(test)]
mod test {
    use super::TestNumber;

    use crate::tests::init_logger;

    #[test]
    fn testnumber() {
        init_logger();

        let number = TestNumber::fixed(0.13);
        assert_eq!(number.value(), 0.13);

        // let number = TestNumber::ranged_random(0.15..0.20);

        // for _ in 0..100 {
        //     let value = number.value();
        //     assert!(value > 0.14, "generated value is not within range!");
        //     assert!(value < 0.21, "generated value is not within range!");
        // }

        // let number = TestNumber::random();

        // for _ in 0..100 {
        //     debug!("rng: generated {}", number.value());
        // }
    }
}
