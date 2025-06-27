use core::f64;

#[derive(Debug, Clone)]
pub struct Averager {
    /// Estimated mean
    pub mean: f64,

    /// Number of averages
    pub count: u64,

    /// internal value
    alpha: f64,
}

impl Averager {
    /// Builds new Averager
    pub fn new(alpha: f64) -> Self {
        Self {
            count: 0,
            mean: 0.0,
            alpha,
        }
    }

    /// Push new value into [Averager]
    pub fn add(&mut self, x: f64) {
        self.count += 1;
        if self.count == 1 {
            self.mean = x;
        } else {
            self.mean = self.alpha * x + (1.0 - self.alpha) * self.mean;
        }
    }

    /// Reset [Averager]
    pub fn reset(&mut self) {
        self.count = 0;
        self.mean = 0.0;
    }

    /// Returns sigma assuming averaged value is gaussian
    pub fn sigma(&self) -> f64 {
        1.0 / (self.count as f64).sqrt()
    }
}

#[cfg(test)]
mod test {
    use super::Averager;

    #[test]
    fn test_averager() {
        let alpha = 0.5;
        let mut avg = Averager::new(alpha);

        let mut mean = 0.0_f64;

        for i in 0..10 {
            mean = alpha * (i as f64) + (1.0 - alpha) * mean;
            avg.add(i as f64);
        }

        assert_eq!(avg.count, 10);
        assert_eq!(avg.mean, mean);
    }
}
