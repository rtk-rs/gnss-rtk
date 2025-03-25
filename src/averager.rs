#[derive(Debug, Clone)]
pub struct Averager {
    pub mean: f64,
    pub count: u64,
}

impl Averager {
    /// Builds new Averager
    pub fn new() -> Self {
        Self {
            count: 0,
            mean: 0.0,
        }
    }

    /// Push new value into [Averager]
    pub fn add(&mut self, x: f64) {
        self.count += 1;
        let k = self.count as f64;
        self.mean = x / k + self.mean * (k - 1.0) / k;
    }

    /// Reset [Averager]
    pub fn reset(&mut self) {
        self.count = 0;
        self.mean = 0.0;
    }
}

#[cfg(test)]
mod test {
    use super::Averager;

    #[test]
    fn test_averager() {
        let mut avg = Averager::new();

        for (x_i, mean, sigma) in [(1.0, 1.0, 0.0), (0.5, 0.75, 0.25)] {
            avg.add(x_i);
            assert_eq!(avg.mean, mean);
        }
    }
}
