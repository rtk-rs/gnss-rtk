use core::f64;

#[derive(Debug, Clone)]
pub struct Averager {
    pub mean: f64,
    pub count: u64,
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
}
