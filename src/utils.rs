/// Computes factorial n!
pub fn factorial(n: usize) -> usize {
    let mut prod = 1;
    for i in 2..=n {
        prod *= i;
    }
    prod
}

/// Returns number of permutations p amongst n (p < n)
pub fn permutations(p: usize, n: usize) -> usize {
    factorial(n) / factorial(p) / factorial(n - p)
}

#[cfg(test)]
mod test {
    use super::{factorial, permutations};
    #[test]
    fn test_factorial() {
        assert_eq!(factorial(3), 1*2*3);
        assert_eq!(factorial(5), 1*2*3*4*5);
        assert_eq!(factorial(10), 1*2*3*4*5*6*7*8*9*10); 
    }
    #[test]
    fn test_permutations() {
        assert_eq!(permutations(2, 3), 3);
        assert_eq!(permutations(2, 5), 10);
        assert_eq!(permutations(4, 5), 120 / 24);
    }
}
