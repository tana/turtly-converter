use na::Vector3;
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct AdaptiveTransform {
    scale: Vector3<f64>,
    origin: Vector3<f64>,
}

impl AdaptiveTransform {
    pub fn apply(&self, point: Vector3<f64>) -> Vector3<f64> {
        todo!()
    }

    pub fn apply_inverse(&self, point: Vector3<f64>) -> Vector3<f64> {
        todo!()
    }

    pub fn jacobian(&self, point: Vector3<f64>) -> f64 {
        todo!()
    }
}

/// Calculate value of $f(x)=\sum_{i=0}^n w_i b_{i,n}(x)$ where $b_{i,n}(x)$ is a Bernstein basis function
/// (de Casteljau's algorithm)
fn bezier(coeffs: &[f64], x: f64) -> f64 {
    let mut coeffs: Vec<f64> = coeffs.into();
    for i in 0..coeffs.len() {
        for j in 0..(coeffs.len() - i - 1) {
            coeffs[j] = (1.0 - x) * coeffs[j] + x * coeffs[j + 1];
        }
    }

    coeffs[0]
}

#[cfg(test)]
mod tests {
    use crate::transform::adaptive::bezier;

    #[test]
    fn test_bezier() {
        let div = 10;
        let coeffs = [1.0, 2.0, 3.0];

        for i in 0..div {
            let x = i as f64 / div as f64;
            println!("{} {} {}", x, bezier(&coeffs, x), bezier_direct(&coeffs, x));
            approx::assert_relative_eq!(
                bezier(&coeffs, x),
                bezier_direct(&coeffs, x),
                max_relative = 0.1,
            )
        }
    }

    fn bezier_direct(coeffs: &[f64], x: f64) -> f64 {
        let mut sum = 0.0;
        for i in 0..coeffs.len() {
            sum += coeffs[i] * bernstein(i as i32, coeffs.len() as i32 - 1, x)
        }

        sum
    }

    fn bernstein(i: i32, n: i32, x: f64) -> f64 {
        (binomial(n, i) as f64) * x.powi(i) * (1.0 - x).powi(n - i)
    }

    fn binomial(n: i32, k: i32) -> u64 {
        factorial(n) / factorial(k) / factorial(n - k)
    }

    fn factorial(n: i32) -> u64 {
        if n <= 1 {
            1
        } else {
            let mut result = 1u64;
            for i in 2..=n {
                result *= i as u64;
            }

            result
        }
    }
}