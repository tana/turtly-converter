use na::{vector, Vector3};
use nalgebra as na;
use serde::{Deserialize, Serialize};

use crate::utils::Mesh;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdaptiveTransform {
    scale: Vector3<f64>,
    origin: Vector3<f64>,
    coeffs: Vec<Vec<Vec<f64>>>,
}

impl AdaptiveTransform {
    pub fn apply(&self, point: Vector3<f64>) -> Vector3<f64> {
        let scaled = self.scale.component_mul(&(point - &self.origin));

        vector![
            point.x,
            point.y,
            bezier3d(&self.coeffs, scaled.x, scaled.y, scaled.z),
        ]
    }

    pub fn apply_inverse(&self, point: Vector3<f64>) -> Vector3<f64> {
        todo!()
    }

    pub fn jacobian(&self, point: Vector3<f64>) -> f64 {
        let scaled = self.scale.component_mul(&(point - &self.origin));

        self.scale.z * bezier3d_dz(&self.coeffs, scaled.x, scaled.y, scaled.z)
    }
}

pub fn fit_adaptive(mesh: &Mesh, center: &Vector3<f64>) -> AdaptiveTransform {
    let aabb = mesh.calc_aabb();

    let coeffs = vec![
        vec![
            vec![1.0, 2.0, 3.0],
            vec![3.0, 2.0, 1.0],
            vec![1.0, 2.0, 3.0],
        ],
        vec![
            vec![3.0, 2.0, 1.0],
            vec![1.0, 2.0, 3.0],
            vec![3.0, 2.0, 1.0],
        ],
        vec![
            vec![1.0, 2.0, 3.0],
            vec![3.0, 2.0, 1.0],
            vec![1.0, 2.0, 3.0],
        ],
    ];

    AdaptiveTransform {
        scale: Vector3::from_element(1.0).component_div(&aabb.size),
        origin: aabb.origin - center,
        coeffs,
    }
}

/// Calculate value of $f(x)=\sum_{i=0}^n w_i b_{i,n}(x)$ where $b_{i,n}(x)$ is a Bernstein basis function
/// (de Casteljau's algorithm)
fn bezier(coeffs: &[f64], x: f64) -> f64 {
    assert!(0.0 <= x && x <= 1.0);

    let mut coeffs: Vec<f64> = coeffs.into();
    for i in 0..coeffs.len() {
        for j in 0..(coeffs.len() - i - 1) {
            coeffs[j] = (1.0 - x) * coeffs[j] + x * coeffs[j + 1];
        }
    }

    coeffs[0]
}

fn bezier3d(coeffs: &[Vec<Vec<f64>>], x: f64, y: f64, z: f64) -> f64 {
    let bezier_i: Vec<_> = coeffs
        .iter()
        .map(|coeffs_i| {
            let bezier_ij: Vec<_> = coeffs_i
                .iter()
                .map(|coeffs_ij| bezier(coeffs_ij, z))
                .collect();
            bezier(&bezier_ij, y)
        })
        .collect();
    bezier(&bezier_i, x)
}

fn bezier3d_dx(coeffs: &[Vec<Vec<f64>>], x: f64, y: f64, z: f64) -> f64 {
    let mut new_coeffs = Vec::new();
    for i in 0..(coeffs.len() - 1) {
        new_coeffs.push(Vec::new());
        for j in 0..coeffs[i].len() {
            new_coeffs[i].push(Vec::new());
            for k in 0..coeffs[i][j].len() {
                new_coeffs[i][j].push(coeffs[i + 1][j][k] - coeffs[i][j][k]);
            }
        }
    }

    (coeffs.len() as f64) * bezier3d(&new_coeffs, x, y, z)
}

fn bezier3d_dy(coeffs: &[Vec<Vec<f64>>], x: f64, y: f64, z: f64) -> f64 {
    let mut new_coeffs = Vec::new();
    for i in 0..coeffs.len() {
        new_coeffs.push(Vec::new());
        for j in 0..(coeffs[i].len() - 1) {
            new_coeffs[i].push(Vec::new());
            for k in 0..coeffs[i][j].len() {
                new_coeffs[i][j].push(coeffs[i][j + 1][k] - coeffs[i][j][k]);
            }
        }
    }

    (coeffs[0].len() as f64) * bezier3d(&new_coeffs, x, y, z)
}

fn bezier3d_dz(coeffs: &[Vec<Vec<f64>>], x: f64, y: f64, z: f64) -> f64 {
    let mut new_coeffs = Vec::new();
    for i in 0..coeffs.len() {
        new_coeffs.push(Vec::new());
        for j in 0..coeffs[i].len() {
            new_coeffs[i].push(Vec::new());
            for k in 0..(coeffs[i][j].len() - 1) {
                new_coeffs[i][j].push(coeffs[i][j][k + 1] - coeffs[i][j][k]);
            }
        }
    }

    (coeffs[0][0].len() as f64) * bezier3d(&new_coeffs, x, y, z)
}

#[cfg(test)]
mod tests {
    use crate::transform::adaptive::{bezier, bezier3d};

    #[test]
    fn test_bezier() {
        let div = 10;
        let coeffs = [1.0, 2.0, 3.0];

        for i in 0..div {
            let x = i as f64 / div as f64;
            approx::assert_relative_eq!(
                bezier(&coeffs, x),
                bezier_direct(&coeffs, x),
                max_relative = 0.1,
            )
        }
    }

    #[test]
    fn test_bezier3d() {
        let div = 10;
        let coeffs = vec![
            vec![
                vec![1.0, 2.0, 3.0],
                vec![3.0, 2.0, 1.0],
                vec![1.0, 2.0, 3.0],
            ],
            vec![
                vec![3.0, 2.0, 1.0],
                vec![1.0, 2.0, 3.0],
                vec![3.0, 2.0, 1.0],
            ],
            vec![
                vec![1.0, 2.0, 3.0],
                vec![3.0, 2.0, 1.0],
                vec![1.0, 2.0, 3.0],
            ],
        ];

        for i in 0..div {
            let x = i as f64 / div as f64;
            for j in 0..div {
                let y = j as f64 / div as f64;
                for k in 0..div {
                    let z = k as f64 / div as f64;
                    approx::assert_relative_eq!(
                        bezier3d(&coeffs, x, y, z),
                        bezier3d_direct(&coeffs, x, y, z),
                        max_relative = 0.1,
                    )
                }
            }
        }
    }

    fn bezier_direct(coeffs: &[f64], x: f64) -> f64 {
        let mut sum = 0.0;
        for i in 0..coeffs.len() {
            sum += coeffs[i] * bernstein(i as i32, coeffs.len() as i32 - 1, x)
        }

        sum
    }

    fn bezier3d_direct(coeffs: &[Vec<Vec<f64>>], x: f64, y: f64, z: f64) -> f64 {
        let mut sum = 0.0;
        for i in 0..coeffs.len() {
            for j in 0..coeffs[i].len() {
                for k in 0..coeffs[i][j].len() {
                    sum += coeffs[i][j][k]
                        * bernstein(i as i32, coeffs.len() as i32 - 1, x)
                        * bernstein(j as i32, coeffs[i].len() as i32 - 1, y)
                        * bernstein(k as i32, coeffs[i][j].len() as i32 - 1, z)
                }
            }
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
