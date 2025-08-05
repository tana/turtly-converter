use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use na::{vector, Vector3};
use nalgebra::{self as na, DMatrix, DVector};
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
            point.z + bezier3d(&self.coeffs, scaled.x, scaled.y, scaled.z),
        ]
    }

    pub fn apply_inverse(&self, _point: Vector3<f64>) -> Vector3<f64> {
        todo!()
    }

    pub fn jacobian(&self, point: Vector3<f64>) -> f64 {
        let scaled = self.scale.component_mul(&(point - &self.origin));

        1.0 + self.scale.z * bezier3d_dz(&self.coeffs, scaled.x, scaled.y, scaled.z)
    }
}

pub fn fit_adaptive(
    mesh: &Mesh,
    center: &Vector3<f64>,
    order: usize,
    lambda: f64,
) -> AdaptiveTransform {
    let aabb = mesh.calc_aabb();
    let origin = aabb.origin - center;
    let scale = Vector3::from_element(1.0).component_div(&aabb.size);
    let num_coeffs = (order + 1) * (order + 1) * (order + 1);

    let targets = target_normals(mesh, *center);

    // Fill A and b of Ax=b
    let mut a_mat = DMatrix::zeros(3 * targets.len(), num_coeffs);
    let mut b_vec = DVector::zeros(3 * targets.len());
    for (i, (point, normal)) in targets.iter().enumerate() {
        let scaled = scale.component_mul(&(point - &origin));

        // ∂f/∂x = ΣΣΣ w_{i,j,k} s_x b'_{i,n}(s_x x) b_{j,n}(s_y y) b_{k,n}(s_z z)
        let dx_dw = bezier3d_dx_dw(order, scaled.x, scaled.y, scaled.z);
        // ∂f/∂y = ΣΣΣ w_{i,j,k} s_y b_{i,n}(s_x x) b'_{j,n}(s_y y) b_{k,n}(s_z z)
        let dy_dw = bezier3d_dy_dw(order, scaled.x, scaled.y, scaled.z);
        // ∂f/∂z = 1 + ΣΣΣ w_{i,j,k} s_z b_{i,n}(s_x x) b_{j,n}(s_y y) b'_{k,n}(s_z z)
        let dz_dw = bezier3d_dz_dw(order, scaled.x, scaled.y, scaled.z);

        dx_dw
            .iter()
            .flatten()
            .flatten()
            .map(|b| scale.x * b)
            .zip(a_mat.row_mut(3 * i).iter_mut())
            .for_each(|(val, dst)| *dst = val);
        b_vec[3 * i] = normal.x;

        dy_dw
            .iter()
            .flatten()
            .flatten()
            .map(|b| scale.y * b)
            .zip(a_mat.row_mut(3 * i + 1).iter_mut())
            .for_each(|(val, dst)| *dst = val);
        b_vec[3 * i + 1] = normal.y;

        dz_dw
            .iter()
            .flatten()
            .flatten()
            .map(|b| scale.z * b)
            .zip(a_mat.row_mut(3 * i + 2).iter_mut())
            .for_each(|(val, dst)| *dst = val);
        b_vec[3 * i + 2] = normal.z - 1.0;
    }

    let mut gram_mat = a_mat.clone().transpose() * a_mat.clone();
    // Add L2-norm regularization
    for i in 0..num_coeffs {
        gram_mat[(i, i)] += lambda;
    }

    // Solve
    let coeffs = gram_mat
        .qr()
        .solve(&(a_mat.transpose() * b_vec))
        .expect("Singular matrix in least squares");

    let coeffs: Vec<Vec<Vec<_>>> = coeffs
        .as_slice()
        .chunks(order + 1)
        .map(|coeffs_ij| coeffs_ij.into())
        .collect::<Vec<_>>()
        .chunks(order + 1)
        .map(|coeffs_i| coeffs_i.into())
        .collect();

    AdaptiveTransform {
        scale,
        origin,
        coeffs,
    }
}

fn target_normals(mesh: &Mesh, center: Vector3<f64>) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    let mut target = Vec::with_capacity(mesh.triangles.len());

    for tri in mesh.triangles.iter() {
        let [v1, v2, v3] = tri.map(|idx| mesh.vertices[idx]);
        // Center of the triangle
        let tri_center = (v1 + v2 + v3) / 3.0;
        // Normal vector of the triangle
        let tri_normal = (v2 - v1).cross(&(v3 - v2)).normalize();

        if (tri_center - center).z < 0.1 {
            // close to the bed, probably bottom surface
            // TODO: variable threshold
            continue;
        }

        // Overhang angle (positive means overhang)
        let angle = tri_normal.z.acos() - FRAC_PI_2;

        if angle < 0.0 {
            // Ignore non-overhang
            continue;
        }

        let side = Vector3::z().cross(&tri_normal).cross(&Vector3::z());
        if side.norm() < std::f64::EPSILON {
            continue;
        }
        let side = side.normalize();

        let target_angle = angle.clamp(-FRAC_PI_4, FRAC_PI_4);
        let target_normal = target_angle.cos() * Vector3::z() + target_angle.sin() * side;

        target.push((tri_center, target_normal));
    }

    target
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

/// Calculate values of each Bernstein bases $b_{i,n}(x)$ for all $i$ using de Casteljau-like algorithm
fn bernstein_all(order: usize, x: f64) -> Vec<f64> {
    let mut values = vec![1.0; order + 1];

    for n in 1..=order {
        values[n] = x * values[n - 1];
        for i in (1..n).rev() {
            values[i] = x * values[i - 1] + (1.0 - x) * values[i];
        }
        values[0] = (1.0 - x) * values[0];
    }

    values
}

/// Calculate derivatives of each Bernstein bases $b'_{i,n}(x)$
/// using $b'_{i,n}(x) = n(b_{i-1,n-1}(x)-b_{i,n-1}(x))$
fn bernstein_deriv_all(order: usize, x: f64) -> Vec<f64> {
    let low_bases = bernstein_all(order - 1, x);
    let mut bases_i_minus1 = low_bases.clone();
    bases_i_minus1.insert(0, 0.0);
    let mut bases_i = low_bases.clone();
    bases_i.push(0.0);

    bases_i_minus1
        .iter()
        .zip(bases_i.iter())
        .map(|(b_i_minus1, b_i)| (order as f64) * (b_i_minus1 - b_i))
        .collect()
}

fn bezier3d_dx_dw(order: usize, x: f64, y: f64, z: f64) -> Vec<Vec<Vec<f64>>> {
    let bxs = bernstein_deriv_all(order, x);
    let bys = bernstein_all(order, y);
    let bzs = bernstein_all(order, z);

    bxs.iter()
        .map(|bx| {
            bys.iter()
                .map(|by| bzs.iter().map(|bz| bx * by * bz).collect())
                .collect()
        })
        .collect()
}

fn bezier3d_dy_dw(order: usize, x: f64, y: f64, z: f64) -> Vec<Vec<Vec<f64>>> {
    let bxs = bernstein_all(order, x);
    let bys = bernstein_deriv_all(order, y);
    let bzs = bernstein_all(order, z);

    bxs.iter()
        .map(|bx| {
            bys.iter()
                .map(|by| bzs.iter().map(|bz| bx * by * bz).collect())
                .collect()
        })
        .collect()
}

fn bezier3d_dz_dw(order: usize, x: f64, y: f64, z: f64) -> Vec<Vec<Vec<f64>>> {
    let bxs = bernstein_all(order, x);
    let bys = bernstein_all(order, y);
    let bzs = bernstein_deriv_all(order, z);

    bxs.iter()
        .map(|bx| {
            bys.iter()
                .map(|by| bzs.iter().map(|bz| bx * by * bz).collect())
                .collect()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use crate::transform::adaptive::{bernstein_all, bezier, bezier3d};

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

    #[test]
    fn test_bernstein_all() {
        let div = 10;
        let order = 4;

        for j in 0..div {
            let x = j as f64 / div as f64;
            let values = bernstein_all(order, x);
            assert_eq!(values.len(), order + 1);

            for (i, b) in values.iter().enumerate() {
                approx::assert_relative_eq!(
                    *b,
                    bernstein_direct(i as i32, order as i32, x),
                    max_relative = 0.1,
                )
            }
        }
    }

    fn bezier_direct(coeffs: &[f64], x: f64) -> f64 {
        let mut sum = 0.0;
        for i in 0..coeffs.len() {
            sum += coeffs[i] * bernstein_direct(i as i32, coeffs.len() as i32 - 1, x)
        }

        sum
    }

    fn bezier3d_direct(coeffs: &[Vec<Vec<f64>>], x: f64, y: f64, z: f64) -> f64 {
        let mut sum = 0.0;
        for i in 0..coeffs.len() {
            for j in 0..coeffs[i].len() {
                for k in 0..coeffs[i][j].len() {
                    sum += coeffs[i][j][k]
                        * bernstein_direct(i as i32, coeffs.len() as i32 - 1, x)
                        * bernstein_direct(j as i32, coeffs[i].len() as i32 - 1, y)
                        * bernstein_direct(k as i32, coeffs[i][j].len() as i32 - 1, z)
                }
            }
        }

        sum
    }

    fn bernstein_direct(i: i32, n: i32, x: f64) -> f64 {
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
