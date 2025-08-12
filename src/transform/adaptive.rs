use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use na::{vector, Vector3};
use nalgebra::{self as na, stack, DMatrix, DVector};
use nalgebra_sparse_linalg::{iteratives::conjugate_gradient, CsrMatrix};
use serde::{Deserialize, Serialize};

use crate::utils::Mesh;

const RANGE_MARGIN: f64 = 0.1;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdaptiveTransform {
    knots: (Vec<f64>, Vec<f64>, Vec<f64>),
    coeffs: Vec<Vec<Vec<f64>>>,
}

impl AdaptiveTransform {
    pub fn apply(&self, point: Vector3<f64>) -> Vector3<f64> {
        vector![
            point.x,
            point.y,
            point.z + bspline3d(&self.knots, &self.coeffs, point.x, point.y, point.z),
        ]
    }

    pub fn apply_inverse(&self, _point: Vector3<f64>) -> Vector3<f64> {
        todo!()
    }

    pub fn jacobian(&self, point: Vector3<f64>) -> f64 {
        1.0 + bspline3d_dz(&self.knots, &self.coeffs, point.x, point.y, point.z)
    }
}

pub fn fit_adaptive(
    mesh: &Mesh,
    center: &Vector3<f64>,
    deg: usize,
    div: usize,
    lambda: f64,
) -> AdaptiveTransform {
    let aabb = mesh.calc_aabb();
    let origin = aabb.origin - center;
    let pos_min = origin - RANGE_MARGIN * aabb.size;
    let pos_max = origin + (1.0 + RANGE_MARGIN) * aabb.size;

    let knots_x = make_knots(pos_min.x, pos_max.x, deg, div);
    let knots_y = make_knots(pos_min.y, pos_max.y, deg, div);
    let knots_z = make_knots(pos_min.z, pos_max.z, deg, div);

    let num_coeffs_x = knots_x.len() - deg - 1;
    let num_coeffs_y = knots_y.len() - deg - 1;
    let num_coeffs_z = knots_z.len() - deg - 1;
    let num_coeffs = num_coeffs_x * num_coeffs_y * num_coeffs_z;

    let targets = target_normals(mesh, *center);

    // Fill A and b of Ax=b
    let mut a_mat = DMatrix::<f64>::zeros(3 * targets.len(), num_coeffs);
    let mut b_vec = DVector::zeros(3 * targets.len());
    for (target_idx, (point, normal)) in targets.iter().enumerate() {
        let point = point - center;

        for i in 0..num_coeffs_x {
            let basis_x = bspline_basis(&knots_x, i, deg, point.x);
            let basis_x_dx = bspline_basis_deriv(&knots_x, i, deg, point.x);

            for j in 0..num_coeffs_y {
                let basis_y = bspline_basis(&knots_y, j, deg, point.y);
                let basis_y_dy = bspline_basis_deriv(&knots_y, j, deg, point.y);

                for k in 0..num_coeffs_z {
                    let basis_z = bspline_basis(&knots_z, k, deg, point.z);
                    let basis_z_dz = bspline_basis_deriv(&knots_z, k, deg, point.z);

                    let col = (i * num_coeffs_y + j) * num_coeffs_z + k;
                    // ∂f/∂x = ΣΣΣ w_{i,j,k} b'_{i,p}(x) b_{j,p}(y) b_{k,p}(z)
                    a_mat[(3 * target_idx, col)] = basis_x_dx * basis_y * basis_z;
                    // ∂f/∂y = ΣΣΣ w_{i,j,k} b_{i,p}(x) b'_{j,p}(y) b_{k,p}(z)
                    a_mat[(3 * target_idx + 1, col)] = basis_x * basis_y_dy * basis_z;
                    // ∂f/∂z = 1 + ΣΣΣ w_{i,j,k} b_{i,p}(x) b_{j,p}(y) b'_{k,p}(z)
                    a_mat[(3 * target_idx + 2, col)] = basis_x * basis_y * basis_z_dz;
                }
            }
        }

        b_vec[3 * target_idx] = normal.x;
        b_vec[3 * target_idx + 1] = normal.y;
        b_vec[3 * target_idx + 2] = normal.z - 1.0;
    }

    // Fill C and d of constraint equation Cx=d
    let z0 = origin.z;
    let mut c_mat = DMatrix::<f64>::zeros(num_coeffs_x * num_coeffs_y * num_coeffs_z, num_coeffs);
    let mut d_vec = DVector::zeros(num_coeffs_x * num_coeffs_y * num_coeffs_z);
    for i in 0..num_coeffs_x {
        for j in 0..num_coeffs_y {
            for k in 0..num_coeffs_z {
                // f(x,y,z0) = z0 + ΣΣΣ w_{i,j,k} b_{i,p}(x) b_{j,p}(y) b_{k,p}(z0) = 0
                c_mat[(
                    (i * num_coeffs_y + j) * num_coeffs_z + k,
                    (i * num_coeffs_y + j) * num_coeffs_z + k,
                )] = bspline_basis(&knots_z, k, deg, z0);
                d_vec[(i * num_coeffs_y + j) * num_coeffs_z + k] = -z0;
            }
        }
    }

    let mut gram_mat = a_mat.transpose() * &a_mat;
    // Add L2-norm regularization
    for i in 0..num_coeffs {
        gram_mat[(i, i)] += lambda;
    }

    let solve_mat = stack![
        gram_mat, c_mat.transpose();
        c_mat, 0;
    ];

    let solve_mat = CsrMatrix::from(&solve_mat);
    let solve_vec = stack![a_mat.transpose() * b_vec; d_vec];

    // Solve
    let coeffs = conjugate_gradient::solve(&solve_mat, &solve_vec, 5000, 1e-5).expect("CG failed");
    // Remove Lagrange multipliers
    let coeffs = coeffs.rows(0, num_coeffs);

    let coeffs: Vec<Vec<Vec<_>>> = coeffs
        .as_slice()
        .chunks(num_coeffs_z)
        .map(|coeffs_ij| coeffs_ij.into())
        .collect::<Vec<_>>()
        .chunks(num_coeffs_y)
        .map(|coeffs_i| coeffs_i.into())
        .collect();

    AdaptiveTransform {
        knots: (knots_x, knots_y, knots_z),
        coeffs,
    }
}

fn target_normals(mesh: &Mesh, center: Vector3<f64>) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    let mut target = Vec::with_capacity(mesh.triangles.len());

    // Vertex normals
    let mut vert_normals = vec![Vector3::zeros(); mesh.vertices.len()];
    // Number of triangles containing the vertex
    let mut vert_num_tri = vec![0; mesh.vertices.len()];
    // Calculate vertex normals from triangles
    for tri in mesh.triangles.iter() {
        let [v1, v2, v3] = tri.map(|idx| mesh.vertices[idx]);
        // Normal vector of the triangle
        let tri_normal = (v2 - v1).cross(&(v3 - v2)).normalize();

        // Vertex normal is the average of the normals of all triangles containing the vertex
        for &v_idx in tri {
            vert_normals[v_idx] = (vert_num_tri[v_idx] as f64 * vert_normals[v_idx] + tri_normal)
                / (vert_num_tri[v_idx] + 1) as f64;
            vert_num_tri[v_idx] += 1;
        }
    }

    for (pos, normal) in mesh.vertices.iter().zip(vert_normals.iter()) {
        if (pos - center).z < 0.1 {
            // close to the bed, probably bottom surface
            // TODO: variable threshold
            continue;
        }

        // Overhang angle (positive means overhang)
        let angle = normal.z.acos() - FRAC_PI_2;

        if angle < 0.0 {
            // Ignore non-overhang
            continue;
        }

        let side = Vector3::z().cross(&normal).cross(&Vector3::z());
        if side.norm() < std::f64::EPSILON {
            continue;
        }
        let side = side.normalize();

        let target_angle = angle.clamp(-FRAC_PI_4, FRAC_PI_4);
        let target_normal = target_angle.cos() * Vector3::z() + target_angle.sin() * side;

        target.push((*pos, target_normal));
    }

    target
}

/// Calculate value of a B-spline $f(x)=\sum_{i=0}^{m-p-1} w_i B_{i,p}(x)$ using de Boor-Cox algorithm
/// Reference: https://en.wikipedia.org/w/index.php?title=De_Boor%27s_algorithm&oldid=1304012252
fn bspline(knots: &[f64], coeffs: &[f64], x: f64) -> f64 {
    let deg = knots.len() - coeffs.len() - 1;

    // Find segment x belongs to
    let k = knots
        .iter()
        .position(|knot| *knot > x)
        .expect("x out of bound")
        - 1;

    let mut coeffs = coeffs.to_vec();
    for i in 1..=deg {
        for j in ((k - deg + i)..=k).rev() {
            let ratio = (x - knots[j]) / (knots[j + 1 + deg - i] - knots[j]);
            coeffs[j] = (1.0 - ratio) * coeffs[j - 1] + ratio * coeffs[j];
        }
    }

    coeffs[k]
}

/// Calculate derivative of a B-spline through converting it into a B-spline of lower degree
/// Reference:
///     https://en.wikipedia.org/w/index.php?title=B-spline&oldid=1303386230#Derivative_expressions
///     https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
fn bspline_deriv(knots: &[f64], coeffs: &[f64], x: f64) -> f64 {
    let deg = knots.len() - coeffs.len() - 1;
    let new_coeffs: Vec<_> = (0..(coeffs.len() - 1))
        .map(|i| deg as f64 * (coeffs[i + 1] - coeffs[i]) / (knots[i + deg] - knots[i]))
        .collect();

    bspline(&knots[1..knots.len() - 2], &new_coeffs, x)
}

fn bspline3d(
    knots: &(Vec<f64>, Vec<f64>, Vec<f64>),
    coeffs: &[Vec<Vec<f64>>],
    x: f64,
    y: f64,
    z: f64,
) -> f64 {
    let (knots_x, knots_y, knots_z) = knots;

    let value_i: Vec<_> = coeffs
        .iter()
        .map(|coeffs_i| {
            let value_ij: Vec<_> = coeffs_i
                .iter()
                .map(|coeffs_ij| bspline(knots_z, coeffs_ij, z))
                .collect();
            bspline(knots_y, &value_ij, y)
        })
        .collect();
    bspline(knots_x, &value_i, x)
}

fn bspline3d_dz(
    knots: &(Vec<f64>, Vec<f64>, Vec<f64>),
    coeffs: &[Vec<Vec<f64>>],
    x: f64,
    y: f64,
    z: f64,
) -> f64 {
    let (knots_x, knots_y, knots_z) = knots;

    let value_i: Vec<_> = coeffs
        .iter()
        .map(|coeffs_i| {
            let value_ij: Vec<_> = coeffs_i
                .iter()
                .map(|coeffs_ij| bspline_deriv(knots_z, coeffs_ij, z))
                .collect();
            bspline(knots_y, &value_ij, y)
        })
        .collect();
    bspline(knots_x, &value_i, x)
}

fn make_knots(min: f64, max: f64, deg: usize, div: usize) -> Vec<f64> {
    std::iter::repeat_n(min, deg)
        .chain((0..=div).map(|i| min + (max - min) * i as f64 / div as f64))
        .chain(std::iter::repeat_n(max, deg))
        .collect()
}

fn bspline_basis(knots: &[f64], i: usize, deg: usize, x: f64) -> f64 {
    if deg == 0 {
        if knots[i] <= x && x < knots[i + 1] {
            1.0
        } else {
            0.0
        }
    } else {
        // Avoid division by zero
        // Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.BSpline.html
        let a = if knots[i + deg] == knots[i] {
            0.0
        } else {
            (x - knots[i]) / (knots[i + deg] - knots[i])
        };
        let b = if knots[i + deg + 1] == knots[i + 1] {
            0.0
        } else {
            (knots[i + deg + 1] - x) / (knots[i + deg + 1] - knots[i + 1])
        };
        a * bspline_basis(knots, i, deg - 1, x) + b * bspline_basis(knots, i + 1, deg - 1, x)
    }
}

fn bspline_basis_deriv(knots: &[f64], i: usize, deg: usize, x: f64) -> f64 {
    // deg as f64
    //     * (bspline_basis(knots, i, deg - 1, x) / (knots[i + deg] - knots[i])
    //         - bspline_basis(knots, i + 1, deg - 1, x) / (knots[i + deg + 1] - knots[i + 1]))
    // FIXME:
    let dx = 1e-10;
    (bspline_basis(knots, i, deg, x + dx) - bspline_basis(knots, i, deg, x)) / dx
}

#[cfg(test)]
mod tests {
    use crate::transform::adaptive::{
        bspline, bspline3d, bspline_basis, bspline_basis_deriv, bspline_deriv, make_knots,
    };

    #[test]
    fn test_bspline() {
        let div = 10;
        let coeffs = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        ];
        let knots = make_knots(0.0, 1.0, 2, 10);

        for i in 0..div {
            let x = i as f64 / div as f64;
            approx::assert_relative_eq!(
                bspline(&knots, &coeffs, x),
                bspline_direct(&knots, &coeffs, x),
                max_relative = 0.1,
            )
        }
    }

    #[test]
    fn test_bspline3d() {
        let div = 10;
        let coeffs: Vec<Vec<Vec<f64>>> = (1..=12)
            .map(|i| {
                (1..=12)
                    .map(|j| (1..=12).map(|k| (i * j * k) as f64).collect())
                    .collect()
            })
            .collect();
        let knots1d = make_knots(0.0, 1.0, 2, 10);
        let knots = (knots1d.clone(), knots1d.clone(), knots1d.clone());

        for i in 0..div {
            let x = i as f64 / div as f64;
            for j in 0..div {
                let y = j as f64 / div as f64;
                for k in 0..div {
                    let z = k as f64 / div as f64;
                    approx::assert_relative_eq!(
                        bspline3d(&knots, &coeffs, x, y, z),
                        bspline3d_direct(&knots, &coeffs, x, y, z),
                        max_relative = 0.1,
                    )
                }
            }
        }
    }

    #[test]
    fn test_bspline_deriv() {
        let dx = 1e-7;
        let div = 10;
        let coeffs = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        ];
        let knots = make_knots(0.0, 1.0, 2, 10);

        for i in 0..div {
            let x = i as f64 / div as f64;
            approx::assert_relative_eq!(
                bspline_deriv(&knots, &coeffs, x),
                (bspline(&knots, &coeffs, x + dx) - bspline(&knots, &coeffs, x)) / dx,
                max_relative = 0.1,
            )
        }
    }

    #[test]
    fn test_bspline_basis_deriv() {
        let dx = 1e-7;
        let div = 10;
        let knots = make_knots(0.0, 1.0, 2, 10);

        for i in 0..div {
            let x = i as f64 / div as f64;
            // It needs more lax comparison than others
            approx::assert_abs_diff_eq!(
                bspline_basis_deriv(&knots, 5, 2, x),
                (bspline_basis(&knots, 5, 2, x + dx) - bspline_basis(&knots, 5, 2, x)) / dx,
                epsilon = 1e-4
            )
        }
    }

    #[test]
    fn test_bspline_deriv2() {
        let div = 10;
        let coeffs = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        ];
        let knots = make_knots(0.0, 1.0, 2, 10);

        for i in 0..div {
            let x = i as f64 / div as f64;
            approx::assert_relative_eq!(
                bspline_deriv(&knots, &coeffs, x),
                coeffs
                    .iter()
                    .enumerate()
                    .map(|(i, w)| w * bspline_basis_deriv(&knots, i, 2, x))
                    .sum(),
                max_relative = 0.1,
            )
        }
    }

    fn bspline_direct(knots: &[f64], coeffs: &[f64], x: f64) -> f64 {
        let deg = knots.len() - coeffs.len() - 1;

        let mut sum = 0.0;
        for i in 0..coeffs.len() {
            sum += coeffs[i] * bspline_basis(&knots, i, deg, x);
        }

        sum
    }

    fn bspline3d_direct(
        knots: &(Vec<f64>, Vec<f64>, Vec<f64>),
        coeffs: &[Vec<Vec<f64>>],
        x: f64,
        y: f64,
        z: f64,
    ) -> f64 {
        let (knots_x, knots_y, knots_z) = knots;
        let deg_x = knots_x.len() - coeffs.len() - 1;
        let deg_y = knots_x.len() - coeffs.len() - 1;
        let deg_z = knots_x.len() - coeffs.len() - 1;

        let mut sum = 0.0;
        for i in 0..coeffs.len() {
            for j in 0..coeffs[i].len() {
                for k in 0..coeffs[i][j].len() {
                    sum += coeffs[i][j][k]
                        * bspline_basis(knots_x, i, deg_x, x)
                        * bspline_basis(knots_y, j, deg_y, y)
                        * bspline_basis(knots_z, k, deg_z, z)
                }
            }
        }

        sum
    }
}
