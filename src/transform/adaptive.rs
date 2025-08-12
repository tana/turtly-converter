use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use na::{vector, Vector3};
use nalgebra::{self as na, stack, DMatrix, DVector};
use nalgebra_sparse_linalg::{iteratives::conjugate_gradient, CsrMatrix};
use serde::{Deserialize, Serialize};

use crate::{
    transform::adaptive::spline::{
        bspline3d, bspline3d_dz, bspline_basis, bspline_basis_deriv, make_knots,
    },
    utils::Mesh,
};

mod spline;

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
    let mut c_mat = DMatrix::<f64>::zeros(num_coeffs_x * num_coeffs_y, num_coeffs);
    let mut d_vec = DVector::zeros(num_coeffs_x * num_coeffs_y);
    for i in 0..num_coeffs_x {
        for j in 0..num_coeffs_y {
            for k in 0..num_coeffs_z {
                // f(x,y,z0) = z0 + ΣΣΣ w_{i,j,k} b_{i,p}(x) b_{j,p}(y) b_{k,p}(z0) = 0
                c_mat[(
                    i * num_coeffs_y + j,
                    (i * num_coeffs_y + j) * num_coeffs_z + k,
                )] = bspline_basis(&knots_z, k, deg, z0);
                d_vec[i * num_coeffs_y + j] = -z0;
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
