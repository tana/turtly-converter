use std::f64::consts::FRAC_PI_2;

use clarabel::{
    algebra::CscMatrix,
    solver::{DefaultSettings, DefaultSolver, IPSolver as _, SupportedConeT::NonnegativeConeT},
};
use na::{vector, Vector3};
use nalgebra::{self as na, DMatrix, DVector};
use serde::{Deserialize, Serialize};

use crate::{
    transform::adaptive::spline::{
        bspline3d, bspline3d_integ_z, bspline_basis, bspline_basis_deriv, bspline_basis_integ,
        make_knots,
    },
    utils::Mesh,
};

mod spline;

const RANGE_MARGIN: f64 = 0.1;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdaptiveTransform {
    knots: (Vec<f64>, Vec<f64>, Vec<f64>),
    coeffs: Vec<Vec<Vec<f64>>>,
    origin: Vector3<f64>,
}

impl AdaptiveTransform {
    pub fn apply(&self, point: Vector3<f64>) -> Vector3<f64> {
        vector![
            point.x,
            point.y,
            point.z + bspline3d_integ_z(&self.knots, &self.coeffs, self.origin.z, point.x, point.y, point.z),
        ]
    }

    pub fn apply_inverse(&self, _point: Vector3<f64>) -> Vector3<f64> {
        todo!()
    }

    pub fn jacobian(&self, point: Vector3<f64>) -> f64 {
        1.0 + bspline3d(&self.knots, &self.coeffs, point.x, point.y, point.z)
    }
}

pub fn fit_adaptive(
    mesh: &Mesh,
    center: &Vector3<f64>,
    deg: usize,
    div: usize,
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
                    let basis_z = bspline_basis_integ(&knots_z, k, deg, origin.z, point.z);
                    let basis_z_dz = bspline_basis(&knots_z, k, deg, point.z);

                    let col = (i * num_coeffs_y + j) * num_coeffs_z + k;
                    // ∂f/∂x = ΣΣΣ w_{i,j,k} b'_{i,p}(x) b_{j,p}(y) (b_{k,p}(z) - b_{k,p}(z0))
                    a_mat[(3 * target_idx, col)] = basis_x_dx * basis_y * basis_z;
                    // ∂f/∂y = ΣΣΣ w_{i,j,k} b_{i,p}(x) b'_{j,p}(y) (b_{k,p}(z) - b_{k,p}(z0))
                    a_mat[(3 * target_idx + 1, col)] = basis_x * basis_y_dy * basis_z;
                    // ∂f/∂z = ΣΣΣ w_{i,j,k} b_{i,p}(x) b_{j,p}(y) b'_{k,p}(z)
                    a_mat[(3 * target_idx + 2, col)] = basis_x * basis_y * basis_z_dz;
                }
            }
        }

        b_vec[3 * target_idx] = normal.x;
        b_vec[3 * target_idx + 1] = normal.y;
        b_vec[3 * target_idx + 2] = normal.z - 1.0;
    }

    // Convert non-negative least squares into quadratic programming
    let p_mat = to_clarabel(&(&a_mat.transpose() * &a_mat));
    let q_vec = (-&a_mat.transpose() * &b_vec).as_slice().to_vec();
    // Initialize Clarabel solver
    let mut solver = DefaultSolver::new(
        &p_mat,
        &q_vec,
        &to_clarabel(&-DMatrix::identity(num_coeffs, num_coeffs)),
        &vec![0.0; num_coeffs],
        &[NonnegativeConeT(num_coeffs)],
        DefaultSettings {
            verbose: false,
            ..Default::default()
        },
    )
    .expect("Solver initialization failed");

    solver.solve();

    let coeffs: Vec<Vec<Vec<_>>> = solver
        .solution
        .x
        .chunks(num_coeffs_z)
        .map(|coeffs_ij| coeffs_ij.into())
        .collect::<Vec<_>>()
        .chunks(num_coeffs_y)
        .map(|coeffs_i| coeffs_i.into())
        .collect();

    AdaptiveTransform {
        knots: (knots_x, knots_y, knots_z),
        coeffs,
        origin,
    }
}

fn target_normals(mesh: &Mesh, _center: Vector3<f64>) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    let mut target = Vec::with_capacity(mesh.triangles.len());

    // Vertex normals
    let vert_normals = mesh.calc_vert_normals();

    for (pos, normal) in mesh.vertices.iter().zip(vert_normals.iter()) {
        // Overhang angle (positive means overhang)
        let angle = normal.z.acos() - FRAC_PI_2;

        let side = Vector3::z().cross(&normal).cross(&Vector3::z());
        if side.norm() < std::f64::EPSILON {
            continue;
        }
        let side = side.normalize();

        let target_normal = angle.cos() * Vector3::z() + angle.sin() * side;

        target.push((*pos, target_normal));
    }

    target
}

fn to_clarabel(mat: &DMatrix<f64>) -> CscMatrix {
    // Row-major
    CscMatrix::from(
        mat.transpose()
            .as_slice()
            .chunks(mat.ncols())
            .collect::<Vec<_>>(),
    )
}
