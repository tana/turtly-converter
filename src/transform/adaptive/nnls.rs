use nalgebra::DVector;
use nalgebra_sparse_linalg::CsrMatrix;

/// Solve a non-negative least squares problem (Ax=b s.t. x>=0)
/// It uses projected gradient descent algorithm.
pub fn nnls(
    a_mat: &CsrMatrix<f64>,
    b_vec: &DVector<f64>,
    alpha: f64,
    max_iter: u64,
    stop_norm: f64,
) -> Option<DVector<f64>> {
    let mut x = DVector::zeros(a_mat.ncols());

    for _ in 0..max_iter {
        let grad = 2.0 * a_mat.transpose() * (a_mat * &x - b_vec);
        let new_x = (&x - alpha * grad).sup(&DVector::zeros(a_mat.ncols()));

        if (&new_x - &x).norm() < alpha * stop_norm {
            return Some(new_x)
        }

        x = new_x;
    }

    None
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;
    use nalgebra::{dmatrix, dvector};
    use nalgebra_sparse_linalg::CsrMatrix;

    use crate::transform::adaptive::nnls::nnls;

    #[test]
    fn test_nnls() {
        let a_mat = CsrMatrix::from(&dmatrix![1.0, 2.0; 3.0, 4.0; 5.0, 6.0]);
        let b_vec = dvector![7.0, 8.0, 9.0];

        let solution = nnls(&a_mat, &b_vec, 1e-2, 1000, 1e-5).unwrap();

        assert_abs_diff_eq!(solution[0], 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(solution[1], 1.78571429, epsilon = 1e-5);
    }
}
