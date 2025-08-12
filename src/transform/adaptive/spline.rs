/// Calculate value of a B-spline $f(x)=\sum_{i=0}^{m-p-1} w_i B_{i,p}(x)$ using de Boor-Cox algorithm
/// Reference: https://en.wikipedia.org/w/index.php?title=De_Boor%27s_algorithm&oldid=1304012252
pub fn bspline(knots: &[f64], coeffs: &[f64], x: f64) -> f64 {
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
pub fn bspline_deriv(knots: &[f64], coeffs: &[f64], x: f64) -> f64 {
    let deg = knots.len() - coeffs.len() - 1;
    let new_coeffs: Vec<_> = (0..(coeffs.len() - 1))
        .map(|i| deg as f64 * (coeffs[i + 1] - coeffs[i]) / (knots[i + deg] - knots[i]))
        .collect();

    bspline(&knots[1..knots.len() - 2], &new_coeffs, x)
}

pub fn bspline3d(
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

pub fn bspline3d_dz(
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

pub fn make_knots(min: f64, max: f64, deg: usize, div: usize) -> Vec<f64> {
    std::iter::repeat_n(min, deg)
        .chain((0..=div).map(|i| min + (max - min) * i as f64 / div as f64))
        .chain(std::iter::repeat_n(max, deg))
        .collect()
}

pub fn bspline_basis(knots: &[f64], i: usize, deg: usize, x: f64) -> f64 {
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

pub fn bspline_basis_deriv(knots: &[f64], i: usize, deg: usize, x: f64) -> f64 {
    // deg as f64
    //     * (bspline_basis(knots, i, deg - 1, x) / (knots[i + deg] - knots[i])
    //         - bspline_basis(knots, i + 1, deg - 1, x) / (knots[i + deg + 1] - knots[i + 1]))
    // FIXME:
    let dx = 1e-10;
    (bspline_basis(knots, i, deg, x + dx) - bspline_basis(knots, i, deg, x)) / dx
}

#[cfg(test)]
mod tests {
    use crate::transform::adaptive::spline::{
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
