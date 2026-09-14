//! Small dense linear algebra, a Levenberg–Marquardt solver for the surface
//! fits, B-spline basis functions and the projection of a point onto the
//! intersection of several surfaces.

use super::surface::Surface;
use crate::geom::fitting::eigen_3x3;
use glam::DVec3;

/// Solves the dense system `a x = b` (Gaussian elimination with partial
/// pivoting). Returns `None` for a (numerically) singular matrix.
pub(super) fn solve_dense(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    let scale = a
        .iter()
        .flat_map(|r| r.iter())
        .fold(0.0f64, |m, v| m.max(v.abs()))
        .max(1e-300);
    for col in 0..n {
        let piv = (col..n).max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))?;
        if a[piv][col].abs() <= scale * 1e-15 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for row in col + 1..n {
            let f = a[row][col] / a[col][col];
            if f == 0.0 {
                continue;
            }
            for k in col..n {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = vec![0.0f64; n];
    for row in (0..n).rev() {
        let mut s = b[row];
        for k in row + 1..n {
            s -= a[row][k] * x[k];
        }
        x[row] = s / a[row][row];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// Minimises `Σ r(x, p)²` over the points with Levenberg–Marquardt and a
/// central-difference Jacobian. Parameters flagged in `frozen` keep their
/// value. Returns the RMS residual at the solution.
pub(super) fn levenberg_marquardt(
    x: &mut [f64],
    frozen: &[bool],
    pts: &[DVec3],
    resid: impl Fn(&[f64], DVec3) -> f64,
    iters: usize,
) -> f64 {
    let n = pts.len().max(1) as f64;
    let cost_of = |x: &[f64]| -> f64 { pts.iter().map(|&p| resid(x, p).powi(2)).sum() };
    let free: Vec<usize> = (0..x.len())
        .filter(|&i| !frozen.get(i).copied().unwrap_or(false))
        .collect();
    let mut cost = cost_of(x);
    if free.is_empty() || !cost.is_finite() {
        return (cost / n).sqrt();
    }
    let m = free.len();
    let mut lambda = 1e-3f64;
    let mut xp = x.to_vec();
    let mut xm = x.to_vec();
    for _ in 0..iters {
        let mut jtj = vec![vec![0.0f64; m]; m];
        let mut jtr = vec![0.0f64; m];
        let steps: Vec<f64> = free.iter().map(|&i| 1e-6 * x[i].abs().max(1.0)).collect();
        let mut row = vec![0.0f64; m];
        for &p in pts {
            let r = resid(x, p);
            for (k, &i) in free.iter().enumerate() {
                xp.copy_from_slice(x);
                xm.copy_from_slice(x);
                xp[i] += steps[k];
                xm[i] -= steps[k];
                row[k] = (resid(&xp, p) - resid(&xm, p)) / (2.0 * steps[k]);
            }
            for a in 0..m {
                jtr[a] += row[a] * r;
                for b in a..m {
                    jtj[a][b] += row[a] * row[b];
                }
            }
        }
        for a in 0..m {
            for b in 0..a {
                jtj[a][b] = jtj[b][a];
            }
        }
        let mut improved = false;
        while lambda < 1e12 {
            let mut aug = jtj.clone();
            for (a, r) in aug.iter_mut().enumerate() {
                r[a] += lambda * jtj[a][a].max(1e-12);
            }
            let rhs: Vec<f64> = jtr.iter().map(|v| -v).collect();
            let Some(delta) = solve_dense(aug, rhs) else {
                lambda *= 10.0;
                continue;
            };
            let mut trial = x.to_vec();
            for (k, &i) in free.iter().enumerate() {
                trial[i] += delta[k];
            }
            let c = cost_of(&trial);
            if c.is_finite() && c < cost {
                let gain = cost - c;
                x.copy_from_slice(&trial);
                cost = c;
                lambda = (lambda * 0.3).max(1e-12);
                improved = gain > cost * 1e-12 + 1e-300;
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    (cost / n).sqrt()
}

/// Knot span index `s` with `knots[s] <= t < knots[s + 1]` for a clamped
/// knot vector (the last span for `t` at the end of the domain).
pub(super) fn find_span(knots: &[f64], n_ctrl: usize, deg: usize, t: f64) -> usize {
    let n = n_ctrl - 1;
    if t >= knots[n + 1] {
        return n;
    }
    if t <= knots[deg] {
        return deg;
    }
    let (mut lo, mut hi) = (deg, n + 1);
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// Non-zero basis functions `N[span - deg ..= span]` and their first
/// derivatives at `t` (The NURBS Book, A2.2 / A2.3).
pub(super) fn basis_ders(knots: &[f64], span: usize, deg: usize, t: f64) -> (Vec<f64>, Vec<f64>) {
    let p = deg;
    let mut ndu = vec![vec![0.0f64; p + 1]; p + 1];
    let mut left = vec![0.0f64; p + 1];
    let mut right = vec![0.0f64; p + 1];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r] != 0.0 {
                ndu[r][j - 1] / ndu[j][r]
            } else {
                0.0
            };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let n: Vec<f64> = (0..=p).map(|j| ndu[j][p]).collect();
    let mut d = vec![0.0f64; p + 1];
    if p >= 1 {
        for r in 0..=p {
            let mut v = 0.0;
            if r >= 1 && ndu[p][r - 1] != 0.0 {
                v += ndu[r - 1][p - 1] / ndu[p][r - 1];
            }
            if r < p && ndu[p][r] != 0.0 {
                v -= ndu[r][p - 1] / ndu[p][r];
            }
            d[r] = v * p as f64;
        }
    }
    (n, d)
}

/// Directions whose `JᵀJ` eigenvalue is below this fraction of the largest
/// are left alone by [`project_onto`] (two unit normals less than ~3.6°
/// apart, or three almost coplanar normals).
const WEAK_DIRECTION: f64 = 1e-3;

/// Moves `p0` onto the common intersection of the surfaces. Gauss–Newton on
/// the signed distances with the pseudo-inverse of `JᵀJ`, so the result
/// stays close to `p0` where the intersection is a curve or a surface, and
/// along any direction the surfaces barely constrain: at a corner where a
/// face split by a shallow crease meets a third face, the exact common point
/// can lie far away, and the corner stays next to the mesh with a small gap
/// instead. Returns the point and the largest remaining distance to any of
/// the surfaces.
pub(super) fn project_onto(surfs: &[&Surface], p0: DVec3, max_step: f64) -> (DVec3, f64) {
    let mut x = p0;
    if surfs.is_empty() {
        return (x, 0.0);
    }
    let residual = |x: DVec3| {
        surfs
            .iter()
            .map(|s| s.signed_distance(x).abs())
            .fold(0.0f64, f64::max)
    };
    for _ in 0..40 {
        let f: Vec<f64> = surfs.iter().map(|s| s.signed_distance(x)).collect();
        if f.iter().all(|v| v.abs() < 1e-11 * (1.0 + x.length())) {
            break;
        }
        let mut jtj = [[0.0f64; 3]; 3];
        let mut jtf = DVec3::ZERO;
        for (s, &fa) in surfs.iter().zip(&f) {
            let ga = s.gradient(x);
            let a = ga.to_array();
            for r in 0..3 {
                for c in 0..3 {
                    jtj[r][c] += a[r] * a[c];
                }
            }
            jtf += ga * fa;
        }
        let eig = eigen_3x3(jtj);
        let lmax = eig[2].0;
        if lmax <= 0.0 {
            break;
        }
        let step = eig.iter().filter(|(l, _)| *l > WEAK_DIRECTION * lmax).fold(
            DVec3::ZERO,
            |acc, (l, v)| {
                let v = DVec3::from_array(*v);
                acc - v * (v.dot(jtf) / l)
            },
        );
        let len = step.length();
        let step = if len > max_step && len > 0.0 {
            step * (max_step / len)
        } else {
            step
        };
        let next = x + step;
        if !next.is_finite() {
            break;
        }
        // Never accept a step that makes things worse (tangent surfaces).
        if residual(next) > residual(x) && len < 1e-9 * (1.0 + x.length()) {
            break;
        }
        x = next;
        if len < 1e-13 * (1.0 + x.length()) {
            break;
        }
    }
    (x, residual(x))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dense_solve_and_lm_line_fit() {
        let x = solve_dense(
            vec![
                vec![2.0, 1.0, 0.0],
                vec![1.0, 3.0, 1.0],
                vec![0.0, 1.0, 4.0],
            ],
            vec![3.0, 5.0, 5.0],
        )
        .unwrap();
        assert!((x[0] - 1.0).abs() < 1e-12 && (x[1] - 1.0).abs() < 1e-12);
        // Fit y = a x + b to exact samples encoded as points (x, y, 0).
        let pts: Vec<DVec3> = (0..20)
            .map(|i| DVec3::new(i as f64, 3.0 * i as f64 - 2.0, 0.0))
            .collect();
        let mut params = [0.0, 0.0];
        let rms = levenberg_marquardt(
            &mut params,
            &[false, false],
            &pts,
            |x, p| x[0] * p.x + x[1] - p.y,
            50,
        );
        assert!(rms < 1e-6, "rms {rms}");
        assert!((params[0] - 3.0).abs() < 1e-6 && (params[1] + 2.0).abs() < 1e-5);
    }

    #[test]
    fn corner_of_nearly_coplanar_normals_stays_at_the_mesh_corner() {
        let plane = |origin: DVec3, normal: DVec3| Surface::Plane {
            origin,
            normal: normal.normalize(),
        };
        // A clean box corner is solved exactly.
        let (x, y, z) = (
            plane(DVec3::new(10.0, 0.0, 0.0), DVec3::X),
            plane(DVec3::new(0.0, 5.0, 0.0), DVec3::Y),
            plane(DVec3::new(0.0, 0.0, 3.0), DVec3::Z),
        );
        let (p, res) = project_onto(&[&x, &y, &z], DVec3::new(9.7, 5.2, 2.9), 5.0);
        assert!((p - DVec3::new(10.0, 5.0, 3.0)).length() < 1e-9, "{p}");
        assert!(res < 1e-9);
        // A scan face split by a 5° crease (a, b) meeting a wall (c) that
        // almost contains the crease line: the exact common point lies far
        // away, the corner must stay next to the mesh with a small gap.
        let tilt = 5f64.to_radians();
        let a = plane(DVec3::ZERO, DVec3::Z);
        let b = plane(DVec3::ZERO, DVec3::new(0.0, tilt.sin(), tilt.cos()));
        let c = plane(DVec3::new(0.0, 0.02, 0.0), DVec3::new(0.002, 1.0, 0.0));
        // The exact common point is (10, 0, 0).
        let mesh_corner = DVec3::new(0.05, 0.03, 0.02);
        let (p, res) = project_onto(&[&a, &b, &c], mesh_corner, 50.0);
        assert!((p - mesh_corner).length() < 0.2, "moved to {p}");
        assert!(res < 0.05, "gap {res}");
    }

    #[test]
    fn basis_functions_sum_to_one_and_derivatives_to_zero() {
        let knots = [0.0, 0.0, 0.0, 0.0, 0.3, 0.6, 1.0, 1.0, 1.0, 1.0];
        for &t in &[0.0, 0.1, 0.3, 0.45, 0.99, 1.0] {
            let s = find_span(&knots, 6, 3, t);
            let (n, d) = basis_ders(&knots, s, 3, t);
            assert!((n.iter().sum::<f64>() - 1.0).abs() < 1e-12);
            assert!(d.iter().sum::<f64>().abs() < 1e-9);
        }
    }
}
