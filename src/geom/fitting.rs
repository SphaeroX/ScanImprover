use glam::Vec3;
use rayon::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct PlaneFit {
    pub point: Vec3,
    pub normal: Vec3,
    pub rms: f32,
    pub max_dev: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct CircleFit {
    pub center: Vec3,
    pub normal: Vec3,
    pub radius: f32,
    pub plane_rms: f32,
    pub radial_rms: f32,
    pub radial_max: f32,
    /// True when the points were recognized as a cylindrical surface and the
    /// circle was fitted as the cross-section perpendicular to its axis.
    pub cylinder: bool,
}

#[derive(Clone, Debug)]
pub struct FittedPlane {
    pub id: u64,
    pub name: String,
    pub fit: PlaneFit,
    pub visible: bool,
    pub color: [f32; 4],
}

#[derive(Clone, Debug)]
pub struct FittedCircle {
    pub id: u64,
    pub name: String,
    pub fit: CircleFit,
    pub visible: bool,
    pub color: [f32; 4],
}

pub fn eigen_3x3(m: [[f64; 3]; 3]) -> [(f64, [f64; 3]); 3] {
    let mut a = m;
    let mut v: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-15 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            let apq = a[p][q];
            if apq.abs() < 1e-18 {
                continue;
            }
            let app = a[p][p];
            let aqq = a[q][q];
            let theta = (aqq - app) / (2.0 * apq);
            let t = if theta >= 0.0 {
                1.0 / (theta + (theta * theta + 1.0).sqrt())
            } else {
                1.0 / (theta - (theta * theta + 1.0).sqrt())
            };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            a[p][p] = app - t * apq;
            a[q][q] = aqq + t * apq;
            a[p][q] = 0.0;
            a[q][p] = 0.0;
            let other = 3 - p - q;
            let aop = a[other][p];
            let aoq = a[other][q];
            a[other][p] = c * aop - s * aoq;
            a[other][q] = s * aop + c * aoq;
            a[p][other] = a[other][p];
            a[q][other] = a[other][q];
            for k in 0..3 {
                let vkp = v[k][p];
                let vkq = v[k][q];
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    let mut out = [
        (a[0][0], [1.0, 0.0, 0.0]),
        (a[1][1], [0.0, 1.0, 0.0]),
        (a[2][2], [0.0, 0.0, 1.0]),
    ];
    for i in 0..3 {
        let col = [v[0][i], v[1][i], v[2][i]];
        let len = (col[0] * col[0] + col[1] * col[1] + col[2] * col[2]).sqrt();
        let len = if len > 1e-15 { len } else { 1.0 };
        out[i].1 = [col[0] / len, col[1] / len, col[2] / len];
    }
    out.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
    out
}

fn centroid_f64(points: &[[f32; 3]]) -> [f64; 3] {
    let mut c = [0.0f64; 3];
    for p in points {
        for a in 0..3 {
            c[a] += p[a] as f64;
        }
    }
    let n = points.len().max(1) as f64;
    [c[0] / n, c[1] / n, c[2] / n]
}

pub fn fit_plane(points: &[[f32; 3]]) -> Option<PlaneFit> {
    if points.len() < 3 {
        return None;
    }
    let c = centroid_f64(points);
    let mut m = [[0.0f64; 3]; 3];
    for p in points {
        let d = [p[0] as f64 - c[0], p[1] as f64 - c[1], p[2] as f64 - c[2]];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += d[i] * d[j];
            }
        }
    }
    let eig = eigen_3x3(m);
    let (lam, ev) = eig[0];
    if lam.is_nan() {
        return None;
    }
    let n = Vec3::new(ev[0] as f32, ev[1] as f32, ev[2] as f32);
    let len = n.length();
    if len < 1e-12 {
        return None;
    }
    let normal = n / len;
    let point = Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32);
    let mut ss = 0.0f64;
    let mut max_dev = 0.0f32;
    for p in points {
        let d = (Vec3::from(*p) - point).dot(normal);
        ss += (d * d) as f64;
        max_dev = max_dev.max(d.abs());
    }
    Some(PlaneFit {
        point,
        normal,
        rms: (ss / points.len() as f64) as f32,
        max_dev,
    })
}

pub fn solve_3x3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let mut m = [a[0], a[1], a[2]];
    let mut r = b;
    for col in 0..3 {
        let mut piv = col;
        for row in col + 1..3 {
            if m[row][col].abs() > m[piv][col].abs() {
                piv = row;
            }
        }
        if m[piv][col].abs() < 1e-14 {
            return None;
        }
        m.swap(col, piv);
        r.swap(col, piv);
        for row in 0..3 {
            if row == col {
                continue;
            }
            let f = m[row][col] / m[col][col];
            for k in col..3 {
                m[row][k] -= f * m[col][k];
            }
            r[row] -= f * r[col];
        }
    }
    Some([r[0] / m[0][0], r[1] / m[1][1], r[2] / m[2][2]])
}

pub fn plane_basis(normal: Vec3) -> (Vec3, Vec3) {
    let up = if normal.x.abs() < 0.9 {
        Vec3::X
    } else {
        Vec3::Y
    };
    let u = normal.cross(up).normalize_or_zero();
    if u.length_squared() < 0.5 {
        return (Vec3::X, Vec3::Y);
    }
    let v = normal.cross(u);
    (u.normalize(), v.normalize())
}

pub fn fit_circle(points: &[[f32; 3]]) -> Option<CircleFit> {
    if points.len() < 3 {
        return None;
    }
    let plane = fit_plane(points)?;
    let in_plane = circle_in_plane(points, plane.point, plane.normal).map(
        |(center, radius, radial_ms, radial_max)| CircleFit {
            center,
            normal: plane.normal,
            radius,
            plane_rms: plane.rms,
            radial_rms: radial_ms as f32,
            radial_max,
            cylinder: false,
        },
    );

    // A curved selection (e.g. a patch of a cylinder wall) must not be fitted
    // in the PCA plane: that plane cuts the surface lengthwise or at an angle
    // and produces a crooked circle with a wrong radius. Instead, search for
    // the direction whose orthogonal plane holds all points on one circle:
    // for a cylinder that direction is the axis, and the circle becomes the
    // true cross-section. The cross-section wins only if it beats the plane
    // fit clearly, so planar rings keep their exact plane-based result.
    let scale = points_scale(points);
    let cyl = fit_circle_as_cylinder_section(points, plane.normal, scale);
    match (in_plane, cyl) {
        (Some(pca), Some(cyl)) => {
            let min_gain = ((1e-4 * scale as f64) * (1e-4 * scale as f64)) as f32;
            let better = cyl.radial_rms <= pca.radial_rms * 0.7
                && pca.radial_rms - cyl.radial_rms >= min_gain;
            if better { Some(cyl) } else { Some(pca) }
        }
        (None, Some(cyl)) => Some(cyl),
        (Some(pca), None) => Some(pca),
        (None, None) => None,
    }
}

/// Fits a circle into the given plane (origin + normal) using the algebraic
/// (Kasa) method. Returns (center, radius, mean squared radial residual, max
/// radial deviation).
fn circle_in_plane(
    points: &[[f32; 3]],
    origin: Vec3,
    normal: Vec3,
) -> Option<(Vec3, f32, f64, f32)> {
    let (u, v) = plane_basis(normal);
    let mut su = 0.0f64;
    let mut sv = 0.0f64;
    let mut suu = 0.0f64;
    let mut svv = 0.0f64;
    let mut suv = 0.0f64;
    let mut sb = 0.0f64;
    let mut bu = 0.0f64;
    let mut bv = 0.0f64;
    for p in points {
        let d = Vec3::from(*p) - origin;
        let x = d.dot(u) as f64;
        let y = d.dot(v) as f64;
        let q = x * x + y * y;
        su += x;
        sv += y;
        suu += x * x;
        svv += y * y;
        suv += x * y;
        sb -= q;
        bu -= q * x;
        bv -= q * y;
    }
    let n = points.len() as f64;
    let mat = [[suu, suv, su], [suv, svv, sv], [su, sv, n]];
    let rhs = [bu, bv, sb];
    let sol = solve_3x3(mat, rhs)?;
    let (d_coef, e_coef, f_coef) = (sol[0], sol[1], sol[2]);
    let cx = -d_coef * 0.5;
    let cy = -e_coef * 0.5;
    let r2 = cx * cx + cy * cy - f_coef;
    if r2 <= 1e-12 {
        return None;
    }
    let radius = r2.sqrt() as f32;
    let center = origin + u * cx as f32 + v * cy as f32;
    let mut ss = 0.0f64;
    let mut rmax = 0.0f32;
    for p in points {
        let d = Vec3::from(*p) - center;
        let radial = (d - normal * d.dot(normal)).length() - radius;
        ss += (radial * radial) as f64;
        rmax = rmax.max(radial.abs());
    }
    Some((center, radius, ss / n, rmax))
}

/// Number of directions probed on the sphere while searching for the axis.
const AXIS_GRID_SAMPLES: usize = 256;
/// Upper bound on the points used while scanning directions; the final fit
/// always uses every point.
const AXIS_SEARCH_MAX_POINTS: usize = 16384;
/// Minimum angular wrap of the projections around the fitted center for a
/// direction to count as a valid circle plane (10 degrees). Rejects
/// near-collinear projections that a gigantic circle would "fit".
const MIN_CIRCLE_ARC_RAD: f64 = 0.17453292519943295;

/// Recognizes a cylindrical selection and returns the circle as its
/// cross-section: plane perpendicular to the axis, radius = cylinder radius,
/// center on the axis. `plane_normal` (PCA plane normal) is used as an extra
/// search seed.
fn fit_circle_as_cylinder_section(
    points: &[[f32; 3]],
    plane_normal: Vec3,
    scale: f32,
) -> Option<CircleFit> {
    let c = centroid_f64(points);
    let centroid = Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32);
    let stride = points.len().div_ceil(AXIS_SEARCH_MAX_POINTS);
    let sub: Vec<[f32; 3]> = points.iter().step_by(stride).copied().collect();

    let mut seeds = fibonacci_directions(AXIS_GRID_SAMPLES);
    seeds.push(plane_normal);

    // Scoring the candidate directions is embarrassingly parallel.
    let mut ranked: Vec<(f64, Vec3)> = seeds
        .into_par_iter()
        .filter_map(|d| kasa_radial_ms(&sub, centroid, d).map(|ms| (ms, d)))
        .collect();
    if ranked.is_empty() {
        return None;
    }
    ranked.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut best_dir = Vec3::ZERO;
    let mut best_ms = f64::MAX;
    for (_, dir) in ranked.iter().take(3) {
        let (dir, ms) = refine_axis_dir(&sub, centroid, *dir);
        if ms < best_ms {
            best_ms = ms;
            best_dir = dir;
        }
    }
    if best_dir.length_squared() < 0.5 {
        return None;
    }

    // Final fit on the full point set, in the plane orthogonal to the axis.
    let (u, v) = plane_basis(best_dir);
    let mut xy: Vec<(f64, f64)> = Vec::with_capacity(points.len());
    for p in points {
        let d = Vec3::from(*p) - centroid;
        xy.push((d.dot(u) as f64, d.dot(v) as f64));
    }
    let (cx0, cy0, r0) = fit_circle_2d(&xy)?;
    let (cx, cy, r) = refine_circle_2d(&xy, cx0, cy0, r0);

    if !(r > 1e-9 && r <= scale as f64 * 4.0) {
        return None;
    }
    if angular_coverage(&xy, cx, cy) < MIN_CIRCLE_ARC_RAD {
        return None;
    }

    let center = centroid + u * cx as f32 + v * cy as f32;
    let mut radial_ss = 0.0f64;
    let mut radial_max = 0.0f64;
    for &(x, y) in &xy {
        let dx = x - cx;
        let dy = y - cy;
        let e = (dx * dx + dy * dy).sqrt() - r;
        radial_ss += e * e;
        radial_max = radial_max.max(e.abs());
    }
    let mut plane_ss = 0.0f64;
    for p in points {
        let a = (Vec3::from(*p) - center).dot(best_dir) as f64;
        plane_ss += a * a;
    }
    let n = points.len() as f64;
    Some(CircleFit {
        center,
        normal: best_dir,
        radius: r as f32,
        plane_rms: (plane_ss / n) as f32,
        radial_rms: (radial_ss / n) as f32,
        radial_max: radial_max as f32,
        cylinder: true,
    })
}

/// Mean squared radial residual of the Kasa circle fit of the points projected
/// along `dir` onto the plane through `origin`. Serves as the score of a
/// candidate axis/circle-plane direction: for the true cylinder axis every
/// point of the wall projects onto the same circle, so the score drops to the
/// noise level.
fn kasa_radial_ms(points: &[[f32; 3]], origin: Vec3, dir: Vec3) -> Option<f64> {
    let (u, v) = plane_basis(dir);
    let mut su = 0.0f64;
    let mut sv = 0.0f64;
    let mut suu = 0.0f64;
    let mut svv = 0.0f64;
    let mut suv = 0.0f64;
    let mut sb = 0.0f64;
    let mut bu = 0.0f64;
    let mut bv = 0.0f64;
    for p in points {
        let d = Vec3::from(*p) - origin;
        let x = d.dot(u) as f64;
        let y = d.dot(v) as f64;
        let q = x * x + y * y;
        su += x;
        sv += y;
        suu += x * x;
        svv += y * y;
        suv += x * y;
        sb -= q;
        bu -= q * x;
        bv -= q * y;
    }
    let n = points.len() as f64;
    let mat = [[suu, suv, su], [suv, svv, sv], [su, sv, n]];
    let sol = solve_3x3(mat, [bu, bv, sb])?;
    let cx = -sol[0] * 0.5;
    let cy = -sol[1] * 0.5;
    let r2 = cx * cx + cy * cy - sol[2];
    if r2.is_nan() || r2 <= 1e-12 {
        return None;
    }
    let r = r2.sqrt();
    let mut ss = 0.0f64;
    for p in points {
        let d = Vec3::from(*p) - origin;
        let x = d.dot(u) as f64 - cx;
        let y = d.dot(v) as f64 - cy;
        let e = (x * x + y * y).sqrt() - r;
        ss += e * e;
    }
    Some(ss / n)
}

/// Local pattern search on the direction sphere: walks downhill on the Kasa
/// radial score by stepping along the tangent directions, halving the step
/// once no neighbor improves. Returns the refined direction and its score.
fn refine_axis_dir(points: &[[f32; 3]], origin: Vec3, dir0: Vec3) -> (Vec3, f64) {
    let mut dir = dir0.normalize_or_zero();
    if dir.length_squared() < 0.5 {
        return (dir0, f64::MAX);
    }
    let mut best = kasa_radial_ms(points, origin, dir).unwrap_or(f64::MAX);
    let mut step = 0.2f64;
    let mut rounds = 0;
    while step > 1e-5 && rounds < 400 {
        rounds += 1;
        let (u, v) = plane_basis(dir);
        let mut round_best: Option<(f64, Vec3)> = None;
        for w in [u, -u, v, -v] {
            let cand = (dir + w * step as f32).normalize_or_zero();
            if cand.length_squared() < 0.5 {
                continue;
            }
            if let Some(ms) = kasa_radial_ms(points, origin, cand)
                && ms < best
                && round_best.is_none_or(|(m, _)| ms < m)
            {
                round_best = Some((ms, cand));
            }
        }
        match round_best {
            Some((ms, cand)) => {
                best = ms;
                dir = cand;
            }
            None => step *= 0.5,
        }
    }
    (dir, best)
}

/// Algebraic (Kasa) circle fit in 2D. Returns (cx, cy, radius).
pub fn fit_circle_2d(pts: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let mut su = 0.0f64;
    let mut sv = 0.0f64;
    let mut suu = 0.0f64;
    let mut svv = 0.0f64;
    let mut suv = 0.0f64;
    let mut sb = 0.0f64;
    let mut bu = 0.0f64;
    let mut bv = 0.0f64;
    for &(x, y) in pts {
        let q = x * x + y * y;
        su += x;
        sv += y;
        suu += x * x;
        svv += y * y;
        suv += x * y;
        sb -= q;
        bu -= q * x;
        bv -= q * y;
    }
    let n = pts.len().max(1) as f64;
    let mat = [[suu, suv, su], [suv, svv, sv], [su, sv, n]];
    let rhs = [bu, bv, sb];
    let sol = solve_3x3(mat, rhs)?;
    let (d_coef, e_coef, f_coef) = (sol[0], sol[1], sol[2]);
    let cx = -d_coef * 0.5;
    let cy = -e_coef * 0.5;
    let r2 = cx * cx + cy * cy - f_coef;
    if r2 <= 1e-12 {
        return None;
    }
    Some((cx, cy, r2.sqrt()))
}

/// Geometric (least squares) refinement of a 2D circle: a few damped
/// Gauss-Newton steps minimizing the radial distances. Removes the algebraic
/// bias of the Kasa start, which matters for shallow arcs.
fn refine_circle_2d(pts: &[(f64, f64)], cx0: f64, cy0: f64, r0: f64) -> (f64, f64, f64) {
    let ms = |cx: f64, cy: f64, r: f64| -> f64 {
        let mut ss = 0.0f64;
        for &(x, y) in pts {
            let e = ((x - cx) * (x - cx) + (y - cy) * (y - cy)).sqrt() - r;
            ss += e * e;
        }
        ss / pts.len().max(1) as f64
    };
    let (mut cx, mut cy, mut r) = (cx0, cy0, r0);
    let mut best = ms(cx, cy, r);
    for _ in 0..12 {
        let mut a = [[0.0f64; 3]; 3];
        let mut b = [0.0f64; 3];
        for &(x, y) in pts {
            let dx = x - cx;
            let dy = y - cy;
            let d = (dx * dx + dy * dy).sqrt();
            if d < 1e-12 {
                continue;
            }
            let e = d - r;
            let j = [-dx / d, -dy / d, -1.0];
            for i in 0..3 {
                for k in 0..3 {
                    a[i][k] += j[i] * j[k];
                }
                b[i] += j[i] * e;
            }
        }
        let tr = a[0][0] + a[1][1] + a[2][2];
        for i in 0..3 {
            a[i][i] += 1e-9 * (tr.abs() + 1.0);
        }
        let Some(delta) = solve_3x3(a, [-b[0], -b[1], -b[2]]) else {
            break;
        };
        let (nx, ny, nr) = (cx + delta[0], cy + delta[1], r + delta[2]);
        if !nr.is_finite() || nr <= 1e-12 {
            break;
        }
        let nms = ms(nx, ny, nr);
        if nms >= best {
            break;
        }
        let gain = best - nms;
        (cx, cy, r, best) = (nx, ny, nr, nms);
        if gain <= best * 1e-12 {
            break;
        }
    }
    (cx, cy, r)
}

/// Angular span the 2D points wrap around the given center (0..2*PI).
fn angular_coverage(pts: &[(f64, f64)], cx: f64, cy: f64) -> f64 {
    if pts.is_empty() {
        return 0.0;
    }
    let mut angles: Vec<f64> = pts.iter().map(|&(x, y)| (y - cy).atan2(x - cx)).collect();
    angles.sort_by(|a, b| a.total_cmp(b));
    let n = angles.len();
    let mut max_gap = 2.0 * std::f64::consts::PI - (angles[n - 1] - angles[0]);
    for i in 1..n {
        max_gap = max_gap.max(angles[i] - angles[i - 1]);
    }
    (2.0 * std::f64::consts::PI - max_gap).max(0.0)
}

/// Evenly distributed directions on the unit sphere (Fibonacci lattice).
fn fibonacci_directions(count: usize) -> Vec<Vec3> {
    let ga = 2.399963229728653f64;
    (0..count)
        .map(|i| {
            let z = 1.0 - (2.0 * i as f64 + 1.0) / count as f64;
            let rad = (1.0 - z * z).sqrt().max(0.0);
            let a = ga * i as f64;
            Vec3::new((rad * a.cos()) as f32, (rad * a.sin()) as f32, z as f32)
        })
        .collect()
}

/// Bounding box diagonal of the point set.
fn points_scale(points: &[[f32; 3]]) -> f32 {
    let mut mn = [f32::MAX; 3];
    let mut mx = [f32::MIN; 3];
    for p in points {
        for k in 0..3 {
            mn[k] = mn[k].min(p[k]);
            mx[k] = mx[k].max(p[k]);
        }
    }
    let d = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt().max(1e-9)
}
