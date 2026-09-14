//! Edge curves: analytic intersections of neighbouring face surfaces where
//! a closed form exists (line, circle, ellipse), otherwise a cubic B-spline
//! interpolating points of the true intersection seeded from the mesh
//! boundary between the two groups.

use super::numeric::{basis_ders, find_span, project_onto, solve_dense};
use super::surface::{Surface, perp_basis};
use glam::DVec3;
use std::f64::consts::TAU;

#[derive(Clone)]
pub enum Curve {
    Line {
        origin: DVec3,
        dir: DVec3,
    },
    /// `center + radius (cos t xdir + sin t (axis × xdir))`.
    Circle {
        center: DVec3,
        axis: DVec3,
        xdir: DVec3,
        radius: f64,
    },
    /// `center + major cos t xdir + minor sin t (axis × xdir)`.
    Ellipse {
        center: DVec3,
        axis: DVec3,
        xdir: DVec3,
        major: f64,
        minor: f64,
    },
    /// Clamped non-rational B-spline over 0..=1 (full knot vector).
    Spline {
        degree: usize,
        cps: Vec<DVec3>,
        knots: Vec<f64>,
    },
}

impl Curve {
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn label(&self) -> &'static str {
        match self {
            Curve::Line { .. } => "Line",
            Curve::Circle { .. } => "Circle",
            Curve::Ellipse { .. } => "Ellipse",
            Curve::Spline { .. } => "B-spline",
        }
    }

    /// Index into the curve counters (line, circle, ellipse, B-spline).
    pub fn kind_index(&self) -> usize {
        match self {
            Curve::Line { .. } => 0,
            Curve::Circle { .. } => 1,
            Curve::Ellipse { .. } => 2,
            Curve::Spline { .. } => 3,
        }
    }

    fn conic_param(p: DVec3, center: DVec3, axis: DVec3, xdir: DVec3, a: f64, b: f64) -> f64 {
        let d = p - center;
        let y = axis.cross(xdir);
        (d.dot(y) / b).atan2(d.dot(xdir) / a)
    }

    fn at(&self, t: f64) -> DVec3 {
        match self {
            Curve::Line { origin, dir } => *origin + *dir * t,
            Curve::Circle {
                center,
                axis,
                xdir,
                radius,
            } => *center + (*xdir * t.cos() + axis.cross(*xdir) * t.sin()) * *radius,
            Curve::Ellipse {
                center,
                axis,
                xdir,
                major,
                minor,
            } => *center + *xdir * (major * t.cos()) + axis.cross(*xdir) * (minor * t.sin()),
            Curve::Spline { degree, cps, knots } => {
                let s = find_span(knots, cps.len(), *degree, t);
                let (n, _) = basis_ders(knots, s, *degree, t);
                (0..=*degree).fold(DVec3::ZERO, |acc, k| acc + cps[s - degree + k] * n[k])
            }
        }
    }

    /// `n + 1` points from `a` to `b` along the parametrisation direction
    /// (a full turn when `closed`).
    pub fn sample(&self, a: DVec3, b: DVec3, closed: bool, n: usize) -> Vec<DVec3> {
        let n = n.max(1);
        let (t0, t1) = match self {
            Curve::Line { origin, dir } => ((a - *origin).dot(*dir), (b - *origin).dot(*dir)),
            Curve::Circle {
                center,
                axis,
                xdir,
                radius,
            } => span(
                Self::conic_param(a, *center, *axis, *xdir, *radius, *radius),
                Self::conic_param(b, *center, *axis, *xdir, *radius, *radius),
                closed,
            ),
            Curve::Ellipse {
                center,
                axis,
                xdir,
                major,
                minor,
            } => span(
                Self::conic_param(a, *center, *axis, *xdir, *major, *minor),
                Self::conic_param(b, *center, *axis, *xdir, *major, *minor),
                closed,
            ),
            Curve::Spline { .. } => (0.0, 1.0),
        };
        (0..=n)
            .map(|i| self.at(t0 + (t1 - t0) * i as f64 / n as f64))
            .collect()
    }

    /// Distance from `p` to the (unbounded) curve.
    pub fn distance(&self, p: DVec3) -> f64 {
        match self {
            Curve::Line { origin, dir } => {
                let d = p - *origin;
                (d - *dir * d.dot(*dir)).length()
            }
            Curve::Circle {
                center,
                axis,
                radius,
                ..
            } => {
                let d = p - *center;
                let h = d.dot(*axis);
                let rho = (d - *axis * h).length();
                (h * h + (rho - radius).powi(2)).sqrt()
            }
            Curve::Ellipse { .. } | Curve::Spline { .. } => {
                // Dense sampling, then a local refinement around the best sample.
                let (lo, hi) = match self {
                    Curve::Spline { .. } => (0.0, 1.0),
                    _ => (0.0, TAU),
                };
                let n = 512;
                let mut best = (f64::MAX, 0.0);
                for i in 0..=n {
                    let t = lo + (hi - lo) * i as f64 / n as f64;
                    let d = (self.at(t) - p).length();
                    if d < best.0 {
                        best = (d, t);
                    }
                }
                let mut step = (hi - lo) / n as f64;
                let mut t = best.1;
                for _ in 0..40 {
                    let mut moved = false;
                    for c in [t - step, t + step] {
                        let c = if matches!(self, Curve::Spline { .. }) {
                            c.clamp(lo, hi)
                        } else {
                            c
                        };
                        let d = (self.at(c) - p).length();
                        if d < best.0 {
                            best = (d, c);
                            t = c;
                            moved = true;
                        }
                    }
                    if !moved {
                        step *= 0.5;
                    }
                }
                best.0
            }
        }
    }
}

/// Parameter range from `t0` to `t1` going forwards (full turn if closed).
fn span(t0: f64, mut t1: f64, closed: bool) -> (f64, f64) {
    if closed {
        return (t0, t0 + TAU);
    }
    while t1 <= t0 + 1e-12 {
        t1 += TAU;
    }
    (t0, t1)
}

/// Rotation sense of the chain around `axis` through `center` (> 0 = CCW).
fn winding(chain: &[DVec3], center: DVec3, axis: DVec3) -> f64 {
    chain
        .windows(2)
        .map(|w| (w[0] - center).cross(w[1] - center).dot(axis))
        .sum()
}

/// Revolution profile of a surface around a common axis: in (axial t,
/// radius rho) coordinates.
#[derive(Clone, Copy)]
enum Profile {
    /// Plane perpendicular to the axis: t = const.
    Const(f64),
    /// rho = m t + b (cylinder: m = 0, cone).
    Line(f64, f64),
    /// Sphere centred on the axis: (t - t0)^2 + rho^2 = r^2.
    Circle(f64, f64),
}

const PARALLEL_EPS: f64 = 1e-9;

fn profile(s: &Surface, o: DVec3, a: DVec3, eps: f64) -> Option<Profile> {
    let off_axis = |p: DVec3| {
        let d = p - o;
        (d - a * d.dot(a)).length()
    };
    match s {
        Surface::Plane { origin, normal } => {
            let c = normal.dot(a);
            (c.abs() >= 1.0 - PARALLEL_EPS).then(|| Profile::Const((*origin - o).dot(a)))
        }
        Surface::Cylinder {
            origin,
            axis,
            radius,
        } => (axis.dot(a).abs() >= 1.0 - PARALLEL_EPS && off_axis(*origin) <= eps)
            .then_some(Profile::Line(0.0, *radius)),
        Surface::Cone {
            origin,
            axis,
            radius,
            half_angle,
        } => {
            let s = axis.dot(a);
            if s.abs() < 1.0 - PARALLEL_EPS || off_axis(*origin) > eps {
                return None;
            }
            let s = s.signum();
            let t0 = (*origin - o).dot(a);
            let m = s * half_angle.tan();
            Some(Profile::Line(m, radius - m * t0))
        }
        Surface::Sphere { center, radius } => {
            (off_axis(*center) <= eps).then(|| Profile::Circle((*center - o).dot(a), *radius))
        }
        Surface::Spline(_) => None,
    }
}

/// Intersections of two profiles with rho > 0.
fn profile_hits(p: Profile, q: Profile) -> Vec<(f64, f64)> {
    use Profile::*;
    let mut out = Vec::new();
    let quad = |a: f64, b: f64, c: f64, out: &mut Vec<f64>| {
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 || a.abs() < 1e-300 {
            return;
        }
        let r = disc.sqrt();
        out.push((-b + r) / (2.0 * a));
        out.push((-b - r) / (2.0 * a));
    };
    match (p, q) {
        (Const(_), Const(_)) => {}
        (Const(t), Line(m, b)) | (Line(m, b), Const(t)) => out.push((t, m * t + b)),
        (Const(t), Circle(t0, r)) | (Circle(t0, r), Const(t)) => {
            let h = r * r - (t - t0).powi(2);
            if h >= 0.0 {
                out.push((t, h.sqrt()));
            }
        }
        (Line(m1, b1), Line(m2, b2)) => {
            if (m1 - m2).abs() > 1e-12 {
                let t = (b2 - b1) / (m1 - m2);
                out.push((t, m1 * t + b1));
            }
        }
        (Line(m, b), Circle(t0, r)) | (Circle(t0, r), Line(m, b)) => {
            let mut ts = Vec::new();
            quad(
                1.0 + m * m,
                2.0 * m * b - 2.0 * t0,
                t0 * t0 + b * b - r * r,
                &mut ts,
            );
            out.extend(ts.into_iter().map(|t| (t, m * t + b)));
        }
        (Circle(t1, r1), Circle(t2, r2)) => {
            if (t2 - t1).abs() > 1e-12 {
                let t = (r1 * r1 - r2 * r2 - t1 * t1 + t2 * t2) / (2.0 * (t2 - t1));
                let h = r1 * r1 - (t - t1).powi(2);
                if h >= 0.0 {
                    out.push((t, h.sqrt()));
                }
            }
        }
    }
    out.retain(|&(_, rho)| rho > 1e-9);
    out
}

/// Common axis of two surfaces for the coaxial (circle) case.
fn common_axis(sa: &Surface, sb: &Surface) -> Option<(DVec3, DVec3)> {
    if let Some(ax) = sa.axis().or_else(|| sb.axis()) {
        return Some(ax);
    }
    match (sa, sb) {
        (Surface::Plane { normal, .. }, Surface::Sphere { center, .. })
        | (Surface::Sphere { center, .. }, Surface::Plane { normal, .. }) => {
            Some((*center, *normal))
        }
        (Surface::Sphere { center: c1, .. }, Surface::Sphere { center: c2, .. }) => {
            let d = *c2 - *c1;
            (d.length() > 1e-9).then(|| (*c1, d.normalize()))
        }
        _ => None,
    }
}

/// Parameters for [`intersect`].
pub(super) struct EdgeInput<'a> {
    pub sa: &'a Surface,
    pub sb: &'a Surface,
    /// Mesh boundary points from the start to the end vertex.
    pub chain: &'a [DVec3],
    /// Solved start / end vertex positions.
    pub a: DVec3,
    pub b: DVec3,
    pub closed: bool,
    /// Model scale (bounding box diagonal) for tolerances.
    pub scale: f64,
    /// Largest accepted distance of the mesh chain from an analytic curve
    /// before falling back to the interpolated intersection.
    pub dev_limit: f64,
}

/// Edge curve of two neighbouring faces, parametrised from `a` to `b`.
pub(super) fn intersect(inp: &EdgeInput) -> Result<Curve, String> {
    if let Some(c) = analytic(inp)
        && inp.chain.iter().all(|&p| c.distance(p) <= inp.dev_limit)
    {
        return Ok(c);
    }
    spline_intersection(inp)
}

fn analytic(inp: &EdgeInput) -> Option<Curve> {
    let (sa, sb) = (inp.sa, inp.sb);
    let mean = inp.chain.iter().fold(DVec3::ZERO, |acc, &p| acc + p) / inp.chain.len() as f64;
    if let (Surface::Plane { normal: na, .. }, Surface::Plane { normal: nb, .. }) = (sa, sb) {
        let d = na.cross(*nb);
        if d.length() < 1e-6 {
            return None;
        }
        let mut dir = d.normalize();
        let ab = inp.b - inp.a;
        if ab.dot(dir) < 0.0 {
            dir = -dir;
        }
        let (origin, _) = project_onto(&[sa, sb], inp.a, inp.scale);
        return Some(Curve::Line { origin, dir });
    }
    // Coaxial surfaces of revolution (incl. planes perpendicular to the
    // axis and spheres centred on it) meet in circles.
    if let Some((o, a)) = common_axis(sa, sb) {
        let eps = inp.scale * 1e-9;
        if let (Some(p), Some(q)) = (profile(sa, o, a, eps), profile(sb, o, a, eps)) {
            let coords: Vec<(f64, f64)> = inp
                .chain
                .iter()
                .map(|&p| {
                    let d = p - o;
                    let t = d.dot(a);
                    (t, (d - a * t).length())
                })
                .collect();
            let n = coords.len() as f64;
            let (mt, mr) = coords
                .iter()
                .fold((0.0, 0.0), |acc, c| (acc.0 + c.0 / n, acc.1 + c.1 / n));
            let (t, rho) = profile_hits(p, q).into_iter().min_by(|x, y| {
                let dx = (x.0 - mt).powi(2) + (x.1 - mr).powi(2);
                let dy = (y.0 - mt).powi(2) + (y.1 - mr).powi(2);
                dx.total_cmp(&dy)
            })?;
            let center = o + a * t;
            let axis = if winding(inp.chain, center, a) < 0.0 {
                -a
            } else {
                a
            };
            let r = inp.a - center;
            let r = r - axis * r.dot(axis);
            let xdir = if r.length() > 1e-12 {
                r.normalize()
            } else {
                perp_basis(axis).0
            };
            return Some(Curve::Circle {
                center,
                axis,
                xdir,
                radius: rho,
            });
        }
    }
    // Plane and cylinder at an angle: ellipse, or axis-parallel lines.
    let (plane, cyl) = match (sa, sb) {
        (Surface::Plane { .. }, Surface::Cylinder { .. }) => (sa, sb),
        (Surface::Cylinder { .. }, Surface::Plane { .. }) => (sb, sa),
        _ => return None,
    };
    let (
        Surface::Plane { origin: po, normal },
        Surface::Cylinder {
            origin,
            axis,
            radius,
        },
    ) = (plane, cyl)
    else {
        return None;
    };
    let c = normal.dot(*axis);
    if c.abs() < 1e-9 {
        // The plane contains the axis direction: up to two rulings.
        let h = normal.dot(*origin - *po);
        let q = *origin - *normal * h;
        let w = normal.cross(*axis).normalize();
        let s2 = radius * radius - h * h;
        let s = if s2 >= 0.0 {
            s2.sqrt()
        } else if h.abs() - radius <= inp.dev_limit {
            0.0
        } else {
            return None;
        };
        let cand = [q + w * s, q - w * s];
        let base = if (cand[0] - mean).length_squared() <= (cand[1] - mean).length_squared() {
            cand[0]
        } else {
            cand[1]
        };
        let mut dir = *axis;
        if (inp.b - inp.a).dot(dir) < 0.0 {
            dir = -dir;
        }
        let origin = base + dir * (inp.a - base).dot(dir);
        return Some(Curve::Line { origin, dir });
    }
    let t = normal.dot(*po - *origin) / c;
    let center = *origin + *axis * t;
    let major_dir = (*axis - *normal * c).normalize();
    let ax = if winding(inp.chain, center, *normal) < 0.0 {
        -*normal
    } else {
        *normal
    };
    Some(Curve::Ellipse {
        center,
        axis: ax,
        xdir: major_dir,
        major: radius / c.abs(),
        minor: *radius,
    })
}

/// Cubic B-spline through points of the true intersection: the mesh chain
/// is resampled by arc length and every sample is moved onto both surfaces.
fn spline_intersection(inp: &EdgeInput) -> Result<Curve, String> {
    let chain = inp.chain;
    let segs = chain.len().saturating_sub(1).max(1);
    let m = segs.clamp(3, 32);
    let mut cum = vec![0.0f64; chain.len()];
    for i in 1..chain.len() {
        cum[i] = cum[i - 1] + (chain[i] - chain[i - 1]).length();
    }
    let total = *cum.last().unwrap_or(&0.0);
    let mut pts = vec![inp.a];
    for j in 1..m {
        let s = total * j as f64 / m as f64;
        let i = cum.partition_point(|&c| c < s).clamp(1, chain.len() - 1);
        let span = (cum[i] - cum[i - 1]).max(1e-300);
        let q = chain[i - 1] + (chain[i] - chain[i - 1]) * ((s - cum[i - 1]) / span);
        let (p, res) = project_onto(&[inp.sa, inp.sb], q, inp.scale * 0.05);
        if res > inp.dev_limit {
            return Err(format!(
                "the surfaces do not meet along the edge (gap {res:.3} mm)"
            ));
        }
        pts.push(p);
    }
    pts.push(if inp.closed { inp.a } else { inp.b });
    // Drop coincident samples (they would make the interpolation singular).
    let tiny = inp.scale * 1e-9;
    let mut clean: Vec<DVec3> = Vec::with_capacity(pts.len());
    for (i, &p) in pts.iter().enumerate() {
        let last = i == pts.len() - 1;
        match clean.last() {
            Some(&q) if (p - q).length() <= tiny && !last => {}
            Some(&q) if (p - q).length() <= tiny && last && clean.len() > 1 => {
                clean.pop();
                clean.push(p);
            }
            _ => clean.push(p),
        }
    }
    if clean.len() < 2 {
        return Err("degenerate edge".to_string());
    }
    interpolate(&clean).ok_or_else(|| "could not interpolate the edge curve".to_string())
}

/// Global cubic (lower degree for few points) B-spline interpolation with
/// chord-length parameters and averaged knots (The NURBS Book, 9.2.1).
pub(super) fn interpolate(points: &[DVec3]) -> Option<Curve> {
    let n = points.len() - 1;
    let deg = n.min(3);
    let mut u = vec![0.0f64; n + 1];
    for k in 1..=n {
        u[k] = u[k - 1] + (points[k] - points[k - 1]).length();
    }
    let total = u[n];
    for (k, v) in u.iter_mut().enumerate() {
        *v = if total > 0.0 {
            *v / total
        } else {
            k as f64 / n as f64
        };
    }
    u[n] = 1.0;
    let mut knots = vec![0.0f64; n + deg + 2];
    for k in 0..=deg {
        knots[n + 1 + k] = 1.0;
    }
    for j in 1..=(n - deg) {
        knots[j + deg] = (j..j + deg).map(|i| u[i]).sum::<f64>() / deg as f64;
    }
    let mut a = vec![vec![0.0f64; n + 1]; n + 1];
    for (k, row) in a.iter_mut().enumerate() {
        let s = find_span(&knots, n + 1, deg, u[k]);
        let (nb, _) = basis_ders(&knots, s, deg, u[k]);
        for i in 0..=deg {
            row[s - deg + i] = nb[i];
        }
    }
    let mut cps = vec![DVec3::ZERO; n + 1];
    for c in 0..3 {
        let rhs: Vec<f64> = points.iter().map(|p| p[c]).collect();
        let sol = solve_dense(a.clone(), rhs)?;
        for (i, v) in sol.into_iter().enumerate() {
            cps[i][c] = v;
        }
    }
    // Clamped ends: the curve starts and ends exactly at the vertices.
    cps[0] = points[0];
    cps[n] = points[n];
    Some(Curve::Spline {
        degree: deg,
        cps,
        knots,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolation_passes_through_points() {
        let pts: Vec<DVec3> = (0..9)
            .map(|i| {
                let t = i as f64 * 0.4;
                DVec3::new(t.cos() * 3.0, t.sin() * 3.0, t)
            })
            .collect();
        let c = interpolate(&pts).unwrap();
        for p in &pts {
            assert!(c.distance(*p) < 1e-6, "{}", c.distance(*p));
        }
    }

    #[test]
    fn plane_meets_coaxial_cylinder_in_a_circle() {
        let plane = Surface::Plane {
            origin: DVec3::new(0.0, 0.0, 2.0),
            normal: DVec3::Z,
        };
        let cyl = Surface::Cylinder {
            origin: DVec3::ZERO,
            axis: DVec3::Z,
            radius: 4.0,
        };
        let chain: Vec<DVec3> = (0..=16)
            .map(|i| {
                let t = i as f64 / 16.0 * TAU;
                DVec3::new(4.0 * t.cos(), 4.0 * t.sin(), 2.0)
            })
            .collect();
        let inp = EdgeInput {
            sa: &plane,
            sb: &cyl,
            chain: &chain,
            a: chain[0],
            b: chain[0],
            closed: true,
            scale: 10.0,
            dev_limit: 0.1,
        };
        let c = intersect(&inp).unwrap();
        let Curve::Circle {
            center,
            axis,
            radius,
            ..
        } = c
        else {
            panic!("expected a circle, got {}", c.label());
        };
        assert!((center - DVec3::new(0.0, 0.0, 2.0)).length() < 1e-12);
        assert!((radius - 4.0).abs() < 1e-12);
        // The chain runs counter-clockwise around +Z.
        assert!(axis.dot(DVec3::Z) > 0.99);
    }
}
