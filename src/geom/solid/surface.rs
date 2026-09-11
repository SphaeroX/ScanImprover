//! Face surfaces of the reconstructed solid: analytic primitives (plane,
//! cylinder, cone, sphere) and bicubic B-spline patches, with their robust
//! least-squares fits.
//!
//! Every surface has a *natural normal* (the normal of its STEP
//! parametrisation): the plane normal, radially outwards for cylinders,
//! cones and spheres, `S_u × S_v` for B-spline patches. Signed distances are
//! positive on the side the natural normal points to.

use super::numeric::{basis_ders, find_span, levenberg_marquardt};
use crate::geom::fitting::eigen_3x3;
use crate::geom::freeform::{
    BicubicNet, FreeformGrid, FreeformParams, bicubic_control_net, fit_freeform_grid,
};
use glam::DVec3;

/// Upper bound on the points used by the iterative fits (the deviation
/// report always uses every vertex).
pub(super) const MAX_FIT_POINTS: usize = 4000;

#[derive(Clone)]
pub enum Surface {
    Plane {
        origin: DVec3,
        normal: DVec3,
    },
    Cylinder {
        origin: DVec3,
        axis: DVec3,
        radius: f64,
    },
    /// Radius `radius` at `origin`, growing by `tan(half_angle)` per unit
    /// along `axis`.
    Cone {
        origin: DVec3,
        axis: DVec3,
        radius: f64,
        half_angle: f64,
    },
    Sphere {
        center: DVec3,
        radius: f64,
    },
    Spline(Box<SplineSurface>),
}

/// Two unit vectors completing `a` to a right-handed orthonormal frame.
pub(super) fn perp_basis(a: DVec3) -> (DVec3, DVec3) {
    let up = if a.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
    let u = a.cross(up).normalize_or_zero();
    let v = a.cross(u).normalize_or_zero();
    (u, v)
}

/// Unit radial direction of `d` perpendicular to `axis` (any perpendicular
/// direction on the axis itself).
fn radial_dir(d: DVec3, axis: DVec3) -> (DVec3, f64) {
    let r = d - axis * d.dot(axis);
    let len = r.length();
    if len > 1e-300 {
        (r / len, len)
    } else {
        (perp_basis(axis).0, 0.0)
    }
}

impl Surface {
    pub fn label(&self) -> &'static str {
        match self {
            Surface::Plane { .. } => "Plane",
            Surface::Cylinder { .. } => "Cylinder",
            Surface::Cone { .. } => "Cone",
            Surface::Sphere { .. } => "Sphere",
            Surface::Spline(_) => "B-spline",
        }
    }

    /// Index into the surface counters of the report
    /// (plane, cylinder, cone, sphere, B-spline).
    pub fn kind_index(&self) -> usize {
        match self {
            Surface::Plane { .. } => 0,
            Surface::Cylinder { .. } => 1,
            Surface::Cone { .. } => 2,
            Surface::Sphere { .. } => 3,
            Surface::Spline(_) => 4,
        }
    }

    /// Axis line (point, unit direction) of a surface of revolution.
    pub fn axis(&self) -> Option<(DVec3, DVec3)> {
        match self {
            Surface::Cylinder { origin, axis, .. } | Surface::Cone { origin, axis, .. } => {
                Some((*origin, *axis))
            }
            _ => None,
        }
    }

    pub fn signed_distance(&self, p: DVec3) -> f64 {
        match self {
            Surface::Plane { origin, normal } => (p - *origin).dot(*normal),
            Surface::Cylinder {
                origin,
                axis,
                radius,
            } => radial_dir(p - *origin, *axis).1 - radius,
            Surface::Cone {
                origin,
                axis,
                radius,
                half_angle,
            } => {
                let d = p - *origin;
                let h = d.dot(*axis);
                let rho = radial_dir(d, *axis).1;
                rho * half_angle.cos() - radius * half_angle.cos() - h * half_angle.sin()
            }
            Surface::Sphere { center, radius } => (p - *center).length() - radius,
            Surface::Spline(s) => {
                let (q, n) = s.closest(p);
                (p - q).dot(n)
            }
        }
    }

    /// Unit natural normal at the surface point closest to `p` (the
    /// gradient of the signed distance).
    pub fn gradient(&self, p: DVec3) -> DVec3 {
        match self {
            Surface::Plane { normal, .. } => *normal,
            Surface::Cylinder { origin, axis, .. } => radial_dir(p - *origin, *axis).0,
            Surface::Cone {
                origin,
                axis,
                half_angle,
                ..
            } => {
                let e = radial_dir(p - *origin, *axis).0;
                e * half_angle.cos() - *axis * half_angle.sin()
            }
            Surface::Sphere { center, .. } => {
                let d = p - *center;
                if d.length_squared() > 1e-300 {
                    d.normalize()
                } else {
                    DVec3::Z
                }
            }
            Surface::Spline(s) => s.closest(p).1,
        }
    }
}

// ---------------------------------------------------------------------------
// B-spline patch
// ---------------------------------------------------------------------------

/// A clamped bicubic B-spline patch (from the freeform grid fit) with the
/// fit-plane frame used to seed closest-point queries.
#[derive(Clone)]
pub struct SplineSurface {
    /// Control net as written to STEP.
    pub net: BicubicNet,
    cps: Vec<Vec<DVec3>>,
    origin: DVec3,
    u: DVec3,
    v: DVec3,
    x0: f64,
    y0: f64,
    sx: f64,
    sy: f64,
}

impl SplineSurface {
    fn new(net: BicubicNet, grid: &FreeformGrid) -> SplineSurface {
        let cps = net
            .cps
            .iter()
            .map(|row| row.iter().map(|p| p.as_dvec3()).collect())
            .collect();
        SplineSurface {
            cps,
            origin: grid.origin.as_dvec3(),
            u: grid.u.as_dvec3(),
            v: grid.v.as_dvec3(),
            x0: grid.x0 as f64,
            y0: grid.y0 as f64,
            sx: (grid.nx as f64 * grid.gw as f64).max(1e-12),
            sy: (grid.ny as f64 * grid.gh as f64).max(1e-12),
            net,
        }
    }

    /// Point and first partial derivatives at (u, v) in 0..=1.
    fn eval(&self, u: f64, v: f64) -> (DVec3, DVec3, DVec3) {
        let (nu, nv) = (self.net.nx + 1, self.net.ny + 1);
        let (pu, pv) = (self.net.deg_u, self.net.deg_v);
        let su = find_span(&self.net.u_knots_full, nu, pu, u);
        let sv = find_span(&self.net.v_knots_full, nv, pv, v);
        let (bu, du) = basis_ders(&self.net.u_knots_full, su, pu, u);
        let (bv, dv) = basis_ders(&self.net.v_knots_full, sv, pv, v);
        let (mut s, mut s_u, mut s_v) = (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
        for b in 0..=pv {
            let row = &self.cps[sv - pv + b];
            for a in 0..=pu {
                let p = row[su - pu + a];
                s += p * (bu[a] * bv[b]);
                s_u += p * (du[a] * bv[b]);
                s_v += p * (bu[a] * dv[b]);
            }
        }
        (s, s_u, s_v)
    }

    /// Closest surface point and the unit natural normal there
    /// (Gauss–Newton on the parameters, seeded from the fit plane).
    fn closest(&self, p: DVec3) -> (DVec3, DVec3) {
        let d = p - self.origin;
        let mut u = ((d.dot(self.u) - self.x0) / self.sx).clamp(0.0, 1.0);
        let mut v = ((d.dot(self.v) - self.y0) / self.sy).clamp(0.0, 1.0);
        let (mut s, mut s_u, mut s_v) = self.eval(u, v);
        for _ in 0..24 {
            let f = s - p;
            let (a11, a12, a22) = (s_u.dot(s_u), s_u.dot(s_v), s_v.dot(s_v));
            let (b1, b2) = (-s_u.dot(f), -s_v.dot(f));
            let det = a11 * a22 - a12 * a12;
            if det.abs() < 1e-300 {
                break;
            }
            let du = (b1 * a22 - b2 * a12) / det;
            let dv = (a11 * b2 - a12 * b1) / det;
            let (nu, nv) = ((u + du).clamp(0.0, 1.0), (v + dv).clamp(0.0, 1.0));
            let moved = (nu - u).abs() + (nv - v).abs();
            u = nu;
            v = nv;
            (s, s_u, s_v) = self.eval(u, v);
            if moved < 1e-13 {
                break;
            }
        }
        let n = s_u.cross(s_v).normalize_or_zero();
        (s, if n == DVec3::ZERO { DVec3::Z } else { n })
    }
}

// ---------------------------------------------------------------------------
// Fitting
// ---------------------------------------------------------------------------

/// Every `k`-th point so at most `max` remain.
pub(super) fn subsample(pts: &[DVec3], max: usize) -> Vec<DVec3> {
    let stride = pts.len().div_ceil(max.max(1)).max(1);
    pts.iter().step_by(stride).copied().collect()
}

/// Points whose |residual| is within 3 robust standard deviations (median
/// absolute deviation); all points when that would drop too many.
fn trimmed(s: &Surface, pts: &[DVec3]) -> Vec<DVec3> {
    if pts.len() < 20 {
        return pts.to_vec();
    }
    let mut r: Vec<f64> = pts.iter().map(|&p| s.signed_distance(p).abs()).collect();
    let mid = r.len() / 2;
    let mad = *r.select_nth_unstable_by(mid, |a, b| a.total_cmp(b)).1;
    let cut = (3.0 * 1.4826 * mad).max(1e-12);
    let kept: Vec<DVec3> = pts
        .iter()
        .copied()
        .filter(|&p| s.signed_distance(p).abs() <= cut)
        .collect();
    if kept.len() * 10 >= pts.len() * 8 {
        kept
    } else {
        pts.to_vec()
    }
}

/// RMS of the signed distances.
pub(super) fn rms_of(s: &Surface, pts: &[DVec3]) -> f64 {
    if pts.is_empty() {
        return 0.0;
    }
    (pts.iter()
        .map(|&p| s.signed_distance(p).powi(2))
        .sum::<f64>()
        / pts.len() as f64)
        .sqrt()
}

/// Least-squares plane (PCA), refitted once on the trimmed points.
pub(super) fn fit_plane(pts: &[DVec3]) -> Option<Surface> {
    let pca = |pts: &[DVec3]| -> Option<Surface> {
        if pts.len() < 3 {
            return None;
        }
        let c = pts.iter().fold(DVec3::ZERO, |a, &p| a + p) / pts.len() as f64;
        let mut m = [[0.0f64; 3]; 3];
        for &p in pts {
            let d = (p - c).to_array();
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] += d[i] * d[j];
                }
            }
        }
        let (_, ev) = eigen_3x3(m)[0];
        let n = DVec3::from_array(ev).normalize_or_zero();
        (n != DVec3::ZERO).then_some(Surface::Plane {
            origin: c,
            normal: n,
        })
    };
    let first = pca(pts)?;
    pca(&trimmed(&first, pts)).or(Some(first))
}

/// Which parameters of a refit are held fixed.
#[derive(Clone, Copy, Default)]
pub(super) struct Freeze {
    /// Plane normal / axis direction.
    pub dir: bool,
    /// Axis position (cylinder, cone) or sphere center.
    pub pos: bool,
}

/// Robust least-squares refit of an analytic surface, starting from `seed`.
/// B-spline patches are returned unchanged.
pub(super) fn refit(seed: &Surface, pts: &[DVec3], freeze: Freeze) -> Surface {
    let once = |seed: &Surface, pts: &[DVec3]| -> Surface {
        match seed {
            Surface::Plane { normal, .. } => {
                if freeze.dir {
                    if freeze.pos || pts.is_empty() {
                        return seed.clone();
                    }
                    let c = pts.iter().fold(DVec3::ZERO, |a, &p| a + p) / pts.len() as f64;
                    Surface::Plane {
                        origin: c,
                        normal: *normal,
                    }
                } else {
                    fit_plane(pts).map_or_else(|| seed.clone(), |s| orient_like(s, seed))
                }
            }
            Surface::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let (a0, o0, r0) = (*axis, *origin, *radius);
                let (e1, e2) = perp_basis(a0);
                let model = move |x: &[f64]| {
                    (
                        (a0 + e1 * x[0] + e2 * x[1]).normalize(),
                        o0 + e1 * x[2] + e2 * x[3],
                        r0 + x[4],
                    )
                };
                let mut x = [0.0; 5];
                let frozen = [freeze.dir, freeze.dir, freeze.pos, freeze.pos, false];
                levenberg_marquardt(
                    &mut x,
                    &frozen,
                    pts,
                    |x, p| {
                        let (a, o, r) = model(x);
                        radial_dir(p - o, a).1 - r
                    },
                    60,
                );
                let (axis, origin, radius) = model(&x);
                Surface::Cylinder {
                    origin,
                    axis,
                    radius: radius.abs(),
                }
            }
            Surface::Cone {
                origin,
                axis,
                radius,
                half_angle,
            } => {
                let (a0, o0, r0, h0) = (*axis, *origin, *radius, *half_angle);
                let (e1, e2) = perp_basis(a0);
                let model = move |x: &[f64]| {
                    (
                        (a0 + e1 * x[0] + e2 * x[1]).normalize(),
                        o0 + e1 * x[2] + e2 * x[3],
                        r0 + x[4],
                        h0 + x[5],
                    )
                };
                let mut x = [0.0; 6];
                let frozen = [freeze.dir, freeze.dir, freeze.pos, freeze.pos, false, false];
                levenberg_marquardt(
                    &mut x,
                    &frozen,
                    pts,
                    |x, p| {
                        let (a, o, r, h) = model(x);
                        let d = p - o;
                        let rho = radial_dir(d, a).1;
                        (rho - r - d.dot(a) * h.tan()) * h.cos()
                    },
                    60,
                );
                let (axis, origin, radius, half_angle) = model(&x);
                Surface::Cone {
                    origin,
                    axis,
                    radius,
                    half_angle,
                }
            }
            Surface::Sphere { center, radius } => {
                let (c0, r0) = (*center, *radius);
                let mut x = [0.0; 4];
                let frozen = [freeze.pos, freeze.pos, freeze.pos, false];
                levenberg_marquardt(
                    &mut x,
                    &frozen,
                    pts,
                    |x, p| (p - c0 - DVec3::new(x[0], x[1], x[2])).length() - (r0 + x[3]),
                    60,
                );
                Surface::Sphere {
                    center: c0 + DVec3::new(x[0], x[1], x[2]),
                    radius: (r0 + x[3]).abs(),
                }
            }
            Surface::Spline(_) => seed.clone(),
        }
    };
    let first = once(seed, pts);
    let kept = trimmed(&first, pts);
    if kept.len() == pts.len() {
        first
    } else {
        once(&first, &kept)
    }
}

/// Keeps the plane normal pointing the same way as the reference plane.
fn orient_like(s: Surface, reference: &Surface) -> Surface {
    match (s, reference) {
        (Surface::Plane { origin, normal }, Surface::Plane { normal: n0, .. }) => Surface::Plane {
            origin,
            normal: if normal.dot(*n0) < 0.0 {
                -normal
            } else {
                normal
            },
        },
        (s, _) => s,
    }
}

/// Cone fit for a group the segmentation left as freeform. `normals` are
/// (point, unit normal, area) samples of the group's triangles.
///
/// Seed: the normals of a cone lie on a circle of the unit sphere, so the
/// axis is the smallest-variance direction of the normal covariance; the
/// projected normal lines all pass through the axis; the radius grows
/// linearly along it. A Levenberg–Marquardt refit follows.
pub(super) fn fit_cone(pts: &[DVec3], normals: &[(DVec3, DVec3, f64)]) -> Option<Surface> {
    if pts.len() < 12 || normals.len() < 6 {
        return None;
    }
    let wsum: f64 = normals.iter().map(|t| t.2).sum();
    if wsum <= 0.0 {
        return None;
    }
    let mean_n = normals.iter().fold(DVec3::ZERO, |a, t| a + t.1 * t.2) / wsum;
    let mut cov = [[0.0f64; 3]; 3];
    for (_, n, w) in normals {
        let d = (*n - mean_n).to_array();
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] += w * d[i] * d[j];
            }
        }
    }
    let eig = eigen_3x3(cov);
    // A cone's normals span a circle: two clearly non-zero eigenvalues.
    if eig[1].0 <= eig[2].0 * 1e-3 {
        return None;
    }
    let mut axis = DVec3::from_array(eig[0].1).normalize_or_zero();
    if axis == DVec3::ZERO {
        return None;
    }
    let centroid = pts.iter().fold(DVec3::ZERO, |a, &p| a + p) / pts.len() as f64;
    // Least-squares intersection of the normal lines projected along the axis.
    let (e1, e2) = perp_basis(axis);
    let (mut m11, mut m12, mut m22, mut b1, mut b2) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (p, n, w) in normals {
        let n2 = DVec3::new(n.dot(e1), n.dot(e2), 0.0).normalize_or_zero();
        if n2 == DVec3::ZERO {
            continue;
        }
        let (x, y) = ((*p - centroid).dot(e1), (*p - centroid).dot(e2));
        // Projector onto the line normal: I - n n^T.
        let (p11, p12, p22) = (1.0 - n2.x * n2.x, -n2.x * n2.y, 1.0 - n2.y * n2.y);
        m11 += w * p11;
        m12 += w * p12;
        m22 += w * p22;
        b1 += w * (p11 * x + p12 * y);
        b2 += w * (p12 * x + p22 * y);
    }
    let det = m11 * m22 - m12 * m12;
    if det.abs() < 1e-300 {
        return None;
    }
    let qx = (b1 * m22 - b2 * m12) / det;
    let qy = (m11 * b2 - m12 * b1) / det;
    let origin = centroid + e1 * qx + e2 * qy;
    // Linear radius profile rho = r + h * k.
    let (mut sh, mut sr, mut shh, mut shr) = (0.0, 0.0, 0.0, 0.0);
    for &p in pts {
        let d = p - origin;
        let (h, rho) = (d.dot(axis), radial_dir(d, axis).1);
        sh += h;
        sr += rho;
        shh += h * h;
        shr += h * rho;
    }
    let n = pts.len() as f64;
    let den = n * shh - sh * sh;
    if den.abs() < 1e-300 {
        return None;
    }
    let mut k = (n * shr - sh * sr) / den;
    let r = (sr - k * sh) / n;
    if k < 0.0 {
        axis = -axis;
        k = -k;
    }
    if r <= 0.0 {
        return None;
    }
    let seed = Surface::Cone {
        origin,
        axis,
        radius: r,
        half_angle: k.atan(),
    };
    let s = refit(&seed, pts, Freeze::default());
    match s {
        Surface::Cone {
            radius, half_angle, ..
        } if radius > 0.0 && half_angle > 1e-4 && half_angle < 1.45 => Some(s),
        _ => None,
    }
}

/// Bicubic B-spline patch over the group, extended beyond its boundary so
/// the neighbouring faces intersect it inside the patch.
pub(super) fn fit_spline(pts: &[DVec3]) -> Result<Surface, String> {
    let f32pts: Vec<[f32; 3]> = pts.iter().map(|p| p.as_vec3().to_array()).collect();
    let (mn, mx) = pts.iter().fold(
        (DVec3::splat(f64::MAX), DVec3::splat(f64::MIN)),
        |(a, b), &p| (a.min(p), b.max(p)),
    );
    let extent = (mx - mn).length().max(1e-6);
    let mut params = FreeformParams::default_for(extent as f32);
    params.overshoot_mm = (extent * 0.2) as f32;
    params.resolution = 48;
    params.smoothness = 1;
    let grid = fit_freeform_grid(&f32pts, &params)?;
    let net = bicubic_control_net(&grid);
    Ok(Surface::Spline(Box::new(SplineSurface::new(net, &grid))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cone_fit_recovers_parameters() {
        let axis = DVec3::new(0.2, 0.1, 1.0).normalize();
        let origin = DVec3::new(1.0, -2.0, 3.0);
        let (e1, e2) = perp_basis(axis);
        let (r0, alpha) = (5.0f64, 0.4f64);
        let mut pts = Vec::new();
        let mut normals = Vec::new();
        for i in 0..40 {
            for j in 0..10 {
                let t = i as f64 / 40.0 * std::f64::consts::TAU;
                let h = j as f64 * 0.8;
                let rho = r0 + h * alpha.tan();
                let e = e1 * t.cos() + e2 * t.sin();
                pts.push(origin + axis * h + e * rho);
                normals.push((
                    origin + axis * h + e * rho,
                    e * alpha.cos() - axis * alpha.sin(),
                    1.0,
                ));
            }
        }
        let s = fit_cone(&pts, &normals).expect("cone");
        assert!(rms_of(&s, &pts) < 1e-6, "rms {}", rms_of(&s, &pts));
        let Surface::Cone {
            axis: a,
            half_angle,
            ..
        } = s
        else {
            panic!("not a cone")
        };
        assert!(a.dot(axis) > 1.0 - 1e-9);
        assert!((half_angle - alpha).abs() < 1e-6);
    }

    #[test]
    fn cylinder_refit_with_frozen_axis_keeps_direction() {
        let pts: Vec<DVec3> = (0..200)
            .map(|i| {
                let t = i as f64 * 0.37;
                DVec3::new(2.0 + 3.0 * t.cos(), -1.0 + 3.0 * t.sin(), (i % 13) as f64)
            })
            .collect();
        let seed = Surface::Cylinder {
            origin: DVec3::new(2.2, -0.9, 0.0),
            axis: DVec3::Z,
            radius: 2.5,
        };
        let s = refit(
            &seed,
            &pts,
            Freeze {
                dir: true,
                pos: false,
            },
        );
        let Surface::Cylinder {
            origin,
            axis,
            radius,
        } = s
        else {
            panic!()
        };
        assert_eq!(axis, DVec3::Z);
        assert!((radius - 3.0).abs() < 1e-6);
        assert!((origin.truncate() - glam::DVec2::new(2.0, -1.0)).length() < 1e-6);
    }
}
