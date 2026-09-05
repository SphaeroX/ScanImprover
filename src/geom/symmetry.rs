use crate::geom::bvh::Bvh;
use crate::mesh::Mesh;
use crate::rng::Rng;
use glam::Vec3;
use rayon::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct SymPlane {
    pub normal: Vec3,
    pub point: Vec3,
}

pub fn reflect(p: Vec3, sp: &SymPlane) -> Vec3 {
    let d = (p - sp.point).dot(sp.normal);
    p - sp.normal * (2.0 * d)
}

fn plane_from_params(theta: f64, phi: f64, d: f64, center: Vec3) -> SymPlane {
    let ct = theta.cos();
    let n = Vec3::new(
        (ct * phi.cos()) as f32,
        theta.sin() as f32,
        (ct * phi.sin()) as f32,
    );
    let n = if n.length_squared() > 1e-12 {
        n.normalize()
    } else {
        Vec3::Y
    };
    SymPlane {
        normal: n,
        point: center + n * d as f32,
    }
}

fn params_from_plane(sp: &SymPlane, center: Vec3) -> (f64, f64, f64) {
    let n = sp.normal;
    let ny = (n.y as f64).clamp(-1.0, 1.0);
    let theta = ny.asin();
    let phi = if theta.abs() > 1.56 {
        0.0f64
    } else {
        (n.z as f64).atan2(n.x as f64)
    };
    let d = (sp.point - center).dot(n) as f64;
    (theta, phi, d)
}

/// Samples surface points, ignoring triangles where mask[t] > 0.
pub fn sample_surface_masked(
    mesh: &Mesh,
    n: usize,
    mask: Option<&[u8]>,
    rng: &mut Rng,
) -> Vec<Vec3> {
    let nt = mesh.triangle_count();
    if nt == 0 || n == 0 {
        return Vec::new();
    }
    let mut cum = Vec::with_capacity(nt + 1);
    cum.push(0.0f64);
    for t in 0..nt {
        let is_excluded = mask.map_or(false, |m| m.get(t).copied().unwrap_or(0) > 0);
        let area = if is_excluded {
            0.0f64
        } else {
            let [a, b, c] = mesh.triangle(t);
            (b - a).cross(c - a).length() as f64 * 0.5
        };
        cum.push(cum[t] + area);
    }
    let total = cum[nt];
    if total <= 1e-12 {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let r = rng.f64() * total;
        let t = match cum.partition_point(|x| *x < r) {
            i if i >= nt => nt - 1,
            i => i.saturating_sub(1).max(0),
        };
        let [a, b, c] = mesh.triangle(t);
        let mut u = rng.f32();
        let mut v = rng.f32();
        if u + v > 1.0 {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        out.push(a + (b - a) * u + (c - a) * v);
    }
    out
}

/// Evaluates robust loss (truncated quadratic) for a candidate symmetry plane.
pub fn robust_sym_loss(
    bvh: &Bvh,
    samples: &[Vec3],
    plane: &SymPlane,
    mask: Option<&[u8]>,
    c_cutoff: f32,
) -> f64 {
    if samples.is_empty() {
        return f64::INFINITY;
    }
    let c_sq = (c_cutoff * c_cutoff) as f64;
    let sum_loss: f64 = samples
        .par_iter()
        .map(|&s| {
            let r = reflect(s, plane);
            let (_, hit_tri, dist) = bvh.closest_point(r);
            let hit_excluded =
                mask.map_or(false, |m| m.get(hit_tri as usize).copied().unwrap_or(0) > 0);
            if hit_excluded {
                c_sq
            } else {
                let d2 = (dist * dist) as f64;
                d2.min(c_sq)
            }
        })
        .sum();
    sum_loss / samples.len() as f64
}

/// Computes the RMS deviation of mirrored points against the mesh.
pub fn compute_sym_rms(
    bvh: &Bvh,
    samples: &[Vec3],
    plane: &SymPlane,
    mask: Option<&[u8]>,
    c_cutoff: f32,
) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let c_sq = (c_cutoff * c_cutoff) as f64;
    let (sum_sq, count): (f64, usize) = samples
        .par_iter()
        .map(|&s| {
            let r = reflect(s, plane);
            let (_, hit_tri, dist) = bvh.closest_point(r);
            let hit_excluded =
                mask.map_or(false, |m| m.get(hit_tri as usize).copied().unwrap_or(0) > 0);
            let d2 = (dist * dist) as f64;
            if !hit_excluded && d2 <= c_sq {
                (d2, 1usize)
            } else {
                (0.0, 0usize)
            }
        })
        .reduce(|| (0.0, 0), |a, b| (a.0 + b.0, a.1 + b.1));

    if count > 0 {
        (sum_sq / count as f64).sqrt()
    } else {
        c_cutoff as f64
    }
}


fn nelder_mead(
    f: &dyn Fn([f64; 3]) -> f64,
    x0: [f64; 3],
    step: [f64; 3],
    max_iter: usize,
) -> [f64; 3] {
    let mut pts: [[f64; 3]; 4] = [x0, x0, x0, x0];
    pts[1][0] += step[0];
    pts[2][1] += step[1];
    pts[3][2] += step[2];
    let mut vals: [f64; 4] = std::array::from_fn(|i| f(pts[i]));
    for _ in 0..max_iter {
        let mut order: [usize; 4] = [0, 1, 2, 3];
        order.sort_by(|&a, &b| vals[a].partial_cmp(&vals[b]).unwrap());
        let best = order[0];
        let worst = order[3];
        if (vals[worst] - vals[best]).abs() < 1e-9 * vals[best].abs().max(1e-12) {
            break;
        }
        let mut c = [0.0f64; 3];
        for i in 0..3 {
            for a in 0..3 {
                c[a] += pts[order[i]][a];
            }
        }
        for a in 0..3 {
            c[a] /= 3.0;
        }
        let mut xr = c;
        for a in 0..3 {
            xr[a] = c[a] + (c[a] - pts[worst][a]);
        }
        let fr = f(xr);
        if fr < vals[best] {
            let mut xe = c;
            for a in 0..3 {
                xe[a] = c[a] + 2.0 * (c[a] - pts[worst][a]);
            }
            let fe = f(xe);
            if fe < fr {
                pts[worst] = xe;
                vals[worst] = fe;
            } else {
                pts[worst] = xr;
                vals[worst] = fr;
            }
        } else if fr < vals[order[2]] {
            pts[worst] = xr;
            vals[worst] = fr;
        } else {
            let mut xc = c;
            for a in 0..3 {
                xc[a] = c[a] + 0.5 * (pts[worst][a] - c[a]);
            }
            let fc = f(xc);
            if fc < vals[worst] {
                pts[worst] = xc;
                vals[worst] = fc;
            } else {
                for i in 1..4 {
                    let oi = order[i];
                    for a in 0..3 {
                        pts[oi][a] = pts[best][a] + 0.5 * (pts[oi][a] - pts[best][a]);
                    }
                    vals[oi] = f(pts[oi]);
                }
            }
        }
    }
    let mut order: [usize; 4] = [0, 1, 2, 3];
    order.sort_by(|&a, &b| vals[a].partial_cmp(&vals[b]).unwrap());
    pts[order[0]]
}

/// Detects the optimal symmetry plane containing a user-specified 3D line segment (A - B).
pub fn detect_symmetry_from_line(
    mesh: &Mesh,
    bvh: &Bvh,
    a: Vec3,
    b: Vec3,
    mask: Option<&[u8]>,
) -> Option<(SymPlane, f64)> {
    let bbox = mesh.bbox();
    let diag = bbox.diagonal();
    let diff = b - a;
    let len = diff.length();
    if len < diag * 1e-5 {
        return None;
    }
    let u = diff / len;
    let p0 = (a + b) * 0.5;

    let v_ref = if u.x.abs() < 0.9 {
        Vec3::X
    } else {
        Vec3::Y
    };
    let e1 = u.cross(v_ref).normalize();
    let e2 = u.cross(e1).normalize();

    let mut rng = Rng::new(0xBEEF_1234);
    let coarse = sample_surface_masked(mesh, 768, mask, &mut rng);
    let fine = sample_surface_masked(mesh, 4096, mask, &mut rng);
    if coarse.is_empty() {
        return None;
    }

    let c_coarse = diag * 0.035;
    let c_fine = diag * 0.018;

    // 1. Global 1D angular sweep over alpha in [0, pi)
    let num_steps = 90; // 2 degree steps
    let step_rad = std::f64::consts::PI / (num_steps as f64);
    let mut best_sweep_err = f64::INFINITY;
    let mut sweep_plane = SymPlane { normal: e1, point: p0 };

    for i in 0..num_steps {
        let alpha = (i as f64) * step_rad;
        let cos_a = alpha.cos() as f32;
        let sin_a = alpha.sin() as f32;
        let normal = (e1 * cos_a + e2 * sin_a).normalize();
        let sp = SymPlane { normal, point: p0 };
        let err = robust_sym_loss(bvh, &coarse, &sp, mask, c_coarse);
        if err < best_sweep_err {
            best_sweep_err = err;
            sweep_plane = sp;
        }
    }

    // 2. 3-DOF local refinement around sweep result:
    // delta_alpha (roll around u), delta_beta (tilt around v0), delta_d (normal offset along n0)
    let n0 = sweep_plane.normal;
    let v0 = u.cross(n0).normalize();
    let max_angle = 6.0f64.to_radians();
    let max_offset = (diag * 0.02) as f64;

    let eval_fn = |p: [f64; 3]| -> f64 {
        let da = p[0];
        let db = p[1];
        let dd = p[2];

        let mut penalty = 0.0f64;
        if da.abs() > max_angle {
            let exc = da.abs() - max_angle;
            penalty += exc * exc * 1e4;
        }
        if db.abs() > max_angle {
            let exc = db.abs() - max_angle;
            penalty += exc * exc * 1e4;
        }
        if dd.abs() > max_offset {
            let exc = dd.abs() - max_offset;
            penalty += exc * exc * 1e4;
        }

        let n = (n0 + v0 * (da.tan() as f32) + u * (db.tan() as f32)).normalize();
        let pt = p0 + n0 * (dd as f32);
        let plane = SymPlane { normal: n, point: pt };
        robust_sym_loss(bvh, &fine, &plane, mask, c_fine) + penalty
    };

    let step = [0.005, 0.005, (diag * 0.001) as f64];
    let opt = nelder_mead(&eval_fn, [0.0, 0.0, 0.0], step, 160);

    let mut final_n = (n0 + v0 * (opt[0].tan() as f32) + u * (opt[1].tan() as f32)).normalize();
    let final_pt = p0 + n0 * (opt[2] as f32);

    // Canonicalize orientation relative to e1
    if final_n.dot(e1) < 0.0 {
        final_n = -final_n;
    }

    let final_plane = SymPlane {
        normal: final_n,
        point: final_pt,
    };
    let rms = compute_sym_rms(bvh, &fine, &final_plane, mask, c_fine);
    Some((final_plane, rms))
}

/// Refines an existing symmetry plane candidate using robust loss and an optional exclusion mask.
pub fn refine_symmetry_masked(
    mesh: &Mesh,
    bvh: &Bvh,
    init: &SymPlane,
    mask: Option<&[u8]>,
) -> (SymPlane, f64) {
    let bbox = mesh.bbox();
    let center = bbox.center();
    let diag = bbox.diagonal();
    let mut rng = Rng::new(0xBEEF);
    let samples = sample_surface_masked(mesh, 8192, mask, &mut rng);
    if samples.is_empty() {
        return (*init, 0.0);
    }
    let c_fine = diag * 0.02;
    let (t0, p0, d0) = params_from_plane(init, center);
    let x = nelder_mead(
        &|p: [f64; 3]| {
            let sp = plane_from_params(p[0], p[1], p[2], center);
            robust_sym_loss(bvh, &samples, &sp, mask, c_fine)
        },
        [t0, p0, d0],
        [0.02, 0.02, (diag * 0.01) as f64],
        240,
    );
    let final_plane = plane_from_params(x[0], x[1], x[2], center);
    let rms = compute_sym_rms(bvh, &samples, &final_plane, mask, c_fine);
    (final_plane, rms)
}

/// Automatically detects the best symmetry plane using robust loss and an optional exclusion mask.
pub fn detect_symmetry_masked(
    mesh: &Mesh,
    bvh: &Bvh,
    mask: Option<&[u8]>,
) -> Option<(SymPlane, f64)> {
    let bbox = mesh.bbox();
    let center = bbox.center();
    let diag = bbox.diagonal();
    let mut rng = Rng::new(0xCAFE);
    let coarse = sample_surface_masked(mesh, 768, mask, &mut rng);
    let fine = sample_surface_masked(mesh, 6144, mask, &mut rng);
    if coarse.is_empty() {
        return None;
    }
    let c_coarse = diag * 0.04;
    let c_fine = diag * 0.02;

    let mut c = [0.0f64; 3];
    for p in &coarse {
        for a in 0..3 {
            c[a] += p[a] as f64;
        }
    }
    for a in 0..3 {
        c[a] /= coarse.len() as f64;
    }
    let mut m = [[0.0f64; 3]; 3];
    for p in &coarse {
        let d = [
            p.x as f64 - c[0],
            p.y as f64 - c[1],
            p.z as f64 - c[2],
        ];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += d[i] * d[j];
            }
        }
    }
    let eig = crate::geom::fitting::eigen_3x3(m);
    let mut candidates: Vec<Vec3> = Vec::new();
    for e in &eig {
        let v = Vec3::new(e.1[0] as f32, e.1[1] as f32, e.1[2] as f32);
        if v.length_squared() > 1e-12 {
            let v = v.normalize();
            candidates.push(v);
            candidates.push(-v);
        }
    }
    candidates.push(Vec3::X);
    candidates.push(-Vec3::X);
    candidates.push(Vec3::Y);
    candidates.push(-Vec3::Y);
    candidates.push(Vec3::Z);
    candidates.push(-Vec3::Z);

    let mut scored: Vec<(f64, Vec3)> = candidates
        .into_iter()
        .map(|n| {
            let sp = SymPlane {
                normal: n,
                point: center,
            };
            (robust_sym_loss(bvh, &coarse, &sp, mask, c_coarse), n)
        })
        .collect();
    scored.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let d_step = diag * 0.01;
    let mut best: Option<(SymPlane, f64)> = None;
    for (_, n) in scored.into_iter().take(3) {
        let init = SymPlane {
            normal: n,
            point: center,
        };
        let (t0, p0, d0) = params_from_plane(&init, center);
        let x = nelder_mead(
            &|p: [f64; 3]| {
                let sp = plane_from_params(p[0], p[1], p[2], center);
                robust_sym_loss(bvh, &fine, &sp, mask, c_fine)
            },
            [t0, p0, d0],
            [0.02, 0.02, d_step as f64],
            220,
        );
        let cand_plane = plane_from_params(x[0], x[1], x[2], center);
        let loss = robust_sym_loss(bvh, &fine, &cand_plane, mask, c_fine);
        match &best {
            Some((_, e)) if *e <= loss => {}
            _ => best = Some((cand_plane, loss)),
        }
    }
    best.map(|(plane, _)| {
        let rms = compute_sym_rms(bvh, &fine, &plane, mask, c_fine);
        (plane, rms)
    })
}

pub fn refine_symmetry(mesh: &Mesh, bvh: &Bvh, init: &SymPlane) -> (SymPlane, f64) {
    refine_symmetry_masked(mesh, bvh, init, None)
}

pub fn detect_symmetry(mesh: &Mesh, bvh: &Bvh) -> Option<(SymPlane, f64)> {
    detect_symmetry_masked(mesh, bvh, None)
}
