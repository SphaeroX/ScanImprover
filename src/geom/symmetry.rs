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

fn sym_err(
    bvh: &Bvh,
    samples: &[Vec3],
    theta: f64,
    phi: f64,
    d: f64,
    center: Vec3,
) -> f64 {
    let sp = plane_from_params(theta, phi, d, center);
    let sum = samples
        .par_iter()
        .map(|&s| {
            let r = reflect(s, &sp);
            let dist = bvh.closest_distance(r);
            (dist * dist) as f64
        })
        .sum::<f64>();
    sum / samples.len() as f64
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

fn optimize_from(
    bvh: &Bvh,
    samples: &[Vec3],
    center: Vec3,
    init: &SymPlane,
    d_step: f32,
) -> (SymPlane, f64) {
    let (t0, p0, d0) = params_from_plane(init, center);
    let x = nelder_mead(
        &|p: [f64; 3]| sym_err(bvh, samples, p[0], p[1], p[2], center),
        [t0, p0, d0],
        [0.02, 0.02, d_step as f64],
        220,
    );
    let err = sym_err(bvh, samples, x[0], x[1], x[2], center);
    (plane_from_params(x[0], x[1], x[2], center), err.sqrt())
}

pub fn refine_symmetry(mesh: &Mesh, bvh: &Bvh, init: &SymPlane) -> (SymPlane, f64) {
    let center = mesh.bbox().center();
    let mut rng = Rng::new(0xBEEF);
    let samples = mesh.sample_surface(8192, &mut rng);
    let d_step = mesh.bbox().diagonal() * 0.01;
    optimize_from(bvh, &samples, center, init, d_step)
}

pub fn detect_symmetry(mesh: &Mesh, bvh: &Bvh) -> Option<(SymPlane, f64)> {
    let bbox = mesh.bbox();
    let center = bbox.center();
    let mut rng = Rng::new(0xCAFE);
    let coarse = mesh.sample_surface(768, &mut rng);
    let fine = mesh.sample_surface(6144, &mut rng);
    if coarse.is_empty() {
        return None;
    }
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
            let (t, p, d) = params_from_plane(&sp, center);
            (sym_err(bvh, &coarse, t, p, d, center), n)
        })
        .collect();
    scored.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let d_step = bbox.diagonal() * 0.01;
    let mut best: Option<(SymPlane, f64)> = None;
    for (_, n) in scored.into_iter().take(2) {
        let init = SymPlane {
            normal: n,
            point: center,
        };
        let res = optimize_from(bvh, &fine, center, &init, d_step);
        match &best {
            Some((_, e)) if *e <= res.1 => {}
            _ => best = Some(res),
        }
    }
    best
}
