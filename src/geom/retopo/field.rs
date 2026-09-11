//! Extrinsic 4-RoSy orientation and 4-PoSy position fields (Jakob et al.
//! 2015, `field.cpp`), smoothed coarse to fine on the hierarchy.
//!
//! Every vertex carries a tangent direction `q` (defined up to 90°
//! rotations about its normal) and a lattice origin `o` in its tangent
//! plane: the local quad grid is `o + (a·q + b·(n×q))·s` for integers a, b.
//! Smoothing makes neighbouring grids agree; extraction then reads the
//! quad mesh off the grids.

use super::hierarchy::{Constraint, FREE, LINE, Level, POINT};
use crate::rng::Rng;
use glam::Vec3;
use rayon::prelude::*;

/// Gauss–Seidel sweeps per level (the coarsest level gets more).
const ORIENT_ITERS: usize = 10;
const POS_ITERS: usize = 10;
const COARSEST_ITERS: usize = 60;

/// Vertex frame used by the position compatibility test.
#[derive(Clone, Copy)]
pub(super) struct Frame {
    pub p: Vec3,
    pub n: Vec3,
    pub q: Vec3,
    pub o: Vec3,
}

#[inline]
pub(super) fn tangent(v: Vec3, n: Vec3) -> Vec3 {
    v - n * n.dot(v)
}

/// Unit tangent of `v` at normal `n`, or any tangent when `v` is parallel to `n`.
#[inline]
fn unit_tangent(v: Vec3, n: Vec3) -> Vec3 {
    let t = tangent(v, n).normalize_or_zero();
    if t == Vec3::ZERO {
        n.any_orthonormal_vector()
    } else {
        t
    }
}

/// `q` rotated by `k` quarter turns about `n`.
#[inline]
fn rotate90_by(q: Vec3, n: Vec3, k: usize) -> Vec3 {
    let r = if k & 1 == 1 { n.cross(q) } else { q };
    if k < 2 { r } else { -r }
}

/// The 4-RoSy representatives of two orientations that match best.
#[inline]
pub(super) fn compat_orientation(q0: Vec3, n0: Vec3, q1: Vec3, n1: Vec3) -> (Vec3, Vec3) {
    let a = [q0, n0.cross(q0)];
    let b = [q1, n1.cross(q1)];
    let mut best = (-1.0f32, 0, 0);
    for i in 0..2 {
        for j in 0..2 {
            let score = a[i].dot(b[j]).abs();
            if score > best.0 {
                best = (score, i, j);
            }
        }
    }
    let dp = a[best.1].dot(b[best.2]);
    (a[best.1], b[best.2] * if dp < 0.0 { -1.0 } else { 1.0 })
}

/// The quarter-turn rotation of `d` (about `n`) closest to `r`.
#[inline]
pub(super) fn align_to(r: Vec3, d: Vec3, n: Vec3) -> Vec3 {
    let mut best = (f32::MIN, d);
    for k in 0..4 {
        let c = rotate90_by(d, n, k);
        let s = c.dot(r);
        if s > best.0 {
            best = (s, c);
        }
    }
    best.1
}

/// Point closest to both vertices that lies on both tangent planes.
#[inline]
fn middle_point(p0: Vec3, n0: Vec3, p1: Vec3, n1: Vec3) -> Vec3 {
    let n0p0 = n0.dot(p0);
    let n0p1 = n0.dot(p1);
    let n1p0 = n1.dot(p0);
    let n1p1 = n1.dot(p1);
    let n0n1 = n0.dot(n1);
    let denom = 1.0 / (1.0 - n0n1 * n0n1 + 1e-4);
    let l0 = 2.0 * (n0p1 - n0p0 - n0n1 * (n1p0 - n1p1)) * denom;
    let l1 = 2.0 * (n1p0 - n1p1 - n0n1 * (n0p1 - n0p0)) * denom;
    0.5 * (p0 + p1) - 0.25 * (n0 * l0 + n1 * l1)
}

/// Lattice point `o + (a·q + b·t)·s` below `p` (floor of the lattice coordinates).
#[inline]
fn lattice_floor(o: Vec3, q: Vec3, t: Vec3, p: Vec3, s: f32) -> Vec3 {
    let d = p - o;
    o + q * ((q.dot(d) / s).floor() * s) + t * ((t.dot(d) / s).floor() * s)
}

/// Lattice point nearest to `p`.
#[inline]
pub(super) fn lattice_round(o: Vec3, q: Vec3, t: Vec3, p: Vec3, s: f32) -> Vec3 {
    let d = p - o;
    o + q * ((q.dot(d) / s).round() * s) + t * ((t.dot(d) / s).round() * s)
}

/// Lattice points of the two grids that lie closest to each other near the
/// middle of the edge.
#[inline]
pub(super) fn compat_position(a: Frame, b: Frame, s: f32) -> (Vec3, Vec3) {
    let ta = a.n.cross(a.q);
    let tb = b.n.cross(b.q);
    let m = middle_point(a.p, a.n, b.p, b.n);
    let oa = lattice_floor(a.o, a.q, ta, m, s);
    let ob = lattice_floor(b.o, b.q, tb, m, s);
    let mut best = (f32::MAX, oa, ob);
    for i in 0..4 {
        let ca = oa + (a.q * (i & 1) as f32 + ta * (i >> 1) as f32) * s;
        for j in 0..4 {
            let cb = ob + (b.q * (j & 1) as f32 + tb * (j >> 1) as f32) * s;
            let d = ca.distance_squared(cb);
            if d < best.0 {
                best = (d, ca, cb);
            }
        }
    }
    (best.1, best.2)
}

/// Runs `iters` Gauss–Seidel sweeps: one parallel update per colour phase.
fn gauss_seidel<T: Copy + Send + Sync>(
    level: &Level,
    x: &mut [T],
    iters: usize,
    update: impl Fn(usize, &[T]) -> T + Sync,
) {
    for _ in 0..iters {
        for phase in &level.phases {
            let new: Vec<T> = phase.par_iter().map(|&i| update(i as usize, x)).collect();
            for (&i, v) in phase.iter().zip(new) {
                x[i as usize] = v;
            }
        }
    }
}

/// Direction a constraint locks the orientation to, if any.
#[inline]
fn locked_dir(c: &Constraint) -> Option<Vec3> {
    (c.kind != FREE && c.dir != Vec3::ZERO).then_some(c.dir)
}

fn orientation_update(lv: &Level, i: usize, q: &[Vec3]) -> Vec3 {
    if let Some(d) = locked_dir(&lv.cons[i]) {
        return d;
    }
    let n = lv.n[i];
    let mut acc = q[i];
    let mut wsum = 0.0f32;
    for (j, w) in lv.neighbors(i) {
        let (a, b) = compat_orientation(acc, n, q[j], lv.n[j]);
        acc = tangent(a * wsum + b * w, n);
        wsum += w;
        let len = acc.length();
        acc = if len > 1e-20 { acc / len } else { q[i] };
    }
    acc
}

/// Solves the orientation field on every level; returns `q` per level.
pub(super) fn solve_orientation(levels: &[Level]) -> Vec<Vec<Vec3>> {
    let top = levels.len() - 1;
    let mut qs: Vec<Vec<Vec3>> = vec![Vec::new(); levels.len()];
    let mut rng = Rng::new(0x5eed_0001);
    let lt = &levels[top];
    qs[top] = (0..lt.len())
        .map(|i| {
            let r = Vec3::new(rng.f32() - 0.5, rng.f32() - 0.5, rng.f32() - 0.5);
            locked_dir(&lt.cons[i]).unwrap_or_else(|| unit_tangent(r, lt.n[i]))
        })
        .collect();
    for l in (0..=top).rev() {
        let lv = &levels[l];
        if l < top {
            let coarse = &qs[l + 1];
            qs[l] = (0..lv.len())
                .into_par_iter()
                .map(|i| {
                    locked_dir(&lv.cons[i])
                        .unwrap_or_else(|| unit_tangent(coarse[lv.parent[i] as usize], lv.n[i]))
                })
                .collect();
        }
        let iters = if l == top {
            COARSEST_ITERS
        } else {
            ORIENT_ITERS
        };
        gauss_seidel(lv, &mut qs[l], iters, |i, q| orientation_update(lv, i, q));
    }
    qs
}

/// Moves a lattice origin so the lattice honours the vertex constraint.
#[inline]
fn constrain_position(c: Constraint, o: Vec3, p: Vec3, s: f32) -> Vec3 {
    match c.kind {
        LINE => {
            // Onto the feature line, then the lattice point on it nearest p.
            let on = c.pt + c.dir * c.dir.dot(o - c.pt);
            on + c.dir * ((c.dir.dot(p - on) / s).round() * s)
        }
        POINT => c.pt,
        _ => o,
    }
}

fn position_update(lv: &Level, q: &[Vec3], s: f32, i: usize, o: &[Vec3]) -> Vec3 {
    let (p, n, qi) = (lv.p[i], lv.n[i], q[i]);
    let mut acc = o[i];
    let mut wsum = 0.0f32;
    for (j, w) in lv.neighbors(i) {
        let (a, b) = compat_position(
            Frame {
                p,
                n,
                q: qi,
                o: acc,
            },
            Frame {
                p: lv.p[j],
                n: lv.n[j],
                q: q[j],
                o: o[j],
            },
            s,
        );
        acc = (a * wsum + b * w) / (wsum + w);
        wsum += w;
        acc -= n * n.dot(acc - p);
    }
    let r = lattice_round(acc, qi, n.cross(qi), p, s);
    constrain_position(lv.cons[i], r, p, s)
}

/// Solves the position field (lattice scale `s`); returns `o` of level 0.
pub(super) fn solve_positions(levels: &[Level], qs: &[Vec<Vec3>], s: f32) -> Vec<Vec3> {
    let top = levels.len() - 1;
    let mut rng = Rng::new(0x5eed_0002);
    let lt = &levels[top];
    let mut o: Vec<Vec3> = (0..lt.len())
        .map(|i| {
            let (p, n, q) = (lt.p[i], lt.n[i], qs[top][i]);
            let t = n.cross(q);
            let r = p + (q * (rng.f32() - 0.5) + t * (rng.f32() - 0.5)) * s;
            constrain_position(lt.cons[i], r, p, s)
        })
        .collect();
    for l in (0..=top).rev() {
        let lv = &levels[l];
        let q = &qs[l];
        if l < top {
            let coarse = o;
            o = (0..lv.len())
                .into_par_iter()
                .map(|i| {
                    let (p, n) = (lv.p[i], lv.n[i]);
                    let c = coarse[lv.parent[i] as usize];
                    let c = c - n * n.dot(c - p);
                    let r = lattice_round(c, q[i], n.cross(q[i]), p, s);
                    constrain_position(lv.cons[i], r, p, s)
                })
                .collect();
        }
        let iters = if l == top { COARSEST_ITERS } else { POS_ITERS };
        gauss_seidel(lv, &mut o, iters, |i, o| position_update(lv, q, s, i, o));
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orientation_compat_matches_quarter_turns() {
        let n = Vec3::Z;
        let q0 = Vec3::X;
        let q1 = Vec3::new(-0.1, 1.0, 0.0).normalize(); // ~90° from q0
        let (a, b) = compat_orientation(q0, n, q1, n);
        assert!(a.dot(b) > 0.99, "{a:?} {b:?}");
        let r = align_to(Vec3::X, Vec3::NEG_Y, n);
        assert!(r.dot(Vec3::X) > 0.999);
    }

    #[test]
    fn position_compat_finds_shared_lattice_point() {
        let n = Vec3::Z;
        // Two copies of the same unit lattice, stored with different origins.
        let a = Frame {
            p: Vec3::new(0.1, 0.2, 0.0),
            n,
            q: Vec3::X,
            o: Vec3::new(0.25, 0.25, 0.0),
        };
        let b = Frame {
            p: Vec3::new(0.9, 0.3, 0.0),
            n,
            q: Vec3::Y,
            o: Vec3::new(3.25, -1.75, 0.0),
        };
        let (x, y) = compat_position(a, b, 1.0);
        assert!(x.distance(y) < 1e-5, "{x:?} {y:?}");
        let m = middle_point(a.p, n, b.p, n);
        assert!(x.distance(m) < 1.5);
    }

    #[test]
    fn line_constraint_puts_a_lattice_line_on_the_feature() {
        let c = Constraint {
            dir: Vec3::X,
            pt: Vec3::ZERO,
            kind: LINE,
        };
        let o = constrain_position(c, Vec3::new(0.3, 0.4, 0.0), Vec3::new(2.2, 0.0, 0.0), 1.0);
        assert!(o.y.abs() < 1e-6);
        assert!((o.x - 2.3).abs() < 1e-5, "{o:?}");
    }
}
