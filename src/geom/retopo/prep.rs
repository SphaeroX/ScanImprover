//! Work mesh and level-0 graph: resolution matching, feature detection and
//! vertex constraints.

use super::field::{align_to, tangent};
use super::hierarchy::{Constraint, LINE, Level, POINT};
use crate::decimate;
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::HashMap;

/// Work mesh edge length relative to the lattice scale: a few vertices per
/// lattice cell so every lattice point is claimed by at least one vertex.
const WORK_EDGE_FRACTION: f32 = 1.0 / 3.5;
/// Longest work mesh edge relative to the lattice scale.
const MAX_EDGE_FRACTION: f32 = 0.5;
/// A triangle whose middle vertex lies closer than this (relative to its
/// longest edge) to that edge is degenerate.
const COLLINEAR_EPS: f32 = 1e-4;
/// Feature chains shorter than this (relative to the scale) are noise.
const MIN_FEATURE_LENGTH: f32 = 1.0;
/// A feature turning by less than this angle along its chain is straight.
const STRAIGHT_COS: f32 = 0.866; // 30°
/// A corner within this angle of 90° also fixes the orientation.
const FEATURE_AGREE_COS: f32 = 0.906; // 25°
const MAX_WORK_TRIS: usize = 6_000_000;

#[inline]
pub(super) fn edge_key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    ((lo as u64) << 32) | hi as u64
}

#[inline]
fn key_pair(k: u64) -> (u32, u32) {
    ((k >> 32) as u32, (k & 0xffff_ffff) as u32)
}

/// A copy of `input` at a resolution suited to lattice scale `scale`:
/// dense scans are decimated, long edges are split.
pub(super) fn work_mesh(input: &Mesh, area: f32, scale: f32) -> Mesh {
    let edge = scale * WORK_EDGE_FRACTION;
    // Equilateral triangles of edge length `edge` cover the area.
    let wanted = (area / (0.433 * edge * edge)).ceil() as usize;
    let tris = input.triangle_count();
    let work = if tris as f32 > 1.6 * wanted as f32 {
        decimate::decimate(input, wanted as f32 / tris as f32, scale * 0.25, false)
            .map(|(m, _)| m)
            .unwrap_or_else(|_| input.clone())
    } else {
        input.clone()
    };
    let work = drop_degenerate(&repair_degenerate(&work, scale), scale);
    split_long_edges(&work, scale * MAX_EDGE_FRACTION)
}

/// Removes collapsed triangles and unreferenced vertices.
fn drop_degenerate(mesh: &Mesh, scale: f32) -> Mesh {
    let min_area2 = (1e-10 * scale * scale).powi(2);
    let keep: Vec<bool> = (0..mesh.triangle_count())
        .into_par_iter()
        .map(|t| {
            let i = &mesh.indices[3 * t..3 * t + 3];
            if i[0] == i[1] || i[1] == i[2] || i[0] == i[2] {
                return false;
            }
            let [a, b, c] = mesh.triangle(t);
            (b - a).cross(c - a).length_squared() > min_area2
        })
        .collect();
    let mut remap = vec![u32::MAX; mesh.vertex_count()];
    let mut positions = Vec::with_capacity(mesh.vertex_count());
    let mut indices = Vec::with_capacity(mesh.indices.len());
    for (t, _) in keep.iter().enumerate().filter(|(_, k)| **k) {
        for &v in &mesh.indices[3 * t..3 * t + 3] {
            if remap[v as usize] == u32::MAX {
                remap[v as usize] = positions.len() as u32;
                positions.push(mesh.positions[v as usize]);
            }
            indices.push(remap[v as usize]);
        }
    }
    Mesh::from_indexed(positions, indices)
}

/// Removes degenerate (collinear) triangles without opening the surface.
///
/// CAD exports fan faces into slivers whose vertices lie on one line.
/// Dropping them leaves slits inside the faces, which the feature detection
/// takes for open boundaries and turns into hard constraints across the
/// face. Instead, each degenerate triangle is deleted and the triangle
/// across its longest edge is split at the middle vertex, so its two short
/// edges pair with the split. Triangles with repeated vertices are dropped.
fn repair_degenerate(mesh: &Mesh, scale: f32) -> Mesh {
    let p = |i: u32| Vec3::from(mesh.positions[i as usize]);
    let tiny = 1e-9 * scale;
    // (a, b, c) with the longest edge a -> b and c its middle vertex, when
    // the triangle is degenerate.
    let collinear = |v: [u32; 3]| -> Option<(u32, u32, u32)> {
        let k = (0..3).max_by(|&i, &j| {
            let li = p(v[i]).distance_squared(p(v[(i + 1) % 3]));
            let lj = p(v[j]).distance_squared(p(v[(j + 1) % 3]));
            li.total_cmp(&lj)
        })?;
        let (a, b, c) = (v[k], v[(k + 1) % 3], v[(k + 2) % 3]);
        let len = p(a).distance(p(b));
        let height = (p(b) - p(a)).cross(p(c) - p(a)).length() / len.max(tiny);
        (height <= COLLINEAR_EPS * len.max(tiny)).then_some((a, b, c))
    };
    let mut tris: Vec<[u32; 3]> = mesh
        .indices
        .chunks_exact(3)
        .map(|t| [t[0], t[1], t[2]])
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        .collect();
    for _ in 0..64 {
        let mut edge_tri: HashMap<(u32, u32), usize> = HashMap::with_capacity(tris.len() * 3);
        for (t, v) in tris.iter().enumerate() {
            for k in 0..3 {
                edge_tri.insert((v[k], v[(k + 1) % 3]), t);
            }
        }
        let mut touched = vec![false; tris.len()];
        let mut added: Vec<[u32; 3]> = Vec::new();
        let mut changed = false;
        for t in 0..tris.len() {
            if touched[t] {
                continue;
            }
            let Some((a, b, c)) = collinear(tris[t]) else {
                continue;
            };
            match edge_tri.get(&(b, a)) {
                Some(&s) if s != t && !touched[s] => {
                    let sv = tris[s];
                    let Some(k) = (0..3).find(|&i| sv[i] == b && sv[(i + 1) % 3] == a) else {
                        continue;
                    };
                    let x = sv[(k + 2) % 3];
                    if x != c {
                        added.push([b, c, x]);
                        added.push([c, a, x]);
                    }
                    touched[s] = true;
                }
                // Open boundary: the outline stays the same without it.
                None => {}
                _ => continue,
            }
            touched[t] = true;
            changed = true;
        }
        if !changed {
            break;
        }
        tris = tris
            .into_iter()
            .zip(touched)
            .filter(|(_, gone)| !gone)
            .map(|(v, _)| v)
            .chain(added)
            .collect();
    }
    Mesh::from_indexed(mesh.positions.clone(), tris.concat())
}

/// Splits every edge longer than `max_len` at its midpoint (conforming
/// red/green refinement), repeated until no long edge is left.
fn split_long_edges(mesh: &Mesh, max_len: f32) -> Mesh {
    let max2 = max_len * max_len;
    let mut positions = mesh.positions.clone();
    let mut indices = mesh.indices.clone();
    for _ in 0..16 {
        let nt = indices.len() / 3;
        let pos = &positions;
        let idx = &indices;
        let mut long: Vec<u64> = (0..nt)
            .into_par_iter()
            .flat_map_iter(|t| {
                let v = [idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]];
                (0..3).filter_map(move |k| {
                    let (a, b) = (v[k], v[(k + 1) % 3]);
                    let d = Vec3::from(pos[a as usize]) - Vec3::from(pos[b as usize]);
                    (d.length_squared() > max2).then(|| edge_key(a, b))
                })
            })
            .collect();
        if long.is_empty() {
            break;
        }
        long.par_sort_unstable();
        long.dedup();
        if nt + 3 * long.len() > MAX_WORK_TRIS {
            break;
        }
        let base = positions.len() as u32;
        for &k in &long {
            let (a, b) = key_pair(k);
            let m = (Vec3::from(positions[a as usize]) + Vec3::from(positions[b as usize])) * 0.5;
            positions.push(m.to_array());
        }
        let mid = |a: u32, b: u32| {
            long.binary_search(&edge_key(a, b))
                .ok()
                .map(|i| base + i as u32)
        };
        let p = |i: u32| Vec3::from(positions[i as usize]);
        let mut out = Vec::with_capacity(indices.len() * 2);
        for t in 0..nt {
            let v = [indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]];
            let m = [mid(v[0], v[1]), mid(v[1], v[2]), mid(v[2], v[0])];
            match m.iter().filter(|x| x.is_some()).count() {
                0 => out.extend_from_slice(&v),
                1 => {
                    let k = m.iter().position(|x| x.is_some()).unwrap_or(0);
                    let (a, b, c) = (v[k], v[(k + 1) % 3], v[(k + 2) % 3]);
                    let mm = m[k].unwrap_or(a);
                    out.extend_from_slice(&[a, mm, c, mm, b, c]);
                }
                2 => {
                    // Unsplit edge u = (c, a); split edges (a, b) and (b, c).
                    let u = m.iter().position(|x| x.is_none()).unwrap_or(0);
                    let (a, b, c) = (v[(u + 1) % 3], v[(u + 2) % 3], v[u]);
                    let m1 = m[(u + 1) % 3].unwrap_or(a);
                    let m2 = m[(u + 2) % 3].unwrap_or(b);
                    out.extend_from_slice(&[m1, b, m2]);
                    if p(a).distance_squared(p(m2)) < p(m1).distance_squared(p(c)) {
                        out.extend_from_slice(&[a, m1, m2, a, m2, c]);
                    } else {
                        out.extend_from_slice(&[a, m1, c, m1, m2, c]);
                    }
                }
                _ => {
                    let (m01, m12, m20) = (
                        m[0].unwrap_or(v[0]),
                        m[1].unwrap_or(v[1]),
                        m[2].unwrap_or(v[2]),
                    );
                    out.extend_from_slice(&[
                        v[0], m01, m20, m01, v[1], m12, m20, m12, v[2], m01, m12, m20,
                    ]);
                }
            }
        }
        indices = out;
    }
    Mesh::from_indexed(positions, indices)
}

/// Builds the level-0 graph of the work mesh, including the feature
/// constraints (sharp creases and open boundaries) when `crease_cos` is set.
pub(super) fn build_level0(work: &Mesh, scale: f32, crease_cos: Option<f32>) -> Level {
    let nv = work.vertex_count();
    let nt = work.triangle_count();
    let p: Vec<Vec3> = work.positions.iter().map(|&v| Vec3::from(v)).collect();
    let n: Vec<Vec3> = work
        .normals
        .iter()
        .map(|&v| Vec3::from(v).normalize_or(Vec3::Y))
        .collect();
    let face: Vec<(Vec3, f32)> = (0..nt)
        .into_par_iter()
        .map(|t| {
            let [a, b, c] = work.triangle(t);
            let cr = (b - a).cross(c - a);
            (cr.normalize_or_zero(), cr.length() * 0.5)
        })
        .collect();
    let mut area = vec![0.0f32; nv];
    for t in 0..nt {
        for &v in &work.indices[3 * t..3 * t + 3] {
            area[v as usize] += face[t].1 / 3.0;
        }
    }

    // Undirected edges with their incident triangles.
    let mut half: Vec<(u64, u32)> = (0..nt)
        .into_par_iter()
        .flat_map_iter(|t| {
            let i = &work.indices[3 * t..3 * t + 3];
            [
                (edge_key(i[0], i[1]), t as u32),
                (edge_key(i[1], i[2]), t as u32),
                (edge_key(i[2], i[0]), t as u32),
            ]
        })
        .collect();
    half.par_sort_unstable();
    let mut pairs: Vec<(u32, u32, f32)> = Vec::with_capacity(half.len());
    let mut features: Vec<(u32, u32)> = Vec::new();
    let mut s = 0;
    while s < half.len() {
        let mut e = s + 1;
        while e < half.len() && half[e].0 == half[s].0 {
            e += 1;
        }
        let (a, b) = key_pair(half[s].0);
        pairs.push((a, b, 1.0));
        pairs.push((b, a, 1.0));
        if let Some(cos) = crease_cos {
            let is_feature = match e - s {
                1 => true,
                2 => {
                    face[half[s].1 as usize]
                        .0
                        .dot(face[half[s + 1].1 as usize].0)
                        < cos
                }
                _ => false,
            };
            if is_feature {
                features.push((a, b));
            }
        }
        s = e;
    }
    let features = drop_short_chains(&features, &p, nv, scale * MIN_FEATURE_LENGTH);
    let cons = vertex_constraints(&features, &p, &n, scale);
    Level::new(p, n, area, cons, pairs)
}

/// Removes connected feature-edge chains whose total length is below `min_len`.
fn drop_short_chains(edges: &[(u32, u32)], p: &[Vec3], nv: usize, min_len: f32) -> Vec<(u32, u32)> {
    let mut parent: Vec<u32> = (0..nv as u32).collect();
    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }
    for &(a, b) in edges {
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            parent[ra as usize] = rb;
        }
    }
    let mut length = vec![0.0f32; nv];
    for &(a, b) in edges {
        let r = find(&mut parent, a);
        length[r as usize] += p[a as usize].distance(p[b as usize]);
    }
    edges
        .iter()
        .copied()
        .filter(|&(a, _)| length[find(&mut parent, a) as usize] >= min_len)
        .collect()
}

/// Turns feature edges into per-vertex constraints.
///
/// Directions are measured with chords reaching half a lattice cell along
/// the feature chain in both directions, so jagged boundaries and noisy
/// creases still give a clean direction. Chain vertices that go straight
/// follow the chain (LINE); the sharpest turn of a chain becomes a lattice
/// point (POINT), with the orientation locked only when the turn is close
/// to a right angle; junctions of three or more chains (box corners) are
/// lattice points with a free orientation.
fn vertex_constraints(edges: &[(u32, u32)], p: &[Vec3], n: &[Vec3], scale: f32) -> Vec<Constraint> {
    let nv = p.len();
    let mut cons = vec![Constraint::default(); nv];
    if edges.is_empty() {
        return cons;
    }
    let mut pairs: Vec<(u32, u32)> = edges.iter().flat_map(|&(a, b)| [(a, b), (b, a)]).collect();
    pairs.par_sort_unstable();
    pairs.dedup();
    let mut off = vec![0usize; nv + 1];
    for &(a, _) in &pairs {
        off[a as usize + 1] += 1;
    }
    for i in 0..nv {
        off[i + 1] += off[i];
    }
    let degree = |v: u32| off[v as usize + 1] - off[v as usize];
    let reach = 0.5 * scale;
    // Follows the chain from `v` through `first` for about `reach`; returns
    // the vertices passed (last one is the chord end).
    let walk = |v: u32, first: u32| -> Vec<u32> {
        let (mut prev, mut cur) = (v, first);
        let mut len = p[v as usize].distance(p[cur as usize]);
        let mut seen = vec![cur];
        while len < reach && degree(cur) == 2 {
            let o = off[cur as usize];
            let next = if pairs[o].1 != prev {
                pairs[o].1
            } else {
                pairs[o + 1].1
            };
            if next == v {
                break;
            }
            len += p[cur as usize].distance(p[next as usize]);
            prev = cur;
            cur = next;
            seen.push(cur);
        }
        seen
    };

    // Turn cosine (1 = straight) of every chain vertex, corner candidates.
    let mut turn = vec![1.0f32; nv];
    let mut corner = Vec::new();
    for v in 0..nv as u32 {
        let (vi, nvv) = (v as usize, n[v as usize]);
        let nb: Vec<u32> = pairs[off[vi]..off[vi + 1]].iter().map(|x| x.1).collect();
        match nb.len() {
            0 => {}
            1 => {
                let end = p[*walk(v, nb[0]).last().unwrap_or(&nb[0]) as usize];
                let dir = tangent(end - p[vi], nvv).normalize_or_zero();
                if dir != Vec3::ZERO {
                    cons[vi] = Constraint {
                        dir,
                        pt: p[vi],
                        kind: LINE,
                    };
                }
            }
            2 => {
                let a = p[*walk(v, nb[0]).last().unwrap_or(&nb[0]) as usize];
                let b = p[*walk(v, nb[1]).last().unwrap_or(&nb[1]) as usize];
                let din = tangent(p[vi] - a, nvv).normalize_or_zero();
                let dout = tangent(b - p[vi], nvv).normalize_or_zero();
                turn[vi] = din.dot(dout);
                if turn[vi] > STRAIGHT_COS {
                    let dir = tangent(b - a, nvv).normalize_or_zero();
                    if dir != Vec3::ZERO {
                        cons[vi] = Constraint {
                            dir,
                            pt: p[vi],
                            kind: LINE,
                        };
                    }
                } else {
                    // Right-angle corners also fix the orientation.
                    let dir = if align_to(din, dout, nvv).dot(din) > FEATURE_AGREE_COS {
                        tangent(din + align_to(din, dout, nvv), nvv).normalize_or_zero()
                    } else {
                        Vec3::ZERO
                    };
                    cons[vi] = Constraint {
                        dir,
                        pt: p[vi],
                        kind: POINT,
                    };
                    corner.push(v);
                }
            }
            _ => {
                cons[vi] = Constraint {
                    dir: Vec3::ZERO,
                    pt: p[vi],
                    kind: POINT,
                }
            }
        }
    }
    // Only the sharpest corner within reach survives (junctions win).
    for &v in &corner {
        let vi = v as usize;
        let beaten = pairs[off[vi]..off[vi + 1]].iter().any(|&(_, first)| {
            walk(v, first).iter().any(|&u| {
                let ui = u as usize;
                degree(u) > 2
                    || (turn[ui] < turn[vi] || (turn[ui] == turn[vi] && u < v))
                        && cons[ui].kind == POINT
            })
        });
        if beaten {
            cons[vi] = Constraint::default();
        }
    }
    cons
}
