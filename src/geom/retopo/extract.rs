//! Quad mesh extraction from the orientation and position fields
//! (following Instant Meshes `extract_graph` / `extract_faces`).
//!
//! 1. Every edge of the work mesh is classified by the integer lattice
//!    offset between its two vertices: 0 means both vertices claim the same
//!    lattice point (collapse), a unit offset is an edge of the quad mesh,
//!    anything else (diagonals, singular regions) is ignored.
//! 2. Collapses are applied cheapest first with union-find; a collapse that
//!    would merge two clusters already joined by a lattice edge is skipped.
//! 3. Clusters become vertices, unit offsets become edges; dangling edges
//!    and redundant edges spanning a nearly collinear vertex are removed.
//! 4. Faces are traced in the resulting graph by walking around each vertex
//!    in counter-clockwise order.

use super::field::{Frame, compat_orientation, compat_position};
use super::hierarchy::{FREE, Level};
use super::prep::edge_key;
use crate::geom::bvh::Bvh;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::HashMap;

/// Largest face kept (larger loops are holes).
const MAX_FACE: usize = 12;
/// Longest loop the face walk follows; longer loops are open boundaries.
const MAX_WALK: usize = 64;
/// A vertex closer than this (relative to the scale) to an edge of a graph
/// triangle is snapped onto it and the edge is removed.
const SNAP_DIST: f32 = 0.3;

/// Polygon mesh produced by the extraction and edited by the clean-up.
pub(super) struct PolyMesh {
    pub pos: Vec<Vec3>,
    pub nrm: Vec<Vec3>,
    /// Vertex lies on a sharp feature or open boundary (not relaxed).
    pub feature: Vec<bool>,
    pub faces: Vec<Vec<u32>>,
}

struct Dsu {
    parent: Vec<u32>,
    size: Vec<u32>,
}

impl Dsu {
    fn new(n: usize) -> Dsu {
        Dsu {
            parent: (0..n as u32).collect(),
            size: vec![1; n],
        }
    }

    fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            let g = self.parent[self.parent[x as usize] as usize];
            self.parent[x as usize] = g;
            x = g;
        }
        x
    }

    /// Joins two roots, returns the new root.
    fn union_roots(&mut self, a: u32, b: u32) -> u32 {
        let (big, small) = if self.size[a as usize] >= self.size[b as usize] {
            (a, b)
        } else {
            (b, a)
        };
        self.parent[small as usize] = big;
        self.size[big as usize] += self.size[small as usize];
        big
    }
}

enum EdgeClass {
    Collapse(f32),
    Unit,
}

fn classify(lv: &Level, q: &[Vec3], o: &[Vec3], s: f32, i: usize, j: usize) -> Option<EdgeClass> {
    let (qi, qj) = compat_orientation(q[i], lv.n[i], q[j], lv.n[j]);
    let (oi, oj) = compat_position(
        Frame {
            p: lv.p[i],
            n: lv.n[i],
            q: qi,
            o: o[i],
        },
        Frame {
            p: lv.p[j],
            n: lv.n[j],
            q: qj,
            o: o[j],
        },
        s,
    );
    // o_j - o_i expressed as lattice steps of vertex i.
    let d = (oi - o[i]) - (oj - o[j]);
    let ti = lv.n[i].cross(qi);
    let u = (qi.dot(d) / s).round() as i64;
    let v = (ti.dot(d) / s).round() as i64;
    match u.abs() + v.abs() {
        0 => Some(EdgeClass::Collapse(oi.distance(oj))),
        1 => Some(EdgeClass::Unit),
        _ => None,
    }
}

/// Extracts a polygon mesh from the fields of level 0 (lattice scale `s`).
/// `surface` is used to reject large faces that span a hole.
pub(super) fn extract(lv: &Level, q: &[Vec3], o: &[Vec3], s: f32, surface: &Bvh) -> PolyMesh {
    let n = lv.len();
    let classified: Vec<(u32, u32, EdgeClass)> = (0..n)
        .into_par_iter()
        .flat_map_iter(|i| {
            lv.neighbors(i)
                .filter(move |&(j, _)| j > i)
                .filter_map(move |(j, _)| {
                    classify(lv, q, o, s, i, j).map(|c| (i as u32, j as u32, c))
                })
        })
        .collect();
    let mut collapses: Vec<(f32, u32, u32)> = Vec::new();
    let mut units: Vec<(u32, u32)> = Vec::new();
    for (i, j, c) in classified {
        match c {
            EdgeClass::Collapse(err) => collapses.push((err, i, j)),
            EdgeClass::Unit => units.push((i, j)),
        }
    }
    collapses.par_sort_unstable_by(|a, b| a.0.total_cmp(&b.0));

    // Collapse, refusing merges between clusters already joined by an edge.
    let mut dsu = Dsu::new(n);
    let mut unit_adj: Vec<Vec<u32>> = vec![Vec::new(); n];
    for &(i, j) in &units {
        unit_adj[i as usize].push(j);
        unit_adj[j as usize].push(i);
    }
    for &(_, i, j) in &collapses {
        let (a, b) = (dsu.find(i), dsu.find(j));
        if a == b {
            continue;
        }
        let (small, big) = if unit_adj[a as usize].len() <= unit_adj[b as usize].len() {
            (a, b)
        } else {
            (b, a)
        };
        let mut adjacent = false;
        for k in 0..unit_adj[small as usize].len() {
            let x = unit_adj[small as usize][k];
            if dsu.find(x) == big {
                adjacent = true;
                break;
            }
        }
        if adjacent {
            continue;
        }
        let r = dsu.union_roots(a, b);
        let other = if r == a { b } else { a };
        let moved = std::mem::take(&mut unit_adj[other as usize]);
        unit_adj[r as usize].extend(moved);
    }

    // Clusters -> vertices, weighted towards members close to their lattice point.
    let mut id = vec![u32::MAX; n];
    let mut cluster = vec![0u32; n];
    let mut count = 0u32;
    for v in 0..n {
        let r = dsu.find(v as u32) as usize;
        if id[r] == u32::MAX {
            id[r] = count;
            count += 1;
        }
        cluster[v] = id[r];
    }
    let nc = count as usize;
    let mut psum = vec![Vec3::ZERO; nc];
    let mut nsum = vec![Vec3::ZERO; nc];
    let mut wsum = vec![0.0f32; nc];
    let mut feature = vec![false; nc];
    let inv_s2 = 1.0 / (s * s);
    for v in 0..n {
        let c = cluster[v] as usize;
        let w = (-9.0 * o[v].distance_squared(lv.p[v]) * inv_s2)
            .exp()
            .max(1e-6);
        psum[c] += o[v] * w;
        nsum[c] += lv.n[v] * w;
        wsum[c] += w;
        feature[c] |= lv.cons[v].kind != FREE;
    }
    let mut pos: Vec<Vec3> = (0..nc).map(|c| psum[c] / wsum[c]).collect();
    let nrm: Vec<Vec3> = (0..nc).map(|c| nsum[c].normalize_or(Vec3::Y)).collect();

    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); nc];
    for &(i, j) in &units {
        let (a, b) = (cluster[i as usize], cluster[j as usize]);
        if a != b {
            adj[a as usize].push(b);
            adj[b as usize].push(a);
        }
    }
    for list in adj.iter_mut() {
        list.sort_unstable();
        list.dedup();
    }
    prune_dangling(&mut adj);
    snap_collinear(&mut adj, &mut pos, s);
    prune_dangling(&mut adj);

    let faces = trace_faces(&adj, &pos, &nrm)
        .into_iter()
        .flat_map(|f| split_pinched(&f))
        .filter(|f| {
            if f.len() < 3 || f.len() > MAX_FACE {
                return false;
            }
            // Loops running clockwise around the normals are the outside
            // of an open boundary, not faces.
            let avg_n: Vec3 = f.iter().map(|&v| nrm[v as usize]).sum();
            if newell_normal(&pos, f).dot(avg_n) <= 0.0 {
                return false;
            }
            if f.len() <= 4 {
                return true;
            }
            // Large loops are only faces when they cover actual surface.
            let c = f.iter().map(|&v| pos[v as usize]).sum::<Vec3>() / f.len() as f32;
            surface.closest_distance(c) < 0.4 * s
        })
        .collect();
    PolyMesh {
        pos,
        nrm,
        feature,
        faces,
    }
}

/// Splits a loop that visits a vertex more than once (two faces pinched
/// together at a vertex) into simple loops; pieces with fewer than three
/// vertices (spikes) are dropped.
pub(super) fn split_pinched(face: &[u32]) -> Vec<Vec<u32>> {
    let mut out = Vec::new();
    let mut stack: Vec<u32> = Vec::with_capacity(face.len());
    for &v in face {
        if let Some(i) = stack.iter().position(|&x| x == v) {
            let piece = stack.split_off(i);
            if piece.len() >= 3 {
                out.push(piece);
            }
        }
        stack.push(v);
    }
    if stack.len() >= 3 {
        out.push(stack);
    }
    out
}

/// Area-weighted normal of a polygon (Newell's method, not normalised).
pub(super) fn newell_normal(pos: &[Vec3], face: &[u32]) -> Vec3 {
    let k = face.len();
    let mut n = Vec3::ZERO;
    for c in 0..k {
        let a = pos[face[c] as usize];
        let b = pos[face[(c + 1) % k] as usize];
        n += Vec3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }
    n * 0.5
}

/// Removes vertices of degree 1 (repeatedly), so no face walk runs into a spike.
fn prune_dangling(adj: &mut [Vec<u32>]) {
    let mut stack: Vec<u32> = (0..adj.len() as u32)
        .filter(|&v| adj[v as usize].len() == 1)
        .collect();
    while let Some(v) = stack.pop() {
        if adj[v as usize].len() != 1 {
            continue;
        }
        let u = adj[v as usize][0];
        adj[v as usize].clear();
        adj[u as usize].retain(|&x| x != v);
        if adj[u as usize].len() == 1 {
            stack.push(u);
        }
    }
}

/// For graph triangles i-j-k where i lies almost on the longest edge j-k,
/// moves i onto that edge and removes the edge (it duplicates j-i-k).
fn snap_collinear(adj: &mut [Vec<u32>], pos: &mut [Vec3], s: f32) {
    let mut remove: Vec<u64> = Vec::new();
    let mut moves: Vec<(u32, Vec3)> = Vec::new();
    for j in 0..adj.len() {
        for &k in &adj[j] {
            if (k as usize) <= j {
                continue;
            }
            let (pj, pk) = (pos[j], pos[k as usize]);
            let len2 = pj.distance_squared(pk);
            for &i in &adj[j] {
                if i == k || adj[k as usize].binary_search(&i).is_err() {
                    continue;
                }
                let pi = pos[i as usize];
                if pi.distance_squared(pj) >= len2 || pi.distance_squared(pk) >= len2 {
                    continue;
                }
                let d = pk - pj;
                let t = ((pi - pj).dot(d) / len2).clamp(0.0, 1.0);
                let on = pj + d * t;
                if on.distance(pi) < SNAP_DIST * s {
                    remove.push(edge_key(j as u32, k));
                    moves.push((i, on));
                    break;
                }
            }
        }
    }
    for (i, p) in moves {
        pos[i as usize] = p;
    }
    remove.sort_unstable();
    remove.dedup();
    for key in remove {
        let (a, b) = ((key >> 32) as usize, (key & 0xffff_ffff) as u32);
        adj[a].retain(|&x| x != b);
        adj[b as usize].retain(|&x| x != a as u32);
    }
}

/// Walks the faces of the graph: every directed edge belongs to exactly one
/// loop; at each vertex the walk continues with the neighbour preceding the
/// one it came from in counter-clockwise order, which traces faces
/// counter-clockwise around the vertex normals.
pub(super) fn trace_faces(adj: &[Vec<u32>], pos: &[Vec3], nrm: &[Vec3]) -> Vec<Vec<u32>> {
    let nv = adj.len();
    // Neighbours sorted counter-clockwise around each vertex normal (CSR).
    let sorted: Vec<Vec<u32>> = (0..nv)
        .into_par_iter()
        .map(|v| {
            let n = nrm[v];
            let e1 = n.any_orthonormal_vector();
            let e2 = n.cross(e1);
            let mut list: Vec<(f32, u32)> = adj[v]
                .iter()
                .map(|&u| {
                    let d = pos[u as usize] - pos[v];
                    (d.dot(e2).atan2(d.dot(e1)), u)
                })
                .collect();
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            list.into_iter().map(|(_, u)| u).collect()
        })
        .collect();
    let mut off = vec![0usize; nv + 1];
    for v in 0..nv {
        off[v + 1] = off[v] + sorted[v].len();
    }
    let darts: Vec<u32> = sorted.concat();
    let mut index: HashMap<u64, usize> = HashMap::with_capacity(darts.len());
    for v in 0..nv {
        for (k, &u) in sorted[v].iter().enumerate() {
            index.insert(((v as u64) << 32) | u as u64, off[v] + k);
        }
    }
    let mut used = vec![false; darts.len()];
    let mut faces = Vec::new();
    for v in 0..nv {
        for d0 in off[v]..off[v + 1] {
            if used[d0] {
                continue;
            }
            let mut face = Vec::new();
            let (mut from, mut d) = (v as u32, d0);
            loop {
                used[d] = true;
                face.push(from);
                let to = darts[d];
                // Position of `from` in the list of `to`, then its predecessor.
                let Some(&back) = index.get(&(((to as u64) << 32) | from as u64)) else {
                    face.clear();
                    break;
                };
                let (lo, hi) = (off[to as usize], off[to as usize + 1]);
                let k = back - lo;
                let next = lo + (k + (hi - lo) - 1) % (hi - lo);
                from = to;
                d = next;
                if d == d0 || used[d] || face.len() > MAX_WALK {
                    if d != d0 {
                        face.clear();
                    }
                    break;
                }
            }
            if face.len() >= 3 {
                faces.push(face);
            }
        }
    }
    faces
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_walk_traces_a_grid_counter_clockwise() {
        // 3x3 vertex grid in the xy plane, normals +z.
        let pos: Vec<Vec3> = (0..9)
            .map(|i| Vec3::new((i % 3) as f32, (i / 3) as f32, 0.0))
            .collect();
        let nrm = vec![Vec3::Z; 9];
        let mut adj = vec![Vec::new(); 9];
        for i in 0..9u32 {
            if i % 3 < 2 {
                adj[i as usize].push(i + 1);
                adj[i as usize + 1].push(i);
            }
            if i < 6 {
                adj[i as usize].push(i + 3);
                adj[i as usize + 3].push(i);
            }
        }
        let faces = trace_faces(&adj, &pos, &nrm);
        let quads: Vec<&Vec<u32>> = faces.iter().filter(|f| f.len() == 4).collect();
        assert_eq!(quads.len(), 4, "{faces:?}");
        for f in quads {
            let [a, b, c] = [f[0], f[1], f[2]].map(|v| pos[v as usize]);
            assert!((b - a).cross(c - b).z > 0.0, "face {f:?} is clockwise");
        }
    }
}
