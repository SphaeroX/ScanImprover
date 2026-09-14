//! Clean-up of the extracted polygon mesh and conversion to a [`Mesh`]:
//! triangle pairs become quads, remaining polygons are split (or the whole
//! mesh is subdivided once into pure quads), vertices are projected onto
//! the input surface and relaxed tangentially.

use super::RetopoParams;
use super::extract::{PolyMesh, newell_normal, split_pinched};
use super::prep::edge_key;
use crate::geom::bvh::Bvh;
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;

/// Triangle pairs merge into a quad when every corner of the quad is within
/// about 50°..130° (|cos| of the corner angle below this).
const MERGE_MAX_COS: f32 = 0.64;
/// Step size of the tangential relaxation.
const RELAX_STEP: f32 = 0.5;
/// Largest hole (in edges) closed after the extraction.
const MAX_HOLE: usize = 16;
/// Holes up to this many edges on the surface are closed whatever their
/// winding.
const SMALL_HOLE: usize = 6;
/// Rounds of the untangling of folded faces.
const UNTANGLE_ROUNDS: usize = 12;

/// Largest |cos| of the corner angles of a polygon, or 2 when it is not
/// convex around `n`.
fn corner_score(pos: &[Vec3], face: &[u32], n: Vec3) -> f32 {
    let k = face.len();
    let mut worst = 0.0f32;
    for c in 0..k {
        let p = pos[face[c] as usize];
        let a = (pos[face[(c + k - 1) % k] as usize] - p).normalize_or_zero();
        let b = (pos[face[(c + 1) % k] as usize] - p).normalize_or_zero();
        if b.cross(a).dot(n) <= 0.0 {
            return 2.0;
        }
        worst = worst.max(a.dot(b).abs());
    }
    worst
}

/// Directed edges of all faces as (edge key, face), sorted by key.
fn face_edges(faces: &[Vec<u32>]) -> Vec<(u64, u32)> {
    let mut edges: Vec<(u64, u32)> = faces
        .iter()
        .enumerate()
        .flat_map(|(f, face)| {
            let k = face.len();
            (0..k).map(move |c| (edge_key(face[c], face[(c + 1) % k]), f as u32))
        })
        .collect();
    edges.par_sort_unstable();
    edges
}

/// Merges adjacent triangles into well-shaped quads, best pairs first.
pub(super) fn merge_triangle_pairs(poly: &mut PolyMesh) {
    let edges = face_edges(&poly.faces);
    let mut cand: Vec<(f32, u32, u32, [u32; 4])> = Vec::new();
    for w in edges.windows(2) {
        if w[0].0 != w[1].0 {
            continue;
        }
        let (f1, f2) = (w[0].1, w[1].1);
        let (t1, t2) = (&poly.faces[f1 as usize], &poly.faces[f2 as usize]);
        if t1.len() != 3 || t2.len() != 3 || f1 == f2 {
            continue;
        }
        let (x, y) = ((w[0].0 >> 32) as u32, (w[0].0 & 0xffff_ffff) as u32);
        // Orient so that t1 runs x -> y and t2 runs y -> x.
        let runs = |t: &[u32], a: u32, b: u32| (0..3).find(|&c| t[c] == a && t[(c + 1) % 3] == b);
        let (x, y) = if runs(t1, x, y).is_some() {
            (x, y)
        } else {
            (y, x)
        };
        let (Some(c1), Some(c2)) = (runs(t1, x, y), runs(t2, y, x)) else {
            continue;
        };
        let z1 = t1[(c1 + 2) % 3];
        let z2 = t2[(c2 + 2) % 3];
        let quad = [y, z1, x, z2];
        let n = quad
            .iter()
            .map(|&v| poly.nrm[v as usize])
            .sum::<Vec3>()
            .normalize_or_zero();
        let score = corner_score(&poly.pos, &quad, n);
        if score < MERGE_MAX_COS {
            cand.push((score, f1, f2, quad));
        }
    }
    cand.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut taken = vec![false; poly.faces.len()];
    let mut removed = vec![false; poly.faces.len()];
    for (_, f1, f2, quad) in cand {
        let (f1, f2) = (f1 as usize, f2 as usize);
        if taken[f1] || taken[f2] {
            continue;
        }
        taken[f1] = true;
        taken[f2] = true;
        poly.faces[f1] = quad.to_vec();
        removed[f2] = true;
    }
    let mut k = 0;
    poly.faces.retain(|_| {
        k += 1;
        !removed[k - 1]
    });
}

/// Splits polygons with more than four corners into quads (plus one
/// triangle for odd corner counts), choosing the best-shaped fan.
pub(super) fn split_polygons(poly: &mut PolyMesh) {
    let mut out = Vec::with_capacity(poly.faces.len());
    for face in std::mem::take(&mut poly.faces) {
        let k = face.len();
        if k <= 4 {
            out.push(face);
            continue;
        }
        let n = newell_normal(&poly.pos, &face).normalize_or_zero();
        let mut best: (f32, Vec<Vec<u32>>) = (f32::MAX, Vec::new());
        for r in 0..k {
            let v: Vec<u32> = (0..k).map(|i| face[(i + r) % k]).collect();
            let mut pieces = Vec::new();
            let mut i = 1;
            while i + 2 < k {
                pieces.push(vec![v[0], v[i], v[i + 1], v[i + 2]]);
                i += 2;
            }
            if i + 1 < k {
                pieces.push(vec![v[0], v[i], v[i + 1]]);
            }
            let score = pieces
                .iter()
                .map(|p| corner_score(&poly.pos, p, n))
                .fold(0.0f32, f32::max);
            if score < best.0 {
                best = (score, pieces);
            }
        }
        out.extend(best.1);
    }
    poly.faces = out;
}

/// Face point of a polygon for the subdivision: the centroid, or for a
/// non-convex face (the centroid of an L-shaped face can lie outside it) the
/// point between the centroid and a corner from which the sub-quads turn
/// consistently, so none of them folds over.
fn face_point(pos: &[Vec3], face: &[u32], centroid: Vec3) -> Vec3 {
    let k = face.len();
    let n = newell_normal(pos, face).normalize_or_zero();
    if n == Vec3::ZERO {
        return centroid;
    }
    let p = |i: usize| pos[face[i % k] as usize];
    // Smallest turn of the sub-quads [v, mid next, c, mid prev] at their
    // midpoints and at c (the corner at v is the face's own corner).
    let score = |c: Vec3| {
        let mut worst = f32::MAX;
        for i in 0..k {
            let q = [
                p(i),
                (p(i) + p(i + 1)) * 0.5,
                c,
                (p(i + k - 1) + p(i)) * 0.5,
            ];
            for j in 0..3 {
                let (a, b, d) = (q[j], q[j + 1], q[(j + 2) % 4]);
                worst = worst.min((b - a).cross(d - b).dot(n));
            }
        }
        worst
    };
    let base = score(centroid);
    if base > 0.0 {
        return centroid;
    }
    let mut best = (base, centroid);
    for i in 0..k {
        for t in [0.25f32, 0.5, 0.75] {
            let c = centroid.lerp(p(i), t);
            let s = score(c);
            if s > best.0 {
                best = (s, c);
            }
        }
    }
    best.1
}

/// One linear subdivision step: every n-gon becomes n quads (face point,
/// edge midpoints). Vertices are projected onto the surface afterwards.
pub(super) fn subdivide(poly: &PolyMesh) -> PolyMesh {
    let mut keys: Vec<u64> = face_edges(&poly.faces).into_iter().map(|e| e.0).collect();
    keys.dedup();
    let nv = poly.pos.len() as u32;
    let mut pos = poly.pos.clone();
    let mut nrm = poly.nrm.clone();
    let mut feature = poly.feature.clone();
    for &k in &keys {
        let (a, b) = ((k >> 32) as usize, (k & 0xffff_ffff) as usize);
        pos.push((poly.pos[a] + poly.pos[b]) * 0.5);
        nrm.push((poly.nrm[a] + poly.nrm[b]).normalize_or(poly.nrm[a]));
        feature.push(poly.feature[a] && poly.feature[b]);
    }
    let mid = |a: u32, b: u32| nv + keys.binary_search(&edge_key(a, b)).unwrap_or(0) as u32;
    let mut faces = Vec::with_capacity(poly.faces.len() * 4);
    for face in &poly.faces {
        let k = face.len();
        let c = pos.len() as u32;
        let centroid = face.iter().map(|&v| poly.pos[v as usize]).sum::<Vec3>() / k as f32;
        let n = face.iter().map(|&v| poly.nrm[v as usize]).sum::<Vec3>();
        pos.push(face_point(&poly.pos, face, centroid));
        nrm.push(n.normalize_or(Vec3::Y));
        feature.push(false);
        for i in 0..k {
            let prev = face[(i + k - 1) % k];
            let (v, next) = (face[i], face[(i + 1) % k]);
            faces.push(vec![v, mid(v, next), c, mid(prev, v)]);
        }
    }
    PolyMesh {
        pos,
        nrm,
        feature,
        faces,
    }
}

/// Projects `p` onto the input surface: (point, surface normal).
fn project(p: Vec3, fallback_n: Vec3, input: &Mesh, bvh: &Bvh) -> (Vec3, Vec3) {
    let (q, tri, d) = bvh.closest_point(p);
    if !d.is_finite() {
        return (p, fallback_n);
    }
    let n = input.face_normal(tri as usize).normalize_or_zero();
    (q, if n == Vec3::ZERO { fallback_n } else { n })
}

/// Tangential Laplacian relaxation with re-projection; feature and boundary
/// vertices stay where they are.
fn relax(poly: &mut PolyMesh, iterations: u32, input: &Mesh, bvh: &Bvh) {
    let nv = poly.pos.len();
    let edges = face_edges(&poly.faces);
    let mut fixed = poly.feature.clone();
    let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(edges.len());
    let mut s = 0;
    while s < edges.len() {
        let mut e = s + 1;
        while e < edges.len() && edges[e].0 == edges[s].0 {
            e += 1;
        }
        let (a, b) = ((edges[s].0 >> 32) as u32, (edges[s].0 & 0xffff_ffff) as u32);
        if e - s == 1 {
            fixed[a as usize] = true;
            fixed[b as usize] = true;
        }
        pairs.push((a, b));
        pairs.push((b, a));
        s = e;
    }
    pairs.par_sort_unstable();
    let mut off = vec![0usize; nv + 1];
    for &(a, _) in &pairs {
        off[a as usize + 1] += 1;
    }
    for i in 0..nv {
        off[i + 1] += off[i];
    }
    for _ in 0..iterations {
        let pos = &poly.pos;
        let nrm = &poly.nrm;
        let next: Vec<(Vec3, Vec3)> = (0..nv)
            .into_par_iter()
            .map(|v| {
                let r = off[v]..off[v + 1];
                if fixed[v] || r.is_empty() {
                    return (pos[v], nrm[v]);
                }
                let avg = pairs[r.clone()]
                    .iter()
                    .map(|&(_, u)| pos[u as usize])
                    .sum::<Vec3>()
                    / r.len() as f32;
                let n = nrm[v];
                let mut d = avg - pos[v];
                d -= n * n.dot(d);
                project(pos[v] + d * RELAX_STEP, n, input, bvh)
            })
            .collect();
        for (v, (p, n)) in next.into_iter().enumerate() {
            poly.pos[v] = p;
            poly.nrm[v] = n;
        }
    }
}

/// Closes holes of up to [`MAX_HOLE`] edges left by the extraction. Loops
/// that run clockwise (open boundaries) or do not cover surface (real
/// holes of the scan) stay open. The winding is judged against the input
/// surface under the hole as well as the extracted vertex normals, which
/// can be unreliable next to sharp edges.
fn fill_small_holes(poly: &mut PolyMesh, s: f32, input: &Mesh, bvh: &Bvh) {
    let mut darts: Vec<(u32, u32)> = poly
        .faces
        .iter()
        .flat_map(|f| (0..f.len()).map(move |c| (f[c], f[(c + 1) % f.len()])))
        .collect();
    darts.par_sort_unstable();
    // Hole darts run against the boundary edges.
    let mut out: Vec<(u32, u32)> = darts
        .iter()
        .filter(|&&(a, b)| darts.binary_search(&(b, a)).is_err())
        .map(|&(a, b)| (b, a))
        .collect();
    out.sort_unstable();
    let mut used = vec![false; out.len()];
    let take = |from: u32, used: &mut [bool]| -> Option<u32> {
        let start = out.partition_point(|x| x.0 < from);
        (start..out.len())
            .take_while(|&k| out[k].0 == from)
            .find(|&k| !used[k])
            .map(|k| {
                used[k] = true;
                out[k].1
            })
    };
    let mut filled = Vec::new();
    for k in 0..out.len() {
        if used[k] {
            continue;
        }
        used[k] = true;
        let start = out[k].0;
        let mut lp = vec![start];
        let mut cur = out[k].1;
        let mut closed = false;
        while lp.len() <= 4 * MAX_HOLE {
            if cur == start {
                closed = true;
                break;
            }
            lp.push(cur);
            match take(cur, &mut used) {
                Some(next) => cur = next,
                None => break,
            }
        }
        if !closed {
            continue;
        }
        for piece in split_pinched(&lp) {
            if piece.len() > MAX_HOLE {
                continue;
            }
            // The outside of an open patch winds against the normals; small
            // holes at creases can be twisted, so only reject clear cases.
            let avg_n = piece
                .iter()
                .map(|&v| poly.nrm[v as usize])
                .sum::<Vec3>()
                .normalize_or_zero();
            let loop_n = newell_normal(&poly.pos, &piece).normalize_or_zero();
            let facing = loop_n.dot(avg_n);
            let c = piece.iter().map(|&v| poly.pos[v as usize]).sum::<Vec3>() / piece.len() as f32;
            let (_, tri, dist) = bvh.closest_point(c);
            let surface_facing = if dist.is_finite() {
                loop_n.dot(input.face_normal(tri as usize).normalize_or_zero())
            } else {
                0.0
            };
            // A loop of a few edges lying on the surface is a gap left where
            // the extracted faces fold next to a sharp edge: closing it keeps
            // the mesh closed, the untangling unfolds it. Longer loops must
            // wind with the surface (open boundaries of the input stay open).
            let small = piece.len() <= SMALL_HOLE;
            if (small || facing > -0.5 || surface_facing > 0.5) && dist < 0.4 * s {
                filled.push(piece);
            }
        }
    }
    poly.faces.extend(filled);
}

/// Faces whose normal points against the input surface below them.
fn folded_faces(poly: &PolyMesh, input: &Mesh, bvh: &Bvh) -> Vec<usize> {
    (0..poly.faces.len())
        .into_par_iter()
        .filter(|&f| {
            let face = &poly.faces[f];
            let n = newell_normal(&poly.pos, face);
            let c = face.iter().map(|&v| poly.pos[v as usize]).sum::<Vec3>() / face.len() as f32;
            let (_, tri, d) = bvh.closest_point(c);
            d.is_finite() && n.dot(input.face_normal(tri as usize)) < 0.0
        })
        .collect()
}

/// Unfolds faces that point against the input surface (next to sharp edges
/// after the subdivision or the relaxation): their free corners move to the
/// average of their neighbours, projected onto the input, until no face is
/// folded or the rounds run out. Feature and boundary vertices stay.
fn untangle(poly: &mut PolyMesh, input: &Mesh, bvh: &Bvh) {
    let nv = poly.pos.len();
    let mut nbrs: Vec<Vec<u32>> = vec![Vec::new(); nv];
    let mut boundary = vec![false; nv];
    let edges = face_edges(&poly.faces);
    let mut s = 0;
    while s < edges.len() {
        let mut e = s + 1;
        while e < edges.len() && edges[e].0 == edges[s].0 {
            e += 1;
        }
        let (a, b) = ((edges[s].0 >> 32) as u32, (edges[s].0 & 0xffff_ffff) as u32);
        nbrs[a as usize].push(b);
        nbrs[b as usize].push(a);
        if e - s == 1 {
            boundary[a as usize] = true;
            boundary[b as usize] = true;
        }
        s = e;
    }
    for _ in 0..UNTANGLE_ROUNDS {
        let folded = folded_faces(poly, input, bvh);
        let mut movable: Vec<u32> = folded
            .iter()
            .flat_map(|&f| poly.faces[f].iter().copied())
            .filter(|&v| !poly.feature[v as usize] && !boundary[v as usize])
            .collect();
        movable.sort_unstable();
        movable.dedup();
        if movable.is_empty() {
            break;
        }
        let pos = &poly.pos;
        let nrm = &poly.nrm;
        let moves: Vec<(Vec3, Vec3)> = movable
            .par_iter()
            .map(|&v| {
                let nb = &nbrs[v as usize];
                let avg =
                    nb.iter().map(|&u| pos[u as usize]).sum::<Vec3>() / nb.len().max(1) as f32;
                project(avg, nrm[v as usize], input, bvh)
            })
            .collect();
        for (&v, (p, n)) in movable.iter().zip(moves) {
            poly.pos[v as usize] = p;
            poly.nrm[v as usize] = n;
        }
    }
}

/// Turns the extracted polygons (lattice scale `s`) into the final
/// quad(-dominant) mesh.
pub(super) fn finish(
    mut poly: PolyMesh,
    params: &RetopoParams,
    s: f32,
    input: &Mesh,
    bvh: &Bvh,
) -> PolyMesh {
    fill_small_holes(&mut poly, s, input, bvh);
    merge_triangle_pairs(&mut poly);
    if params.pure_quads {
        poly = subdivide(&poly);
    } else {
        split_polygons(&mut poly);
    }
    let projected: Vec<(Vec3, Vec3)> = poly
        .pos
        .par_iter()
        .zip(poly.nrm.par_iter())
        .map(|(&p, &n)| project(p, n, input, bvh))
        .collect();
    for (v, (p, n)) in projected.into_iter().enumerate() {
        poly.pos[v] = p;
        poly.nrm[v] = n;
    }
    relax(&mut poly, params.relax_iterations, input, bvh);
    untangle(&mut poly, input, bvh);
    poly
}

/// Final mesh plus (quad count, irregular interior vertex count).
pub(super) fn to_mesh(poly: &PolyMesh) -> (Mesh, usize, usize) {
    let mut remap = vec![u32::MAX; poly.pos.len()];
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut faces: Vec<Vec<u32>> = Vec::with_capacity(poly.faces.len());
    for face in &poly.faces {
        faces.push(
            face.iter()
                .map(|&v| {
                    if remap[v as usize] == u32::MAX {
                        remap[v as usize] = positions.len() as u32;
                        positions.push(poly.pos[v as usize].to_array());
                    }
                    remap[v as usize]
                })
                .collect(),
        );
    }
    let p = |v: u32| Vec3::from(positions[v as usize]);
    let mut indices = Vec::new();
    let mut quads = 0;
    for f in faces.iter().filter(|f| f.len() == 4) {
        // Split along the shorter diagonal, which must be a-c.
        let [a, b, c, d] = if p(f[1]).distance_squared(p(f[3])) < p(f[0]).distance_squared(p(f[2]))
        {
            [f[1], f[2], f[3], f[0]]
        } else {
            [f[0], f[1], f[2], f[3]]
        };
        indices.extend_from_slice(&[a, b, c, a, c, d]);
        quads += 1;
    }
    for f in faces.iter().filter(|f| f.len() != 4) {
        for i in 1..f.len() - 1 {
            indices.extend_from_slice(&[f[0], f[i], f[i + 1]]);
        }
    }

    // Valence of interior vertices (boundary vertices are not counted).
    let edges = face_edges(&faces);
    let mut valence = vec![0u32; positions.len()];
    let mut boundary = vec![false; positions.len()];
    let mut s = 0;
    while s < edges.len() {
        let mut e = s + 1;
        while e < edges.len() && edges[e].0 == edges[s].0 {
            e += 1;
        }
        let (a, b) = (
            (edges[s].0 >> 32) as usize,
            (edges[s].0 & 0xffff_ffff) as usize,
        );
        valence[a] += 1;
        valence[b] += 1;
        if e - s == 1 {
            boundary[a] = true;
            boundary[b] = true;
        }
        s = e;
    }
    let irregular = (0..positions.len())
        .filter(|&v| !boundary[v] && valence[v] != 4)
        .count();

    let mut mesh = Mesh::from_indexed(positions, indices);
    mesh.set_quad_layout(quads);
    (mesh, quads, irregular)
}
