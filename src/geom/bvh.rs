//! Bounding volume hierarchy over mesh triangles for closest-point queries,
//! ray casting and sphere queries.
//!
//! The tree is built with a binned surface area heuristic (SAH), which
//! produces much tighter nodes than a median split and therefore faster
//! queries. Large subtrees are built in parallel. Nodes are stored in a
//! flat, cache friendly array: the left child of an interior node always
//! directly follows it, only the right child index is stored.

use glam::Vec3;
use rayon::prelude::*;

/// Number of SAH bins per split evaluation.
const BINS: usize = 12;
/// Leaves hold at most this many triangles.
const LEAF_MAX: usize = 4;
/// Leaves may grow to this size when the SAH says splitting does not pay.
const LEAF_MAX_SAH: usize = 8;
/// Subtrees with more triangles than this are built on two rayon tasks.
const PARALLEL_MIN: usize = 24_000;
/// Traversal stack depth (tree depth is O(log n) and well below this).
const STACK: usize = 96;

#[derive(Clone, Copy)]
struct Node {
    min: [f32; 3],
    max: [f32; 3],
    /// Interior: index of the right child. Leaf: first triangle in `tri_order`.
    right_or_first: u32,
    /// Leaf triangle count, 0 for interior nodes.
    count: u32,
}

pub struct Bvh {
    nodes: Vec<Node>,
    tri_order: Vec<u32>,
    verts: Vec<[f32; 3]>,
    tris: Vec<[u32; 3]>,
}

struct BuildCtx<'a> {
    verts: &'a [[f32; 3]],
    tris: &'a [[u32; 3]],
    centroids: &'a [[f32; 3]],
}

impl Bvh {
    pub fn new(positions: &[[f32; 3]], indices: &[u32]) -> Bvh {
        let tris: Vec<[u32; 3]> = indices
            .chunks_exact(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();
        let nt = tris.len();
        let mut tri_order: Vec<u32> = (0..nt as u32).collect();
        let nodes = if nt == 0 {
            Vec::new()
        } else {
            let centroids: Vec<[f32; 3]> = tris
                .par_iter()
                .map(|t| {
                    let a = Vec3::from(positions[t[0] as usize]);
                    let b = Vec3::from(positions[t[1] as usize]);
                    let c = Vec3::from(positions[t[2] as usize]);
                    ((a + b + c) * (1.0 / 3.0)).to_array()
                })
                .collect();
            let ctx = BuildCtx {
                verts: positions,
                tris: &tris,
                centroids: &centroids,
            };
            build_subtree(&ctx, &mut tri_order, 0)
        };
        Bvh {
            nodes,
            tri_order,
            verts: positions.to_vec(),
            tris,
        }
    }

    #[allow(dead_code)]
    pub fn triangle_count(&self) -> usize {
        self.tris.len()
    }

    #[inline]
    fn tri_verts(&self, t: u32) -> [Vec3; 3] {
        let [a, b, c] = self.tris[t as usize];
        [
            Vec3::from(self.verts[a as usize]),
            Vec3::from(self.verts[b as usize]),
            Vec3::from(self.verts[c as usize]),
        ]
    }

    /// Closest point on the mesh to `p`: (point, triangle, distance).
    pub fn closest_point(&self, p: Vec3) -> (Vec3, u32, f32) {
        if self.nodes.is_empty() {
            return (p, 0, f32::INFINITY);
        }
        let mut stack: [(u32, f32); STACK] = [(0, 0.0); STACK];
        let mut sp = 1usize;
        let mut best_d2 = f32::MAX;
        let mut best_point = p;
        let mut best_tri = 0u32;
        while sp > 0 {
            sp -= 1;
            let (node, nd2) = stack[sp];
            if nd2 >= best_d2 {
                continue;
            }
            let n = &self.nodes[node as usize];
            if n.count > 0 {
                let first = n.right_or_first as usize;
                for &t in &self.tri_order[first..first + n.count as usize] {
                    let [a, b, c] = self.tri_verts(t);
                    let (q, d2) = closest_point_triangle(p, a, b, c);
                    if d2 < best_d2 {
                        best_d2 = d2;
                        best_point = q;
                        best_tri = t;
                    }
                }
            } else {
                let l = node + 1;
                let r = n.right_or_first;
                let ln = &self.nodes[l as usize];
                let rn = &self.nodes[r as usize];
                let ld2 = point_aabb_dist_sq(p, &ln.min, &ln.max);
                let rd2 = point_aabb_dist_sq(p, &rn.min, &rn.max);
                // Push the farther child first so the nearer one is visited next.
                let (near, near_d, far, far_d) = if ld2 <= rd2 {
                    (l, ld2, r, rd2)
                } else {
                    (r, rd2, l, ld2)
                };
                if far_d < best_d2 && sp < STACK {
                    stack[sp] = (far, far_d);
                    sp += 1;
                }
                if near_d < best_d2 && sp < STACK {
                    stack[sp] = (near, near_d);
                    sp += 1;
                }
            }
        }
        (best_point, best_tri, best_d2.sqrt())
    }

    pub fn closest_distance(&self, p: Vec3) -> f32 {
        self.closest_point(p).2
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn ray_cast(&self, ro: Vec3, rd: Vec3, tmax: f32) -> Option<(f32, u32)> {
        self.ray_cast_filtered(ro, rd, tmax, |_| false)
    }

    /// Nearest ray hit, skipping triangles for which `is_hidden` is true.
    pub fn ray_cast_filtered(
        &self,
        ro: Vec3,
        rd: Vec3,
        tmax: f32,
        is_hidden: impl Fn(u32) -> bool,
    ) -> Option<(f32, u32)> {
        if self.nodes.is_empty() {
            return None;
        }
        let inv = Vec3::new(1.0 / rd.x, 1.0 / rd.y, 1.0 / rd.z);
        let mut stack: [(u32, f32); STACK] = [(0, 0.0); STACK];
        let mut sp = 1usize;
        let mut best_t = tmax;
        let mut best_tri = 0u32;
        let mut hit = false;
        while sp > 0 {
            sp -= 1;
            let (node, tmin) = stack[sp];
            if tmin > best_t {
                continue;
            }
            let n = &self.nodes[node as usize];
            if n.count > 0 {
                let first = n.right_or_first as usize;
                for &t in &self.tri_order[first..first + n.count as usize] {
                    if is_hidden(t) {
                        continue;
                    }
                    let [a, b, c] = self.tri_verts(t);
                    if let Some(th) = ray_triangle(ro, rd, a, b, c)
                        && th < best_t
                    {
                        best_t = th;
                        best_tri = t;
                        hit = true;
                    }
                }
            } else {
                let l = node + 1;
                let r = n.right_or_first;
                let ln = &self.nodes[l as usize];
                let rn = &self.nodes[r as usize];
                let tl = ray_aabb(ro, inv, &ln.min, &ln.max);
                let tr = ray_aabb(ro, inv, &rn.min, &rn.max);
                let mut push = |node: u32, t: Option<f32>| {
                    if let Some(t) = t
                        && t <= best_t
                        && sp < STACK
                    {
                        stack[sp] = (node, t);
                        sp += 1;
                    }
                };
                match (tl, tr) {
                    (Some(a), Some(b)) if a <= b => {
                        push(r, tr);
                        push(l, tl);
                    }
                    _ => {
                        push(l, tl);
                        push(r, tr);
                    }
                }
            }
        }
        if hit { Some((best_t, best_tri)) } else { None }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn query_sphere(&self, center: Vec3, radius: f32, out: &mut Vec<u32>) {
        self.query_sphere_filtered(center, radius, out, |_| false);
    }

    /// Collects triangles intersecting the sphere.
    pub fn query_sphere_filtered(
        &self,
        center: Vec3,
        radius: f32,
        out: &mut Vec<u32>,
        is_hidden: impl Fn(u32) -> bool,
    ) {
        if self.nodes.is_empty() {
            return;
        }
        let r2 = radius * radius;
        let mut stack: [u32; STACK] = [0; STACK];
        let mut sp = 1usize;
        while sp > 0 {
            sp -= 1;
            let node = stack[sp];
            let n = &self.nodes[node as usize];
            if point_aabb_dist_sq(center, &n.min, &n.max) > r2 {
                continue;
            }
            if n.count > 0 {
                let first = n.right_or_first as usize;
                for &t in &self.tri_order[first..first + n.count as usize] {
                    if is_hidden(t) {
                        continue;
                    }
                    let [a, b, c] = self.tri_verts(t);
                    if (a - center).length_squared() <= r2
                        || (b - center).length_squared() <= r2
                        || (c - center).length_squared() <= r2
                        || closest_point_triangle(center, a, b, c).1 <= r2
                    {
                        out.push(t);
                    }
                }
            } else if sp + 2 <= STACK {
                stack[sp] = node + 1;
                stack[sp + 1] = n.right_or_first;
                sp += 2;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Bounds {
    min: [f32; 3],
    max: [f32; 3],
}

impl Bounds {
    const EMPTY: Bounds = Bounds {
        min: [f32::MAX; 3],
        max: [f32::MIN; 3],
    };

    #[inline]
    fn grow_point(&mut self, p: &[f32; 3]) {
        for a in 0..3 {
            self.min[a] = self.min[a].min(p[a]);
            self.max[a] = self.max[a].max(p[a]);
        }
    }

    #[inline]
    fn grow(&mut self, o: &Bounds) {
        for a in 0..3 {
            self.min[a] = self.min[a].min(o.min[a]);
            self.max[a] = self.max[a].max(o.max[a]);
        }
    }

    #[inline]
    fn half_area(&self) -> f32 {
        if self.min[0] > self.max[0] {
            return 0.0;
        }
        let dx = self.max[0] - self.min[0];
        let dy = self.max[1] - self.min[1];
        let dz = self.max[2] - self.min[2];
        dx * dy + dy * dz + dz * dx
    }
}

#[inline]
fn tri_bounds(ctx: &BuildCtx, t: u32) -> Bounds {
    let [a, b, c] = ctx.tris[t as usize];
    let mut bb = Bounds::EMPTY;
    bb.grow_point(&ctx.verts[a as usize]);
    bb.grow_point(&ctx.verts[b as usize]);
    bb.grow_point(&ctx.verts[c as usize]);
    bb
}

/// Builds the subtree for `order` (which starts at absolute index `first`
/// in the final triangle order). Returns the nodes with the subtree root at
/// index 0 and right-child indices relative to the returned vector.
fn build_subtree(ctx: &BuildCtx, order: &mut [u32], first: u32) -> Vec<Node> {
    let n = order.len();
    let mut bb = Bounds::EMPTY;
    let mut cb = Bounds::EMPTY;
    for &t in order.iter() {
        bb.grow(&tri_bounds(ctx, t));
        cb.grow_point(&ctx.centroids[t as usize]);
    }
    let leaf = |count: usize| Node {
        min: bb.min,
        max: bb.max,
        right_or_first: first,
        count: count as u32,
    };
    if n <= LEAF_MAX {
        return vec![leaf(n)];
    }

    let split = match find_sah_split(ctx, order, &bb, &cb) {
        Split::Leaf => return vec![leaf(n)],
        Split::At(k) => k,
    };
    let (left, right) = order.split_at_mut(split);
    let (mut left_nodes, right_nodes) = if n >= PARALLEL_MIN {
        rayon::join(
            || build_subtree(ctx, left, first),
            || build_subtree(ctx, right, first + split as u32),
        )
    } else {
        (
            build_subtree(ctx, left, first),
            build_subtree(ctx, right, first + split as u32),
        )
    };

    // Layout: [root, left subtree..., right subtree...]
    let right_offset = 1 + left_nodes.len() as u32;
    let mut nodes = Vec::with_capacity(1 + left_nodes.len() + right_nodes.len());
    nodes.push(Node {
        min: bb.min,
        max: bb.max,
        right_or_first: right_offset,
        count: 0,
    });
    for nd in left_nodes.iter_mut() {
        if nd.count == 0 {
            nd.right_or_first += 1;
        }
    }
    nodes.extend_from_slice(&left_nodes);
    nodes.extend(right_nodes.into_iter().map(|mut nd| {
        if nd.count == 0 {
            nd.right_or_first += right_offset;
        }
        nd
    }));
    nodes
}

enum Split {
    Leaf,
    At(usize),
}

/// Binned SAH split along the widest centroid axis. Partitions `order` in
/// place and returns the split position, or `Leaf` when a leaf is cheaper.
fn find_sah_split(ctx: &BuildCtx, order: &mut [u32], bb: &Bounds, cb: &Bounds) -> Split {
    let n = order.len();
    let extent = [
        cb.max[0] - cb.min[0],
        cb.max[1] - cb.min[1],
        cb.max[2] - cb.min[2],
    ];
    let mut axis = 0;
    if extent[1] > extent[axis] {
        axis = 1;
    }
    if extent[2] > extent[axis] {
        axis = 2;
    }
    if extent[axis] <= 1e-12 {
        // All centroids coincide: split the list in half.
        return Split::At(n / 2);
    }

    let scale = BINS as f32 / extent[axis];
    let bin_of = |t: u32| -> usize {
        (((ctx.centroids[t as usize][axis] - cb.min[axis]) * scale) as usize).min(BINS - 1)
    };
    let mut bin_bounds = [Bounds::EMPTY; BINS];
    let mut bin_count = [0usize; BINS];
    for &t in order.iter() {
        let b = bin_of(t);
        bin_count[b] += 1;
        bin_bounds[b].grow(&tri_bounds(ctx, t));
    }

    // Sweep from the right to get suffix bounds, then from the left.
    let mut right_area = [0.0f32; BINS];
    let mut right_count = [0usize; BINS];
    let mut acc = Bounds::EMPTY;
    let mut cnt = 0usize;
    for i in (1..BINS).rev() {
        acc.grow(&bin_bounds[i]);
        cnt += bin_count[i];
        right_area[i] = acc.half_area();
        right_count[i] = cnt;
    }
    let mut best_cost = f32::MAX;
    let mut best_bin = 0usize;
    let mut left_acc = Bounds::EMPTY;
    let mut left_cnt = 0usize;
    for i in 0..BINS - 1 {
        left_acc.grow(&bin_bounds[i]);
        left_cnt += bin_count[i];
        let rc = right_count[i + 1];
        if left_cnt == 0 || rc == 0 {
            continue;
        }
        let cost = left_acc.half_area() * left_cnt as f32 + right_area[i + 1] * rc as f32;
        if cost < best_cost {
            best_cost = cost;
            best_bin = i;
        }
    }
    if best_cost == f32::MAX {
        return Split::At(n / 2);
    }
    // Leaf is cheaper than splitting (traversal cost ~ 1 triangle test).
    let leaf_cost = bb.half_area() * n as f32;
    if n <= LEAF_MAX_SAH && best_cost >= leaf_cost {
        return Split::Leaf;
    }

    // Partition: bins <= best_bin go left.
    let mut i = 0usize;
    let mut j = n;
    while i < j {
        if bin_of(order[i]) <= best_bin {
            i += 1;
        } else {
            j -= 1;
            order.swap(i, j);
        }
    }
    if i == 0 || i == n {
        return Split::At(n / 2);
    }
    Split::At(i)
}

// ---------------------------------------------------------------------------
// Primitive tests
// ---------------------------------------------------------------------------

#[inline]
fn point_aabb_dist_sq(p: Vec3, mn: &[f32; 3], mx: &[f32; 3]) -> f32 {
    let mn = Vec3::from(*mn);
    let mx = Vec3::from(*mx);
    let d = (mn - p).max(p - mx).max(Vec3::ZERO);
    d.length_squared()
}

pub fn closest_point_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> (Vec3, f32) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, ap.length_squared());
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, bp.length_squared());
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let q = a + ab * v;
        return (q, (p - q).length_squared());
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, cp.length_squared());
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let q = a + ac * w;
        return (q, (p - q).length_squared());
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let q = b + (c - b) * w;
        return (q, (p - q).length_squared());
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let q = a + ab * v + ac * w;
    (q, (p - q).length_squared())
}

/// Slab test with a precomputed inverse direction. Returns the entry
/// distance (clamped to 0 when the origin is inside the box).
#[inline]
fn ray_aabb(ro: Vec3, inv: Vec3, mn: &[f32; 3], mx: &[f32; 3]) -> Option<f32> {
    let mn = Vec3::from(*mn);
    let mx = Vec3::from(*mx);
    let t0 = (mn - ro) * inv;
    let t1 = (mx - ro) * inv;
    let tmin_v = t0.min(t1);
    let tmax_v = t0.max(t1);
    let tmin = tmin_v.x.max(tmin_v.y).max(tmin_v.z).max(0.0);
    let tmax = tmax_v.x.min(tmax_v.y).min(tmax_v.z);
    if tmax < tmin || tmax.is_nan() || tmin.is_nan() {
        None
    } else {
        Some(tmin)
    }
}

pub fn ray_triangle(ro: Vec3, rd: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    let e1 = b - a;
    let e2 = c - a;
    let pvec = rd.cross(e2);
    let det = e1.dot(pvec);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let tvec = ro - a;
    let u = tvec.dot(pvec) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qvec = tvec.cross(e1);
    let v = rd.dot(qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(qvec) * inv_det;
    if t > 1e-6 { Some(t) } else { None }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    fn random_mesh(n: usize, seed: u64) -> (Vec<[f32; 3]>, Vec<u32>) {
        let mut rng = Rng::new(seed);
        let mut pos = Vec::with_capacity(n * 3);
        let mut idx = Vec::with_capacity(n * 3);
        for i in 0..n {
            let c = [rng.f32() * 100.0, rng.f32() * 100.0, rng.f32() * 100.0];
            for _ in 0..3 {
                pos.push([
                    c[0] + rng.f32() * 3.0,
                    c[1] + rng.f32() * 3.0,
                    c[2] + rng.f32() * 3.0,
                ]);
            }
            idx.extend_from_slice(&[(3 * i) as u32, (3 * i + 1) as u32, (3 * i + 2) as u32]);
        }
        (pos, idx)
    }

    #[test]
    fn every_triangle_is_in_exactly_one_leaf() {
        let (pos, idx) = random_mesh(5000, 7);
        let bvh = Bvh::new(&pos, &idx);
        let mut seen = vec![0u32; 5000];
        for n in &bvh.nodes {
            if n.count > 0 {
                let first = n.right_or_first as usize;
                for &t in &bvh.tri_order[first..first + n.count as usize] {
                    seen[t as usize] += 1;
                }
            }
        }
        assert!(seen.iter().all(|&s| s == 1));
        // Child bounds are contained in their parent's bounds.
        for (i, n) in bvh.nodes.iter().enumerate() {
            if n.count == 0 {
                for child in [i + 1, n.right_or_first as usize] {
                    let c = &bvh.nodes[child];
                    for a in 0..3 {
                        assert!(c.min[a] >= n.min[a] - 1e-6 && c.max[a] <= n.max[a] + 1e-6);
                    }
                }
            }
        }
    }

    #[test]
    fn sah_closest_point_matches_bruteforce() {
        let (pos, idx) = random_mesh(3000, 11);
        let bvh = Bvh::new(&pos, &idx);
        let mut rng = Rng::new(99);
        for _ in 0..200 {
            let p = Vec3::new(
                rng.f32() * 120.0 - 10.0,
                rng.f32() * 120.0 - 10.0,
                rng.f32() * 120.0 - 10.0,
            );
            let (_, _, d) = bvh.closest_point(p);
            let mut best = f32::MAX;
            for t in idx.chunks_exact(3) {
                let a = Vec3::from(pos[t[0] as usize]);
                let b = Vec3::from(pos[t[1] as usize]);
                let c = Vec3::from(pos[t[2] as usize]);
                best = best.min(closest_point_triangle(p, a, b, c).1);
            }
            assert!(
                (d - best.sqrt()).abs() < 1e-3,
                "bvh {d} vs brute {}",
                best.sqrt()
            );
        }
    }

    #[test]
    fn sah_ray_cast_matches_bruteforce() {
        let (pos, idx) = random_mesh(3000, 13);
        let bvh = Bvh::new(&pos, &idx);
        let mut rng = Rng::new(5);
        let mut hits = 0;
        for _ in 0..300 {
            let ro = Vec3::new(rng.f32() * 100.0, rng.f32() * 100.0, -20.0);
            let rd = Vec3::new(rng.f32() - 0.5, rng.f32() - 0.5, 1.0).normalize();
            let got = bvh.ray_cast(ro, rd, f32::INFINITY);
            let mut best: Option<f32> = None;
            for t in idx.chunks_exact(3) {
                let a = Vec3::from(pos[t[0] as usize]);
                let b = Vec3::from(pos[t[1] as usize]);
                let c = Vec3::from(pos[t[2] as usize]);
                if let Some(th) = ray_triangle(ro, rd, a, b, c) {
                    best = Some(best.map_or(th, |x: f32| x.min(th)));
                }
            }
            match (got, best) {
                (Some((t, _)), Some(b)) => {
                    hits += 1;
                    assert!((t - b).abs() < 1e-3);
                }
                (None, None) => {}
                other => panic!("mismatch {other:?}"),
            }
        }
        assert!(hits > 10);
    }

    #[test]
    fn sphere_query_finds_all_nearby_triangles() {
        let (pos, idx) = random_mesh(2000, 17);
        let bvh = Bvh::new(&pos, &idx);
        let center = Vec3::new(50.0, 50.0, 50.0);
        let r = 8.0;
        let mut out = Vec::new();
        bvh.query_sphere(center, r, &mut out);
        out.sort_unstable();
        let mut brute = Vec::new();
        for (t, tri) in idx.chunks_exact(3).enumerate() {
            let a = Vec3::from(pos[tri[0] as usize]);
            let b = Vec3::from(pos[tri[1] as usize]);
            let c = Vec3::from(pos[tri[2] as usize]);
            if closest_point_triangle(center, a, b, c).1 <= r * r {
                brute.push(t as u32);
            }
        }
        assert_eq!(out, brute);
    }

    #[test]
    fn empty_and_degenerate_inputs() {
        let bvh = Bvh::new(&[], &[]);
        assert!(bvh.ray_cast(Vec3::ZERO, Vec3::Z, 10.0).is_none());
        assert_eq!(bvh.closest_distance(Vec3::ZERO), f32::INFINITY);
        // All centroids identical.
        let pos = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let idx: Vec<u32> = (0..40).flat_map(|_| [0u32, 1, 2]).collect();
        let bvh = Bvh::new(&pos, &idx);
        assert!(bvh.closest_distance(Vec3::new(0.2, 0.2, 5.0)) - 5.0 < 1e-5);
    }
}
