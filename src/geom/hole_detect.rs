use crate::geom::boundary::{find_boundary_edges, EdgeKey};
use crate::mesh::{Aabb, Mesh};
use glam::Vec3;
use std::collections::HashMap;

/// A detected closed boundary loop (hole) on a 3D mesh.
#[derive(Clone, Debug)]
pub struct HoleLoop {
    /// 1-based sequential identifier for user display.
    pub id: usize,
    /// Ordered list of vertex indices tracing the boundary cycle:
    /// v_0 -> v_1 -> ... -> v_{n-1} -> v_0
    pub vertices: Vec<u32>,
    /// Total perimeter of the hole in world units (mm).
    pub perimeter: f32,
    /// Approximate planar surface area enclosed by the loop (in mm^2).
    pub approx_area: f32,
    /// Centroid of the boundary vertices in 3D.
    pub centroid: Vec3,
    /// Estimated average normal vector of the hole boundary.
    pub normal: Vec3,
    /// Axis-aligned bounding box enclosing all loop vertices.
    pub bbox: Aabb,
}

impl HoleLoop {
    /// Returns the number of edges / vertices in this hole loop.
    #[inline]
    pub fn edge_count(&self) -> usize {
        self.vertices.len()
    }
}

/// Detects all closed boundary loops (holes) in the given mesh.
/// Loops are ordered cycles of directed edges along the boundary.
pub fn detect_holes(mesh: &Mesh) -> Vec<HoleLoop> {
    let b_edges = find_boundary_edges(mesh);
    if b_edges.is_empty() {
        return Vec::new();
    }

    // Build directed adjacency map from boundary half-edges: u -> list of v
    // In a manifold boundary, each vertex has exactly one outgoing boundary edge (u -> v).
    // In non-manifold boundary vertices (figure-8 / pinch points), a vertex may have > 1 outgoing edges.
    let mut adj: HashMap<u32, Vec<u32>> = HashMap::with_capacity(b_edges.len());
    for e in &b_edges {
        // e.directed is (u, v) from the single triangle that owns the boundary edge
        adj.entry(e.directed.0).or_default().push(e.directed.1);
    }

    let mut visited_edges: HashMap<EdgeKey, usize> = HashMap::with_capacity(b_edges.len());
    let mut raw_loops: Vec<Vec<u32>> = Vec::new();

    // Trace loops
    for e in &b_edges {
        let key = EdgeKey::new(e.directed.0, e.directed.1);
        if visited_edges.contains_key(&key) {
            continue;
        }

        let start_v = e.directed.0;
        let mut loop_verts = Vec::new();
        let mut curr_v = start_v;

        while let Some(next_candidates) = adj.get_mut(&curr_v) {
            if next_candidates.is_empty() {
                break;
            }

            // Find an unvisited outgoing edge
            let mut chosen_idx = None;
            for (idx, &next_v) in next_candidates.iter().enumerate() {
                let ek = EdgeKey::new(curr_v, next_v);
                if !visited_edges.contains_key(&ek) {
                    chosen_idx = Some(idx);
                    break;
                }
            }

            match chosen_idx {
                Some(idx) => {
                    let next_v = next_candidates[idx];
                    let ek = EdgeKey::new(curr_v, next_v);
                    visited_edges.insert(ek, raw_loops.len());
                    loop_verts.push(curr_v);
                    curr_v = next_v;

                    if curr_v == start_v {
                        // Loop completed!
                        break;
                    }
                }
                None => break,
            }
        }

        if loop_verts.len() >= 3 && curr_v == start_v {
            raw_loops.push(loop_verts);
        }
    }

    let positions = &mesh.positions;
    let mut holes = Vec::with_capacity(raw_loops.len());

    for (i, verts) in raw_loops.into_iter().enumerate() {
        let n = verts.len();
        if n < 3 {
            continue;
        }

        let mut perimeter = 0.0f32;
        let mut sum_pos = Vec3::ZERO;
        let mut min_pt = Vec3::splat(f32::MAX);
        let mut max_pt = Vec3::splat(f32::MIN);

        for idx in 0..n {
            let v0 = verts[idx] as usize;
            let v1 = verts[(idx + 1) % n] as usize;
            let p0 = Vec3::from(positions[v0]);
            let p1 = Vec3::from(positions[v1]);

            perimeter += (p1 - p0).length();
            sum_pos += p0;
            min_pt = min_pt.min(p0);
            max_pt = max_pt.max(p0);
        }

        let centroid = sum_pos / (n as f32);

        // Compute polygon normal and approximate area using Newell's method
        let mut normal_acc = Vec3::ZERO;
        for idx in 0..n {
            let v0 = verts[idx] as usize;
            let v1 = verts[(idx + 1) % n] as usize;
            let p0 = Vec3::from(positions[v0]) - centroid;
            let p1 = Vec3::from(positions[v1]) - centroid;
            normal_acc += p0.cross(p1);
        }

        let cross_len = normal_acc.length();
        let approx_area = cross_len * 0.5;
        let normal = if cross_len > 1e-12 {
            normal_acc / cross_len
        } else {
            Vec3::Y
        };

        holes.push(HoleLoop {
            id: i + 1,
            vertices: verts,
            perimeter,
            approx_area,
            centroid,
            normal,
            bbox: Aabb {
                min: min_pt,
                max: max_pt,
            },
        });
    }

    // Sort holes by perimeter descending so largest holes appear first, but preserve sequential IDs
    holes.sort_by(|a, b| b.perimeter.partial_cmp(&a.perimeter).unwrap_or(std::cmp::Ordering::Equal));
    for (i, h) in holes.iter_mut().enumerate() {
        h.id = i + 1;
    }

    holes
}
