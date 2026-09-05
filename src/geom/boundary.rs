use crate::mesh::Mesh;
use rayon::prelude::*;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct EdgeKey(pub u32, pub u32);

impl EdgeKey {
    #[inline]
    pub fn new(u: u32, v: u32) -> Self {
        if u < v {
            EdgeKey(u, v)
        } else {
            EdgeKey(v, u)
        }
    }
}

pub struct BoundaryEdge {
    #[allow(dead_code)]
    pub key: EdgeKey,
    pub directed: (u32, u32),
    pub triangle: u32,
}

/// Finds all open boundary edges of a triangle mesh (edges incident to exactly one triangle).
pub fn find_boundary_edges(mesh: &Mesh) -> Vec<BoundaryEdge> {
    let nt = mesh.triangle_count();
    if nt == 0 {
        return Vec::new();
    }

    let mut raw_edges: Vec<(EdgeKey, (u32, u32), u32)> = Vec::with_capacity(nt * 3);
    for t in 0..nt {
        let i0 = mesh.indices[3 * t];
        let i1 = mesh.indices[3 * t + 1];
        let i2 = mesh.indices[3 * t + 2];
        raw_edges.push((EdgeKey::new(i0, i1), (i0, i1), t as u32));
        raw_edges.push((EdgeKey::new(i1, i2), (i1, i2), t as u32));
        raw_edges.push((EdgeKey::new(i2, i0), (i2, i0), t as u32));
    }

    raw_edges.par_sort_unstable_by_key(|e| e.0);

    let mut boundary = Vec::new();
    let n = raw_edges.len();
    let mut i = 0;
    while i < n {
        let mut j = i + 1;
        while j < n && raw_edges[j].0 == raw_edges[i].0 {
            j += 1;
        }
        if j - i == 1 {
            boundary.push(BoundaryEdge {
                key: raw_edges[i].0,
                directed: raw_edges[i].1,
                triangle: raw_edges[i].2,
            });
        }
        i = j;
    }
    boundary
}

/// Generates a triangle exclusion mask for all holes and open boundaries with k-ring dilation.
/// Triangles marked with 1 are excluded; triangles with 0 are kept.
pub fn generate_hole_mask(mesh: &Mesh, dilation_rings: usize) -> Vec<u8> {
    let nt = mesh.triangle_count();
    let nv = mesh.vertex_count();
    if nt == 0 || nv == 0 {
        return Vec::new();
    }

    let b_edges = find_boundary_edges(mesh);
    if b_edges.is_empty() {
        return vec![0u8; nt];
    }

    // Build CSR vertex -> triangles adjacency
    let mut offsets = vec![0u32; nv + 1];
    for &idx in &mesh.indices {
        offsets[idx as usize + 1] += 1;
    }
    for v in 0..nv {
        offsets[v + 1] += offsets[v];
    }
    let mut fill = vec![0u32; nv];
    let mut tris_of_vert = vec![0u32; mesh.indices.len()];
    for t in 0..nt {
        for k in 0..3 {
            let v = mesh.indices[3 * t + k] as usize;
            tris_of_vert[offsets[v] as usize + fill[v] as usize] = t as u32;
            fill[v] += 1;
        }
    }

    // Mark boundary vertices
    let mut active_verts = vec![false; nv];
    for e in &b_edges {
        active_verts[e.directed.0 as usize] = true;
        active_verts[e.directed.1 as usize] = true;
    }

    // Perform k-ring dilation to buffer hole edges
    let mut tri_mask = vec![0u8; nt];
    let rings = dilation_rings.max(1);

    for _ in 0..rings {
        let mut next_verts = vec![false; nv];
        for v in 0..nv {
            if !active_verts[v] {
                continue;
            }
            for k in offsets[v]..offsets[v + 1] {
                let t = tris_of_vert[k as usize] as usize;
                tri_mask[t] = 1;
                let i0 = mesh.indices[3 * t] as usize;
                let i1 = mesh.indices[3 * t + 1] as usize;
                let i2 = mesh.indices[3 * t + 2] as usize;
                next_verts[i0] = true;
                next_verts[i1] = true;
                next_verts[i2] = true;
            }
        }
        active_verts = next_verts;
    }

    // Safeguard: If heavily fragmented scan causes > 40% of triangles to be masked,
    // fallback to 0-ring dilation (only direct boundary triangles)
    let masked_count = tri_mask.iter().filter(|&&m| m > 0).count();
    if masked_count > (nt * 4) / 10 {
        let mut fallback_mask = vec![0u8; nt];
        for e in &b_edges {
            fallback_mask[e.triangle as usize] = 1;
        }
        return fallback_mask;
    }

    tri_mask
}
