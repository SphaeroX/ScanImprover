use crate::geom::boundary::EdgeKey;
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;

/// Mesh topology dual graph providing fast triangle-to-triangle edge adjacency
/// and dihedral angle calculations for selection operations.
pub struct MeshTopology {
    /// For each triangle index t, the edge-adjacent neighbors [neighbor_e0, neighbor_e1, neighbor_e2].
    /// u32::MAX indicates an open boundary edge (no neighbor).
    pub neighbors: Vec<[u32; 3]>,
    /// Unit face normal for each triangle.
    pub face_normals: Vec<Vec3>,
    #[allow(dead_code)]
    pub centroids: Vec<Vec3>,
}

impl MeshTopology {
    /// Builds the topology dual-graph from a mesh.
    pub fn build(mesh: &Mesh) -> Self {
        let nt = mesh.triangle_count();
        if nt == 0 {
            return Self {
                neighbors: Vec::new(),
                face_normals: Vec::new(),
                centroids: Vec::new(),
            };
        }

        let positions = &mesh.positions;
        let indices = &mesh.indices;

        // Compute face normals and centroids in parallel
        let (face_normals, centroids): (Vec<Vec3>, Vec<Vec3>) = (0..nt)
            .into_par_iter()
            .map(|t| {
                let i0 = indices[3 * t] as usize;
                let i1 = indices[3 * t + 1] as usize;
                let i2 = indices[3 * t + 2] as usize;
                let a = Vec3::from(positions[i0]);
                let b = Vec3::from(positions[i1]);
                let c = Vec3::from(positions[i2]);
                let normal = (b - a).cross(c - a).normalize_or_zero();
                let centroid = (a + b + c) * (1.0 / 3.0);
                (normal, centroid)
            })
            .unzip();

        // Build edge to triangles mapping
        // We collect all (EdgeKey, tri_idx, edge_local_idx)
        let mut raw_edges: Vec<(EdgeKey, u32, u8)> = Vec::with_capacity(nt * 3);
        for t in 0..nt {
            let i0 = indices[3 * t];
            let i1 = indices[3 * t + 1];
            let i2 = indices[3 * t + 2];
            let t_u32 = t as u32;
            raw_edges.push((EdgeKey::new(i0, i1), t_u32, 0));
            raw_edges.push((EdgeKey::new(i1, i2), t_u32, 1));
            raw_edges.push((EdgeKey::new(i2, i0), t_u32, 2));
        }

        raw_edges.par_sort_unstable_by_key(|e| e.0);

        let mut neighbors = vec![[u32::MAX; 3]; nt];
        let n = raw_edges.len();
        let mut i = 0;
        while i < n {
            let mut j = i + 1;
            while j < n && raw_edges[j].0 == raw_edges[i].0 {
                j += 1;
            }
            if j - i == 2 {
                let (_, t0, e0) = raw_edges[i];
                let (_, t1, e1) = raw_edges[i + 1];
                neighbors[t0 as usize][e0 as usize] = t1;
                neighbors[t1 as usize][e1 as usize] = t0;
            } else if j - i > 2 {
                // Non-manifold edge: connect adjacent pairs
                for k in i..j {
                    let next = if k + 1 < j { k + 1 } else { i };
                    let (_, t0, e0) = raw_edges[k];
                    let (_, t1, _) = raw_edges[next];
                    neighbors[t0 as usize][e0 as usize] = t1;
                }
            }
            i = j;
        }

        Self {
            neighbors,
            face_normals,
            centroids,
        }
    }

    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.neighbors.len()
    }

    #[inline]
    #[allow(dead_code)]
    pub fn triangle_normal(&self, tri: u32) -> Vec3 {
        self.face_normals[tri as usize]
    }

    /// Dihedral angle in radians between two triangles.
    #[inline]
    #[allow(dead_code)]
    pub fn dihedral_angle(&self, tri_a: u32, tri_b: u32) -> f32 {
        let na = self.face_normals[tri_a as usize];
        let nb = self.face_normals[tri_b as usize];
        let dot = na.dot(nb).clamp(-1.0, 1.0);
        dot.acos()
    }
}

/// Expands the current selection by 1 ring of adjacent triangles across shared edges,
/// stopping at any edge where the dihedral angle exceeds `angle_threshold_rad`.
pub fn grow_selection(
    topo: &MeshTopology,
    current_sel: &[u8],
    angle_threshold_rad: f32,
) -> Vec<u8> {
    let nt = topo.triangle_count();
    if current_sel.len() != nt {
        return current_sel.to_vec();
    }

    let mut next_sel = current_sel.to_vec();
    for t in 0..nt {
        if current_sel[t] == 0 {
            let na = topo.face_normals[t];
            for &nb in &topo.neighbors[t] {
                if nb != u32::MAX && current_sel[nb as usize] > 0 {
                    let nb_normal = topo.face_normals[nb as usize];
                    let angle = na.dot(nb_normal).clamp(-1.0, 1.0).acos();
                    if angle <= angle_threshold_rad {
                        next_sel[t] = 1;
                        break;
                    }
                }
            }
        }
    }
    next_sel
}

/// Shrinks the current selection by 1 ring of triangles along the selection boundary.
/// Any selected triangle adjacent to an unselected triangle or an open mesh edge is deselected.
pub fn shrink_selection(
    topo: &MeshTopology,
    current_sel: &[u8],
) -> Vec<u8> {
    let nt = topo.triangle_count();
    if current_sel.len() != nt {
        return current_sel.to_vec();
    }

    let mut next_sel = current_sel.to_vec();
    for t in 0..nt {
        if current_sel[t] > 0 {
            let mut is_boundary = false;
            for &nb in &topo.neighbors[t] {
                if nb == u32::MAX || current_sel[nb as usize] == 0 {
                    is_boundary = true;
                    break;
                }
            }
            if is_boundary {
                next_sel[t] = 0;
            }
        }
    }
    next_sel
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_topology_grow_and_shrink() {
        // Two triangles sharing an edge, flat (angle = 0)
        let positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ];
        // Triangle 0: (0, 1, 2), Triangle 1: (1, 3, 2)
        let indices = vec![0, 1, 2, 1, 3, 2];
        let mesh = Mesh::from_indexed(positions, indices);
        let topo = MeshTopology::build(&mesh);

        assert_eq!(topo.triangle_count(), 2);
        assert_eq!(topo.neighbors[0].iter().filter(|&&nb| nb == 1).count(), 1);
        assert_eq!(topo.neighbors[1].iter().filter(|&&nb| nb == 0).count(), 1);

        let sel = vec![1u8, 0u8];
        let grown = grow_selection(&topo, &sel, 0.1);
        assert_eq!(grown, vec![1u8, 1u8]);

        let shrunk = shrink_selection(&topo, &grown);
        // Since both have open boundary edges, both shrink
        assert_eq!(shrunk, vec![0u8, 0u8]);
    }

    #[test]
    fn test_topology_angle_threshold() {
        // Two triangles at 90 degrees (angle = PI/2)
        let positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        // T0 on XY plane: (0, 1, 2)
        // T1 on XZ plane: (0, 3, 1)
        let indices = vec![0, 1, 2, 0, 3, 1];
        let mesh = Mesh::from_indexed(positions, indices);
        let topo = MeshTopology::build(&mesh);

        let angle = topo.dihedral_angle(0, 1);
        assert!((angle - PI * 0.5).abs() < 1e-4);

        let sel = vec![1u8, 0u8];
        // Threshold 45 deg < 90 deg -> should NOT grow
        let grown_blocked = grow_selection(&topo, &sel, 45.0f32.to_radians());
        assert_eq!(grown_blocked, vec![1u8, 0u8]);

        // Threshold 100 deg > 90 deg -> SHOULD grow
        let grown_allowed = grow_selection(&topo, &sel, 100.0f32.to_radians());
        assert_eq!(grown_allowed, vec![1u8, 1u8]);
    }
}
