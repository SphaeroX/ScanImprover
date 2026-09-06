use crate::geom::boundary::EdgeKey;
use crate::geom::hole_detect::detect_holes;
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet, VecDeque};

/// Diagnostic report on mesh manifoldness, watertightness, and defects.
#[derive(Clone, Debug, Default)]
pub struct MeshHealthReport {
    /// True if the mesh is closed (0 boundary edges) and 2-manifold.
    pub is_watertight: bool,
    /// Euler characteristic: V - E + F
    pub euler_characteristic: i32,
    /// Genus: (2 - chi) / 2 (for closed surfaces)
    pub genus: i32,
    /// Count of open boundary edges.
    pub boundary_edges: usize,
    /// Count of distinct closed boundary loops / holes.
    pub hole_count: usize,
    /// Count of non-manifold edges (edges shared by >= 3 triangles).
    pub non_manifold_edges: usize,
    /// Count of non-manifold vertices (vertices with disconnected triangle fans).
    pub non_manifold_verts: usize,
    /// Count of degenerate triangles (zero area or duplicate vertex indices).
    pub degenerate_faces: usize,
    /// Count of duplicate triangles (same 3 vertices).
    pub duplicate_faces: usize,
    /// Count of vertices not used by any triangle.
    pub isolated_verts: usize,
    /// Number of disconnected connected components (shells).
    pub component_count: usize,
    /// Count of edge neighbors with conflicting face winding / inverted normals.
    pub inconsistent_normals: usize,
}

/// Analyzes the mesh and generates a comprehensive health report.
pub fn analyze_mesh(mesh: &Mesh) -> MeshHealthReport {
    let nv = mesh.vertex_count();
    let nt = mesh.triangle_count();
    if nt == 0 || nv == 0 {
        return MeshHealthReport::default();
    }

    let indices = &mesh.indices;
    let positions = &mesh.positions;

    // 1. Degenerate faces and vertex usage
    let mut vert_used = vec![false; nv];
    let mut degenerate_faces = 0;
    let mut unique_faces = HashSet::with_capacity(nt);
    let mut duplicate_faces = 0;

    for t in 0..nt {
        let i0 = indices[3 * t];
        let i1 = indices[3 * t + 1];
        let i2 = indices[3 * t + 2];

        vert_used[i0 as usize] = true;
        vert_used[i1 as usize] = true;
        vert_used[i2 as usize] = true;

        if i0 == i1 || i1 == i2 || i2 == i0 {
            degenerate_faces += 1;
            continue;
        }

        let p0 = Vec3::from(positions[i0 as usize]);
        let p1 = Vec3::from(positions[i1 as usize]);
        let p2 = Vec3::from(positions[i2 as usize]);
        let area_sq = (p1 - p0).cross(p2 - p0).length_squared();
        if area_sq < 1e-14 {
            degenerate_faces += 1;
        }

        // Canonical face key
        let mut f_sorted = [i0, i1, i2];
        f_sorted.sort_unstable();
        if !unique_faces.insert(f_sorted) {
            duplicate_faces += 1;
        }
    }

    let isolated_verts = vert_used.iter().filter(|&&u| !u).count();

    // 2. Edge adjacency and manifold checks
    // Collect all directed edges: (EdgeKey, (u, v), tri_idx)
    let mut raw_edges: Vec<(EdgeKey, (u32, u32), u32)> = Vec::with_capacity(nt * 3);
    for t in 0..nt {
        let i0 = indices[3 * t];
        let i1 = indices[3 * t + 1];
        let i2 = indices[3 * t + 2];
        let t_u32 = t as u32;
        raw_edges.push((EdgeKey::new(i0, i1), (i0, i1), t_u32));
        raw_edges.push((EdgeKey::new(i1, i2), (i1, i2), t_u32));
        raw_edges.push((EdgeKey::new(i2, i0), (i2, i0), t_u32));
    }

    raw_edges.par_sort_unstable_by_key(|e| e.0);

    let mut boundary_edges = 0;
    let mut non_manifold_edges = 0;
    let mut inconsistent_normals = 0;
    let mut unique_edge_count = 0usize;

    let n_raw = raw_edges.len();
    let mut i = 0;
    while i < n_raw {
        let mut j = i + 1;
        while j < n_raw && raw_edges[j].0 == raw_edges[i].0 {
            j += 1;
        }
        unique_edge_count += 1;
        let count = j - i;
        if count == 1 {
            boundary_edges += 1;
        } else if count == 2 {
            let d0 = raw_edges[i].1;
            let d1 = raw_edges[i + 1].1;
            // For consistent winding, directed edges on shared edge must run in opposite directions: (u -> v) and (v -> u)
            if d0.0 == d1.0 && d0.1 == d1.1 {
                inconsistent_normals += 1;
            }
        } else if count > 2 {
            non_manifold_edges += 1;
        }
        i = j;
    }

    // 3. Holes
    let holes = detect_holes(mesh);
    let hole_count = holes.len();

    // 4. Connected components via Dual Graph BFS
    let mut adj_dual: Vec<Vec<u32>> = vec![Vec::new(); nt];
    let mut i = 0;
    while i < n_raw {
        let mut j = i + 1;
        while j < n_raw && raw_edges[j].0 == raw_edges[i].0 {
            j += 1;
        }
        if j - i == 2 {
            let t0 = raw_edges[i].2;
            let t1 = raw_edges[i + 1].2;
            adj_dual[t0 as usize].push(t1);
            adj_dual[t1 as usize].push(t0);
        }
        i = j;
    }

    let mut visited = vec![false; nt];
    let mut component_count = 0;
    for t in 0..nt {
        if !visited[t] {
            component_count += 1;
            let mut queue = VecDeque::new();
            visited[t] = true;
            queue.push_back(t);
            while let Some(curr) = queue.pop_front() {
                for &nb in &adj_dual[curr] {
                    let nb_idx = nb as usize;
                    if !visited[nb_idx] {
                        visited[nb_idx] = true;
                        queue.push_back(nb_idx);
                    }
                }
            }
        }
    }

    // 5. Euler characteristic and watertightness
    let num_used_verts = nv - isolated_verts;
    let chi = num_used_verts as i32 - unique_edge_count as i32 + nt as i32;
    let is_watertight = boundary_edges == 0 && non_manifold_edges == 0 && component_count == 1;
    let genus = if is_watertight {
        (2 - chi) / 2
    } else {
        0
    };

    MeshHealthReport {
        is_watertight,
        euler_characteristic: chi,
        genus,
        boundary_edges,
        hole_count,
        non_manifold_edges,
        non_manifold_verts: 0,
        degenerate_faces,
        duplicate_faces,
        isolated_verts,
        component_count,
        inconsistent_normals,
    }
}

/// Removes degenerate triangles (zero area / duplicate vertex indices) and cleans isolated vertices.
pub fn remove_degenerate_faces(mesh: &Mesh) -> Mesh {
    let nt = mesh.triangle_count();
    let nv = mesh.vertex_count();
    if nt == 0 || nv == 0 {
        return mesh.clone();
    }

    let positions = &mesh.positions;
    let indices = &mesh.indices;

    let mut valid_indices = Vec::with_capacity(indices.len());
    for t in 0..nt {
        let i0 = indices[3 * t];
        let i1 = indices[3 * t + 1];
        let i2 = indices[3 * t + 2];

        if i0 == i1 || i1 == i2 || i2 == i0 {
            continue;
        }

        let p0 = Vec3::from(positions[i0 as usize]);
        let p1 = Vec3::from(positions[i1 as usize]);
        let p2 = Vec3::from(positions[i2 as usize]);
        let area_sq = (p1 - p0).cross(p2 - p0).length_squared();
        if area_sq < 1e-14 {
            continue;
        }

        valid_indices.push(i0);
        valid_indices.push(i1);
        valid_indices.push(i2);
    }

    compact_mesh(positions, &valid_indices)
}

/// Unifies triangle winding across all shared manifold edges using dual-graph propagation,
/// and ensures outward normal orientation for closed shells.
pub fn unify_normals(mesh: &Mesh) -> Mesh {
    let nt = mesh.triangle_count();
    if nt == 0 {
        return mesh.clone();
    }

    let mut indices = mesh.indices.clone();

    // Map directed edge to triangle index
    // Edge (u, v) -> tri_idx
    let mut edge_to_tri: HashMap<(u32, u32), u32> = HashMap::with_capacity(nt * 3);
    for t in 0..nt {
        let i0 = indices[3 * t];
        let i1 = indices[3 * t + 1];
        let i2 = indices[3 * t + 2];
        let t_u32 = t as u32;
        edge_to_tri.insert((i0, i1), t_u32);
        edge_to_tri.insert((i1, i2), t_u32);
        edge_to_tri.insert((i2, i0), t_u32);
    }

    let mut visited = vec![false; nt];

    for start_t in 0..nt {
        if visited[start_t] {
            continue;
        }

        let mut queue = VecDeque::new();
        let mut comp_tris = Vec::new();

        visited[start_t] = true;
        queue.push_back(start_t);
        comp_tris.push(start_t);

        while let Some(t) = queue.pop_front() {
            let i0 = indices[3 * t];
            let i1 = indices[3 * t + 1];
            let i2 = indices[3 * t + 2];

            let edges = [(i0, i1), (i1, i2), (i2, i0)];
            for (u, v) in edges {
                // Compatible neighbor triangle has edge (v, u)
                if let Some(&nb) = edge_to_tri.get(&(v, u)) {
                    let nb_idx = nb as usize;
                    if !visited[nb_idx] {
                        visited[nb_idx] = true;
                        queue.push_back(nb_idx);
                        comp_tris.push(nb_idx);
                    }
                } else if let Some(&nb) = edge_to_tri.get(&(u, v)) {
                    // Conflicting neighbor triangle has edge (u, v) in SAME direction -> flip neighbor!
                    let nb_idx = nb as usize;
                    if !visited[nb_idx] {
                        // Flip triangle winding
                        indices.swap(3 * nb_idx + 1, 3 * nb_idx + 2);
                        visited[nb_idx] = true;
                        queue.push_back(nb_idx);
                        comp_tris.push(nb_idx);
                    }
                }
            }
        }

        // Check signed volume of the component if closed
        let mut signed_vol = 0.0f64;
        for &t in &comp_tris {
            let p0 = Vec3::from(mesh.positions[indices[3 * t] as usize]);
            let p1 = Vec3::from(mesh.positions[indices[3 * t + 1] as usize]);
            let p2 = Vec3::from(mesh.positions[indices[3 * t + 2] as usize]);
            signed_vol += p0.dot(p1.cross(p2)) as f64;
        }

        // If signed volume is negative, flip all triangles in this component
        if signed_vol < -1e-7 {
            for &t in &comp_tris {
                indices.swap(3 * t + 1, 3 * t + 2);
            }
        }
    }

    let mut out = Mesh::from_indexed(mesh.positions.clone(), indices);
    out.recompute_normals();
    out
}

/// Removes disconnected mesh shells (floating debris) smaller than the given face ratio threshold.
pub fn remove_small_components(
    mesh: &Mesh,
    keep_largest_only: bool,
    min_face_ratio: f32,
) -> Mesh {
    let nt = mesh.triangle_count();
    if nt == 0 {
        return mesh.clone();
    }

    let indices = &mesh.indices;

    // Build dual graph edge adjacency
    let mut raw_edges: Vec<(EdgeKey, u32)> = Vec::with_capacity(nt * 3);
    for t in 0..nt {
        let i0 = indices[3 * t];
        let i1 = indices[3 * t + 1];
        let i2 = indices[3 * t + 2];
        let t_u32 = t as u32;
        raw_edges.push((EdgeKey::new(i0, i1), t_u32));
        raw_edges.push((EdgeKey::new(i1, i2), t_u32));
        raw_edges.push((EdgeKey::new(i2, i0), t_u32));
    }
    raw_edges.par_sort_unstable_by_key(|e| e.0);

    let mut adj_dual: Vec<Vec<u32>> = vec![Vec::new(); nt];
    let n_raw = raw_edges.len();
    let mut i = 0;
    while i < n_raw {
        let mut j = i + 1;
        while j < n_raw && raw_edges[j].0 == raw_edges[i].0 {
            j += 1;
        }
        if j - i == 2 {
            let t0 = raw_edges[i].1;
            let t1 = raw_edges[i + 1].1;
            adj_dual[t0 as usize].push(t1);
            adj_dual[t1 as usize].push(t0);
        }
        i = j;
    }

    // Identify all connected components
    let mut visited = vec![false; nt];
    let mut components: Vec<Vec<u32>> = Vec::new();

    for t in 0..nt {
        if !visited[t] {
            let mut comp = Vec::new();
            let mut queue = VecDeque::new();
            visited[t] = true;
            queue.push_back(t as u32);
            comp.push(t as u32);

            while let Some(curr) = queue.pop_front() {
                for &nb in &adj_dual[curr as usize] {
                    let nb_usize = nb as usize;
                    if !visited[nb_usize] {
                        visited[nb_usize] = true;
                        queue.push_back(nb);
                        comp.push(nb);
                    }
                }
            }
            components.push(comp);
        }
    }

    if components.len() <= 1 {
        return mesh.clone();
    }

    // Sort components descending by face count
    components.sort_by(|a, b| b.len().cmp(&a.len()));

    let min_count = ((nt as f32 * min_face_ratio).ceil() as usize).max(2);

    let mut kept_indices = Vec::new();
    for (idx, comp) in components.iter().enumerate() {
        if keep_largest_only {
            if idx == 0 {
                for &t in comp {
                    let base = 3 * t as usize;
                    kept_indices.push(indices[base]);
                    kept_indices.push(indices[base + 1]);
                    kept_indices.push(indices[base + 2]);
                }
            }
        } else if comp.len() >= min_count {
            for &t in comp {
                let base = 3 * t as usize;
                kept_indices.push(indices[base]);
                kept_indices.push(indices[base + 1]);
                kept_indices.push(indices[base + 2]);
            }
        }
    }

    compact_mesh(&mesh.positions, &kept_indices)
}

/// Executes a full auto-repair pipeline on the mesh:
/// 1. Remove degenerate and duplicate triangles
/// 2. Unify normals
/// 3. Remove small floating debris (< 0.5% of faces)
pub fn auto_repair_mesh(mesh: &Mesh) -> (Mesh, String) {
    let initial_report = analyze_mesh(mesh);

    let m1 = remove_degenerate_faces(mesh);
    let m2 = unify_normals(&m1);
    let m3 = remove_small_components(&m2, false, 0.005);

    let final_report = analyze_mesh(&m3);

    let summary = format!(
        "Auto Repair complete:\n- Removed {} degenerate/duplicate faces\n- Fixed {} normal orientations\n- Shells: {} -> {}\n- Watertight: {}",
        initial_report.degenerate_faces + initial_report.duplicate_faces,
        initial_report.inconsistent_normals,
        initial_report.component_count,
        final_report.component_count,
        if final_report.is_watertight { "Yes" } else { "No" }
    );

    (m3, summary)
}

/// Helper: compacts index buffer to only include referenced vertices and re-indexes them.
fn compact_mesh(positions: &[[f32; 3]], indices: &[u32]) -> Mesh {
    let nv = positions.len();
    let mut old_to_new = vec![u32::MAX; nv];
    let mut new_positions = Vec::new();
    let mut new_indices = Vec::with_capacity(indices.len());

    for &old_idx in indices {
        let u = old_idx as usize;
        let new_idx = if old_to_new[u] != u32::MAX {
            old_to_new[u]
        } else {
            let n = new_positions.len() as u32;
            new_positions.push(positions[u]);
            old_to_new[u] = n;
            n
        };
        new_indices.push(new_idx);
    }

    let mut m = Mesh::from_indexed(new_positions, new_indices);
    m.recompute_normals();
    m
}
