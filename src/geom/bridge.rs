use crate::geom::boundary::EdgeKey;
use crate::geom::hole_fill::MeshPatch;
use crate::geom::topology::MeshTopology;
use crate::mesh::Mesh;
use glam::Vec3;
use std::collections::{HashMap, HashSet, VecDeque};

/// Trajectory / curvature algorithm for the bridge.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BridgeMethod {
    /// Smooth cubic Hermite spline preserving surface tangency and curvature at bridgeheads.
    CubicHermite,
    /// Direct linear interpolation across the span.
    Linear,
    /// Direct interpolation with an adjustable parabolic bulge / arch.
    BulgeArc,
}

impl BridgeMethod {
    pub fn all() -> [BridgeMethod; 3] {
        [
            BridgeMethod::CubicHermite,
            BridgeMethod::Linear,
            BridgeMethod::BulgeArc,
        ]
    }

    pub fn display_name(&self) -> &'static str {
        match self {
            BridgeMethod::CubicHermite => "Cubic Hermite (Curved)",
            BridgeMethod::Linear => "Linear (Straight)",
            BridgeMethod::BulgeArc => "Bulge / Arch",
        }
    }
}

/// Configuration settings for the bridge generation.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct BridgeConfig {
    /// Curvature algorithm.
    pub method: BridgeMethod,
    /// Number of subdivisions along the span (1 to 50).
    pub segments: usize,
    /// Curvature tension factor for Hermite interpolation (0.0 = straight, 1.0 = natural, up to 2.0).
    pub tension: f32,
    /// Bulge height offset factor (-1.0 to 1.0).
    pub bulge: f32,
    /// Flip alignment to eliminate twist.
    pub flip_twist: bool,
    /// Invert triangle winding / surface normals.
    pub flip_normals: bool,
    /// Laplacian smoothing / fairing iterations on interior bridge vertices (0 to 30).
    pub fairing_iterations: usize,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            method: BridgeMethod::CubicHermite,
            segments: 8,
            tension: 1.0,
            bulge: 0.0,
            flip_twist: false,
            flip_normals: false,
            fairing_iterations: 5,
        }
    }
}

/// A detected connected cluster of selected faces and its boundary edge chains.
#[derive(Clone, Debug)]
pub struct SelectionCluster {
    #[allow(dead_code)]
    pub triangles: Vec<u32>,
    pub boundary_chain: Vec<u32>,
    #[allow(dead_code)]
    pub is_open_mesh_boundary: bool,
    #[allow(dead_code)]
    pub centroid: Vec3,
    pub average_normal: Vec3,
}

/// Identifies connected components in the face selection.
/// Returns Ok((cluster_a, cluster_b)) if exactly two components exist,
/// selecting the boundary chain pair that face each other across the gap.
pub fn detect_selection_clusters(
    mesh: &Mesh,
    sel: &[u8],
    topo: Option<&MeshTopology>,
) -> Result<(SelectionCluster, SelectionCluster), String> {
    let nt = mesh.triangle_count();
    if sel.len() != nt {
        return Err("Selection size mismatch".to_string());
    }

    let built_topo;
    let topology = match topo {
        Some(t) => t,
        None => {
            built_topo = MeshTopology::build(mesh);
            &built_topo
        }
    };

    let mut visited = vec![false; nt];
    let mut clusters = Vec::new();

    for start_t in 0..nt {
        if sel[start_t] == 0 || visited[start_t] {
            continue;
        }

        let mut cluster_tris = Vec::new();
        let mut queue = VecDeque::new();
        queue.push_back(start_t as u32);
        visited[start_t] = true;

        while let Some(t) = queue.pop_front() {
            cluster_tris.push(t);
            let t_idx = t as usize;
            for &nb in &topology.neighbors[t_idx] {
                if nb != u32::MAX && sel[nb as usize] > 0 && !visited[nb as usize] {
                    visited[nb as usize] = true;
                    queue.push_back(nb);
                }
            }
        }

        if !cluster_tris.is_empty() {
            clusters.push(cluster_tris);
        }
    }

    if clusters.len() < 2 {
        return Err(format!(
            "Bridge requires 2 separate selections across the gap (found {}).",
            clusters.len()
        ));
    }
    if clusters.len() > 2 {
        clusters.sort_by_key(|c| std::cmp::Reverse(c.len()));
        return Err(format!(
            "Found {} selected regions. Please select only 2 regions to bridge.",
            clusters.len()
        ));
    }

    let raw_cluster_a = extract_cluster_raw(mesh, &clusters[0], topology)?;
    let raw_cluster_b = extract_cluster_raw(mesh, &clusters[1], topology)?;

    // Match the closest pair of chains between cluster A and cluster B
    let (best_chain_a, best_chain_b) = find_closest_chain_pair(
        mesh,
        &raw_cluster_a.all_chains,
        &raw_cluster_b.all_chains,
    )?;

    let cluster_a = SelectionCluster {
        triangles: clusters[0].clone(),
        boundary_chain: best_chain_a,
        is_open_mesh_boundary: raw_cluster_a.is_open_mesh_boundary,
        centroid: raw_cluster_a.centroid,
        average_normal: raw_cluster_a.average_normal,
    };

    let cluster_b = SelectionCluster {
        triangles: clusters[1].clone(),
        boundary_chain: best_chain_b,
        is_open_mesh_boundary: raw_cluster_b.is_open_mesh_boundary,
        centroid: raw_cluster_b.centroid,
        average_normal: raw_cluster_b.average_normal,
    };

    Ok((cluster_a, cluster_b))
}

struct RawCluster {
    all_chains: Vec<Vec<u32>>,
    is_open_mesh_boundary: bool,
    centroid: Vec3,
    average_normal: Vec3,
}

fn extract_cluster_raw(
    mesh: &Mesh,
    cluster_tris: &[u32],
    topo: &MeshTopology,
) -> Result<RawCluster, String> {
    let tri_set: HashSet<u32> = cluster_tris.iter().copied().collect();
    let mut centroid = Vec3::ZERO;
    let mut avg_normal = Vec3::ZERO;

    for &t in cluster_tris {
        let i0 = mesh.indices[3 * t as usize] as usize;
        let i1 = mesh.indices[3 * t as usize + 1] as usize;
        let i2 = mesh.indices[3 * t as usize + 2] as usize;
        let a = Vec3::from(mesh.positions[i0]);
        let b = Vec3::from(mesh.positions[i1]);
        let c = Vec3::from(mesh.positions[i2]);
        centroid += (a + b + c) * (1.0 / 3.0);
        avg_normal += topo.face_normals[t as usize];
    }
    let n = cluster_tris.len() as f32;
    centroid /= n.max(1.0);
    avg_normal = avg_normal.normalize_or_zero();

    let mut open_boundary_half_edges = Vec::new();
    let mut other_boundary_half_edges = Vec::new();

    for &t in cluster_tris {
        let t_idx = t as usize;
        let i0 = mesh.indices[3 * t_idx];
        let i1 = mesh.indices[3 * t_idx + 1];
        let i2 = mesh.indices[3 * t_idx + 2];

        let edges = [(i0, i1, 0), (i1, i2, 1), (i2, i0, 2)];
        for (u, v, e_local) in edges {
            let nb = topo.neighbors[t_idx][e_local];
            if nb == u32::MAX {
                open_boundary_half_edges.push((u, v));
            } else if !tri_set.contains(&nb) {
                other_boundary_half_edges.push((u, v));
            }
        }
    }

    let (use_open, half_edges) = if !open_boundary_half_edges.is_empty() {
        (true, open_boundary_half_edges)
    } else if !other_boundary_half_edges.is_empty() {
        (false, other_boundary_half_edges)
    } else {
        return Err("Cluster has no boundary edges to bridge.".to_string());
    };

    let all_chains = trace_all_chains(&half_edges);
    if all_chains.is_empty() {
        return Err("Failed to extract boundary chains from selection.".to_string());
    }

    Ok(RawCluster {
        all_chains,
        is_open_mesh_boundary: use_open,
        centroid,
        average_normal: avg_normal,
    })
}

/// Traces all contiguous paths / cycles of directed half-edges.
fn trace_all_chains(half_edges: &[(u32, u32)]) -> Vec<Vec<u32>> {
    let mut adj: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut in_degrees: HashMap<u32, usize> = HashMap::new();
    let mut visited_edges: HashSet<EdgeKey> = HashSet::new();

    for &(u, v) in half_edges {
        adj.entry(u).or_default().push(v);
        *in_degrees.entry(v).or_default() += 1;
    }

    let mut chains = Vec::new();

    // First trace open paths starting from vertices with in_degree == 0
    let mut open_starts: Vec<u32> = half_edges
        .iter()
        .map(|e| e.0)
        .filter(|&u| in_degrees.get(&u).copied().unwrap_or(0) == 0)
        .collect();
    open_starts.sort_unstable();
    open_starts.dedup();

    for start_v in open_starts {
        let mut path = vec![start_v];
        let mut curr = start_v;

        while let Some(nexts) = adj.get(&curr) {
            let mut found = None;
            for &next_v in nexts {
                let ek = EdgeKey::new(curr, next_v);
                if !visited_edges.contains(&ek) {
                    visited_edges.insert(ek);
                    found = Some(next_v);
                    break;
                }
            }
            if let Some(next_v) = found {
                path.push(next_v);
                curr = next_v;
            } else {
                break;
            }
        }
        if path.len() >= 2 {
            chains.push(path);
        }
    }

    // Then trace remaining cycles
    for &(u, _) in half_edges {
        let mut path = vec![u];
        let mut curr = u;

        while let Some(nexts) = adj.get(&curr) {
            let mut found = None;
            for &next_v in nexts {
                let ek = EdgeKey::new(curr, next_v);
                if !visited_edges.contains(&ek) {
                    visited_edges.insert(ek);
                    found = Some(next_v);
                    break;
                }
            }
            if let Some(next_v) = found {
                path.push(next_v);
                curr = next_v;
                if curr == u {
                    break;
                }
            } else {
                break;
            }
        }

        // If cycle, remove duplicate tail vertex for polyline bridging
        if path.len() >= 3 && path.first() == path.last() {
            path.pop();
        }

        if path.len() >= 2 {
            chains.push(path);
        }
    }

    chains
}

/// Finds the closest pair of chains between two sets of chains.
fn find_closest_chain_pair(
    mesh: &Mesh,
    chains_a: &[Vec<u32>],
    chains_b: &[Vec<u32>],
) -> Result<(Vec<u32>, Vec<u32>), String> {
    let mut best_dist = f32::MAX;
    let mut best_pair = None;

    for ca in chains_a {
        if ca.len() < 2 {
            continue;
        }
        for cb in chains_b {
            if cb.len() < 2 {
                continue;
            }
            // Compute min distance between vertices of ca and cb
            let mut min_d = f32::MAX;
            for &va in ca {
                let pa = Vec3::from(mesh.positions[va as usize]);
                for &vb in cb {
                    let pb = Vec3::from(mesh.positions[vb as usize]);
                    let d = pa.distance(pb);
                    if d < min_d {
                        min_d = d;
                    }
                }
            }
            if min_d < best_dist {
                best_dist = min_d;
                best_pair = Some((ca.clone(), cb.clone()));
            }
        }
    }

    best_pair.ok_or_else(|| "Could not find compatible boundary chains to bridge.".to_string())
}

/// Generates a triangulated bridge patch between two selection clusters.
pub fn generate_bridge_patch(
    mesh: &Mesh,
    cluster_a: &SelectionCluster,
    cluster_b: &SelectionCluster,
    config: BridgeConfig,
) -> Result<MeshPatch, String> {
    let chain_a = cluster_a.boundary_chain.clone();
    let mut chain_b = cluster_b.boundary_chain.clone();

    if chain_a.len() < 2 || chain_b.len() < 2 {
        return Err("Bridge boundary chains must have at least 2 vertices each.".to_string());
    }

    let pos_a: Vec<Vec3> = chain_a
        .iter()
        .map(|&v| Vec3::from(mesh.positions[v as usize]))
        .collect();
    let mut pos_b: Vec<Vec3> = chain_b
        .iter()
        .map(|&v| Vec3::from(mesh.positions[v as usize]))
        .collect();

    // Check alignment / twist between A and B
    let d_straight = pos_a[0].distance(pos_b[0]) + pos_a.last().unwrap().distance(*pos_b.last().unwrap());
    let d_crossed = pos_a[0].distance(*pos_b.last().unwrap()) + pos_a.last().unwrap().distance(pos_b[0]);

    let mut should_reverse_b = d_crossed < d_straight;
    if config.flip_twist {
        should_reverse_b = !should_reverse_b;
    }

    if should_reverse_b {
        chain_b.reverse();
        pos_b.reverse();
    }

    let n_a = chain_a.len();
    let n_b = chain_b.len();
    let width_res = n_a.max(n_b);
    let segments = config.segments.clamp(1, 60);

    let t_a = compute_arc_lengths(&pos_a);
    let t_b = compute_arc_lengths(&pos_b);

    let norm_a: Vec<Vec3> = chain_a
        .iter()
        .map(|&v| {
            if (v as usize) < mesh.normals.len() {
                Vec3::from(mesh.normals[v as usize])
            } else {
                cluster_a.average_normal
            }
        })
        .collect();

    let norm_b: Vec<Vec3> = chain_b
        .iter()
        .map(|&v| {
            if (v as usize) < mesh.normals.len() {
                Vec3::from(mesh.normals[v as usize])
            } else {
                cluster_b.average_normal
            }
        })
        .collect();

    let mut width_pts_a = Vec::with_capacity(width_res);
    let mut width_norms_a = Vec::with_capacity(width_res);
    let mut width_pts_b = Vec::with_capacity(width_res);
    let mut width_norms_b = Vec::with_capacity(width_res);

    for w_idx in 0..width_res {
        let w_param = w_idx as f32 / (width_res - 1) as f32;
        let (pa, na) = sample_chain(&pos_a, &norm_a, &t_a, w_param);
        let (pb, nb) = sample_chain(&pos_b, &norm_b, &t_b, w_param);
        width_pts_a.push(pa);
        width_norms_a.push(na);
        width_pts_b.push(pb);
        width_norms_b.push(nb);
    }

    // Grid of positions: (segments + 1) rows, width_res columns
    let mut grid: Vec<Vec<Vec3>> = Vec::with_capacity(segments + 1);

    for s_idx in 0..=segments {
        let s = s_idx as f32 / segments as f32;
        let mut row = Vec::with_capacity(width_res);

        for w_idx in 0..width_res {
            let pa = width_pts_a[w_idx];
            let pb = width_pts_b[w_idx];
            let na = width_norms_a[w_idx];
            let nb = width_norms_b[w_idx];

            let chord = pb - pa;
            let chord_len = chord.length();

            let pt = match config.method {
                BridgeMethod::Linear => pa.lerp(pb, s),
                BridgeMethod::CubicHermite => {
                    let mut ta = chord - na * chord.dot(na);
                    if ta.length_squared() > 1e-8 {
                        ta = ta.normalize() * chord_len * config.tension;
                    } else {
                        ta = chord * config.tension;
                    }

                    let mut tb = chord - nb * chord.dot(nb);
                    if tb.length_squared() > 1e-8 {
                        tb = tb.normalize() * chord_len * config.tension;
                    } else {
                        tb = chord * config.tension;
                    }

                    let h00 = 2.0 * s * s * s - 3.0 * s * s + 1.0;
                    let h10 = s * s * s - 2.0 * s * s + s;
                    let h01 = -2.0 * s * s * s + 3.0 * s * s;
                    let h11 = s * s * s - s * s;

                    pa * h00 + ta * h10 + pb * h01 + tb * h11
                }
                BridgeMethod::BulgeArc => {
                    let base = pa.lerp(pb, s);
                    let mid_norm = (na.lerp(nb, s)).normalize_or_zero();
                    let parabolic = 4.0 * s * (1.0 - s);
                    base + mid_norm * (parabolic * config.bulge * chord_len * 0.5)
                }
            };

            let final_pt = if config.method == BridgeMethod::CubicHermite && config.bulge.abs() > 0.001 {
                let mid_norm = (na.lerp(nb, s)).normalize_or_zero();
                let parabolic = 4.0 * s * (1.0 - s);
                pt + mid_norm * (parabolic * config.bulge * chord_len * 0.5)
            } else {
                pt
            };

            row.push(final_pt);
        }
        grid.push(row);
    }

    // Optional Laplacian fairing of interior grid vertices
    if config.fairing_iterations > 0 && segments > 2 && width_res > 2 {
        for _ in 0..config.fairing_iterations {
            let mut next_grid = grid.clone();
            for s in 1..segments {
                for w in 1..(width_res - 1) {
                    let p_curr = grid[s][w];
                    let p_up = grid[s - 1][w];
                    let p_down = grid[s + 1][w];
                    let p_left = grid[s][w - 1];
                    let p_right = grid[s][w + 1];
                    let avg = (p_up + p_down + p_left + p_right) * 0.25;
                    next_grid[s][w] = p_curr.lerp(avg, 0.45);
                }
            }
            grid = next_grid;
        }
    }

    // 1. Preview representation
    let mut preview_positions = Vec::new();
    let mut preview_indices = Vec::new();

    for row in &grid {
        for &pt in row {
            preview_positions.push(pt.to_array());
        }
    }

    for s in 0..segments {
        for w in 0..(width_res - 1) {
            let i00 = (s * width_res + w) as u32;
            let i01 = (s * width_res + w + 1) as u32;
            let i10 = ((s + 1) * width_res + w) as u32;
            let i11 = ((s + 1) * width_res + w + 1) as u32;

            if config.flip_normals {
                preview_indices.extend_from_slice(&[i00, i10, i11]);
                preview_indices.extend_from_slice(&[i00, i11, i01]);
            } else {
                preview_indices.extend_from_slice(&[i00, i11, i10]);
                preview_indices.extend_from_slice(&[i00, i01, i11]);
            }
        }
    }

    // 2. MeshPatch representation for mesh stitching
    let base_nv = mesh.positions.len() as u32;
    let mut new_positions = Vec::new();
    let mut final_indices_grid: Vec<Vec<u32>> = vec![vec![0; width_res]; segments + 1];

    for s in 1..segments {
        for w in 0..width_res {
            let new_idx = base_nv + new_positions.len() as u32;
            new_positions.push(grid[s][w].to_array());
            final_indices_grid[s][w] = new_idx;
        }
    }

    let mut new_indices = Vec::new();

    // Intermediate quad grid (1 <= s < segments - 1)
    if segments > 2 {
        for s in 1..(segments - 1) {
            for w in 0..(width_res - 1) {
                let v00 = final_indices_grid[s][w];
                let v01 = final_indices_grid[s][w + 1];
                let v10 = final_indices_grid[s + 1][w];
                let v11 = final_indices_grid[s + 1][w + 1];

                if config.flip_normals {
                    new_indices.extend_from_slice(&[v00, v10, v11]);
                    new_indices.extend_from_slice(&[v00, v11, v01]);
                } else {
                    new_indices.extend_from_slice(&[v00, v11, v10]);
                    new_indices.extend_from_slice(&[v00, v01, v11]);
                }
            }
        }
    }

    // Stitch Row 0 to Row 1
    let target_row_1 = if segments == 1 {
        &chain_b
    } else {
        &final_indices_grid[1]
    };
    let stitch_0 = stitch_boundary_row(&chain_a, &pos_a, target_row_1, &grid[1.min(segments)], config.flip_normals);
    new_indices.extend(stitch_0);

    // Stitch Row (segments - 1) to Row segments (chain_b)
    if segments > 1 {
        let prev_row = &final_indices_grid[segments - 1];
        let stitch_end = stitch_boundary_row(prev_row, &grid[segments - 1], &chain_b, &pos_b, config.flip_normals);
        new_indices.extend(stitch_end);
    }

    Ok(MeshPatch {
        new_positions,
        new_indices,
        preview_positions,
        preview_indices,
    })
}

fn compute_arc_lengths(pts: &[Vec3]) -> Vec<f32> {
    let n = pts.len();
    if n <= 1 {
        return vec![0.0; n];
    }
    let mut lens = Vec::with_capacity(n);
    lens.push(0.0);
    let mut total = 0.0;
    for i in 1..n {
        let d = pts[i].distance(pts[i - 1]);
        total += d;
        lens.push(total);
    }
    if total > 1e-8 {
        for val in &mut lens {
            *val /= total;
        }
    }
    lens
}

fn sample_chain(pts: &[Vec3], norms: &[Vec3], t_arr: &[f32], t: f32) -> (Vec3, Vec3) {
    let n = pts.len();
    if n == 0 {
        return (Vec3::ZERO, Vec3::Y);
    }
    if n == 1 || t <= t_arr[0] {
        return (pts[0], norms[0]);
    }
    if t >= *t_arr.last().unwrap() {
        return (*pts.last().unwrap(), *norms.last().unwrap());
    }

    let idx = match t_arr.binary_search_by(|val| val.partial_cmp(&t).unwrap()) {
        Ok(i) => i,
        Err(i) => i.max(1) - 1,
    };

    let t0 = t_arr[idx];
    let t1 = t_arr[(idx + 1).min(n - 1)];
    let dt = (t1 - t0).max(1e-8);
    let factor = ((t - t0) / dt).clamp(0.0, 1.0);

    let pt = pts[idx].lerp(pts[(idx + 1).min(n - 1)], factor);
    let norm = norms[idx].lerp(norms[(idx + 1).min(n - 1)], factor).normalize_or_zero();

    (pt, norm)
}

/// Stitches between two ordered vertex rows, maintaining consistent 2-manifold orientation.
fn stitch_boundary_row(
    row_a: &[u32],
    pos_a: &[Vec3],
    row_b: &[u32],
    pos_b: &[Vec3],
    flip_user: bool,
) -> Vec<u32> {
    let n = row_a.len();
    let m = row_b.len();
    let mut indices = Vec::with_capacity((n + m) * 6);

    let mut i = 0;
    let mut j = 0;

    while i < n - 1 || j < m - 1 {
        let can_advance_i = i < n - 1;
        let can_advance_j = j < m - 1;

        let advance_i = if can_advance_i && can_advance_j {
            let dist_i = pos_a[i + 1].distance(pos_b[j]);
            let dist_j = pos_a[i].distance(pos_b[j + 1]);
            dist_i <= dist_j
        } else {
            can_advance_i
        };

        if advance_i {
            // Triangle: (row_a[i], row_a[i+1], row_b[j])
            let v0 = row_a[i];
            let v1 = row_a[i + 1];
            let v2 = row_b[j];
            if v0 != v1 && v1 != v2 && v2 != v0 {
                let p0 = pos_a[i];
                let p1 = pos_a[i + 1];
                let p2 = pos_b[j];
                if (p1 - p0).cross(p2 - p0).length_squared() > 1e-12 {
                    if flip_user {
                        indices.extend_from_slice(&[v0, v1, v2]);
                    } else {
                        indices.extend_from_slice(&[v0, v2, v1]);
                    }
                }
            }
            i += 1;
        } else {
            // Triangle: (row_a[i], row_b[j+1], row_b[j])
            let v0 = row_a[i];
            let v1 = row_b[j + 1];
            let v2 = row_b[j];
            if v0 != v1 && v1 != v2 && v2 != v0 {
                let p0 = pos_a[i];
                let p1 = pos_b[j + 1];
                let p2 = pos_b[j];
                if (p1 - p0).cross(p2 - p0).length_squared() > 1e-12 {
                    if flip_user {
                        indices.extend_from_slice(&[v0, v1, v2]);
                    } else {
                        indices.extend_from_slice(&[v0, v2, v1]);
                    }
                }
            }
            j += 1;
        }
    }

    indices
}

/// Applies the generated bridge patch directly to a mesh.
pub fn apply_bridge_patch(mesh: &mut Mesh, patch: &MeshPatch) {
    if patch.new_indices.is_empty() {
        return;
    }
    mesh.positions.extend(patch.new_positions.iter().copied());
    mesh.indices.extend(patch.new_indices.iter().copied());
    mesh.recompute_normals();
}
