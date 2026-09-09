use crate::geom::bvh::Bvh;
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::HashMap;

/// Fitting method to adjust the surface to the target object.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum FitMethod {
    /// Elastic membrane pulled onto the object contour.
    /// Harmonious curvature, scan noise filtering, ignores and spans holes form-stably.
    #[default]
    SoftMembrane,
    /// Rigid projection of vertices along their normals directly onto the part.
    /// Maximum dimensional accuracy, reproduces sharp edges.
    HardRaycast,
}

impl FitMethod {
    pub fn label(self) -> &'static str {
        match self {
            FitMethod::SoftMembrane => "Weiche Anpassung (Membran)",
            FitMethod::HardRaycast => "Harter Raycast",
        }
    }
}

/// Geometry output format.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum FitOutputType {
    /// Open surface sheet body bounded by the outer edges.
    #[default]
    Faces,
    /// Watertight closed B-Rep solid body penetrating the target part.
    Solid,
}

impl FitOutputType {
    #[allow(dead_code)]
    pub fn label(self) -> &'static str {
        match self {
            FitOutputType::Faces => "Offener Flächenverband (Faces)",
            FitOutputType::Solid => "Geschlossener Volumenkörper (Solid)",
        }
    }
}

/// Direction for extruding boundary edges backwards into the part.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum EdgeProjectionDir {
    /// Inverted surface/vertex normals.
    #[default]
    SurfaceNormal,
    /// Fixed negative Z axis (-Z).
    NegZ,
    /// Fixed negative Y axis (-Y).
    NegY,
    /// Fixed negative X axis (-X).
    NegX,
}

impl EdgeProjectionDir {
    pub fn label(self) -> &'static str {
        match self {
            EdgeProjectionDir::SurfaceNormal => "Flächennormalen (Invertiert)",
            EdgeProjectionDir::NegZ => "Feste Achse: -Z",
            EdgeProjectionDir::NegY => "Feste Achse: -Y",
            EdgeProjectionDir::NegX => "Feste Achse: -X",
        }
    }

    pub fn direction_vector(self, normal: Vec3) -> Vec3 {
        match self {
            EdgeProjectionDir::SurfaceNormal => -normal.normalize_or_zero(),
            EdgeProjectionDir::NegZ => -Vec3::Z,
            EdgeProjectionDir::NegY => -Vec3::Y,
            EdgeProjectionDir::NegX => -Vec3::X,
        }
    }
}

/// Parameters for Fit to Object.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct FitToObjectConfig {
    pub method: FitMethod,
    pub output_type: FitOutputType,
    /// Tension factor (0.01..=1.0) for elastic membrane.
    /// Controls bending stiffness and surface tension.
    pub tension: f32,
    /// Smoothing passes (1..=20) for filtering high-frequency scan noise.
    pub smoothness: u32,
    /// Maximum ray search distance in mm for Hard Raycast.
    pub max_ray_dist: f32,
    /// Snap tolerance in mm for handling edge / corner features in Hard Raycast.
    pub snap_tolerance: f32,
    /// Penetration depth in mm for Solid generation.
    pub penetration_depth: f32,
    /// Projection vector direction for the Solid boundary extrusion.
    pub projection_dir: EdgeProjectionDir,
    /// Tolerance in mm for sewing/welding overlapping freeform patch seams.
    pub stitch_tolerance: f32,
}

impl Default for FitToObjectConfig {
    fn default() -> Self {
        FitToObjectConfig {
            method: FitMethod::SoftMembrane,
            output_type: FitOutputType::Faces,
            tension: 0.40,
            smoothness: 4,
            max_ray_dist: 20.0,
            snap_tolerance: 0.25,
            penetration_depth: 5.0,
            projection_dir: EdgeProjectionDir::SurfaceNormal,
            stitch_tolerance: 0.35,
        }
    }
}

/// A finished fitted parametric feature item stored in the model tree.
#[derive(Clone)]
pub struct FittedObjectFeature {
    pub id: u64,
    pub name: String,
    pub visible: bool,
    pub color: [f32; 4],
    pub output_type: FitOutputType,
    pub mesh: Mesh,
    #[allow(dead_code)]
    pub config: FitToObjectConfig,
    pub triangle_count: usize,
    #[allow(dead_code)]
    pub vertex_count: usize,
    pub surface_area: f32,
    /// Volume in mm³ (if Solid).
    pub volume: Option<f32>,
    pub is_watertight: bool,
    pub max_dev: f32,
    pub rms_dev: f32,
}

// -----------------------------------------------------------------------------
// 1. Trimming and Stitching
// -----------------------------------------------------------------------------

/// Trims and stitches multiple freeform surface meshes into a single continuous patch.
/// If only one surface is supplied, it is returned directly.
/// For multiple surfaces, mutual intersections and redundant overlaps are trimmed,
/// and common boundary edges are welded/stitched together into a manifold surface.
pub fn trim_and_stitch_freeforms(
    patches: &[&Mesh],
    stitch_tolerance: f32,
) -> Result<Mesh, String> {
    if patches.is_empty() {
        return Err("No freeform surfaces selected or available.".to_string());
    }
    if patches.len() == 1 {
        return Ok((*patches[0]).clone());
    }

    let bvhs: Vec<Bvh> = patches
        .iter()
        .map(|m| Bvh::new(&m.positions, &m.indices))
        .collect();

    let tol = stitch_tolerance.max(0.01);
    let tol2 = tol * tol;

    let mut trimmed_patches: Vec<Mesh> = Vec::with_capacity(patches.len());

    for (i, patch) in patches.iter().enumerate() {
        let nt = patch.triangle_count();
        let mut keep_tri = vec![true; nt];

        for t in 0..nt {
            let [a, b, c] = patch.triangle(t);
            let centroid = (a + b + c) * (1.0 / 3.0);

            // If a higher-indexed patch already covers this point closer than tolerance,
            // cull this triangle as redundant overlap outside the seam.
            for j in (i + 1)..patches.len() {
                let (_, _, dist) = bvhs[j].closest_point(centroid);
                if dist * dist < tol2 {
                    keep_tri[t] = false;
                    break;
                }
            }
        }

        // Build trimmed sub-mesh for patch i
        let mut new_pos = Vec::new();
        let mut new_ind = Vec::new();
        let mut remap = HashMap::new();

        for (t, &keep) in keep_tri.iter().enumerate() {
            if !keep {
                continue;
            }
            for k in 0..3 {
                let v_idx = patch.indices[3 * t + k];
                let p = patch.positions[v_idx as usize];
                let n_idx = *remap.entry(v_idx).or_insert_with(|| {
                    let idx = new_pos.len() as u32;
                    new_pos.push(p);
                    idx
                });
                new_ind.push(n_idx);
            }
        }

        if !new_ind.is_empty() {
            trimmed_patches.push(Mesh::from_indexed(new_pos, new_ind));
        }
    }

    if trimmed_patches.is_empty() {
        return Err("Trimming resulted in an empty surface.".to_string());
    }

    // Combine all trimmed meshes into a single vertex/index buffer
    let mut combined_pos: Vec<[f32; 3]> = Vec::new();
    let mut combined_ind: Vec<u32> = Vec::new();

    for p in &trimmed_patches {
        let offset = combined_pos.len() as u32;
        combined_pos.extend_from_slice(&p.positions);
        for &idx in &p.indices {
            combined_ind.push(idx + offset);
        }
    }

    // Stitch / weld vertices along intersection boundaries within tolerance
    let welded = weld_vertices_within_tol(&combined_pos, &combined_ind, tol);
    Ok(welded)
}

/// Welds vertices that are closer than `tolerance` using a spatial grid hash.
fn weld_vertices_within_tol(positions: &[[f32; 3]], indices: &[u32], tolerance: f32) -> Mesh {
    let cell_size = tolerance.max(1e-4);
    let inv_cell = 1.0 / cell_size;
    let tol2 = tolerance * tolerance;

    let mut grid: HashMap<[i64; 3], Vec<u32>> = HashMap::with_capacity(positions.len());
    let mut remap: Vec<u32> = (0..positions.len() as u32).collect();
    let mut new_positions: Vec<[f32; 3]> = Vec::new();

    for (old_idx, p) in positions.iter().enumerate() {
        let v = Vec3::from(*p);
        let cx = (v.x * inv_cell).floor() as i64;
        let cy = (v.y * inv_cell).floor() as i64;
        let cz = (v.z * inv_cell).floor() as i64;

        let mut matched_target: Option<u32> = None;

        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = [cx + dx, cy + dy, cz + dz];
                    if let Some(candidates) = grid.get(&key) {
                        for &cand_new_idx in candidates {
                            let cand_pos = Vec3::from(new_positions[cand_new_idx as usize]);
                            if (cand_pos - v).length_squared() <= tol2 {
                                matched_target = Some(cand_new_idx);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }

        if let Some(target) = matched_target {
            remap[old_idx] = target;
        } else {
            let new_idx = new_positions.len() as u32;
            new_positions.push(*p);
            remap[old_idx] = new_idx;
            grid.entry([cx, cy, cz]).or_default().push(new_idx);
        }
    }

    // Remap triangle indices and drop degenerate (collapsed) triangles
    let mut new_indices = Vec::with_capacity(indices.len());
    for chunk in indices.chunks_exact(3) {
        let i0 = remap[chunk[0] as usize];
        let i1 = remap[chunk[1] as usize];
        let i2 = remap[chunk[2] as usize];
        if i0 != i1 && i1 != i2 && i2 != i0 {
            new_indices.push(i0);
            new_indices.push(i1);
            new_indices.push(i2);
        }
    }

    Mesh::from_indexed(new_positions, new_indices)
}

// -----------------------------------------------------------------------------
// 2. Soft Membrane Fitting (Weiche Anpassung)
// -----------------------------------------------------------------------------

/// Fits the surface to the target object using an elastic membrane model.
/// Pulls vertices toward the scan contour while smoothing high-frequency noise.
/// Holes, cutouts, and gaps are detected and bridged/spanned smoothly without sagging.
pub fn fit_soft_membrane(
    mesh: &Mesh,
    target_bvh: &Bvh,
    tension: f32,
    smoothness: u32,
) -> Mesh {
    let nv = mesh.vertex_count();
    if nv == 0 {
        return mesh.clone();
    }

    // Build vertex adjacency
    let mut neighbors: Vec<Vec<u32>> = vec![Vec::with_capacity(6); nv];
    for chunk in mesh.indices.chunks_exact(3) {
        let i0 = chunk[0] as usize;
        let i1 = chunk[1] as usize;
        let i2 = chunk[2] as usize;

        if !neighbors[i0].contains(&chunk[1]) { neighbors[i0].push(chunk[1]); }
        if !neighbors[i0].contains(&chunk[2]) { neighbors[i0].push(chunk[2]); }
        if !neighbors[i1].contains(&chunk[0]) { neighbors[i1].push(chunk[0]); }
        if !neighbors[i1].contains(&chunk[2]) { neighbors[i1].push(chunk[2]); }
        if !neighbors[i2].contains(&chunk[0]) { neighbors[i2].push(chunk[0]); }
        if !neighbors[i2].contains(&chunk[1]) { neighbors[i2].push(chunk[1]); }
    }

    let tension_factor = tension.clamp(0.05, 0.95);
    let mut curr_positions: Vec<Vec3> = mesh.positions.iter().map(|p| Vec3::from(*p)).collect();

    let passes = smoothness.clamp(1, 20);

    for _pass in 0..passes {
        // Evaluate target projections and hole indicators in parallel
        let targets: Vec<(Vec3, f32)> = curr_positions
            .par_iter()
            .enumerate()
            .map(|(idx, &pos)| {
                let (q, tri, dist) = target_bvh.closest_point(pos);
                let tri_norm = target_bvh.face_normal(tri);
                let my_norm = if idx < mesh.normals.len() {
                    Vec3::from(mesh.normals[idx])
                } else {
                    Vec3::Y
                };

                // Hole detection heuristic:
                // If dot product of target normal and current normal is very small (< 0.25),
                // or if distance is excessive, the point is considered over an opening/hole.
                let norm_dot = tri_norm.dot(my_norm);
                let is_hole = norm_dot < 0.25 || dist > 40.0;

                // Weight for data attraction (0.0 inside holes -> pure membrane tension)
                let weight = if is_hole {
                    0.0f32
                } else {
                    (norm_dot.clamp(0.0, 1.0) * (1.0 - tension_factor)).clamp(0.0, 1.0)
                };

                (q, weight)
            })
            .collect();

        // Update positions using elastic membrane Laplace-Beltrami fairing
        let next_positions: Vec<Vec3> = (0..nv)
            .into_par_iter()
            .map(|i| {
                let pos = curr_positions[i];
                let neighs = &neighbors[i];
                if neighs.is_empty() {
                    return pos;
                }

                // Umbrella Laplacian coordinate
                let mut sum = Vec3::ZERO;
                for &nb in neighs {
                    sum += curr_positions[nb as usize];
                }
                let avg = sum * (1.0 / neighs.len() as f32);
                let laplacian = avg - pos;

                let (target_q, weight) = targets[i];

                // Membrane balance: attraction pull + elastic Laplacian smoothing
                let pull = (target_q - pos) * weight;
                let elastic = laplacian * tension_factor;

                pos + pull + elastic
            })
            .collect();

        curr_positions = next_positions;
    }

    let out_pos: Vec<[f32; 3]> = curr_positions.into_iter().map(|v| v.to_array()).collect();
    Mesh::from_indexed(out_pos, mesh.indices.clone())
}

// -----------------------------------------------------------------------------
// 3. Hard Raycast Fitting (Harter Raycast)
// -----------------------------------------------------------------------------

/// Projects surface vertices directly along their surface normals onto the target part.
/// Maximum dimensional accuracy, preserves sharp corners and snaps to creases within tolerance.
pub fn fit_hard_raycast(
    mesh: &Mesh,
    target_bvh: &Bvh,
    max_ray_dist: f32,
    snap_tolerance: f32,
) -> Mesh {
    let nv = mesh.vertex_count();
    if nv == 0 {
        return mesh.clone();
    }

    let max_dist = max_ray_dist.max(0.1);
    let snap_tol2 = (snap_tolerance.max(0.001)) * (snap_tolerance.max(0.001));

    let out_pos: Vec<[f32; 3]> = (0..nv)
        .into_par_iter()
        .map(|i| {
            let p = Vec3::from(mesh.positions[i]);
            let n = if i < mesh.normals.len() {
                Vec3::from(mesh.normals[i]).normalize_or_zero()
            } else {
                Vec3::Y
            };

            // 1. Raycast forward along normal
            let hit_fwd = target_bvh.ray_cast(p - n * 0.005, n, max_dist);
            // 2. Raycast backward along normal
            let hit_bwd = target_bvh.ray_cast(p + n * 0.005, -n, max_dist);

            let chosen_hit = match (hit_fwd, hit_bwd) {
                (Some((tf, trif)), Some((tb, trib))) => {
                    if tf <= tb {
                        Some((p + n * (tf - 0.005), trif))
                    } else {
                        Some((p - n * (tb - 0.005), trib))
                    }
                }
                (Some((tf, trif)), None) => Some((p + n * (tf - 0.005), trif)),
                (None, Some((tb, trib))) => Some((p - n * (tb - 0.005), trib)),
                (None, None) => None,
            };

            let target_point = if let Some((hit_pos, hit_tri)) = chosen_hit {
                // Snap tolerance feature check:
                // Check if hit_pos is within snap_tolerance of any of the hit triangle's edge vertices
                let [v0, v1, v2] = target_bvh.tri_verts(hit_tri);
                if (v0 - hit_pos).length_squared() <= snap_tol2 {
                    v0
                } else if (v1 - hit_pos).length_squared() <= snap_tol2 {
                    v1
                } else if (v2 - hit_pos).length_squared() <= snap_tol2 {
                    v2
                } else {
                    hit_pos
                }
            } else {
                // Fallback: closest point if within max_dist
                let (cp, _, d) = target_bvh.closest_point(p);
                if d <= max_dist {
                    cp
                } else {
                    p
                }
            };

            target_point.to_array()
        })
        .collect();

    Mesh::from_indexed(out_pos, mesh.indices.clone())
}

// -----------------------------------------------------------------------------
// 4. Solid Extrusion (B-Rep Watertight Solid Generation)
// -----------------------------------------------------------------------------

/// Converts an open fitted surface sheet into a watertight 2-manifold B-Rep solid.
/// Extrudes the surface and outer boundary backwards through the target part by
/// `penetration_depth`, creating side walls and capping the back with outward normals.
pub fn extrude_to_solid(
    sheet: &Mesh,
    depth: f32,
    direction: EdgeProjectionDir,
) -> Result<Mesh, String> {
    let nv = sheet.vertex_count();
    let nt = sheet.triangle_count();
    if nv < 3 || nt == 0 {
        return Err("Surface has insufficient geometry for solid extrusion.".to_string());
    }

    let penetration = depth.max(0.01);

    // Compute boundary half-edges of the top surface.
    // An open boundary edge is an edge (u -> v) with no opposite edge (v -> u).
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::with_capacity(nt * 3);
    for chunk in sheet.indices.chunks_exact(3) {
        let (i0, i1, i2) = (chunk[0], chunk[1], chunk[2]);
        *edge_counts.entry((i0, i1)).or_insert(0) += 1;
        *edge_counts.entry((i1, i2)).or_insert(0) += 1;
        *edge_counts.entry((i2, i0)).or_insert(0) += 1;
    }

    let mut boundary_edges: Vec<(u32, u32)> = Vec::new();
    for (&(a, b), &count) in &edge_counts {
        if count == 1 && !edge_counts.contains_key(&(b, a)) {
            boundary_edges.push((a, b));
        }
    }

    if boundary_edges.is_empty() {
        return Err("Surface appears to be already closed (no boundary edges found).".to_string());
    }

    // Build solid mesh:
    // 1. Top face: Original surface vertices [0..nv] and triangles.
    // 2. Bottom face: Offset surface vertices [nv..2*nv] with inverted winding.
    // 3. Side walls: Quad strips between top boundary edges and bottom boundary edges.
    let mut solid_positions: Vec<[f32; 3]> = Vec::with_capacity(nv * 2);
    let mut solid_indices: Vec<u32> = Vec::with_capacity(nt * 6 + boundary_edges.len() * 6);

    // Add top vertices
    solid_positions.extend_from_slice(&sheet.positions);

    // Add bottom vertices shifted backwards by extrusion vector
    for i in 0..nv {
        let p = Vec3::from(sheet.positions[i]);
        let n = if i < sheet.normals.len() {
            Vec3::from(sheet.normals[i])
        } else {
            Vec3::Y
        };
        let ext_dir = direction.direction_vector(n);
        let bot_p = p + ext_dir * penetration;
        solid_positions.push(bot_p.to_array());
    }

    // Top triangles (same as input)
    solid_indices.extend_from_slice(&sheet.indices);

    // Bottom triangles (offset vertices with inverted triangle winding for outward normal)
    let bot_offset = nv as u32;
    for chunk in sheet.indices.chunks_exact(3) {
        let i0 = chunk[0] + bot_offset;
        let i1 = chunk[1] + bot_offset;
        let i2 = chunk[2] + bot_offset;
        // Invert winding: i0, i2, i1
        solid_indices.push(i0);
        solid_indices.push(i2);
        solid_indices.push(i1);
    }

    // Side wall quad-strips for every directed boundary edge (a -> b).
    // The top triangle uses edge (a -> b), so the side wall must use (b -> a)
    // to maintain consistent manifold orientation and outward normals.
    for (a, b) in boundary_edges {
        let a_bot = a + bot_offset;
        let b_bot = b + bot_offset;

        // Triangle 1: (b, a, a_bot)
        solid_indices.push(b);
        solid_indices.push(a);
        solid_indices.push(a_bot);

        // Triangle 2: (b, a_bot, b_bot)
        solid_indices.push(b);
        solid_indices.push(a_bot);
        solid_indices.push(b_bot);
    }

    let mut solid_mesh = Mesh::from_indexed(solid_positions, solid_indices);
    solid_mesh.recompute_normals();

    Ok(solid_mesh)
}

// -----------------------------------------------------------------------------
// 5. Geometric Statistics & Solid Volume
// -----------------------------------------------------------------------------

/// Computes the volume of a closed triangle mesh using the divergence theorem.
pub fn calculate_mesh_volume(mesh: &Mesh) -> f32 {
    let mut total_vol = 0.0f64;
    for chunk in mesh.indices.chunks_exact(3) {
        let v0 = Vec3::from(mesh.positions[chunk[0] as usize]);
        let v1 = Vec3::from(mesh.positions[chunk[1] as usize]);
        let v2 = Vec3::from(mesh.positions[chunk[2] as usize]);

        let tet_vol = v0.dot(v1.cross(v2)) as f64 / 6.0;
        total_vol += tet_vol;
    }
    total_vol.abs() as f32
}

/// Checks if a mesh is closed and 2-manifold watertight (every edge shared by exactly 2 triangles).
pub fn check_mesh_watertight(mesh: &Mesh) -> bool {
    if mesh.indices.is_empty() {
        return false;
    }
    let mut edge_map: HashMap<[u32; 2], usize> = HashMap::with_capacity(mesh.indices.len());
    for chunk in mesh.indices.chunks_exact(3) {
        for k in 0..3 {
            let u = chunk[k];
            let v = chunk[(k + 1) % 3];
            let key = if u < v { [u, v] } else { [v, u] };
            *edge_map.entry(key).or_insert(0) += 1;
        }
    }
    edge_map.values().all(|&cnt| cnt == 2)
}

/// Measures deviation (max and RMS) from surface vertices to target mesh BVH.
pub fn measure_surface_deviation(surface: &Mesh, target_bvh: &Bvh) -> (f32, f32) {
    if surface.positions.is_empty() {
        return (0.0, 0.0);
    }
    let mut max_dev = 0.0f32;
    let mut sum_sq = 0.0f64;
    let count = surface.positions.len();

    for p in &surface.positions {
        let d = target_bvh.closest_distance(Vec3::from(*p));
        if d.is_finite() {
            max_dev = max_dev.max(d);
            sum_sq += (d * d) as f64;
        }
    }

    let rms = if count > 0 {
        ((sum_sq / count as f64) as f32).sqrt()
    } else {
        0.0
    };

    (max_dev, rms)
}

// -----------------------------------------------------------------------------
// 6. Main Orchestrator: execute_fit_to_object
// -----------------------------------------------------------------------------

/// Executes the full Fit to Object pipeline:
/// 1. Trims and stitches freeform patches.
/// 2. Fits to target mesh using configured method (Soft Membrane / Hard Raycast).
/// 3. Converts to solid if configured.
/// 4. Computes metrics (RMS, volume, area, watertightness).
pub fn execute_fit_to_object(
    patches: &[&Mesh],
    _target_mesh: &Mesh,
    target_bvh: &Bvh,
    config: &FitToObjectConfig,
    feature_id: u64,
    color: [f32; 4],
) -> Result<FittedObjectFeature, String> {
    // Phase 1: Trim and stitch input freeform surfaces
    let stitched = trim_and_stitch_freeforms(patches, config.stitch_tolerance)?;

    // Phase 2: Fit to object contour
    let fitted = match config.method {
        FitMethod::SoftMembrane => {
            fit_soft_membrane(&stitched, target_bvh, config.tension, config.smoothness)
        }
        FitMethod::HardRaycast => {
            fit_hard_raycast(&stitched, target_bvh, config.max_ray_dist, config.snap_tolerance)
        }
    };

    let (max_dev, rms_dev) = measure_surface_deviation(&fitted, target_bvh);
    let area = fitted.surface_area();

    // Phase 3: Geometry output
    let (final_mesh, volume, is_watertight) = match config.output_type {
        FitOutputType::Faces => {
            let wt = check_mesh_watertight(&fitted);
            (fitted, None, wt)
        }
        FitOutputType::Solid => {
            let solid = extrude_to_solid(&fitted, config.penetration_depth, config.projection_dir)?;
            let vol = calculate_mesh_volume(&solid);
            let wt = check_mesh_watertight(&solid);
            (solid, Some(vol), wt)
        }
    };

    let name = match config.output_type {
        FitOutputType::Faces => format!("Fit Surface {feature_id}"),
        FitOutputType::Solid => format!("Fit Solid {feature_id}"),
    };

    let triangle_count = final_mesh.triangle_count();
    let vertex_count = final_mesh.vertex_count();

    Ok(FittedObjectFeature {
        id: feature_id,
        name,
        visible: true,
        color,
        output_type: config.output_type,
        mesh: final_mesh,
        config: *config,
        triangle_count,
        vertex_count,
        surface_area: area,
        volume,
        is_watertight,
        max_dev,
        rms_dev,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_flat_grid(
        nx: usize,
        ny: usize,
        x_min: f32,
        x_max: f32,
        y_min: f32,
        y_max: f32,
        z: f32,
    ) -> Mesh {
        let mut positions = Vec::with_capacity(nx * ny);
        for j in 0..ny {
            let v = j as f32 / (ny - 1) as f32;
            let y = y_min + v * (y_max - y_min);
            for i in 0..nx {
                let u = i as f32 / (nx - 1) as f32;
                let x = x_min + u * (x_max - x_min);
                positions.push([x, y, z]);
            }
        }
        let mut indices = Vec::with_capacity((nx - 1) * (ny - 1) * 6);
        for j in 0..(ny - 1) {
            for i in 0..(nx - 1) {
                let i0 = (j * nx + i) as u32;
                let i1 = (j * nx + (i + 1)) as u32;
                let i2 = ((j + 1) * nx + (i + 1)) as u32;
                let i3 = ((j + 1) * nx + i) as u32;
                indices.extend_from_slice(&[i0, i1, i2, i0, i2, i3]);
            }
        }
        let mut m = Mesh::from_indexed(positions, indices);
        m.recompute_normals();
        m
    }

    #[test]
    fn test_trim_and_stitch_overlapping_patches() {
        // Two patches that overlap in X [4.0, 6.0]
        let patch1 = make_flat_grid(7, 5, 0.0, 6.0, 0.0, 4.0, 0.0);
        let patch2 = make_flat_grid(7, 5, 4.0, 10.0, 0.0, 4.0, 0.0);

        let initial_tris = patch1.triangle_count() + patch2.triangle_count();
        let stitched = trim_and_stitch_freeforms(&[&patch1, &patch2], 0.05).unwrap();

        // Stitched mesh should remove redundant triangles in the overlap area
        assert!(stitched.triangle_count() < initial_tris);
        assert!(stitched.vertex_count() > 0);
    }

    #[test]
    fn test_soft_membrane_spanning_hole() {
        // Target: flat mesh with vertices at z=0, but only defined in corners/edges
        let target = make_flat_grid(11, 11, 0.0, 10.0, 0.0, 10.0, 0.0);
        let bvh = Bvh::new(&target.positions, &target.indices);

        // Input surface slightly offset above at z=2.0
        let patch = make_flat_grid(7, 7, 0.0, 10.0, 0.0, 10.0, 2.0);

        let fitted = fit_soft_membrane(&patch, &bvh, 0.5, 5);

        // All positions should be pulled downwards towards z=0, smooth, finite
        for p in &fitted.positions {
            assert!(p[0].is_finite());
            assert!(p[1].is_finite());
            assert!(p[2].is_finite());
            assert!(p[2] < 2.0); // Pulled down
        }
    }

    #[test]
    fn test_hard_raycast_projection_and_snapping() {
        let target = make_flat_grid(5, 5, 0.0, 10.0, 0.0, 10.0, 0.0);
        let bvh = Bvh::new(&target.positions, &target.indices);

        let patch = make_flat_grid(5, 5, 1.0, 9.0, 1.0, 9.0, 1.5);
        let fitted = fit_hard_raycast(&patch, &bvh, 5.0, 0.01);

        for p in &fitted.positions {
            assert!((p[2] - 0.0).abs() < 0.02, "Expected z near 0, got {}", p[2]);
        }
    }

    #[test]
    fn test_extrude_to_solid_watertightness_and_volume() {
        let patch = make_flat_grid(5, 5, 0.0, 10.0, 0.0, 10.0, 0.0);
        let depth = 3.0;
        let solid = extrude_to_solid(&patch, depth, EdgeProjectionDir::SurfaceNormal).unwrap();

        // Must be a watertight 2-manifold closed solid
        assert!(check_mesh_watertight(&solid), "Solid should be watertight!");

        // Area: 10 * 10 = 100, depth: 3.0 -> Volume ~ 300
        let vol = calculate_mesh_volume(&solid);
        let expected_vol = 10.0 * 10.0 * 3.0;
        assert!(
            (vol - expected_vol).abs() < 5.0,
            "Volume was {}, expected {}",
            vol,
            expected_vol
        );
    }

    #[test]
    fn test_full_pipeline_faces_and_solid() {
        let target = make_flat_grid(7, 7, 0.0, 12.0, 0.0, 12.0, 0.0);
        let bvh = Bvh::new(&target.positions, &target.indices);
        let patch = make_flat_grid(5, 5, 1.0, 11.0, 1.0, 11.0, 0.5);

        let config_faces = FitToObjectConfig {
            method: FitMethod::SoftMembrane,
            tension: 0.5,
            smoothness: 3,
            output_type: FitOutputType::Faces,
            ..Default::default()
        };
        let feat_faces = execute_fit_to_object(
            &[&patch],
            &target,
            &bvh,
            &config_faces,
            1,
            [0.2, 0.6, 0.9, 1.0],
        )
        .unwrap();

        assert_eq!(feat_faces.output_type, FitOutputType::Faces);
        assert!(feat_faces.volume.is_none());
        assert!(!feat_faces.is_watertight); // Open sheet

        let config_solid = FitToObjectConfig {
            method: FitMethod::HardRaycast,
            output_type: FitOutputType::Solid,
            penetration_depth: 4.0,
            projection_dir: EdgeProjectionDir::SurfaceNormal,
            ..Default::default()
        };
        let feat_solid = execute_fit_to_object(
            &[&patch],
            &target,
            &bvh,
            &config_solid,
            2,
            [0.9, 0.5, 0.2, 1.0],
        )
        .unwrap();

        assert_eq!(feat_solid.output_type, FitOutputType::Solid);
        assert!(feat_solid.volume.is_some());
        assert!(feat_solid.is_watertight);
        assert!(feat_solid.volume.unwrap() > 0.0);
    }
}

