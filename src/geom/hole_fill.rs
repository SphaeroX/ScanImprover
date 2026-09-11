use crate::geom::hole_detect::HoleLoop;
use crate::mesh::Mesh;
use glam::Vec3;

/// Available algorithms for filling a mesh boundary hole.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HoleFillMethod {
    /// Connects boundary edges to the loop centroid. Very fast and robust.
    PlanarFan,
    /// 2D best-fit plane projection and polygon ear-clipping. No interior vertices.
    EarClipping,
    /// Barequet-Sharir dynamic programming: finds 3D triangulation minimizing total surface area.
    MinimalArea,
    /// Liepa (2003): Initial triangulation followed by edge refinement and Laplacian surface fairing.
    LiepaSmooth,
}

impl HoleFillMethod {
    pub fn all() -> [HoleFillMethod; 4] {
        [
            HoleFillMethod::PlanarFan,
            HoleFillMethod::EarClipping,
            HoleFillMethod::MinimalArea,
            HoleFillMethod::LiepaSmooth,
        ]
    }

    pub fn display_name(&self) -> &'static str {
        match self {
            HoleFillMethod::PlanarFan => "Planar Fan (Centroid)",
            HoleFillMethod::EarClipping => "Ear Clipping (Planar)",
            HoleFillMethod::MinimalArea => "Minimal Area (DP)",
            HoleFillMethod::LiepaSmooth => "Liepa Smooth (Refined & Faired)",
        }
    }
}

/// Direction mode for bulging / curving the hole fill.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FillDirectionMode {
    AutoNormal,
    InvertedNormal,
    AxisX,
    AxisY,
    AxisZ,
}

impl FillDirectionMode {
    pub fn all() -> [FillDirectionMode; 5] {
        [
            FillDirectionMode::AutoNormal,
            FillDirectionMode::InvertedNormal,
            FillDirectionMode::AxisX,
            FillDirectionMode::AxisY,
            FillDirectionMode::AxisZ,
        ]
    }

    pub fn display_name(&self) -> &'static str {
        match self {
            FillDirectionMode::AutoNormal => "Auto (Surface Normal)",
            FillDirectionMode::InvertedNormal => "Inverted Normal",
            FillDirectionMode::AxisX => "Axis X",
            FillDirectionMode::AxisY => "Axis Y",
            FillDirectionMode::AxisZ => "Axis Z",
        }
    }

    pub fn compute_vector(&self, hole_normal: Vec3) -> Vec3 {
        match self {
            FillDirectionMode::AutoNormal => hole_normal.normalize_or_zero(),
            FillDirectionMode::InvertedNormal => -hole_normal.normalize_or_zero(),
            FillDirectionMode::AxisX => Vec3::X,
            FillDirectionMode::AxisY => Vec3::Y,
            FillDirectionMode::AxisZ => Vec3::Z,
        }
    }
}

/// Comprehensive Meshmixer-style configuration for hole filling.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct HoleFillConfig {
    pub method: HoleFillMethod,
    /// Density / resolution scale: 0.2 (coarse) to 3.0 (ultra fine). Default: 1.0
    pub density: f32,
    /// Bulge / roundness factor: -1.0 (concave) .. 0.0 (flat) .. 1.0 (convex dome). Default: 0.0
    pub bulge: f32,
    /// Direction mode for the bulge / curvature
    pub direction_mode: FillDirectionMode,
    /// Fairing / smoothing iterations: 5 to 50. Default: 25
    pub smooth_iterations: usize,
}

impl Default for HoleFillConfig {
    fn default() -> Self {
        Self {
            method: HoleFillMethod::LiepaSmooth,
            density: 1.0,
            bulge: 0.0,
            direction_mode: FillDirectionMode::AutoNormal,
            smooth_iterations: 25,
        }
    }
}

/// Represents a triangulated patch filling a hole.
#[derive(Clone, Debug, Default)]
pub struct MeshPatch {
    /// Newly introduced vertex positions (to be appended to mesh.positions).
    pub new_positions: Vec<[f32; 3]>,
    /// Indices of the newly formed triangles (referencing existing mesh indices or new_positions).
    pub new_indices: Vec<u32>,
    /// Standalone positions used for instant preview rendering in viewport.
    pub preview_positions: Vec<[f32; 3]>,
    /// Standalone triangle indices into preview_positions for viewport rendering.
    pub preview_indices: Vec<u32>,
}

/// Triangulates a hole loop with the specified configuration.
pub fn generate_hole_patch(
    mesh: &Mesh,
    hole: &HoleLoop,
    config: HoleFillConfig,
) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    if n < 3 {
        return Err("Hole loop must have at least 3 vertices".to_string());
    }

    let base_nv = mesh.positions.len() as u32;

    match config.method {
        HoleFillMethod::PlanarFan => fill_planar_fan(mesh, hole, base_nv, config),
        HoleFillMethod::EarClipping => {
            if config.bulge.abs() > 0.01 || config.density > 1.2 {
                fill_liepa_smooth(mesh, hole, base_nv, config)
            } else {
                fill_ear_clipping(mesh, hole, base_nv)
            }
        }
        HoleFillMethod::MinimalArea => {
            if config.bulge.abs() > 0.01 || config.density > 1.2 {
                fill_liepa_smooth(mesh, hole, base_nv, config)
            } else if n <= 120 {
                fill_minimal_area(mesh, hole, base_nv)
            } else {
                fill_ear_clipping(mesh, hole, base_nv)
            }
        }
        HoleFillMethod::LiepaSmooth => fill_liepa_smooth(mesh, hole, base_nv, config),
    }
}

/// Applies a patch directly to the mesh, updating its geometry and normals.
pub fn apply_patch(mesh: &mut Mesh, patch: &MeshPatch) {
    if patch.new_indices.is_empty() {
        return;
    }
    mesh.positions.extend_from_slice(&patch.new_positions);
    mesh.indices.extend_from_slice(&patch.new_indices);
    mesh.recompute_normals();
}

/// Fills all specified holes on the mesh in sequence using the chosen configuration.
#[allow(dead_code)]
pub fn fill_holes(mesh: &Mesh, holes: &[HoleLoop], config: HoleFillConfig) -> Result<Mesh, String> {
    let mut working = mesh.clone();
    for hole in holes {
        let patch = generate_hole_patch(&working, hole, config)?;
        apply_patch(&mut working, &patch);
    }
    Ok(working)
}

// -------------------------------------------------------------------------------------------------
// Algorithm 1: Planar Fan (Centroid Fan with Bulge Option)
// -------------------------------------------------------------------------------------------------

fn fill_planar_fan(
    mesh: &Mesh,
    hole: &HoleLoop,
    base_nv: u32,
    config: HoleFillConfig,
) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    let dir = config.direction_mode.compute_vector(hole.normal);
    let radius = hole.bbox.diagonal() * 0.5;
    let center = hole.centroid + dir * (radius * config.bulge);
    let center_idx = base_nv;

    let mut new_indices = Vec::with_capacity(n * 3);
    for i in 0..n {
        let v0 = hole.vertices[i];
        let v1 = hole.vertices[(i + 1) % n];
        // Reverse winding to match adjacent boundary orientation
        new_indices.push(v1);
        new_indices.push(v0);
        new_indices.push(center_idx);
    }

    // Build standalone preview
    let mut preview_positions = Vec::with_capacity(n + 1);
    for &v in &hole.vertices {
        preview_positions.push(mesh.positions[v as usize]);
    }
    preview_positions.push(center.to_array());

    let mut preview_indices = Vec::with_capacity(n * 3);
    let prev_center = n as u32;
    for i in 0..n {
        let v0 = i as u32;
        let v1 = ((i + 1) % n) as u32;
        preview_indices.push(v1);
        preview_indices.push(v0);
        preview_indices.push(prev_center);
    }

    Ok(MeshPatch {
        new_positions: vec![center.to_array()],
        new_indices,
        preview_positions,
        preview_indices,
    })
}

// -------------------------------------------------------------------------------------------------
// Algorithm 2: Ear Clipping (Best-Fit Plane Projection)
// -------------------------------------------------------------------------------------------------

fn fill_ear_clipping(mesh: &Mesh, hole: &HoleLoop, _base_nv: u32) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    let positions = &mesh.positions;

    let (u, v) = plane_basis(hole.normal);
    let centroid = hole.centroid;

    let mut poly_2d: Vec<glam::Vec2> = Vec::with_capacity(n);
    for &idx in &hole.vertices {
        let p = Vec3::from(positions[idx as usize]) - centroid;
        poly_2d.push(glam::Vec2::new(p.dot(u), p.dot(v)));
    }

    let mut signed_area = 0.0f32;
    for i in 0..n {
        let p0 = poly_2d[i];
        let p1 = poly_2d[(i + 1) % n];
        signed_area += p0.x * p1.y - p1.x * p0.y;
    }

    let tris_2d = ear_clip_polygon(&poly_2d)?;

    let mut new_indices = Vec::with_capacity(tris_2d.len() * 3);
    let mut preview_indices = Vec::with_capacity(tris_2d.len() * 3);

    for [i0, i1, i2] in tris_2d {
        let (m0, m1, m2) = if signed_area > 0.0 {
            (hole.vertices[i0], hole.vertices[i2], hole.vertices[i1])
        } else {
            (hole.vertices[i0], hole.vertices[i1], hole.vertices[i2])
        };
        new_indices.push(m0);
        new_indices.push(m1);
        new_indices.push(m2);

        if signed_area > 0.0 {
            preview_indices.push(i0 as u32);
            preview_indices.push(i2 as u32);
            preview_indices.push(i1 as u32);
        } else {
            preview_indices.push(i0 as u32);
            preview_indices.push(i1 as u32);
            preview_indices.push(i2 as u32);
        }
    }

    let mut preview_positions = Vec::with_capacity(n);
    for &v_idx in &hole.vertices {
        preview_positions.push(positions[v_idx as usize]);
    }

    Ok(MeshPatch {
        new_positions: Vec::new(),
        new_indices,
        preview_positions,
        preview_indices,
    })
}

pub(crate) fn ear_clip_polygon(poly: &[glam::Vec2]) -> Result<Vec<[usize; 3]>, String> {
    let n = poly.len();
    if n < 3 {
        return Err("Cannot triangulate polygon with < 3 vertices".to_string());
    }

    let mut remaining: Vec<usize> = (0..n).collect();
    let mut triangles = Vec::with_capacity(n - 2);

    let mut area = 0.0f32;
    for i in 0..n {
        let p0 = poly[i];
        let p1 = poly[(i + 1) % n];
        area += p0.x * p1.y - p1.x * p0.y;
    }
    let ccw = area >= 0.0;

    let mut max_iters = n * n * 2;
    while remaining.len() > 3 && max_iters > 0 {
        max_iters -= 1;
        let count = remaining.len();
        let mut ear_found = false;

        for i in 0..count {
            let prev = remaining[(i + count - 1) % count];
            let curr = remaining[i];
            let next = remaining[(i + 1) % count];

            let a = poly[prev];
            let b = poly[curr];
            let c = poly[next];

            let cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            let is_convex = if ccw { cross > 1e-9 } else { cross < -1e-9 };
            if !is_convex {
                continue;
            }

            let mut point_inside = false;
            for &p_idx in &remaining {
                if p_idx == prev || p_idx == curr || p_idx == next {
                    continue;
                }
                if point_in_triangle_2d(poly[p_idx], a, b, c) {
                    point_inside = true;
                    break;
                }
            }

            if !point_inside {
                triangles.push([prev, curr, next]);
                remaining.remove(i);
                ear_found = true;
                break;
            }
        }

        if !ear_found {
            triangles.push([remaining[0], remaining[1], remaining[2]]);
            remaining.remove(1);
        }
    }

    if remaining.len() == 3 {
        triangles.push([remaining[0], remaining[1], remaining[2]]);
    }

    Ok(triangles)
}

#[inline]
fn point_in_triangle_2d(p: glam::Vec2, a: glam::Vec2, b: glam::Vec2, c: glam::Vec2) -> bool {
    let cross1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    let cross2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    let cross3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);

    let has_neg = (cross1 < -1e-7) || (cross2 < -1e-7) || (cross3 < -1e-7);
    let has_pos = (cross1 > 1e-7) || (cross2 > 1e-7) || (cross3 > 1e-7);

    !(has_neg && has_pos)
}

// -------------------------------------------------------------------------------------------------
// Algorithm 3: Minimal Area Triangulation (Barequet-Sharir DP)
// -------------------------------------------------------------------------------------------------

fn fill_minimal_area(mesh: &Mesh, hole: &HoleLoop, _base_nv: u32) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    let positions = &mesh.positions;

    let pts: Vec<Vec3> = hole
        .vertices
        .iter()
        .map(|&idx| Vec3::from(positions[idx as usize]))
        .collect();

    let mut cost = vec![vec![0.0f32; n]; n];
    let mut split = vec![vec![0usize; n]; n];

    for len in 2..n {
        for i in 0..(n - len) {
            let j = i + len;
            let mut min_val = f32::MAX;
            let mut best_k = i + 1;

            let pi = pts[i];
            let pj = pts[j];

            for k in (i + 1)..j {
                let pk = pts[k];
                let tri_area = (pk - pi).cross(pj - pi).length() * 0.5;
                let total = cost[i][k] + cost[k][j] + tri_area;
                if total < min_val {
                    min_val = total;
                    best_k = k;
                }
            }

            cost[i][j] = min_val;
            split[i][j] = best_k;
        }
    }

    let mut tri_indices_local: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
    fn reconstruct(i: usize, j: usize, split: &[Vec<usize>], out: &mut Vec<[usize; 3]>) {
        if i + 1 >= j {
            return;
        }
        let k = split[i][j];
        out.push([i, k, j]);
        reconstruct(i, k, split, out);
        reconstruct(k, j, split, out);
    }
    reconstruct(0, n - 1, &split, &mut tri_indices_local);

    let mut new_indices = Vec::with_capacity(tri_indices_local.len() * 3);
    let mut preview_indices = Vec::with_capacity(tri_indices_local.len() * 3);

    for [i0, i1, i2] in tri_indices_local {
        let p0 = pts[i0];
        let p1 = pts[i1];
        let p2 = pts[i2];
        let tri_n = (p1 - p0).cross(p2 - p0);

        let (v0, v1, v2) = if tri_n.dot(hole.normal) < 0.0 {
            (i0, i2, i1)
        } else {
            (i0, i1, i2)
        };

        new_indices.push(hole.vertices[v0]);
        new_indices.push(hole.vertices[v1]);
        new_indices.push(hole.vertices[v2]);

        preview_indices.push(v0 as u32);
        preview_indices.push(v1 as u32);
        preview_indices.push(v2 as u32);
    }

    let mut preview_positions = Vec::with_capacity(n);
    for &p in &pts {
        preview_positions.push(p.to_array());
    }

    Ok(MeshPatch {
        new_positions: Vec::new(),
        new_indices,
        preview_positions,
        preview_indices,
    })
}

// -------------------------------------------------------------------------------------------------
// Algorithm 4: Liepa (2003) Refined & Faired Surface with Meshmixer-Style Bulge & Density
// -------------------------------------------------------------------------------------------------

fn fill_liepa_smooth(
    mesh: &Mesh,
    hole: &HoleLoop,
    base_nv: u32,
    config: HoleFillConfig,
) -> Result<MeshPatch, String> {
    // 1. Initial triangulation via Minimal Area (or Ear Clipping fallback)
    let initial_patch = if hole.vertices.len() <= 80 {
        fill_minimal_area(mesh, hole, base_nv)?
    } else {
        fill_ear_clipping(mesh, hole, base_nv)?
    };

    let n_boundary = hole.vertices.len();
    let mut positions_local: Vec<Vec3> = hole
        .vertices
        .iter()
        .map(|&idx| Vec3::from(mesh.positions[idx as usize]))
        .collect();

    let mut tri_locals: Vec<[usize; 3]> = Vec::new();
    for chunk in initial_patch.preview_indices.chunks_exact(3) {
        tri_locals.push([chunk[0] as usize, chunk[1] as usize, chunk[2] as usize]);
    }

    // 2. Adaptive Refinement:
    // Scale target edge length by config.density
    let density = config.density.clamp(0.2, 4.0);
    let target_edge_len = (hole.perimeter / (n_boundary as f32).max(1.0)) / density;
    let target_area = 0.433 * target_edge_len * target_edge_len * 2.0;

    // Multi-pass refinement if high density requested
    let passes = if density > 1.4 { 2 } else { 1 };
    let mut refined_tris = tri_locals;

    for _ in 0..passes {
        let mut next_tris = Vec::with_capacity(refined_tris.len() * 3);
        for [i0, i1, i2] in refined_tris {
            let p0 = positions_local[i0];
            let p1 = positions_local[i1];
            let p2 = positions_local[i2];
            let area = (p1 - p0).cross(p2 - p0).length() * 0.5;

            if area > target_area {
                let centroid = (p0 + p1 + p2) * (1.0 / 3.0);
                let c_idx = positions_local.len();
                positions_local.push(centroid);

                next_tris.push([i0, i1, c_idx]);
                next_tris.push([i1, i2, c_idx]);
                next_tris.push([i2, i0, c_idx]);
            } else {
                next_tris.push([i0, i1, i2]);
            }
        }
        refined_tris = next_tris;
    }

    // 2b. Edge flipping (Liepa): the centroid refinement produces skinny
    // triangles; flipping interior edges towards the locally Delaunay
    // configuration restores well shaped triangles before fairing.
    improve_triangulation(&mut refined_tris, &positions_local, n_boundary);

    // 3. Bulge / Curvature Application:
    // Displace newly created interior vertices along the chosen direction vector
    let radius = (hole.bbox.diagonal() * 0.5).max(1e-4);
    let dir = config.direction_mode.compute_vector(hole.normal);

    if config.bulge.abs() > 0.001 && positions_local.len() > n_boundary {
        for v in n_boundary..positions_local.len() {
            let p = positions_local[v];
            // Compute minimum distance to boundary loop
            let mut min_dist_sq = f32::MAX;
            for b in 0..n_boundary {
                let d_sq = (positions_local[b] - p).length_squared();
                if d_sq < min_dist_sq {
                    min_dist_sq = d_sq;
                }
            }
            let min_dist = min_dist_sq.sqrt();
            let rel_depth = (min_dist / radius).clamp(0.0, 1.0);
            // Sinusoidal dome height profile: 0 at rim, 1 at center
            let dome_factor = (rel_depth * std::f32::consts::FRAC_PI_2).sin();
            let offset = dir * (radius * config.bulge * dome_factor);
            positions_local[v] += offset;
        }
    }

    // 4. Fairing: Umbrella / Laplacian smoothing on interior vertices (boundary fixed)
    let num_total_verts = positions_local.len();
    if num_total_verts > n_boundary {
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); num_total_verts];
        for &[i0, i1, i2] in &refined_tris {
            adj[i0].push(i1);
            adj[i0].push(i2);
            adj[i1].push(i0);
            adj[i1].push(i2);
            adj[i2].push(i0);
            adj[i2].push(i1);
        }

        for list in &mut adj {
            list.sort_unstable();
            list.dedup();
        }

        let iters = config.smooth_iterations.clamp(2, 60);
        let damp_rate = if config.bulge.abs() > 0.05 { 0.35 } else { 0.5 };

        for _ in 0..iters {
            let mut next_pos = positions_local.clone();
            for v in n_boundary..num_total_verts {
                let neighbors = &adj[v];
                if neighbors.is_empty() {
                    continue;
                }
                let mut sum = Vec3::ZERO;
                for &nb in neighbors {
                    sum += positions_local[nb];
                }
                let avg = sum / (neighbors.len() as f32);
                next_pos[v] = positions_local[v] + (avg - positions_local[v]) * damp_rate;
            }
            positions_local = next_pos;
        }
    }

    // Convert local vertices and triangles to patch
    let mut new_positions = Vec::new();
    for v in n_boundary..num_total_verts {
        new_positions.push(positions_local[v].to_array());
    }

    let mut new_indices = Vec::with_capacity(refined_tris.len() * 3);
    let mut preview_indices = Vec::with_capacity(refined_tris.len() * 3);

    for [i0, i1, i2] in refined_tris {
        let m0 = if i0 < n_boundary {
            hole.vertices[i0]
        } else {
            base_nv + (i0 - n_boundary) as u32
        };
        let m1 = if i1 < n_boundary {
            hole.vertices[i1]
        } else {
            base_nv + (i1 - n_boundary) as u32
        };
        let m2 = if i2 < n_boundary {
            hole.vertices[i2]
        } else {
            base_nv + (i2 - n_boundary) as u32
        };

        new_indices.push(m0);
        new_indices.push(m1);
        new_indices.push(m2);

        preview_indices.push(i0 as u32);
        preview_indices.push(i1 as u32);
        preview_indices.push(i2 as u32);
    }

    let preview_positions: Vec<[f32; 3]> = positions_local.iter().map(|p| p.to_array()).collect();

    Ok(MeshPatch {
        new_positions,
        new_indices,
        preview_positions,
        preview_indices,
    })
}

// -------------------------------------------------------------------------------------------------
// Triangle quality improvement (edge flips)
// -------------------------------------------------------------------------------------------------

/// Smallest interior angle of a triangle (radians).
fn min_angle(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let angle = |p: Vec3, q: Vec3, r: Vec3| {
        let u = q - p;
        let v = r - p;
        let d = u.length() * v.length();
        if d < 1e-20 {
            0.0
        } else {
            (u.dot(v) / d).clamp(-1.0, 1.0).acos()
        }
    };
    angle(a, b, c).min(angle(b, c, a)).min(angle(c, a, b))
}

/// Flips interior edges of the patch whenever the flip raises the minimum
/// angle of the two triangles sharing the edge (the classic Delaunay-style
/// improvement used by Liepa's hole filling). Loop edges between
/// consecutive boundary vertices are never touched. Terminates because the
/// sorted angle vector strictly increases with every flip.
type EdgeUsers = std::collections::HashMap<(usize, usize), Vec<(usize, usize)>>;

fn improve_triangulation(tris: &mut [[usize; 3]], pos: &[Vec3], n_boundary: usize) {
    const MAX_PASSES: usize = 12;
    let is_loop_edge = |a: usize, b: usize| {
        a < n_boundary && b < n_boundary && ((a + 1) % n_boundary == b || (b + 1) % n_boundary == a)
    };
    for _ in 0..MAX_PASSES {
        // Map undirected edge -> (tri index, local edge index) pairs.
        let mut edge_map: EdgeUsers = EdgeUsers::with_capacity(tris.len() * 3);
        for (ti, tri) in tris.iter().enumerate() {
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                let key = if a < b { (a, b) } else { (b, a) };
                edge_map.entry(key).or_default().push((ti, e));
            }
        }
        let mut flipped_any = false;
        let mut touched = vec![false; tris.len()];
        let mut edges: Vec<_> = edge_map.iter().collect();
        edges.sort_unstable_by_key(|(k, _)| **k);
        for (&(a, b), users) in edges {
            if users.len() != 2 || is_loop_edge(a, b) {
                continue;
            }
            let (t0, e0) = users[0];
            let (t1, e1) = users[1];
            if touched[t0] || touched[t1] {
                continue;
            }
            // Opposite vertices of the shared edge.
            let c = tris[t0][(e0 + 2) % 3];
            let d = tris[t1][(e1 + 2) % 3];
            if c == d || c == a || c == b || d == a || d == b {
                continue;
            }
            // The new edge (c, d) must not already exist.
            let new_key = if c < d { (c, d) } else { (d, c) };
            if edge_map.contains_key(&new_key) {
                continue;
            }
            let (pa, pb, pc, pd) = (pos[a], pos[b], pos[c], pos[d]);
            let before = min_angle(pa, pb, pc).min(min_angle(pa, pb, pd));
            let after = min_angle(pc, pd, pa).min(min_angle(pc, pd, pb));
            if after <= before + 1e-6 {
                continue;
            }
            // Reject flips that would fold the quad (normals of the new
            // triangles must agree with the old ones).
            let n_old = (pb - pa).cross(pc - pa) + (pa - pb).cross(pd - pb);
            let t0_new = [tris[t0][e0], d, c];
            let t1_new = [tris[t1][e1], c, d];
            let n_new = (pos[t0_new[1]] - pos[t0_new[0]]).cross(pos[t0_new[2]] - pos[t0_new[0]])
                + (pos[t1_new[1]] - pos[t1_new[0]]).cross(pos[t1_new[2]] - pos[t1_new[0]]);
            if n_old.dot(n_new) <= 0.0 {
                continue;
            }
            // Keep the original winding: t0 was (a, b, c) around edge a->b,
            // t1 was (b, a, d). The flipped pair is (a, d, c) and (b, c, d).
            let orient0 = tris[t0][e0];
            let (x, y) = if orient0 == a { (a, b) } else { (b, a) };
            tris[t0] = [x, d, c];
            tris[t1] = [y, c, d];
            touched[t0] = true;
            touched[t1] = true;
            flipped_any = true;
        }
        if !flipped_any {
            break;
        }
    }
}

// -------------------------------------------------------------------------------------------------
// Geometry Utilities
// -------------------------------------------------------------------------------------------------

fn plane_basis(n: Vec3) -> (Vec3, Vec3) {
    let n = n.normalize_or_zero();
    let up = if n.y.abs() < 0.9 { Vec3::Y } else { Vec3::Z };
    let u = n.cross(up).normalize_or_zero();
    let v = n.cross(u).normalize_or_zero();
    (u, v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edge_flip_raises_minimum_angle_and_keeps_orientation() {
        // Boundary loop A -> C -> B -> D with the long chord A-B as the
        // interior edge: flipping to C-D gives much better triangles.
        let pos = vec![
            Vec3::new(0.0, 0.0, 0.0),  // 0 = A
            Vec3::new(2.0, 1.0, 0.0),  // 1 = C
            Vec3::new(4.0, 0.0, 0.0),  // 2 = B
            Vec3::new(2.0, -1.0, 0.0), // 3 = D
        ];
        let mut tris = vec![[0usize, 2, 1], [0, 3, 2]];
        let before = tris
            .iter()
            .map(|t| min_angle(pos[t[0]], pos[t[1]], pos[t[2]]))
            .fold(f32::MAX, f32::min);
        improve_triangulation(&mut tris, &pos, 4);
        let after = tris
            .iter()
            .map(|t| min_angle(pos[t[0]], pos[t[1]], pos[t[2]]))
            .fold(f32::MAX, f32::min);
        assert!(after > before + 0.3, "min angle {before} -> {after}");
        // Both triangles keep a +Z normal and use the new diagonal.
        for t in &tris {
            let n = (pos[t[1]] - pos[t[0]]).cross(pos[t[2]] - pos[t[0]]);
            assert!(n.z > 0.0);
            assert!(t.contains(&1) && t.contains(&3));
        }
    }

    #[test]
    fn loop_edges_are_never_flipped() {
        // A single boundary triangle plus one interior vertex (fan of 3).
        let pos = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::new(2.0, 3.0, 0.0),
            Vec3::new(2.0, 0.2, 0.0), // interior, very close to the base edge
        ];
        let mut tris = vec![[0usize, 1, 3], [1, 2, 3], [2, 0, 3]];
        improve_triangulation(&mut tris, &pos, 3);
        // Every triangle still contains the interior vertex: the boundary
        // edges (0,1), (1,2), (2,0) were not flipped away.
        assert!(tris.iter().all(|t| t.contains(&3)));
    }
}
