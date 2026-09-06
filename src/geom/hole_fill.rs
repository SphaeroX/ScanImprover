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

/// Triangulates a hole loop with the specified method.
pub fn generate_hole_patch(
    mesh: &Mesh,
    hole: &HoleLoop,
    method: HoleFillMethod,
) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    if n < 3 {
        return Err("Hole loop must have at least 3 vertices".to_string());
    }

    let base_nv = mesh.positions.len() as u32;

    match method {
        HoleFillMethod::PlanarFan => fill_planar_fan(mesh, hole, base_nv),
        HoleFillMethod::EarClipping => fill_ear_clipping(mesh, hole, base_nv),
        HoleFillMethod::MinimalArea => {
            // Barequet-Sharir DP is O(N^3). For very large holes (> 120 verts), fall back to ear clipping
            if n <= 120 {
                fill_minimal_area(mesh, hole, base_nv)
            } else {
                fill_ear_clipping(mesh, hole, base_nv)
            }
        }
        HoleFillMethod::LiepaSmooth => fill_liepa_smooth(mesh, hole, base_nv),
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

/// Fills all specified holes on the mesh in sequence using the chosen method.
pub fn fill_holes(
    mesh: &Mesh,
    holes: &[HoleLoop],
    method: HoleFillMethod,
) -> Result<Mesh, String> {
    let mut working = mesh.clone();
    for hole in holes {
        let patch = generate_hole_patch(&working, hole, method)?;
        apply_patch(&mut working, &patch);
    }
    Ok(working)
}

// -------------------------------------------------------------------------------------------------
// Algorithm 1: Planar Fan (Centroid Fan)
// -------------------------------------------------------------------------------------------------

fn fill_planar_fan(
    mesh: &Mesh,
    hole: &HoleLoop,
    base_nv: u32,
) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    let center = hole.centroid;
    let center_idx = base_nv;

    let mut new_indices = Vec::with_capacity(n * 3);
    for i in 0..n {
        let v0 = hole.vertices[i];
        let v1 = hole.vertices[(i + 1) % n];
        // Note: Boundary edge from find_boundary_edges is directed v0 -> v1.
        // To face outward consistently with the adjacent mesh, the filling triangle
        // traverses the boundary edge in reverse (v1 -> v0 -> center).
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

fn fill_ear_clipping(
    mesh: &Mesh,
    hole: &HoleLoop,
    _base_nv: u32,
) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    let positions = &mesh.positions;

    // Build orthonormal 2D coordinate frame (u, v) from hole.normal
    let (u, v) = plane_basis(hole.normal);
    let centroid = hole.centroid;

    // Project 3D loop into 2D polygon
    let mut poly_2d: Vec<glam::Vec2> = Vec::with_capacity(n);
    for &idx in &hole.vertices {
        let p = Vec3::from(positions[idx as usize]) - centroid;
        poly_2d.push(glam::Vec2::new(p.dot(u), p.dot(v)));
    }

    // Determine signed area in 2D
    let mut signed_area = 0.0f32;
    for i in 0..n {
        let p0 = poly_2d[i];
        let p1 = poly_2d[(i + 1) % n];
        signed_area += p0.x * p1.y - p1.x * p0.y;
    }

    // Triangulate 2D polygon using ear clipping
    let tris_2d = ear_clip_polygon(&poly_2d)?;

    // Map 2D triangle vertex indices to mesh indices
    let mut new_indices = Vec::with_capacity(tris_2d.len() * 3);
    let mut preview_indices = Vec::with_capacity(tris_2d.len() * 3);

    for [i0, i1, i2] in tris_2d {
        let (m0, m1, m2) = if signed_area > 0.0 {
            // CCW in 2D
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

    // Build preview positions
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

fn ear_clip_polygon(poly: &[glam::Vec2]) -> Result<Vec<[usize; 3]>, String> {
    let n = poly.len();
    if n < 3 {
        return Err("Cannot triangulate polygon with < 3 vertices".to_string());
    }

    let mut remaining: Vec<usize> = (0..n).collect();
    let mut triangles = Vec::with_capacity(n - 2);

    // Compute polygon winding
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

            // Convexity check
            let cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            let is_convex = if ccw { cross > 1e-9 } else { cross < -1e-9 };
            if !is_convex {
                continue;
            }

            // Check if any other point lies inside triangle (a, b, c)
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
                // Ear found!
                triangles.push([prev, curr, next]);
                remaining.remove(i);
                ear_found = true;
                break;
            }
        }

        if !ear_found {
            // Degenerate or self-intersecting polygon fallback: clip first triangle
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

fn fill_minimal_area(
    mesh: &Mesh,
    hole: &HoleLoop,
    _base_nv: u32,
) -> Result<MeshPatch, String> {
    let n = hole.vertices.len();
    let positions = &mesh.positions;

    let pts: Vec<Vec3> = hole
        .vertices
        .iter()
        .map(|&idx| Vec3::from(positions[idx as usize]))
        .collect();

    // Table of minimal areas: cost[i][j] is min area for sub-polygon pts[i..=j]
    // Table of split indices: split[i][j] is the vertex k minimizing the cost
    let mut cost = vec![vec![0.0f32; n]; n];
    let mut split = vec![vec![0usize; n]; n];

    // Subproblems of length L from 2 to n-1
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

    // Reconstruct triangles recursively
    let mut tri_indices_local: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
    fn reconstruct(
        i: usize,
        j: usize,
        split: &[Vec<usize>],
        out: &mut Vec<[usize; 3]>,
    ) {
        if i + 1 >= j {
            return;
        }
        let k = split[i][j];
        out.push([i, k, j]);
        reconstruct(i, k, split, out);
        reconstruct(k, j, split, out);
    }
    reconstruct(0, n - 1, &split, &mut tri_indices_local);

    // Ensure normal orientation matches hole.normal
    let mut new_indices = Vec::with_capacity(tri_indices_local.len() * 3);
    let mut preview_indices = Vec::with_capacity(tri_indices_local.len() * 3);

    for [i0, i1, i2] in tri_indices_local {
        let p0 = pts[i0];
        let p1 = pts[i1];
        let p2 = pts[i2];
        let tri_n = (p1 - p0).cross(p2 - p0);

        // We want the fill normal to be oriented in the same direction as hole.normal
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
// Algorithm 4: Liepa (2003) Refined & Faired Surface
// -------------------------------------------------------------------------------------------------

fn fill_liepa_smooth(
    mesh: &Mesh,
    hole: &HoleLoop,
    base_nv: u32,
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

    // Map initial local triangles: (0..n_boundary)
    let mut tri_locals: Vec<[usize; 3]> = Vec::new();
    for chunk in initial_patch.preview_indices.chunks_exact(3) {
        tri_locals.push([chunk[0] as usize, chunk[1] as usize, chunk[2] as usize]);
    }

    // 2. Refinement: Subdivide triangles that are significantly larger than the average boundary edge length
    let target_edge_len = hole.perimeter / (n_boundary as f32).max(1.0);
    let target_area = 0.433 * target_edge_len * target_edge_len * 2.5; // ~ equilateral triangle area * factor

    // Refinement: 1-to-3 centroid subdivision for large triangles
    let mut refined_tris = Vec::new();
    for [i0, i1, i2] in tri_locals {
        let p0 = positions_local[i0];
        let p1 = positions_local[i1];
        let p2 = positions_local[i2];
        let area = (p1 - p0).cross(p2 - p0).length() * 0.5;

        if area > target_area {
            let centroid = (p0 + p1 + p2) * (1.0 / 3.0);
            let c_idx = positions_local.len();
            positions_local.push(centroid);

            refined_tris.push([i0, i1, c_idx]);
            refined_tris.push([i1, i2, c_idx]);
            refined_tris.push([i2, i0, c_idx]);
        } else {
            refined_tris.push([i0, i1, i2]);
        }
    }

    // 3. Fairing: Umbrella / Laplacian smoothing on interior vertices (boundary fixed)
    let num_total_verts = positions_local.len();
    if num_total_verts > n_boundary {
        // Build adjacency for interior vertices
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); num_total_verts];
        for &[i0, i1, i2] in &refined_tris {
            adj[i0].push(i1);
            adj[i0].push(i2);
            adj[i1].push(i0);
            adj[i1].push(i2);
            adj[i2].push(i0);
            adj[i2].push(i1);
        }

        // Clean duplicates in adj
        for list in &mut adj {
            list.sort_unstable();
            list.dedup();
        }

        // 20 iterations of Laplacian smoothing
        for _ in 0..20 {
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
                // Damped update
                next_pos[v] = positions_local[v] + (avg - positions_local[v]) * 0.5;
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
// Geometry Utilities
// -------------------------------------------------------------------------------------------------

fn plane_basis(n: Vec3) -> (Vec3, Vec3) {
    let n = n.normalize_or_zero();
    let up = if n.y.abs() < 0.9 {
        Vec3::Y
    } else {
        Vec3::Z
    };
    let u = n.cross(up).normalize_or_zero();
    let v = n.cross(u).normalize_or_zero();
    (u, v)
}
