use crate::geom::fitting::{FittedCircle, FittedPlane};
use crate::geom::hole_detect::HoleLoop;
use crate::geom::hole_fill::{
    FillDirectionMode, HoleFillConfig, HoleFillMethod, MeshPatch, generate_hole_patch,
};
use crate::mesh::Mesh;
use glam::Vec3;

/// Interpretation mode for a reference circle.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CircleGuideMode {
    /// Bounded circular disk and boundary rim (does not extend to infinite cylinder or plane).
    #[default]
    DiskAndRim,
    /// Cylindrical wall surface (localized to hole vicinity).
    CylinderWall,
}

/// Represents a geometric reference feature that hole fills can be guided by.
#[allow(dead_code)]
#[derive(Clone, Debug)]
pub enum ReferenceGeometry {
    Plane {
        id: u64,
        name: String,
        point: Vec3,
        normal: Vec3,
    },
    Circle {
        id: u64,
        name: String,
        center: Vec3,
        normal: Vec3,
        radius: f32,
        mode: CircleGuideMode,
    },
}

impl ReferenceGeometry {
    pub fn from_plane(plane: &FittedPlane) -> Self {
        Self::Plane {
            id: plane.id,
            name: plane.name.clone(),
            point: plane.fit.point,
            normal: plane.fit.normal.normalize_or_zero(),
        }
    }

    pub fn from_circle(circle: &FittedCircle, mode: CircleGuideMode) -> Self {
        Self::Circle {
            id: circle.id,
            name: circle.name.clone(),
            center: circle.fit.center,
            normal: circle.fit.normal.normalize_or_zero(),
            radius: circle.fit.radius,
            mode,
        }
    }

    #[allow(dead_code)]
    pub fn name(&self) -> &str {
        match self {
            Self::Plane { name, .. } => name.as_str(),
            Self::Circle { name, .. } => name.as_str(),
        }
    }

    /// Calculates the unsigned distance from a 3D point to this reference geometry.
    pub fn distance_to_point(&self, p: Vec3) -> f32 {
        match self {
            Self::Plane { point, normal, .. } => ((p - *point).dot(*normal)).abs(),
            Self::Circle {
                center,
                normal,
                radius,
                mode,
                ..
            } => {
                let v = p - *center;
                let h = v.dot(*normal);
                let r_perp = v - *normal * h;
                let d_perp = r_perp.length();

                match mode {
                    CircleGuideMode::DiskAndRim => {
                        if d_perp <= *radius {
                            // Inside circular disk: perpendicular distance to disk plane
                            h.abs()
                        } else {
                            // Outside circular disk: 3D distance to the circular boundary rim
                            let dr = d_perp - *radius;
                            (dr * dr + h * h).sqrt()
                        }
                    }
                    CircleGuideMode::CylinderWall => {
                        let dr = (d_perp - *radius).abs();
                        // Cylinders in real parts are bounded in height (not infinite into infinity)
                        let max_h = *radius * 2.0;
                        if h.abs() <= max_h {
                            dr
                        } else {
                            let dh = h.abs() - max_h;
                            (dr * dr + dh * dh).sqrt()
                        }
                    }
                }
            }
        }
    }

    /// Projects a 3D point onto the nearest surface of this reference geometry.
    pub fn project_point(&self, p: Vec3) -> Vec3 {
        match self {
            Self::Plane { point, normal, .. } => {
                let dist = (p - *point).dot(*normal);
                p - *normal * dist
            }
            Self::Circle {
                center,
                normal,
                radius,
                mode,
                ..
            } => {
                let v = p - *center;
                let h = v.dot(*normal);
                let r_perp = v - *normal * h;
                let d_perp = r_perp.length();

                let dir = if d_perp > 1e-6 {
                    r_perp / d_perp
                } else {
                    let mut ortho = Vec3::new(normal.y, -normal.x, 0.0);
                    if ortho.length_squared() < 1e-6 {
                        ortho = Vec3::new(0.0, normal.z, -normal.y);
                    }
                    ortho.normalize_or_zero()
                };

                match mode {
                    CircleGuideMode::DiskAndRim => {
                        if d_perp <= *radius {
                            // Project onto circular disk plane
                            p - *normal * h
                        } else {
                            // Project onto circular perimeter rim
                            *center + dir * *radius
                        }
                    }
                    CircleGuideMode::CylinderWall => {
                        // Project onto cylinder wall at height h
                        *center + *normal * h + dir * *radius
                    }
                }
            }
        }
    }
}

/// Computes the minimum distance from a point to a collection of reference geometries.
pub fn distance_to_references(p: Vec3, refs: &[ReferenceGeometry]) -> f32 {
    if refs.is_empty() {
        return 0.0;
    }
    let mut min_d = f32::MAX;
    for r in refs {
        let d = r.distance_to_point(p);
        if d < min_d {
            min_d = d;
        }
    }
    min_d
}

/// Finds the closest reference geometry to a given 3D point.
pub fn closest_reference(p: Vec3, refs: &[ReferenceGeometry]) -> Option<(&ReferenceGeometry, f32)> {
    if refs.is_empty() {
        return None;
    }
    let mut best_ref = None;
    let mut min_d = f32::MAX;
    for r in refs {
        let d = r.distance_to_point(p);
        if d < min_d {
            min_d = d;
            best_ref = Some(r);
        }
    }
    best_ref.map(|r| (r, min_d))
}

/// Evaluation metrics of a generated patch against reference geometry.
#[derive(Clone, Copy, Debug)]
pub struct PatchEvaluation {
    pub rms: f32,
    pub max_dev: f32,
    pub score: f32,
}

/// Evaluates a candidate patch against the reference geometries by sampling
/// the true triangulated patch surface and interior vertices.
pub fn evaluate_patch(patch: &MeshPatch, refs: &[ReferenceGeometry]) -> PatchEvaluation {
    if refs.is_empty() {
        return PatchEvaluation {
            rms: 0.0,
            max_dev: 0.0,
            score: 0.0,
        };
    }

    let mut sample_points = Vec::new();

    // 1. Sample triangle centroids from preview mesh (represents the true triangulated surface)
    let num_tris = patch.preview_indices.len() / 3;
    for t in 0..num_tris {
        let i0 = patch.preview_indices[t * 3] as usize;
        let i1 = patch.preview_indices[t * 3 + 1] as usize;
        let i2 = patch.preview_indices[t * 3 + 2] as usize;
        if let (Some(&p0), Some(&p1), Some(&p2)) = (
            patch.preview_positions.get(i0),
            patch.preview_positions.get(i1),
            patch.preview_positions.get(i2),
        ) {
            let c = (Vec3::from_array(p0) + Vec3::from_array(p1) + Vec3::from_array(p2)) / 3.0;
            sample_points.push(c);
        }
    }

    // 2. Also include newly added interior vertices
    for pos in &patch.new_positions {
        sample_points.push(Vec3::from_array(*pos));
    }

    // Fallback if no triangles or vertices
    if sample_points.is_empty() {
        for pos in &patch.preview_positions {
            sample_points.push(Vec3::from_array(*pos));
        }
    }

    if sample_points.is_empty() {
        return PatchEvaluation {
            rms: 0.0,
            max_dev: 0.0,
            score: 0.0,
        };
    }

    let mut sum_sq = 0.0;
    let mut max_dev: f32 = 0.0;
    for &p in &sample_points {
        let d = distance_to_references(p, refs);
        sum_sq += d * d;
        if d > max_dev {
            max_dev = d;
        }
    }

    let rms = (sum_sq / sample_points.len() as f32).sqrt();
    let score = rms + 0.15 * max_dev;
    PatchEvaluation {
        rms,
        max_dev,
        score,
    }
}

/// Result of running the numerical hole solver.
#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct HoleSolveResult {
    pub best_config: HoleFillConfig,
    pub best_method: HoleFillMethod,
    pub rms_error: f32,
    pub max_error: f32,
    pub tested_count: usize,
    pub summary: String,
}

/// Solves for the optimal hole filling method and parameters to conform
/// as closely as possible to the provided reference geometries.
pub fn solve_best_hole_config(
    mesh: &Mesh,
    hole: &HoleLoop,
    refs: &[ReferenceGeometry],
) -> Result<HoleSolveResult, String> {
    if refs.is_empty() {
        return Err("No reference geometries provided to guide hole fill".to_string());
    }

    let mut best_eval = PatchEvaluation {
        rms: f32::MAX,
        max_dev: f32::MAX,
        score: f32::MAX,
    };
    let mut best_config = HoleFillConfig::default();
    let mut tested_count = 0;

    let test_directions = [
        FillDirectionMode::AutoNormal,
        FillDirectionMode::InvertedNormal,
    ];

    // 1. Evaluate Ear Clipping
    {
        let cfg = HoleFillConfig {
            method: HoleFillMethod::EarClipping,
            density: 1.0,
            bulge: 0.0,
            direction_mode: FillDirectionMode::AutoNormal,
            smooth_iterations: 10,
        };
        if let Ok(patch) = generate_hole_patch(mesh, hole, cfg) {
            tested_count += 1;
            let ev = evaluate_patch(&patch, refs);
            if ev.score < best_eval.score {
                best_eval = ev;
                best_config = cfg;
            }
        }
    }

    // 2. Evaluate Minimal Area (if vertex count suitable)
    if hole.vertices.len() <= 120 {
        let cfg = HoleFillConfig {
            method: HoleFillMethod::MinimalArea,
            density: 1.0,
            bulge: 0.0,
            direction_mode: FillDirectionMode::AutoNormal,
            smooth_iterations: 10,
        };
        if let Ok(patch) = generate_hole_patch(mesh, hole, cfg) {
            tested_count += 1;
            let ev = evaluate_patch(&patch, refs);
            if ev.score < best_eval.score {
                best_eval = ev;
                best_config = cfg;
            }
        }
    }

    // 3. Evaluate and optimize Liepa Smooth across bulge, directions, and iterations
    let liepa_bulges = [-0.6, -0.4, -0.2, -0.1, 0.0, 0.1, 0.2, 0.4, 0.6];
    for &dir in &test_directions {
        for &bulge in &liepa_bulges {
            for &iters in &[15usize, 25] {
                let cfg = HoleFillConfig {
                    method: HoleFillMethod::LiepaSmooth,
                    density: 1.0,
                    bulge,
                    direction_mode: dir,
                    smooth_iterations: iters,
                };
                if let Ok(patch) = generate_hole_patch(mesh, hole, cfg) {
                    tested_count += 1;
                    let ev = evaluate_patch(&patch, refs);
                    if ev.score < best_eval.score {
                        best_eval = ev;
                        best_config = cfg;
                    }
                }
            }
        }
    }

    // 4. Evaluate Planar Fan across modest bulge values with topology regularization penalty
    let fan_bulges = [-0.4, -0.2, 0.0, 0.2, 0.4];
    for &dir in &test_directions {
        for &bulge in &fan_bulges {
            let cfg = HoleFillConfig {
                method: HoleFillMethod::PlanarFan,
                density: 1.0,
                bulge,
                direction_mode: dir,
                smooth_iterations: 15,
            };
            if let Ok(patch) = generate_hole_patch(mesh, hole, cfg) {
                tested_count += 1;
                let ev = evaluate_patch(&patch, refs);
                // PlanarFan creates a single-vertex star fan with poor needle-triangle topology,
                // so we add a 25% regularization penalty compared to smooth fairing.
                let penalized_score = ev.score * 1.25;
                if penalized_score < best_eval.score {
                    best_eval = ev;
                    best_config = cfg;
                }
            }
        }
    }

    // Fine-tune bulge for the winning configuration if LiepaSmooth or PlanarFan
    if best_config.method == HoleFillMethod::LiepaSmooth
        || best_config.method == HoleFillMethod::PlanarFan
    {
        let center_bulge = best_config.bulge;
        let deltas = [-0.08, -0.04, 0.04, 0.08];
        for delta in deltas {
            let tuned_bulge = (center_bulge + delta).clamp(-1.0, 1.0);
            let mut cfg = best_config;
            cfg.bulge = tuned_bulge;
            if let Ok(patch) = generate_hole_patch(mesh, hole, cfg) {
                tested_count += 1;
                let ev = evaluate_patch(&patch, refs);
                let score = if cfg.method == HoleFillMethod::PlanarFan {
                    ev.score * 1.25
                } else {
                    ev.score
                };
                if score < best_eval.score {
                    best_eval = ev;
                    best_config = cfg;
                }
            }
        }
    }

    let summary = format!(
        "Optimal: {} (bulge: {:.2}, dir: {}, RMS: {:.4} mm, max: {:.4} mm)",
        best_config.method.display_name(),
        best_config.bulge,
        best_config.direction_mode.display_name(),
        best_eval.rms,
        best_eval.max_dev
    );

    Ok(HoleSolveResult {
        best_config,
        best_method: best_config.method,
        rms_error: best_eval.rms,
        max_error: best_eval.max_dev,
        tested_count,
        summary,
    })
}

/// Refines the interior vertices of a patch to snap onto the nearest reference geometry
/// while maintaining strict boundary continuity with the original mesh.
pub fn refine_patch_to_references(
    patch: &mut MeshPatch,
    mesh: &Mesh,
    hole: &HoleLoop,
    refs: &[ReferenceGeometry],
) {
    if refs.is_empty() || patch.new_positions.is_empty() {
        return;
    }

    let hole_diag = hole.bbox.diagonal().max(1e-4);
    let transition_radius = 0.35 * hole_diag;

    // Cache hole boundary positions
    let boundary_pts: Vec<Vec3> = hole
        .vertices
        .iter()
        .filter_map(|&vi| {
            mesh.positions
                .get(vi as usize)
                .map(|p| Vec3::from_array(*p))
        })
        .collect();

    if boundary_pts.is_empty() {
        return;
    }

    for pos in &mut patch.new_positions {
        let p = Vec3::from_array(*pos);

        // Find distance to closest boundary vertex
        let mut min_bnd_d = f32::MAX;
        for &bp in &boundary_pts {
            let d = p.distance(bp);
            if d < min_bnd_d {
                min_bnd_d = d;
            }
        }

        // Smooth falloff factor: 0 at boundary, 1 inside hole
        let raw_t = (min_bnd_d / transition_radius).clamp(0.0, 1.0);
        let weight = raw_t * raw_t * (3.0 - 2.0 * raw_t); // Smoothstep

        if weight > 1e-4
            && let Some((closest_ref, _)) = closest_reference(p, refs)
        {
            let proj = closest_ref.project_point(p);
            let refined = p.lerp(proj, weight);
            *pos = refined.to_array();
        }
    }

    // Synchronize preview positions if present
    if !patch.preview_positions.is_empty()
        && patch.preview_positions.len() >= patch.new_positions.len()
    {
        let offset = patch.preview_positions.len() - patch.new_positions.len();
        for (i, pos) in patch.new_positions.iter().enumerate() {
            patch.preview_positions[offset + i] = *pos;
        }
    }
}
