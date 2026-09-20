//! Automated quad retopology (experimental).
//!
//! A field-aligned remesher in the Instant Meshes family (Jakob, Tarini,
//! Panozzo, Sorkine-Hornung: "Instant Field-Aligned Meshes", SIGGRAPH Asia
//! 2015); `docs/quad-retopology.md` explains why this method was chosen.
//!
//! 1. [`prep`]: a work mesh whose resolution matches the target edge length
//!    (dense scans are decimated, long edges split); sharp creases and open
//!    boundaries become hard constraints of the fields.
//! 2. [`hierarchy`]: a multi-resolution graph built by greedy vertex pairing.
//! 3. [`field`]: an extrinsic 4-RoSy orientation field and a 4-PoSy position
//!    field, smoothed coarse to fine with parallel Gauss–Seidel. Extrinsic
//!    smoothing lets the edge flow follow the principal curvature directions.
//! 4. [`extract`]: vertices claiming the same lattice point collapse, unit
//!    lattice offsets become edges and faces are traced in that graph.
//! 5. [`post`]: triangle pairs merge into quads, then either one subdivision
//!    step makes every face a quad or larger polygons are split; vertices
//!    are projected onto the input and relaxed tangentially.

mod extract;
mod field;
mod hierarchy;
mod post;
mod prep;
#[cfg(test)]
mod tests;

use crate::geom::bvh::Bvh;
use crate::mesh::Mesh;

/// Lower bound for the target face count.
pub const MIN_TARGET_FACES: usize = 24;

/// Parameters of [`quad_retopology`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RetopoParams {
    /// Desired number of output faces; 0 derives it from the mesh
    /// ([`default_target_faces`]).
    pub target_faces: usize,
    /// Align the edge flow to sharp creases and open boundaries.
    pub align_features: bool,
    /// Edges whose faces meet at more than this angle are sharp creases.
    pub crease_angle_deg: f32,
    /// Produce only quads (extract at twice the edge length, then split
    /// every face once). Otherwise the result is quad-dominant.
    pub pure_quads: bool,
    /// Tangential relaxation passes after extraction.
    pub relax_iterations: u32,
}

impl Default for RetopoParams {
    fn default() -> Self {
        RetopoParams {
            target_faces: 0,
            align_features: true,
            crease_angle_deg: 40.0,
            pure_quads: true,
            relax_iterations: 3,
        }
    }
}

/// Summary of a retopology run.
#[derive(Clone, Copy, Debug)]
pub struct RetopoStats {
    pub quads: usize,
    /// Triangles that are not part of a quad.
    pub triangles: usize,
    pub vertices: usize,
    /// Interior vertices whose valence is not 4 (singularities).
    pub irregular_vertices: usize,
    /// Target edge length the lattice was built for.
    pub edge_length: f32,
}

pub struct RetopoOutput {
    pub mesh: Mesh,
    pub stats: RetopoStats,
}

/// Face count used when the target is left on automatic.
pub fn default_target_faces(triangles: usize) -> usize {
    (triangles / 20).clamp(500, 20_000)
}

/// Edge length of square faces covering `area` with `faces` faces.
pub fn target_edge_length(area: f32, faces: usize) -> f32 {
    (area / faces.max(1) as f32).sqrt()
}

/// Remeshes `input` into a quad(-dominant) mesh. `progress` receives the
/// fraction done and the current stage.
pub fn quad_retopology(
    input: &Mesh,
    params: &RetopoParams,
    progress: &dyn Fn(f32, &str),
) -> Result<RetopoOutput, String> {
    if input.triangle_count() < 4 {
        return Err("The mesh is too small for quad retopology.".to_string());
    }
    let area = input.surface_area();
    if !(area.is_finite() && area > 0.0) {
        return Err("The mesh has no surface area.".to_string());
    }
    let target = match params.target_faces {
        0 => default_target_faces(input.triangle_count()),
        t => t,
    }
    .max(MIN_TARGET_FACES);
    let h = target_edge_length(area, target);
    // Pure quads: extract at twice the edge length, subdivide once.
    let scale = if params.pure_quads { 2.0 * h } else { h };

    progress(0.02, "preparing work mesh");
    let work = prep::work_mesh(input, area, scale);
    if work.triangle_count() < 4 {
        return Err("The mesh is too small for quad retopology.".to_string());
    }
    progress(0.15, "detecting features");
    let crease_cos = params
        .align_features
        .then(|| params.crease_angle_deg.clamp(1.0, 179.0).to_radians().cos());
    let level0 = prep::build_level0(&work, scale, crease_cos);
    progress(0.2, "building hierarchy");
    let levels = hierarchy::build(level0);
    progress(0.3, "orientation field");
    let qs = field::solve_orientation(&levels);
    progress(0.45, "position field");
    let o = field::solve_positions(&levels, &qs, scale);
    progress(0.65, "spatial index");
    let bvh = Bvh::new(&input.positions, &input.indices);
    progress(0.72, "extracting quads");
    let poly = extract::extract(&levels[0], &qs[0], &o, scale, &bvh);
    if poly.faces.is_empty() {
        return Err(
            "Quad extraction produced no faces. Try a larger target face count.".to_string(),
        );
    }
    progress(0.85, "cleaning up");
    let poly = post::finish(poly, params, scale, input, &bvh);
    let (mesh, quads, irregular) = post::to_mesh(&poly);
    if mesh.triangle_count() == 0 {
        return Err(
            "Quad extraction produced no faces. Try a larger target face count.".to_string(),
        );
    }
    let stats = RetopoStats {
        quads,
        triangles: mesh.triangle_count() - 2 * quads,
        vertices: mesh.vertex_count(),
        irregular_vertices: irregular,
        edge_length: h,
    };
    Ok(RetopoOutput { mesh, stats })
}
