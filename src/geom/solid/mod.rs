//! Experimental solid (B-Rep) reconstruction from face groups.
//!
//! Mesh-guided construction for a closed mesh whose triangles are
//! segmented into face groups:
//!
//! 1. **Topology** from the mesh ([`topo`]): one B-Rep face per connected
//!    group region, one edge per chain of mesh edges between two faces, a
//!    vertex wherever more than two faces meet (a seam vertex on closed
//!    edges such as the circle where a cylinder meets a plane).
//! 2. **Surfaces** ([`surface`]): each group gets an exact analytic surface
//!    (plane, cylinder, cone, sphere) from a robust least-squares refit, or a
//!    bicubic B-spline patch extended past the group boundary.
//! 3. **Relations** ([`regularize`]): near-parallel / perpendicular
//!    directions and near-coaxial axes are snapped to exact relations.
//! 4. **Geometry of edges and vertices**: vertices are solved as the common
//!    point of their faces' surfaces next to the mesh corner, edges are the
//!    analytic intersection curves (line, circle, ellipse) where a closed
//!    form exists, otherwise cubic B-splines through the exact intersection
//!    ([`curve`]).
//! 5. **Validation**: every edge used twice with opposite senses, closed
//!    loops, Euler–Poincaré, and the geometric gaps are reported.
//!
//! Anything that cannot be represented is refused with a message naming the
//! groups involved, rather than producing a broken STEP file.

mod curve;
mod numeric;
mod regularize;
mod surface;
#[cfg(test)]
mod tests;
mod topo;

pub use curve::Curve;
pub use regularize::Relations;
pub use surface::Surface;

use crate::geom::segment::{FaceGroup, GroupKind};
use crate::mesh::Mesh;
use glam::{DVec3, Vec3};
use rayon::prelude::*;
use std::collections::{BTreeMap, HashSet};
use surface::{Freeze, MAX_FIT_POINTS, fit_cone, fit_plane, fit_spline, refit, rms_of, subsample};
use topo::join_ids;

/// Settings of the reconstruction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolidParams {
    /// Relations within this angle are snapped to exactly parallel /
    /// perpendicular / coaxial (0 = keep the independent fits).
    pub snap_angle_deg: f64,
    /// Fit tolerance as a fraction of the bounding box diagonal (the same
    /// tolerance the face group classification uses).
    pub fit_tol: f64,
    /// Represent freeform groups by B-spline faces (otherwise refuse them).
    pub allow_freeform: bool,
}

impl Default for SolidParams {
    fn default() -> Self {
        SolidParams {
            snap_angle_deg: 2.0,
            fit_tol: 0.0015,
            allow_freeform: true,
        }
    }
}

pub struct SolidEdge {
    pub curve: Curve,
    pub start: usize,
    pub end: usize,
    /// `faces[0]` uses the edge in its own direction, `faces[1]` reversed.
    pub faces: [usize; 2],
    /// Closed edge (a full turn from the vertex back to itself).
    pub closed: bool,
    /// Sampled curve for the viewport preview.
    pub preview: Vec<Vec3>,
}

pub struct SolidLoop {
    /// (edge index, used in the edge's own direction).
    pub edges: Vec<(usize, bool)>,
    /// Written as the face's outer bound.
    pub outer: bool,
}

pub struct SolidFace {
    /// Face group id the face was built from.
    pub group: i32,
    pub surface: Surface,
    /// The surface's natural normal points out of the solid.
    pub same_sense: bool,
    pub loops: Vec<SolidLoop>,
    /// Deviation of the group's mesh vertices from the surface (mm).
    pub max_dev: f64,
    pub rms_dev: f64,
}

/// Summary of a reconstruction for the panel.
#[derive(Clone, Debug, Default)]
pub struct SolidReport {
    pub genus: i64,
    /// Plane, cylinder, cone, sphere, B-spline faces.
    pub surface_counts: [usize; 5],
    /// Line, circle, ellipse, B-spline edges.
    pub curve_counts: [usize; 4],
    /// Deviation of the mesh vertices from their face surfaces (mm).
    pub mesh_max_dev: f64,
    pub mesh_rms_dev: f64,
    /// Group with the largest mesh deviation.
    pub worst_group: i32,
    /// Largest distance of a vertex from the surfaces of its faces.
    pub vertex_gap: f64,
    /// Largest distance of an edge's end vertices from its curve.
    pub edge_gap: f64,
    /// Largest distance of the mesh boundary between two groups from the
    /// edge curve.
    pub edge_mesh_dev: f64,
    pub relations: Relations,
    /// Warnings (absorbed ungrouped triangles, flipped orientation, gaps).
    pub notes: Vec<String>,
}

pub struct Solid {
    pub vertices: Vec<DVec3>,
    pub edges: Vec<SolidEdge>,
    pub faces: Vec<SolidFace>,
    pub report: SolidReport,
}

/// Samples of a group used by the fits.
struct GroupData {
    group: i32,
    /// Unique vertex positions.
    pts: Vec<DVec3>,
    /// (centroid, outward unit normal, area) per triangle.
    normals: Vec<(DVec3, DVec3, f64)>,
    tris: Vec<[DVec3; 3]>,
    area: f64,
}

/// Points spread over the group's triangles (the vertices plus a
/// barycentric grid per triangle), about `target` in total, so coarse
/// meshes still give the freeform fit a dense sample.
fn surface_samples(gd: &GroupData, target: usize) -> Vec<DVec3> {
    if gd.pts.len() >= target || gd.tris.is_empty() {
        return subsample(&gd.pts, target);
    }
    let per_tri = (target - gd.pts.len()) as f64 / gd.tris.len() as f64;
    let k = ((2.0 * per_tri).sqrt().ceil() as usize).clamp(3, 16);
    let mut out = gd.pts.clone();
    for [a, b, c] in &gd.tris {
        for i in 1..k {
            for j in 1..(k - i) {
                let (u, v) = (i as f64 / k as f64, j as f64 / k as f64);
                out.push(*a + (*b - *a) * u + (*c - *a) * v);
            }
        }
    }
    subsample(&out, target)
}

/// Reconstructs a solid from a closed mesh and its face groups.
pub fn reconstruct_solid(
    mesh: &Mesh,
    groups: &[FaceGroup],
    group_ids: &[i32],
    params: &SolidParams,
) -> Result<Solid, String> {
    let topo = topo::build(mesh, group_ids)?;
    let pos = |v: u32| Vec3::from(mesh.positions[v as usize]).as_dvec3();
    let diag = mesh.bbox().diagonal().max(1e-9) as f64;
    let tol = (params.fit_tol.max(1e-6) * diag).max(1e-9);
    // Largest gap accepted between surfaces that should meet.
    let gap_limit = (20.0 * tol).max(diag * 1e-4);

    // --- Per-group samples ---------------------------------------------------
    let mut by_group: BTreeMap<i32, Vec<usize>> = BTreeMap::new();
    for (f, face) in topo.faces.iter().enumerate() {
        by_group.entry(face.group).or_default().push(f);
    }
    let data: Vec<GroupData> = by_group
        .iter()
        .map(|(&g, faces)| {
            let mut seen: HashSet<u32> = HashSet::new();
            let mut pts = Vec::new();
            let mut normals = Vec::new();
            let mut tris = Vec::new();
            let mut area = 0.0;
            for &f in faces {
                for &t in &topo.faces[f].tris {
                    let tri = topo.tris[t as usize];
                    let [a, b, c] = tri.map(pos);
                    tris.push([a, b, c]);
                    let n = (b - a).cross(c - a);
                    let ar = n.length() * 0.5;
                    area += ar;
                    if ar > 0.0 {
                        normals.push(((a + b + c) / 3.0, n.normalize(), ar));
                    }
                    for v in tri {
                        if seen.insert(v) {
                            pts.push(pos(v));
                        }
                    }
                }
            }
            GroupData {
                group: g,
                pts,
                normals,
                tris,
                area,
            }
        })
        .collect();

    // --- Surface fits (independent per group) -----------------------------
    let fits: Vec<Result<(Surface, Vec<DVec3>), String>> = data
        .par_iter()
        .map(|gd| {
            let sample = subsample(&gd.pts, MAX_FIT_POINTS);
            let seed = groups.iter().find(|g| g.id == gd.group);
            let s = fit_group(gd, seed, &sample, tol, params.allow_freeform)?;
            Ok((s, sample))
        })
        .collect();
    let mut failures = Vec::new();
    let mut slots = Vec::with_capacity(fits.len());
    for (gd, fit) in data.iter().zip(fits) {
        match fit {
            Ok((surface, pts)) => slots.push(regularize::FitSlot {
                surface,
                pts,
                weight: gd.area,
            }),
            Err(e) => failures.push(format!("Group {}: {e}", gd.group + 1)),
        }
    }
    if !failures.is_empty() {
        return Err(format!(
            "These face groups cannot be represented by a surface:\n{}",
            failures.join("\n")
        ));
    }

    // --- Relationship inference ------------------------------------------------
    let relations = regularize::regularize(
        &mut slots,
        params.snap_angle_deg.max(0.0).to_radians(),
        3.0 * tol,
    );
    let surface_of_group: BTreeMap<i32, usize> = data
        .iter()
        .enumerate()
        .map(|(i, gd)| (gd.group, i))
        .collect();
    let surf_of_face = |f: usize| &slots[surface_of_group[&topo.faces[f].group]].surface;

    // --- Vertices ------------------------------------------------------------
    let solved: Vec<(DVec3, f64)> = topo
        .vertices
        .par_iter()
        .map(|v| {
            let mut groups_here: Vec<i32> = v.faces.iter().map(|&f| topo.faces[f].group).collect();
            groups_here.sort_unstable();
            groups_here.dedup();
            let surfs: Vec<&Surface> = groups_here
                .iter()
                .map(|g| &slots[surface_of_group[g]].surface)
                .collect();
            numeric::project_onto(&surfs, pos(v.mesh_vertex), diag * 0.05)
        })
        .collect();
    let mut errors = Vec::new();
    let mut vertex_gap = 0.0f64;
    for (v, &(p, res)) in topo.vertices.iter().zip(&solved) {
        let moved = (p - pos(v.mesh_vertex)).length();
        vertex_gap = vertex_gap.max(res);
        if res > gap_limit || moved > (0.1 * diag).max(gap_limit) {
            let mut ids: Vec<i32> = v.faces.iter().map(|&f| topo.faces[f].group + 1).collect();
            ids.sort_unstable();
            ids.dedup();
            errors.push(format!(
                "The surfaces of groups {} do not meet near their common corner (gap {:.3} mm, \
                 {:.3} mm from the mesh).",
                join_ids(&ids),
                res,
                moved
            ));
        }
    }
    let vertices: Vec<DVec3> = solved.iter().map(|s| s.0).collect();

    // --- Edges ----------------------------------------------------------------
    let curves: Vec<Result<Curve, String>> = topo
        .edges
        .par_iter()
        .map(|e| {
            let chain: Vec<DVec3> = e.chain.iter().map(|&v| pos(v)).collect();
            curve::intersect(&curve::EdgeInput {
                sa: surf_of_face(e.faces[0]),
                sb: surf_of_face(e.faces[1]),
                chain: &chain,
                a: vertices[e.start],
                b: vertices[e.end],
                closed: e.chain[0] == *e.chain.last().unwrap(),
                scale: diag,
                dev_limit: gap_limit,
            })
        })
        .collect();
    let mut edges = Vec::with_capacity(topo.edges.len());
    let (mut edge_gap, mut edge_mesh_dev) = (0.0f64, 0.0f64);
    for (e, c) in topo.edges.iter().zip(curves) {
        let (ga, gb) = (topo.faces[e.faces[0]].group, topo.faces[e.faces[1]].group);
        let curve = match c {
            Ok(c) => c,
            Err(msg) => {
                let (lo, hi) = (ga.min(gb) + 1, ga.max(gb) + 1);
                errors.push(format!(
                    "Groups {lo} and {hi} are neighbours but their surfaces do not meet along \
                     the shared boundary ({msg}); e.g. parallel planes where a step face is \
                     missing. Adjust the segmentation."
                ));
                continue;
            }
        };
        let closed = e.chain[0] == *e.chain.last().unwrap();
        let (a, b) = (vertices[e.start], vertices[e.end]);
        edge_gap = edge_gap.max(curve.distance(a)).max(curve.distance(b));
        let stride = e.chain.len().div_ceil(64).max(1);
        for &v in e.chain.iter().step_by(stride) {
            edge_mesh_dev = edge_mesh_dev.max(curve.distance(pos(v)));
        }
        let n = if matches!(curve, Curve::Line { .. }) {
            1
        } else {
            64
        };
        let preview = curve
            .sample(a, b, closed, n)
            .into_iter()
            .map(|p| p.as_vec3())
            .collect();
        edges.push(SolidEdge {
            curve,
            start: e.start,
            end: e.end,
            faces: e.faces,
            closed,
            preview,
        });
    }
    if !errors.is_empty() {
        errors.sort();
        errors.dedup();
        return Err(errors.join("\n"));
    }

    // --- Faces -------------------------------------------------------------
    let faces: Vec<SolidFace> = topo
        .faces
        .par_iter()
        .map(|face| {
            let surface = surf_of_face_owned(&slots, &surface_of_group, face.group);
            let mut sense = 0.0f64;
            let mut mean_n = DVec3::ZERO;
            let mut total = 0.0;
            let mut seen: HashSet<u32> = HashSet::new();
            let (mut max_dev, mut ss, mut n) = (0.0f64, 0.0f64, 0usize);
            for &t in &face.tris {
                let tri = topo.tris[t as usize];
                let [a, b, c] = tri.map(pos);
                let nn = (b - a).cross(c - a);
                let ar = nn.length() * 0.5;
                if ar > 0.0 {
                    let centroid = (a + b + c) / 3.0;
                    sense += ar * nn.normalize().dot(surface.gradient(centroid));
                    mean_n += nn * 0.5;
                    total += ar;
                }
                for v in tri {
                    if seen.insert(v) {
                        let d = surface.signed_distance(pos(v)).abs();
                        max_dev = max_dev.max(d);
                        ss += d * d;
                        n += 1;
                    }
                }
            }
            // Outer bound: the loop enclosing the largest area around the
            // mean outward normal, for faces that are not closed around an
            // axis (cylinders between two circles have none).
            let mut loops: Vec<SolidLoop> = face
                .loops
                .iter()
                .map(|l| SolidLoop {
                    edges: l.clone(),
                    outer: false,
                })
                .collect();
            if loops.len() == 1 {
                loops[0].outer = true;
            } else if total > 0.0 && mean_n.length() / total > 0.5 {
                let nref = mean_n.normalize();
                let areas: Vec<f64> = face
                    .loops
                    .iter()
                    .map(|l| loop_area(l, &topo.edges, &pos, nref))
                    .collect();
                if let Some((k, _)) = areas
                    .iter()
                    .enumerate()
                    .filter(|(_, a)| **a > 0.0)
                    .max_by(|a, b| a.1.total_cmp(b.1))
                {
                    loops[k].outer = true;
                }
            }
            SolidFace {
                group: face.group,
                surface,
                same_sense: sense >= 0.0,
                loops,
                max_dev,
                rms_dev: (ss / n.max(1) as f64).sqrt(),
            }
        })
        .collect();

    // --- Report ---------------------------------------------------------------
    let mut report = SolidReport {
        genus: (2 - topo.mesh_euler) / 2,
        vertex_gap,
        edge_gap,
        edge_mesh_dev,
        relations,
        worst_group: -1,
        ..Default::default()
    };
    let (mut ss, mut n) = (0.0f64, 0usize);
    for (face, tf) in faces.iter().zip(&topo.faces) {
        report.surface_counts[face.surface.kind_index()] += 1;
        if face.max_dev >= report.mesh_max_dev {
            report.mesh_max_dev = face.max_dev;
            report.worst_group = face.group;
        }
        let nv = tf.tris.len().max(1);
        ss += face.rms_dev * face.rms_dev * nv as f64;
        n += nv;
    }
    report.mesh_rms_dev = (ss / n.max(1) as f64).sqrt();
    for e in &edges {
        report.curve_counts[e.curve.kind_index()] += 1;
    }
    if topo.absorbed > 0 {
        report.notes.push(format!(
            "{} ungrouped triangles were given to neighbouring groups.",
            topo.absorbed
        ));
    }
    if topo.flipped {
        report
            .notes
            .push("The mesh was inside-out; its orientation was flipped.".to_string());
    }
    if vertex_gap > tol {
        report.notes.push(format!(
            "Some corners join more than three surfaces that do not meet exactly (gap up to \
             {vertex_gap:.3} mm)."
        ));
    }
    if edge_mesh_dev > 3.0 * tol {
        report.notes.push(format!(
            "Some edges lie up to {edge_mesh_dev:.3} mm from the mesh boundary between their \
             groups (fillets or rounded scan edges)."
        ));
    }

    let solid = Solid {
        vertices,
        edges,
        faces,
        report,
    };
    validate(&solid)?;
    Ok(solid)
}

fn surf_of_face_owned(
    slots: &[regularize::FitSlot],
    surface_of_group: &BTreeMap<i32, usize>,
    group: i32,
) -> Surface {
    slots[surface_of_group[&group]].surface.clone()
}

/// Signed area of a loop (its mesh boundary chains) around `n`.
fn loop_area(
    lp: &[(usize, bool)],
    edges: &[topo::TopoEdge],
    pos: &impl Fn(u32) -> DVec3,
    n: DVec3,
) -> f64 {
    let mut pts: Vec<DVec3> = Vec::new();
    for &(e, fwd) in lp {
        let ch = &edges[e].chain;
        if fwd {
            pts.extend(ch[..ch.len() - 1].iter().map(|&v| pos(v)));
        } else {
            pts.extend(ch[1..].iter().rev().map(|&v| pos(v)));
        }
    }
    let Some(&o) = pts.first() else { return 0.0 };
    let mut a = DVec3::ZERO;
    for i in 0..pts.len() {
        a += (pts[i] - o).cross(pts[(i + 1) % pts.len()] - o);
    }
    0.5 * a.dot(n)
}

/// Fits the surface of one group: the segmentation's kind decides the
/// primitive; freeform groups try a cone before a B-spline patch.
fn fit_group(
    gd: &GroupData,
    seed: Option<&FaceGroup>,
    sample: &[DVec3],
    tol: f64,
    allow_freeform: bool,
) -> Result<Surface, String> {
    let outward = gd
        .normals
        .iter()
        .fold(DVec3::ZERO, |acc, t| acc + t.1 * t.2);
    let kind = seed.map_or(GroupKind::Freeform, |g| g.kind);
    let best_of = |a: Surface, b: Option<Surface>| match b {
        Some(b) if rms_of(&b, sample) < rms_of(&a, sample) => b,
        _ => a,
    };
    match kind {
        GroupKind::Plane => {
            let s = fit_plane(sample).ok_or("degenerate plane")?;
            let Surface::Plane { origin, normal } = s else {
                unreachable!()
            };
            let normal = if normal.dot(outward) < 0.0 {
                -normal
            } else {
                normal
            };
            Ok(Surface::Plane { origin, normal })
        }
        GroupKind::Cylinder | GroupKind::Sphere => {
            let g = seed.unwrap();
            let seed_s = if kind == GroupKind::Cylinder {
                Surface::Cylinder {
                    origin: g.point.as_dvec3(),
                    axis: g.normal.as_dvec3().normalize_or(DVec3::Z),
                    radius: g.radius as f64,
                }
            } else {
                Surface::Sphere {
                    center: g.point.as_dvec3(),
                    radius: g.radius as f64,
                }
            };
            let fitted = refit(&seed_s, sample, Freeze::default());
            Ok(best_of(seed_s, Some(fitted)))
        }
        GroupKind::Freeform => {
            if let Some(cone) = fit_cone(sample, &gd.normals)
                && rms_of(&cone, sample) <= tol
            {
                return Ok(cone);
            }
            if !allow_freeform {
                return Err("freeform surface (B-spline faces are disabled)".to_string());
            }
            let s = fit_spline(&surface_samples(gd, 20_000))
                .map_err(|e| format!("freeform surface fit failed: {e}"))?;
            let rms = rms_of(&s, sample);
            if rms > 5.0 * tol {
                return Err(format!(
                    "freeform surface deviates {rms:.3} mm RMS from the mesh (it probably \
                     folds over its fit plane); split the group with a smaller crease angle"
                ));
            }
            Ok(s)
        }
    }
}

/// Checks the B-Rep: loops close up, every edge is used exactly twice with
/// opposite senses by its two faces, and V - E + 2F - L = 2 - 2 genus.
pub fn validate(solid: &Solid) -> Result<(), String> {
    let ne = solid.edges.len();
    let mut uses = vec![[0usize; 2]; ne];
    for (fi, face) in solid.faces.iter().enumerate() {
        if face.loops.is_empty() {
            return Err(format!("Face of group {} has no boundary.", face.group + 1));
        }
        for lp in &face.loops {
            if lp.edges.is_empty() {
                return Err(format!(
                    "Face of group {} has an empty loop.",
                    face.group + 1
                ));
            }
            let ends = |&(e, fwd): &(usize, bool)| {
                let ed = &solid.edges[e];
                if fwd {
                    (ed.start, ed.end)
                } else {
                    (ed.end, ed.start)
                }
            };
            for k in 0..lp.edges.len() {
                let (e, fwd) = lp.edges[k];
                if e >= ne {
                    return Err("Loop references a missing edge.".to_string());
                }
                if solid.edges[e].faces[if fwd { 0 } else { 1 }] != fi {
                    return Err(format!(
                        "Edge {e} is used by the wrong face (group {}).",
                        face.group + 1
                    ));
                }
                uses[e][if fwd { 0 } else { 1 }] += 1;
                let next = &lp.edges[(k + 1) % lp.edges.len()];
                if ends(&lp.edges[k]).1 != ends(next).0 {
                    return Err(format!("A loop of group {} is not closed.", face.group + 1));
                }
            }
        }
    }
    if let Some(e) = uses.iter().position(|u| *u != [1, 1]) {
        return Err(format!(
            "Edge {e} is used {} times forwards and {} times backwards (must be once each).",
            uses[e][0], uses[e][1]
        ));
    }
    let (v, e, f) = (
        solid.vertices.len() as i64,
        ne as i64,
        solid.faces.len() as i64,
    );
    let l: i64 = solid.faces.iter().map(|f| f.loops.len() as i64).sum();
    if v - e + 2 * f - l != 2 - 2 * solid.report.genus {
        return Err(format!(
            "Euler–Poincaré check failed: V - E + 2F - L = {} but the mesh has genus {}.",
            v - e + 2 * f - l,
            solid.report.genus
        ));
    }
    if solid.vertices.iter().any(|p| !p.is_finite()) {
        return Err("A vertex position is not finite.".to_string());
    }
    Ok(())
}
