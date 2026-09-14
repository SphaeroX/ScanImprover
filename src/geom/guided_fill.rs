//! Hole filling guided by the face groups (experimental).
//!
//! A standard fill only sees the hole rim, so a hole across a sharp edge
//! comes out as a rounded bridge. Here the primitives of the face groups
//! around the hole (plane, cylinder, sphere) are extended into the gap and
//! the edges where they meet are rebuilt:
//!
//! 1. **Rim labels.** Every rim edge gets the group of the triangle owning
//!    it. A group without a primitive, or whose primitive misses the rim by
//!    more than the fit tolerance even after a local refit, counts as
//!    freeform. Visible fitted circles act as cylinders ([`Guide`]) and take
//!    over the rim edges they match better than the groups, e.g. a round
//!    profile of a scan whose groups are ragged. Runs of one or two edges
//!    (label noise along a crease) are merged into their neighbours.
//! 2. **Creases.** Where the label changes along the rim an edge enters the
//!    hole. The two transitions between the same pair of groups are joined by
//!    the intersection curve of their primitives (plane/plane line,
//!    plane/cylinder ellipse or circle, ...), traced by Gauss-Newton
//!    projection of the chord between the two rim points. Three groups
//!    meeting once each (a hole at a corner) are joined at the common point of
//!    their primitives. Against a freeform group the crease is the chord
//!    projected onto the primitive side.
//! 3. **Regions.** Walking the rim and the creases gives one polygon per
//!    surface. Each is triangulated in the parameter plane of its primitive
//!    (plane coordinates, unrolled cylinder, sphere cap), refined to the rim
//!    edge length with centroid splits, Delaunay flips and relaxation, and
//!    lifted back onto the primitive: every new vertex lies exactly on its
//!    surface and the rebuilt edge runs along mesh edges, so no triangle
//!    straddles it. Freeform regions are faired like the Liepa fill.
//! 4. **Checks.** The patch must close the hole manifold and must not fold
//!    over the rim. Anything the reconstruction cannot handle falls back to
//!    the standard fill ([`generate_hole_patch`]).

use crate::geom::fitting::{FittedCircle, fit_circle_2d, fit_plane, plane_basis, solve_3x3};
use crate::geom::hole_detect::HoleLoop;
use crate::geom::hole_fill::{
    HoleFillConfig, MeshPatch, apply_patch, ear_clip_polygon, generate_hole_patch,
    improve_triangulation,
};
use crate::geom::segment::{FaceGroup, GroupKind};
use crate::mesh::Mesh;
use glam::{Vec2, Vec3};
use std::collections::{BTreeMap, HashMap, HashSet};

/// Rim label of edges without a usable primitive.
const FREE: i32 = -1;
/// Rim runs shorter than this many edges are merged into their neighbours.
const MIN_RUN_EDGES: usize = 2;
/// Two primitives meeting at less than this angle (sine) are tangent, e.g.
/// a fillet running into a plane: the crease is placed halfway between them.
const TANGENT_SIN: f32 = 0.2;
/// Upper bound for the number of new vertices of one patch.
const MAX_NEW_VERTICES: usize = 250_000;
/// Largest freeform region polygon triangulated with the O(m^3) minimal
/// area fallback.
const MAX_DP_VERTICES: usize = 150;
const REFINE_PASSES: usize = 24;
const RELAX_ITERATIONS: usize = 12;

/// Parameters of the guided fill.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct GuidedFillConfig {
    /// Patch resolution relative to the hole rim (1.0 = rim edge length).
    pub density: f32,
    /// Fit tolerance as a fraction of the mesh bbox diagonal (the face group
    /// fit tolerance): a group whose primitive misses the hole rim by more is
    /// treated as freeform.
    pub fit_tol: f32,
    /// Standard fill used when the hole cannot be rebuilt from primitives.
    /// Its smoothing iterations also fair the freeform parts of a patch.
    pub fallback: HoleFillConfig,
}

impl Default for GuidedFillConfig {
    fn default() -> Self {
        Self {
            density: 1.0,
            fit_tol: 0.0015,
            fallback: HoleFillConfig::default(),
        }
    }
}

/// A primitive surface extended into the hole.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Surface {
    Plane {
        point: Vec3,
        normal: Vec3,
    },
    Cylinder {
        point: Vec3,
        axis: Vec3,
        radius: f32,
    },
    Sphere {
        center: Vec3,
        radius: f32,
    },
}

impl Surface {
    /// The fitted primitive of a face group (`None` for freeform groups).
    pub fn from_group(g: &FaceGroup) -> Option<Surface> {
        match g.kind {
            GroupKind::Plane if g.normal.length_squared() > 0.5 => Some(Surface::Plane {
                point: g.point,
                normal: g.normal.normalize(),
            }),
            GroupKind::Cylinder if g.normal.length_squared() > 0.5 && g.radius > 0.0 => {
                Some(Surface::Cylinder {
                    point: g.point,
                    axis: g.normal.normalize(),
                    radius: g.radius,
                })
            }
            GroupKind::Sphere if g.radius > 0.0 => Some(Surface::Sphere {
                center: g.point,
                radius: g.radius,
            }),
            _ => None,
        }
    }

    pub fn kind(&self) -> GroupKind {
        match self {
            Surface::Plane { .. } => GroupKind::Plane,
            Surface::Cylinder { .. } => GroupKind::Cylinder,
            Surface::Sphere { .. } => GroupKind::Sphere,
        }
    }

    /// Signed distance of `p` and its gradient (the unit surface normal).
    fn eval(&self, p: Vec3) -> (f32, Vec3) {
        match *self {
            Surface::Plane { point, normal } => ((p - point).dot(normal), normal),
            Surface::Cylinder {
                point,
                axis,
                radius,
            } => {
                let q = p - point;
                let radial = q - axis * q.dot(axis);
                let d = radial.length();
                if d < 1e-12 {
                    (-radius, plane_basis(axis).0)
                } else {
                    (d - radius, radial / d)
                }
            }
            Surface::Sphere { center, radius } => {
                let q = p - center;
                let d = q.length();
                if d < 1e-12 {
                    (-radius, Vec3::Z)
                } else {
                    (d - radius, q / d)
                }
            }
        }
    }

    pub fn distance(&self, p: Vec3) -> f32 {
        self.eval(p).0.abs()
    }

    /// Closest point on the surface.
    pub fn project(&self, p: Vec3) -> Vec3 {
        let (d, n) = self.eval(p);
        p - n * d
    }
}

/// A reference surface fitted by the user, used for the rim edges it matches
/// better than the face groups: the cylinder of a fitted circle.
#[derive(Clone, Debug, PartialEq)]
pub struct Guide {
    pub name: String,
    pub surface: Surface,
    /// Distance from the surface still counted as on it (noise of the fit).
    pub tol: f32,
}

impl Guide {
    /// The cylinder through a fitted circle, its axis along the circle normal.
    pub fn from_circle(circle: &FittedCircle) -> Option<Guide> {
        let f = &circle.fit;
        (f.radius > 0.0 && f.normal.length_squared() > 0.5).then(|| Guide {
            name: circle.name.clone(),
            surface: Surface::Cylinder {
                point: f.center,
                axis: f.normal.normalize(),
                radius: f.radius,
            },
            tol: 3.0 * f.radial_rms,
        })
    }
}

/// Summary of a guided fill for the UI and the tests.
#[derive(Clone, Debug, Default)]
pub struct GuidedFillReport {
    /// True when the patch was rebuilt from the primitives, false when the
    /// standard fill was used (see `note`).
    pub guided: bool,
    /// Surface kind of every region of the patch (`Freeform` = faired).
    #[cfg_attr(not(test), allow(dead_code))]
    pub surfaces: Vec<GroupKind>,
    /// Number of rebuilt edges between two surfaces.
    #[cfg_attr(not(test), allow(dead_code))]
    pub creases: usize,
    /// Number of rebuilt corners (three surfaces meeting in the hole).
    #[cfg_attr(not(test), allow(dead_code))]
    pub corners: usize,
    /// Largest distance of a new vertex from the primitive(s) it belongs to.
    pub max_deviation: f32,
    /// Short explanation: what was rebuilt, or why the fill fell back.
    pub note: String,
}

/// A guided hole patch.
#[derive(Clone, Debug)]
pub struct GuidedFillResult {
    pub patch: MeshPatch,
    /// Face group id of every patch triangle (-1 = none), in the order of
    /// `patch.new_indices`.
    pub tri_groups: Vec<i32>,
    /// Rebuilt edges as polylines from rim to rim (or to the corner).
    pub creases: Vec<Vec<Vec3>>,
    pub report: GuidedFillReport,
}

/// Fills `hole` guided by the face groups (`group_ids` = per-triangle group
/// id map of `mesh`) and the fitted reference `guides`. Falls back to the
/// standard fill when the hole cannot be rebuilt from primitives; errors
/// only when that fails too.
pub fn guided_hole_patch(
    mesh: &Mesh,
    hole: &HoleLoop,
    groups: &[FaceGroup],
    group_ids: &[i32],
    guides: &[Guide],
    config: GuidedFillConfig,
) -> Result<GuidedFillResult, String> {
    if hole.vertices.len() < 3 {
        return Err("Hole loop must have at least 3 vertices".to_string());
    }
    if hole
        .vertices
        .iter()
        .any(|&v| v as usize >= mesh.vertex_count())
    {
        return Err("The hole does not belong to this mesh.".to_string());
    }
    match reconstruct(mesh, hole, groups, group_ids, guides, config) {
        Ok(result) => Ok(result),
        Err(reason) => {
            let patch = generate_hole_patch(mesh, hole, config.fallback)?;
            let tris = patch.new_indices.len() / 3;
            Ok(GuidedFillResult {
                patch,
                tri_groups: vec![-1; tris],
                creases: Vec::new(),
                report: GuidedFillReport {
                    guided: false,
                    note: reason,
                    ..Default::default()
                },
            })
        }
    }
}

/// Result of filling several holes in a row.
pub struct GuidedBatch {
    pub mesh: Mesh,
    /// Face groups and group map extended by the patch triangles, `None`
    /// when the input groups did not belong to the mesh.
    pub groups: Option<(Vec<FaceGroup>, Vec<i32>)>,
    /// Holes rebuilt from primitives.
    pub guided: usize,
    /// Holes filled with the standard fill instead.
    pub fallback: usize,
    /// Holes that could not be filled at all.
    pub failed: usize,
}

/// Fills all `holes` one after the other. Patches only append vertices and
/// triangles, so the hole loops and the group map stay valid in between.
/// `progress(i, n)` is called before hole `i`.
pub fn fill_holes_guided(
    mesh: &Mesh,
    holes: &[HoleLoop],
    groups: &[FaceGroup],
    group_ids: &[i32],
    guides: &[Guide],
    config: GuidedFillConfig,
    mut progress: impl FnMut(usize, usize),
) -> GuidedBatch {
    let nt = mesh.triangle_count();
    let groups_valid = !groups.is_empty() && group_ids.len() == nt;
    let mut working = mesh.clone();
    let mut groups = groups.to_vec();
    let mut ids = if groups_valid {
        group_ids.to_vec()
    } else {
        vec![-1; nt]
    };
    let (mut guided, mut fallback, mut failed) = (0, 0, 0);
    for (i, hole) in holes.iter().enumerate() {
        progress(i, holes.len());
        match guided_hole_patch(&working, hole, &groups, &ids, guides, config) {
            Ok(result) => {
                let first = working.triangle_count();
                apply_patch(&mut working, &result.patch);
                record_patch_groups(&mut groups, &mut ids, first, &result.tri_groups);
                if result.report.guided {
                    guided += 1;
                } else {
                    fallback += 1;
                }
            }
            Err(_) => failed += 1,
        }
    }
    GuidedBatch {
        mesh: working,
        groups: groups_valid.then_some((groups, ids)),
        guided,
        fallback,
        failed,
    }
}

/// Adds the triangles of an applied patch (starting at triangle index
/// `first_tri`) to the group map and to the triangle lists of their groups.
pub fn record_patch_groups(
    groups: &mut [FaceGroup],
    ids: &mut Vec<i32>,
    first_tri: usize,
    tri_groups: &[i32],
) {
    for (k, &gid) in tri_groups.iter().enumerate() {
        ids.push(gid);
        if gid >= 0
            && let Some(g) = groups.iter_mut().find(|g| g.id == gid)
        {
            g.tris.push((first_tri + k) as u32);
        }
    }
}

// -------------------------------------------------------------------------------------------------
// Reconstruction
// -------------------------------------------------------------------------------------------------

/// End of a crease: a rim vertex (loop index) or the corner point.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Node {
    Rim(usize),
    Corner,
}

/// A rebuilt edge between the surfaces of two rim labels.
struct Crease {
    ends: [Node; 2],
    labels: (i32, i32),
    /// New vertex ids of the interior points, ordered from `ends[0]`.
    pts: Vec<usize>,
}

impl Crease {
    fn has(&self, label: i32) -> bool {
        self.labels.0 == label || self.labels.1 == label
    }

    /// Interior points walking from `from`, and the node at the other end.
    fn walk_from(&self, from: Node) -> (Vec<usize>, Node) {
        if self.ends[0] == from {
            (self.pts.clone(), self.ends[1])
        } else {
            (self.pts.iter().rev().copied().collect(), self.ends[0])
        }
    }
}

/// One surface region of the patch: a polygon of patch vertices (loop index
/// `< n` = rim vertex, `n + id` = new vertex `id`) in rim order.
struct Region {
    label: i32,
    poly: Vec<usize>,
}

fn reconstruct(
    mesh: &Mesh,
    hole: &HoleLoop,
    groups: &[FaceGroup],
    group_ids: &[i32],
    guides: &[Guide],
    config: GuidedFillConfig,
) -> Result<GuidedFillResult, String> {
    let n = hole.vertices.len();
    let nt = mesh.triangle_count();
    let groups_ok = !groups.is_empty() && group_ids.len() == nt;
    if !groups_ok && guides.is_empty() {
        return Err("no face groups for this mesh (run the detection first)".to_string());
    }
    let rim: Vec<Vec3> = hole
        .vertices
        .iter()
        .map(|&v| Vec3::from(mesh.positions[v as usize]))
        .collect();
    let h = hole.perimeter / n as f32 / config.density.clamp(0.25, 4.0);
    if h.is_nan() || h <= 1e-12 {
        return Err("degenerate hole".to_string());
    }
    let diag = mesh.bbox().diagonal().max(1e-9);

    // --- 1. Rim labels --------------------------------------------------------
    let (owners, chords) = rim_topology(mesh, hole);
    let raw: Vec<i32> = owners
        .iter()
        .map(|o| o.filter(|_| groups_ok).map_or(-1, |t| group_ids[t]))
        .collect();
    let mut gids: Vec<i32> = raw.iter().copied().filter(|&g| g >= 0).collect();
    gids.sort_unstable();
    gids.dedup();
    let mut surfs: Vec<(Surface, i32)> = Vec::new();
    let mut slot_of: HashMap<i32, i32> = HashMap::new();
    let reach = (hole.bbox.diagonal() * 0.5).max(3.0 * h);
    for gid in gids {
        let Some(g) = groups.iter().find(|g| g.id == gid) else {
            continue;
        };
        let tol = (3.0 * g.rms).max(config.fit_tol * diag).max(1e-6 * diag);
        let pts = rim_points_of(&raw, &rim, gid);
        if let Some(s) = rim_surface(mesh, g, &pts, hole, reach, tol) {
            slot_of.insert(gid, surfs.len() as i32);
            surfs.push((s, gid));
        }
    }
    let mut lab: Vec<i32> = raw
        .iter()
        .map(|g| slot_of.get(g).copied().unwrap_or(FREE))
        .collect();
    // Fitted circles (as cylinders) take over the rim edges they match better
    // than the face groups: along a round profile of a scan the groups are
    // often freeform or ragged.
    let edge_dist = |s: &Surface, i: usize| s.distance(rim[i]).max(s.distance(rim[(i + 1) % n]));
    let mut guide_slots: Vec<(i32, &str)> = Vec::new();
    for guide in guides {
        let tol = guide.tol.max(config.fit_tol * diag).max(1e-6 * diag);
        let slot = surfs.len() as i32;
        let mut replaced: HashMap<i32, usize> = HashMap::new();
        for i in 0..n {
            let d = edge_dist(&guide.surface, i);
            if d <= tol && (lab[i] == FREE || edge_dist(&surfs[lab[i] as usize].0, i) > d) {
                lab[i] = slot;
                *replaced.entry(raw[i]).or_default() += 1;
            }
        }
        if replaced.is_empty() {
            continue;
        }
        // Patch triangles join the face group the guide replaced most.
        let gid = replaced
            .iter()
            .filter(|(g, _)| **g >= 0)
            .max_by_key(|(g, c)| (**c, -**g))
            .map_or(-1, |(g, _)| *g);
        surfs.push((guide.surface, gid));
        guide_slots.push((slot, guide.name.as_str()));
    }
    merge_short_runs(&mut lab, MIN_RUN_EDGES);
    let input = PatchInput {
        mesh,
        hole,
        rim: &rim,
        owners: &owners,
        chords: &chords,
        surfs: &surfs,
        h,
        fair_iterations: config.fallback.smooth_iterations,
    };
    // A failed reconstruction (e.g. an edge curve leaving the hole next to a
    // tiny run) is retried with the shortest run merged into its neighbours.
    let mut first_err: Option<String> = None;
    for _ in 0..4 {
        if lab.iter().all(|&l| l == FREE) {
            break;
        }
        match build_patch(&input, &lab) {
            Ok(mut result) => {
                let used: Vec<&str> = guide_slots
                    .iter()
                    .filter(|(slot, _)| lab.contains(slot))
                    .map(|(_, name)| *name)
                    .collect();
                if !used.is_empty() {
                    result
                        .report
                        .note
                        .push_str(&format!(", guided by {}", used.join(", ")));
                }
                return Ok(result);
            }
            Err(e) => {
                first_err.get_or_insert(e);
            }
        }
        if !merge_shortest_run(&mut lab) {
            break;
        }
    }
    Err(first_err
        .unwrap_or_else(|| "no plane, cylinder or sphere group borders the hole".to_string()))
}

/// Inputs of one reconstruction attempt.
#[derive(Clone, Copy)]
struct PatchInput<'a> {
    mesh: &'a Mesh,
    hole: &'a HoleLoop,
    /// Rim positions in loop order.
    rim: &'a [Vec3],
    owners: &'a [Option<usize>],
    chords: &'a RimChords,
    /// Primitive and face group id of every rim label.
    surfs: &'a [(Surface, i32)],
    h: f32,
    fair_iterations: usize,
}

/// Builds the patch for the rim labels `lab` (creases, regions,
/// triangulation, checks).
fn build_patch(input: &PatchInput, lab: &[i32]) -> Result<GuidedFillResult, String> {
    let PatchInput {
        mesh,
        hole,
        rim,
        owners,
        chords,
        surfs,
        h,
        fair_iterations,
    } = *input;
    let n = rim.len();
    let scale = hole.bbox.diagonal().max(h);
    let surf_of = |l: i32| (l >= 0).then(|| surfs[l as usize].0);
    let project = |p: Vec3, a: i32, b: i32| match (surf_of(a), surf_of(b)) {
        (Some(sa), Some(sb)) => project_to_curve(p, &sa, &sb, scale),
        (Some(s), None) | (None, Some(s)) => Some(s.project(p)),
        (None, None) => Some(p),
    };

    // --- 2. Creases -----------------------------------------------------------
    let prev = |i: usize| (i + n - 1) % n;
    let trans: Vec<usize> = (0..n).filter(|&i| lab[i] != lab[prev(i)]).collect();
    let mut by_pair: BTreeMap<(i32, i32), Vec<usize>> = BTreeMap::new();
    for &i in &trans {
        let (a, b) = (lab[prev(i)], lab[i]);
        by_pair.entry((a.min(b), a.max(b))).or_default().push(i);
    }
    let mut new_pos: Vec<Vec3> = Vec::new();
    let mut creases: Vec<Crease> = Vec::new();
    let mut crease_at: HashMap<usize, usize> = HashMap::new();
    let mut corner: Option<usize> = None;
    let unsupported = || "the face groups meet inside the hole in an unsupported way".to_string();
    if by_pair.values().all(|ts| ts.len() == 2) {
        for (&(a, b), ts) in &by_pair {
            let (t0, t1) = (ts[0], ts[1]);
            // The rim has to cross the edge once in each direction.
            if lab[t0] != lab[prev(t1)] || lab[t1] != lab[prev(t0)] {
                return Err(unsupported());
            }
            let pts = sample_crease(rim[t0], rim[t1], h, |p| project(p, a, b))?;
            crease_at.insert(t0, creases.len());
            crease_at.insert(t1, creases.len());
            creases.push(Crease {
                ends: [Node::Rim(t0), Node::Rim(t1)],
                labels: (a, b),
                pts: push_points(&mut new_pos, pts),
            });
        }
    } else if trans.len() == 3 && by_pair.len() == 3 {
        let start = (rim[trans[0]] + rim[trans[1]] + rim[trans[2]]) / 3.0;
        let mut labels: Vec<i32> = trans.iter().map(|&t| lab[t]).collect();
        labels.sort_unstable();
        labels.dedup();
        let corner_surfs: Vec<Surface> = labels.iter().filter_map(|&l| surf_of(l)).collect();
        let c = corner_point(start, &corner_surfs, scale)
            .ok_or("the surfaces around the corner have no common point")?;
        if c.distance(hole.centroid) > scale {
            return Err("the rebuilt corner lies outside the hole".to_string());
        }
        corner = Some(new_pos.len());
        new_pos.push(c);
        for &t in &trans {
            let (a, b) = (lab[prev(t)], lab[t]);
            let pts = sample_crease(rim[t], c, h, |p| project(p, a, b))?;
            crease_at.insert(t, creases.len());
            creases.push(Crease {
                ends: [Node::Rim(t), Node::Corner],
                labels: (a.min(b), a.max(b)),
                pts: push_points(&mut new_pos, pts),
            });
        }
    } else {
        return Err(unsupported());
    }

    // Deviation of the rebuilt edges from their surfaces.
    let mut max_dev = 0.0f32;
    for c in &creases {
        for &id in &c.pts {
            for l in [c.labels.0, c.labels.1] {
                if let Some(s) = surf_of(l) {
                    max_dev = max_dev.max(s.distance(new_pos[id]));
                }
            }
        }
    }
    if let Some(cid) = corner {
        for &t in &trans {
            if let Some(s) = surf_of(lab[t]) {
                max_dev = max_dev.max(s.distance(new_pos[cid]));
            }
        }
    }

    // --- 3. Regions -----------------------------------------------------------
    let regions = trace_regions(lab, &creases, &crease_at, corner)?;

    // --- 4. Triangulation per region --------------------------------------------
    let mut tris: Vec<[usize; 3]> = Vec::new();
    let mut tri_groups: Vec<i32> = Vec::new();
    let mut kinds: Vec<GroupKind> = Vec::new();
    for region in &regions {
        let poly: Vec<Vec3> = region
            .poly
            .iter()
            .map(|&k| if k < n { rim[k] } else { new_pos[k - n] })
            .collect();
        let surf = surf_of(region.label);
        let rm = triangulate_region(&poly, surf.as_ref(), h, fair_iterations)?;
        let m = poly.len();
        let base = new_pos.len();
        for &p in &rm.interior {
            if let Some(s) = &surf {
                max_dev = max_dev.max(s.distance(p));
            }
            new_pos.push(p);
        }
        if new_pos.len() > MAX_NEW_VERTICES {
            return Err("the hole is too large for this density".to_string());
        }
        let map = |j: usize| {
            if j < m {
                region.poly[j]
            } else {
                n + base + (j - m)
            }
        };
        let gid = if region.label >= 0 {
            surfs[region.label as usize].1
        } else {
            -1
        };
        for t in &rm.tris {
            // Region polygons run along the rim, whose edges belong to the
            // surrounding triangles: the patch uses the opposite winding.
            tris.push([map(t[0]), map(t[2]), map(t[1])]);
            tri_groups.push(gid);
        }
        kinds.push(surf.map_or(GroupKind::Freeform, |s| s.kind()));
    }

    // --- 5. Checks ------------------------------------------------------------
    let point = |k: usize| if k < n { rim[k] } else { new_pos[k - n] };
    check_patch(&tris, h, &point, mesh, owners, chords)?;

    let base_nv = mesh.positions.len() as u32;
    let to_mesh = |k: usize| {
        if k < n {
            hole.vertices[k]
        } else {
            base_nv + (k - n) as u32
        }
    };
    let mut new_indices = Vec::with_capacity(tris.len() * 3);
    let mut preview_indices = Vec::with_capacity(tris.len() * 3);
    for t in &tris {
        for &k in t {
            new_indices.push(to_mesh(k));
            // Preview positions are the rim followed by the new vertices, so
            // the local numbering is the preview numbering.
            preview_indices.push(k as u32);
        }
    }
    let new_positions: Vec<[f32; 3]> = new_pos.iter().map(|p| p.to_array()).collect();
    let mut preview_positions: Vec<[f32; 3]> = rim.iter().map(|p| p.to_array()).collect();
    preview_positions.extend_from_slice(&new_positions);

    let crease_lines: Vec<Vec<Vec3>> = creases
        .iter()
        .map(|c| {
            let end = |node: Node| match node {
                Node::Rim(i) => rim[i],
                Node::Corner => new_pos[corner.unwrap_or(0)],
            };
            let mut line = vec![end(c.ends[0])];
            line.extend(c.pts.iter().map(|&id| new_pos[id]));
            line.push(end(c.ends[1]));
            line
        })
        .collect();

    let names: Vec<&str> = kinds.iter().map(|k| k.label()).collect();
    let mut note = format!("Rebuilt from {}", names.join(" + "));
    if !creases.is_empty() {
        note.push_str(&format!(
            ", {} edge{}",
            creases.len(),
            if creases.len() == 1 { "" } else { "s" }
        ));
    }
    if corner.is_some() {
        note.push_str(", 1 corner");
    }
    Ok(GuidedFillResult {
        patch: MeshPatch {
            new_positions,
            new_indices,
            preview_positions,
            preview_indices,
        },
        tri_groups,
        creases: crease_lines,
        report: GuidedFillReport {
            guided: true,
            surfaces: kinds,
            creases: creases.len(),
            corners: usize::from(corner.is_some()),
            max_deviation: max_dev,
            note,
        },
    })
}

fn push_points(new_pos: &mut Vec<Vec3>, pts: Vec<Vec3>) -> Vec<usize> {
    let first = new_pos.len();
    new_pos.extend(pts);
    (first..new_pos.len()).collect()
}

/// Existing mesh edges between two rim vertices that are not rim edges, as
/// sorted loop index pairs. A patch must not add them a second time.
type RimChords = HashSet<(usize, usize)>;

/// The triangle owning every rim edge (`hole.vertices[i] -> [i + 1]`) and
/// the mesh edges joining non-adjacent rim vertices.
fn rim_topology(mesh: &Mesh, hole: &HoleLoop) -> (Vec<Option<usize>>, RimChords) {
    let n = hole.vertices.len();
    let mut edge_index: HashMap<(u32, u32), usize> = HashMap::with_capacity(n);
    let mut loop_index: HashMap<u32, usize> = HashMap::with_capacity(n);
    let mut on_rim = vec![false; mesh.vertex_count()];
    for (i, &v) in hole.vertices.iter().enumerate() {
        edge_index.insert((v, hole.vertices[(i + 1) % n]), i);
        loop_index.entry(v).or_insert(i);
        on_rim[v as usize] = true;
    }
    let mut owners = vec![None; n];
    let mut chords = RimChords::new();
    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        for e in 0..3 {
            let (a, b) = (tri[e], tri[(e + 1) % 3]);
            if !on_rim[a as usize] || !on_rim[b as usize] {
                continue;
            }
            if let Some(&i) = edge_index.get(&(a, b)) {
                owners[i] = Some(t);
            } else if !edge_index.contains_key(&(b, a)) {
                let (ia, ib) = (loop_index[&a], loop_index[&b]);
                chords.insert((ia.min(ib), ia.max(ib)));
            }
        }
    }
    (owners, chords)
}

/// Rim vertices inside the runs of group `gid` (both rim edges in the
/// group), or every vertex of its edges when the runs are very short.
fn rim_points_of(raw: &[i32], rim: &[Vec3], gid: i32) -> Vec<Vec3> {
    let n = raw.len();
    let inner: Vec<Vec3> = (0..n)
        .filter(|&i| raw[i] == gid && raw[(i + n - 1) % n] == gid)
        .map(|i| rim[i])
        .collect();
    if inner.len() >= 3 {
        return inner;
    }
    (0..n)
        .filter(|&i| raw[i] == gid || raw[(i + n - 1) % n] == gid)
        .map(|i| rim[i])
        .collect()
}

/// The primitive of group `g` if it matches the rim within `tol`, using the
/// global fit first and a fit to the group faces around the hole second.
fn rim_surface(
    mesh: &Mesh,
    g: &FaceGroup,
    rim: &[Vec3],
    hole: &HoleLoop,
    reach: f32,
    tol: f32,
) -> Option<Surface> {
    let global = Surface::from_group(g)?;
    if rim_error(&global, rim) <= tol {
        return Some(global);
    }
    let local = local_refit(mesh, g, &global, hole, reach)?;
    (rim_error(&local, rim) <= tol).then_some(local)
}

/// 80th percentile of the rim distances: a few rim vertices on a fillet or
/// a spike do not disqualify a surface.
fn rim_error(s: &Surface, rim: &[Vec3]) -> f32 {
    if rim.is_empty() {
        return f32::MAX;
    }
    let mut d: Vec<f32> = rim.iter().map(|&p| s.distance(p)).collect();
    let k = ((d.len() - 1) as f32 * 0.8).round() as usize;
    *d.select_nth_unstable_by(k, f32::total_cmp).1
}

/// Refits the primitive of `g` to its faces near the hole. Only accepted
/// when it stays close to the global fit (orientation / radius), so a strip
/// of a few faces cannot tilt the surface.
fn local_refit(
    mesh: &Mesh,
    g: &FaceGroup,
    global: &Surface,
    hole: &HoleLoop,
    reach: f32,
) -> Option<Surface> {
    let lo = hole.bbox.min - Vec3::splat(reach);
    let hi = hole.bbox.max + Vec3::splat(reach);
    let mut pts: Vec<[f32; 3]> = Vec::new();
    for &t in &g.tris {
        let t = t as usize;
        if t >= mesh.triangle_count() {
            continue;
        }
        let tri = mesh.triangle(t);
        let c = (tri[0] + tri[1] + tri[2]) / 3.0;
        if c.cmpge(lo).all() && c.cmple(hi).all() {
            pts.extend(tri.iter().map(|p| p.to_array()));
        }
    }
    if pts.len() < 12 {
        return None;
    }
    match *global {
        Surface::Plane { normal, .. } => {
            let first = fit_plane(&pts)?;
            // Drop the worst fifth (fillet faces, noise) and refit.
            let dev: Vec<f32> = pts
                .iter()
                .map(|p| (Vec3::from(*p) - first.point).dot(first.normal).abs())
                .collect();
            let mut sorted = dev.clone();
            let k = ((sorted.len() - 1) as f32 * 0.8) as usize;
            let cut = *sorted.select_nth_unstable_by(k, f32::total_cmp).1;
            let inliers: Vec<[f32; 3]> = pts
                .iter()
                .zip(&dev)
                .filter(|&(_, &d)| d <= cut)
                .map(|(p, _)| *p)
                .collect();
            let fit = fit_plane(&inliers).unwrap_or(first);
            let n = if fit.normal.dot(normal) < 0.0 {
                -fit.normal
            } else {
                fit.normal
            };
            (n.dot(normal) > 15f32.to_radians().cos()).then_some(Surface::Plane {
                point: fit.point,
                normal: n,
            })
        }
        Surface::Cylinder {
            point,
            axis,
            radius,
        } => {
            let (u, v) = plane_basis(axis);
            let xy: Vec<(f64, f64)> = pts
                .iter()
                .map(|p| {
                    let d = Vec3::from(*p) - point;
                    (d.dot(u) as f64, d.dot(v) as f64)
                })
                .collect();
            let (cx, cy, r) = fit_circle_2d(&xy)?;
            let r = r as f32;
            ((r - radius).abs() <= radius * 0.15).then_some(Surface::Cylinder {
                point: point + u * cx as f32 + v * cy as f32,
                axis,
                radius: r,
            })
        }
        Surface::Sphere { .. } => None,
    }
}

/// Runs of equal labels along the (cyclic) rim as (first edge, length).
fn runs_of(lab: &[i32]) -> Vec<(usize, usize)> {
    let n = lab.len();
    let Some(first) = (0..n).find(|&i| lab[i] != lab[(i + n - 1) % n]) else {
        return vec![(0, n)];
    };
    let mut runs = Vec::new();
    let (mut start, mut len) = (first, 0);
    for k in 0..n {
        let i = (first + k) % n;
        if k > 0 && lab[i] != lab[(i + n - 1) % n] {
            runs.push((start, len));
            start = i;
            len = 0;
        }
        len += 1;
    }
    runs.push((start, len));
    runs
}

/// Relabels run `ri` of `runs` with the label of its neighbours (the longer
/// neighbour when those differ).
fn absorb_run(lab: &mut [i32], runs: &[(usize, usize)], ri: usize) {
    let (n, k) = (lab.len(), runs.len());
    let (start, len) = runs[ri];
    let (before, after) = (runs[(ri + k - 1) % k], runs[(ri + 1) % k]);
    let (lb, la) = (lab[before.0], lab[after.0]);
    let target = if lb == la || before.1 >= after.1 {
        lb
    } else {
        la
    };
    for j in 0..len {
        lab[(start + j) % n] = target;
    }
}

/// Relabels runs shorter than `min_edges` (shortest first) with the label of
/// their neighbours.
fn merge_short_runs(lab: &mut [i32], min_edges: usize) {
    for _ in 0..lab.len() {
        let runs = runs_of(lab);
        if runs.len() < 2 {
            return;
        }
        let Some(ri) = (0..runs.len())
            .filter(|&i| runs[i].1 < min_edges)
            .min_by_key(|&i| runs[i].1)
        else {
            return;
        };
        absorb_run(lab, &runs, ri);
    }
}

/// Merges the shortest run into its neighbours; false when only one run is
/// left.
fn merge_shortest_run(lab: &mut [i32]) -> bool {
    let runs = runs_of(lab);
    if runs.len() < 2 {
        return false;
    }
    let ri = (0..runs.len()).min_by_key(|&i| runs[i].1).unwrap_or(0);
    absorb_run(lab, &runs, ri);
    true
}

/// Moves `p` onto the intersection curve of two surfaces (Gauss-Newton on
/// the two signed distances, minimal step). Tangent surfaces meet halfway.
fn project_to_curve(p: Vec3, a: &Surface, b: &Surface, scale: f32) -> Option<Vec3> {
    let mut q = p;
    for _ in 0..40 {
        let (fa, na) = a.eval(q);
        let (fb, nb) = b.eval(q);
        let c = na.dot(nb);
        let det = 1.0 - c * c;
        if det < TANGENT_SIN * TANGENT_SIN {
            let (pa, pb) = (a.project(q), b.project(q));
            return (pa.distance(pb) <= 0.1 * scale).then_some((pa + pb) * 0.5);
        }
        let step = (na * (fa - c * fb) + nb * (fb - c * fa)) / det;
        q -= step;
        if step.length() < scale * 1e-7 {
            break;
        }
    }
    let err = a.distance(q).max(b.distance(q));
    (err < scale * 1e-4 && q.distance(p) < 2.0 * scale).then_some(q)
}

/// Common point of up to three surfaces near `start` (Newton on the signed
/// distances).
fn corner_point(start: Vec3, surfs: &[Surface], scale: f32) -> Option<Vec3> {
    match surfs {
        [s] => Some(s.project(start)),
        [a, b] => project_to_curve(start, a, b, scale),
        [a, b, c] => {
            let mut q = start;
            for _ in 0..40 {
                let (fs, ns): (Vec<f32>, Vec<Vec3>) = [a, b, c].iter().map(|s| s.eval(q)).unzip();
                let m = [0, 1, 2].map(|i| [ns[i].x as f64, ns[i].y as f64, ns[i].z as f64]);
                let r = [-fs[0] as f64, -fs[1] as f64, -fs[2] as f64];
                let d = solve_3x3(m, r)?;
                let step = Vec3::new(d[0] as f32, d[1] as f32, d[2] as f32);
                q += step;
                if step.length() < scale * 1e-7 {
                    break;
                }
            }
            let err = a.distance(q).max(b.distance(q)).max(c.distance(q));
            (err < scale * 1e-4).then_some(q)
        }
        _ => None,
    }
}

/// Points of the rebuilt edge strictly between `p` and `q`, spaced about `h`
/// along the curve: the chord is projected onto the curve densely, then
/// resampled by arc length.
fn sample_crease(
    p: Vec3,
    q: Vec3,
    h: f32,
    project: impl Fn(Vec3) -> Option<Vec3>,
) -> Result<Vec<Vec3>, String> {
    let fail = || "could not trace the edge between two face groups".to_string();
    let chord = q - p;
    let len = chord.length();
    let m = ((len / h * 4.0).ceil() as usize).clamp(8, 4096);
    let mut dense = Vec::with_capacity(m + 1);
    dense.push(p);
    for k in 1..m {
        dense.push(project(p + chord * (k as f32 / m as f32)).ok_or_else(fail)?);
    }
    dense.push(q);
    let mut cum = vec![0.0f32; dense.len()];
    for k in 1..dense.len() {
        cum[k] = cum[k - 1] + dense[k].distance(dense[k - 1]);
    }
    let total = cum[dense.len() - 1];
    if total > 3.0 * len.max(h) {
        return Err(fail());
    }
    let segs = ((total / h).round() as usize).max(2);
    let mut out = Vec::with_capacity(segs - 1);
    let mut j = 1;
    for s in 1..segs {
        let target = total * s as f32 / segs as f32;
        while j < dense.len() - 1 && cum[j] < target {
            j += 1;
        }
        let span = (cum[j] - cum[j - 1]).max(1e-20);
        let t = ((target - cum[j - 1]) / span).clamp(0.0, 1.0);
        out.push(project(dense[j - 1].lerp(dense[j], t)).ok_or_else(fail)?);
    }
    Ok(out)
}

/// Walks the rim and the creases into one polygon per surface region, all in
/// the rotational sense of the rim.
fn trace_regions(
    lab: &[i32],
    creases: &[Crease],
    crease_at: &HashMap<usize, usize>,
    corner: Option<usize>,
) -> Result<Vec<Region>, String> {
    let n = lab.len();
    let bad = || "the face groups meet inside the hole in an unsupported way".to_string();
    let mut visited = vec![false; n];
    let mut regions = Vec::new();
    for start in 0..n {
        if visited[start] {
            continue;
        }
        let label = lab[start];
        let mut poly = Vec::new();
        let mut v = start;
        loop {
            if visited[v] {
                return Err(bad());
            }
            visited[v] = true;
            poly.push(v);
            let w = (v + 1) % n;
            if w == start {
                break;
            }
            if lab[w] == label {
                v = w;
                continue;
            }
            // The rim leaves the region at `w`: follow the edge into the hole.
            poly.push(w);
            let &ci = crease_at.get(&w).ok_or_else(bad)?;
            let (pts, mut far) = creases[ci].walk_from(Node::Rim(w));
            poly.extend(pts.iter().map(|&id| n + id));
            if far == Node::Corner {
                poly.push(n + corner.ok_or_else(bad)?);
                let cj = (0..creases.len())
                    .find(|&cj| {
                        cj != ci
                            && creases[cj].ends.contains(&Node::Corner)
                            && creases[cj].has(label)
                    })
                    .ok_or_else(bad)?;
                let (pts, next) = creases[cj].walk_from(Node::Corner);
                poly.extend(pts.iter().map(|&id| n + id));
                far = next;
            }
            let Node::Rim(k) = far else {
                return Err(bad());
            };
            if lab[k] != label {
                return Err(bad());
            }
            if k == start {
                break;
            }
            v = k;
        }
        if poly.len() < 3 {
            return Err(bad());
        }
        regions.push(Region { label, poly });
    }
    Ok(regions)
}

/// Topology and shape checks of the assembled patch (local numbering: rim
/// loop indices, then `n + new id`).
fn check_patch(
    tris: &[[usize; 3]],
    h: f32,
    point: &impl Fn(usize) -> Vec3,
    mesh: &Mesh,
    owners: &[Option<usize>],
    chords: &RimChords,
) -> Result<(), String> {
    let n = owners.len();
    // Directed edge -> triangle. Each rim edge i -> i + 1 must be used once,
    // backwards; every other edge exactly twice, once in each direction.
    let mut edges: HashMap<(usize, usize), usize> = HashMap::with_capacity(tris.len() * 3);
    for (ti, t) in tris.iter().enumerate() {
        let [a, b, c] = t.map(point);
        if (b - a).cross(c - a).length() < 1e-9 * h * h {
            return Err("the rebuilt patch has degenerate triangles".to_string());
        }
        for e in 0..3 {
            if edges.insert((t[e], t[(e + 1) % 3]), ti).is_some() {
                return Err("the rebuilt patch is not manifold".to_string());
            }
        }
    }
    let is_rim_edge = |a: usize, b: usize| a < n && b < n && (a + 1) % n == b;
    for &(a, b) in edges.keys() {
        if is_rim_edge(a, b) || (!is_rim_edge(b, a) && !edges.contains_key(&(b, a))) {
            return Err("the rebuilt patch does not close the hole".to_string());
        }
        if a < n && b < n && chords.contains(&(a.min(b), a.max(b))) {
            return Err("the rebuilt patch would duplicate an edge of the mesh".to_string());
        }
    }
    // The patch must continue the surface at the rim, not fold back.
    let mut folded = 0usize;
    for (i, owner) in owners.iter().enumerate() {
        let j = (i + 1) % n;
        let Some(&ti) = edges.get(&(j, i)) else {
            return Err("the rebuilt patch does not close the hole".to_string());
        };
        if let Some(t) = owner {
            let [a, b, c] = tris[ti].map(point);
            let pn = (b - a).cross(c - a).normalize_or_zero();
            if pn.dot(mesh.face_normal(*t).normalize_or_zero()) < -0.2 {
                folded += 1;
            }
        }
    }
    if folded * 5 > n {
        return Err("the rebuilt patch folds over the hole rim".to_string());
    }
    Ok(())
}

// -------------------------------------------------------------------------------------------------
// Region triangulation in the surface chart
// -------------------------------------------------------------------------------------------------

/// 2D parameterization of a region: exact for planes, unrolled cylinder,
/// orthographic sphere cap; a best-fit plane for freeform regions.
enum Chart {
    Plane {
        o: Vec3,
        u: Vec3,
        v: Vec3,
    },
    Cylinder {
        o: Vec3,
        axis: Vec3,
        e1: Vec3,
        e2: Vec3,
        r: f32,
    },
    Sphere {
        c: Vec3,
        u: Vec3,
        v: Vec3,
        w: Vec3,
        r: f32,
    },
}

impl Chart {
    fn new(surf: Option<&Surface>, poly: &[Vec3]) -> Result<Chart, String> {
        let centroid = poly.iter().copied().sum::<Vec3>() / poly.len() as f32;
        Ok(match surf {
            Some(Surface::Plane { normal, .. }) => {
                let (u, v) = plane_basis(*normal);
                Chart::Plane {
                    o: surf.map_or(centroid, |s| s.project(centroid)),
                    u,
                    v,
                }
            }
            Some(&Surface::Cylinder {
                point,
                axis,
                radius,
            }) => {
                let q = centroid - point;
                let radial = q - axis * q.dot(axis);
                let e1 = if radial.length() > 1e-9 {
                    radial.normalize()
                } else {
                    plane_basis(axis).0
                };
                let chart = Chart::Cylinder {
                    o: point,
                    axis,
                    e1,
                    e2: axis.cross(e1),
                    r: radius,
                };
                // The unrolled chart must not wrap around the axis.
                if poly
                    .iter()
                    .any(|&p| chart.to_2d(p).x.abs() > 0.9 * std::f32::consts::PI * radius)
                {
                    return Err("the region wraps around the cylinder".to_string());
                }
                chart
            }
            Some(&Surface::Sphere { center, radius }) => {
                let w = (centroid - center).normalize_or_zero();
                if w == Vec3::ZERO
                    || poly
                        .iter()
                        .any(|&p| (p - center).normalize_or_zero().dot(w) < 0.25)
                {
                    return Err("the spherical region is too large".to_string());
                }
                let (u, v) = plane_basis(w);
                Chart::Sphere {
                    c: center,
                    u,
                    v,
                    w,
                    r: radius,
                }
            }
            None => {
                // Newell normal of the region polygon.
                let mut nrm = Vec3::ZERO;
                for (i, &a) in poly.iter().enumerate() {
                    let b = poly[(i + 1) % poly.len()];
                    nrm += (a - centroid).cross(b - centroid);
                }
                let nrm = nrm.normalize_or_zero();
                if nrm == Vec3::ZERO {
                    return Err("degenerate freeform region".to_string());
                }
                let (u, v) = plane_basis(nrm);
                Chart::Plane { o: centroid, u, v }
            }
        })
    }

    fn to_2d(&self, p: Vec3) -> Vec2 {
        match *self {
            Chart::Plane { o, u, v } => Vec2::new((p - o).dot(u), (p - o).dot(v)),
            Chart::Cylinder { o, axis, e1, e2, r } => {
                let q = p - o;
                Vec2::new(r * q.dot(e2).atan2(q.dot(e1)), q.dot(axis))
            }
            Chart::Sphere { c, u, v, r, .. } => {
                let q = (p - c).normalize_or_zero() * r;
                Vec2::new(q.dot(u), q.dot(v))
            }
        }
    }

    fn to_3d(&self, q: Vec2) -> Vec3 {
        match *self {
            Chart::Plane { o, u, v } => o + u * q.x + v * q.y,
            Chart::Cylinder { o, axis, e1, e2, r } => {
                let theta = q.x / r;
                o + axis * q.y + (e1 * theta.cos() + e2 * theta.sin()) * r
            }
            Chart::Sphere { c, u, v, w, r } => {
                let z = (r * r - q.length_squared()).max(0.0).sqrt();
                c + u * q.x + v * q.y + w * z
            }
        }
    }
}

/// Interior vertices (3D) and triangles of one region, numbered like the
/// region polygon (`0..m`) followed by the interior vertices. Triangles have
/// the orientation of the polygon.
struct RegionMesh {
    interior: Vec<Vec3>,
    tris: Vec<[usize; 3]>,
}

fn orient(a: Vec2, b: Vec2, c: Vec2) -> f32 {
    (b - a).perp_dot(c - a)
}

/// Positive when `d` lies inside the circumcircle of the counter-clockwise
/// triangle `a b c`.
fn in_circle(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> f64 {
    let f = |p: Vec2| ((p.x - d.x) as f64, (p.y - d.y) as f64);
    let (ax, ay) = f(a);
    let (bx, by) = f(b);
    let (cx, cy) = f(c);
    (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
        + (cx * cx + cy * cy) * (ax * by - bx * ay)
}

/// Triangulates a region in its surface chart. A freeform region whose
/// polygon folds in its best-fit plane is triangulated in 3D instead.
fn triangulate_region(
    poly: &[Vec3],
    surf: Option<&Surface>,
    h: f32,
    fair_iterations: usize,
) -> Result<RegionMesh, String> {
    match triangulate_in_chart(poly, surf, h, fair_iterations) {
        Err(e) if surf.is_none() => triangulate_free_3d(poly, h, fair_iterations).ok_or(e),
        result => result,
    }
}

/// Freeform region without a usable flat chart: minimal-area triangulation
/// of the 3D polygon (as the standard fill), refined with centroid splits
/// and edge flips and faired like the Liepa fill.
fn triangulate_free_3d(poly: &[Vec3], h: f32, fair_iterations: usize) -> Option<RegionMesh> {
    let m = poly.len();
    if !(3..=MAX_DP_VERTICES).contains(&m) {
        return None;
    }
    let mut tris = min_area_triangulation(poly);
    let mut pts = poly.to_vec();
    let target = 0.866 * h * h;
    for _ in 0..REFINE_PASSES {
        let mut split = false;
        let mut next = Vec::with_capacity(tris.len() * 2);
        for t in tris {
            let [a, b, c] = t.map(|i| pts[i]);
            if (b - a).cross(c - a).length() * 0.5 > target {
                let ci = pts.len();
                pts.push((a + b + c) / 3.0);
                next.extend_from_slice(&[[t[0], t[1], ci], [t[1], t[2], ci], [t[2], t[0], ci]]);
                split = true;
            } else {
                next.push(t);
            }
        }
        tris = next;
        improve_triangulation(&mut tris, &pts, m);
        if pts.len() - m > MAX_NEW_VERTICES {
            return None;
        }
        if !split {
            break;
        }
    }
    let mut interior = pts[m..].to_vec();
    fair(&mut interior, poly, &tris, fair_iterations);
    Some(RegionMesh { interior, tris })
}

/// Minimal-area triangulation of a closed 3D polygon (Barequet-Sharir
/// dynamic programming); the triangles follow the polygon orientation.
fn min_area_triangulation(p: &[Vec3]) -> Vec<[usize; 3]> {
    let m = p.len();
    let mut cost = vec![0.0f32; m * m];
    let mut split = vec![0usize; m * m];
    for len in 2..m {
        for i in 0..m - len {
            let j = i + len;
            let mut best = (f32::MAX, i + 1);
            for k in i + 1..j {
                let area = (p[k] - p[i]).cross(p[j] - p[i]).length() * 0.5;
                let c = cost[i * m + k] + cost[k * m + j] + area;
                if c < best.0 {
                    best = (c, k);
                }
            }
            cost[i * m + j] = best.0;
            split[i * m + j] = best.1;
        }
    }
    let mut tris = Vec::with_capacity(m - 2);
    let mut stack = vec![(0, m - 1)];
    while let Some((i, j)) = stack.pop() {
        if i + 1 >= j {
            continue;
        }
        let k = split[i * m + j];
        tris.push([i, k, j]);
        stack.push((i, k));
        stack.push((k, j));
    }
    tris
}

fn triangulate_in_chart(
    poly: &[Vec3],
    surf: Option<&Surface>,
    h: f32,
    fair_iterations: usize,
) -> Result<RegionMesh, String> {
    let m = poly.len();
    if m < 3 {
        return Err("degenerate region".to_string());
    }
    let chart = Chart::new(surf, poly)?;
    let mut pts: Vec<Vec2> = poly.iter().map(|&p| chart.to_2d(p)).collect();
    let area2: f32 = (0..m).map(|i| pts[i].perp_dot(pts[(i + 1) % m])).sum();
    if area2.abs() < 1e-6 * h * h {
        return Err("a region collapses in its surface chart".to_string());
    }
    let sign = area2.signum();
    let min_area = 1e-7 * h * h;
    let valid =
        |t: &[usize; 3], pts: &[Vec2]| orient(pts[t[0]], pts[t[1]], pts[t[2]]) * sign > min_area;
    let mut tris = ear_clip_polygon(&pts)?;
    if tris.len() != m - 2 || !tris.iter().all(|t| valid(t, &pts)) {
        return Err("a region polygon overlaps itself in its surface chart".to_string());
    }
    let is_rim = |a: usize, b: usize| a < m && b < m && ((a + 1) % m == b || (b + 1) % m == a);

    // Refinement: centroid splits of triangles larger than twice an
    // equilateral triangle of edge `h`, each pass followed by Delaunay flips.
    let target = 0.866 * h * h;
    for _ in 0..REFINE_PASSES {
        let mut split = false;
        let mut next = Vec::with_capacity(tris.len() * 2);
        for t in tris {
            let [a, b, c] = t.map(|i| pts[i]);
            if orient(a, b, c).abs() * 0.5 > target {
                let ci = pts.len();
                pts.push((a + b + c) / 3.0);
                next.extend_from_slice(&[[t[0], t[1], ci], [t[1], t[2], ci], [t[2], t[0], ci]]);
                split = true;
            } else {
                next.push(t);
            }
        }
        tris = next;
        delaunay_flips(&mut tris, &pts, sign, h, &is_rim);
        if pts.len() - m > MAX_NEW_VERTICES {
            return Err("the hole is too large for this density".to_string());
        }
        if !split {
            break;
        }
    }
    relax(&mut pts, &tris, m, &|pts: &[Vec2]| {
        tris.iter().all(|t| valid(t, pts))
    });
    delaunay_flips(&mut tris, &pts, sign, h, &is_rim);
    if !tris.iter().all(|t| valid(t, &pts)) {
        return Err("a region could not be triangulated".to_string());
    }

    let mut interior: Vec<Vec3> = pts[m..].iter().map(|&q| chart.to_3d(q)).collect();
    if surf.is_none() {
        fair(&mut interior, poly, &tris, fair_iterations);
    }
    Ok(RegionMesh { interior, tris })
}

/// Lawson flips towards the Delaunay triangulation; `fixed` edges (the
/// region polygon) are kept. Only flips of strictly convex quads are made,
/// so the orientation of every triangle is preserved.
fn delaunay_flips(
    tris: &mut [[usize; 3]],
    pts: &[Vec2],
    sign: f32,
    h: f32,
    fixed: &impl Fn(usize, usize) -> bool,
) {
    let eps_area = 1e-7 * h * h;
    let eps_circle = 1e-9 * (h as f64).powi(4);
    for _ in 0..64 {
        let mut edges: HashMap<(usize, usize), (usize, usize)> =
            HashMap::with_capacity(tris.len() * 3);
        for (ti, t) in tris.iter().enumerate() {
            for e in 0..3 {
                edges.insert((t[e], t[(e + 1) % 3]), (ti, e));
            }
        }
        let mut keys: Vec<(usize, usize)> = edges.keys().copied().filter(|(a, b)| a < b).collect();
        keys.sort_unstable();
        let mut touched = vec![false; tris.len()];
        let mut flipped = false;
        for (a, b) in keys {
            if fixed(a, b) {
                continue;
            }
            let (Some(&(t0, e0)), Some(&(t1, e1))) = (edges.get(&(a, b)), edges.get(&(b, a)))
            else {
                continue;
            };
            if touched[t0] || touched[t1] {
                continue;
            }
            let c = tris[t0][(e0 + 2) % 3];
            let d = tris[t1][(e1 + 2) % 3];
            if c == d || edges.contains_key(&(c, d)) || edges.contains_key(&(d, c)) {
                continue;
            }
            let (pa, pb, pc, pd) = (pts[a], pts[b], pts[c], pts[d]);
            if in_circle(pa, pb, pc, pd) * sign as f64 <= eps_circle
                || orient(pa, pd, pc) * sign <= eps_area
                || orient(pb, pc, pd) * sign <= eps_area
            {
                continue;
            }
            tris[t0] = [a, d, c];
            tris[t1] = [b, c, d];
            touched[t0] = true;
            touched[t1] = true;
            flipped = true;
        }
        if !flipped {
            break;
        }
    }
}

/// Vertex neighbours (sorted, unique) of every vertex of a triangle list.
fn neighbours(count: usize, tris: &[[usize; 3]]) -> Vec<Vec<usize>> {
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); count];
    for t in tris {
        for e in 0..3 {
            adj[t[e]].push(t[(e + 1) % 3]);
            adj[t[e]].push(t[(e + 2) % 3]);
        }
    }
    for list in &mut adj {
        list.sort_unstable();
        list.dedup();
    }
    adj
}

/// Damped umbrella relaxation of the interior chart points (the first `m`
/// stay fixed); stops before an iteration that would invalidate a triangle.
fn relax(pts: &mut [Vec2], tris: &[[usize; 3]], m: usize, valid: &impl Fn(&[Vec2]) -> bool) {
    let adj = neighbours(pts.len(), tris);
    for _ in 0..RELAX_ITERATIONS {
        let before = pts.to_vec();
        for v in m..pts.len() {
            if adj[v].is_empty() {
                continue;
            }
            let avg = adj[v].iter().map(|&u| before[u]).sum::<Vec2>() / adj[v].len() as f32;
            pts[v] = before[v] + (avg - before[v]) * 0.5;
        }
        if !valid(pts) {
            pts.copy_from_slice(&before);
            return;
        }
    }
}

/// Laplacian fairing of the interior vertices of a freeform region (the
/// region polygon stays fixed), as in the Liepa fill.
fn fair(interior: &mut [Vec3], poly: &[Vec3], tris: &[[usize; 3]], iterations: usize) {
    let m = poly.len();
    let mut all: Vec<Vec3> = poly.to_vec();
    all.extend_from_slice(interior);
    let adj = neighbours(all.len(), tris);
    for _ in 0..iterations.clamp(2, 60) {
        let before = all.clone();
        for v in m..all.len() {
            if adj[v].is_empty() {
                continue;
            }
            let avg = adj[v].iter().map(|&u| before[u]).sum::<Vec3>() / adj[v].len() as f32;
            all[v] = before[v] + (avg - before[v]) * 0.5;
        }
    }
    interior.copy_from_slice(&all[m..]);
}

#[cfg(test)]
pub(crate) mod fixtures {
    //! Synthetic meshes for the guided fill tests (integer lattice points, so
    //! shared edges weld exactly).
    use crate::mesh::Mesh;
    use glam::Vec3;

    /// Unit squares spanning `nu` x `nv` cells from `o` along the integer
    /// directions `u`, `v` (outward normal `u x v`).
    fn add_rect(
        corners: &mut Vec<[f32; 3]>,
        o: [i32; 3],
        u: [i32; 3],
        v: [i32; 3],
        nu: i32,
        nv: i32,
    ) {
        let p = |i: i32, j: i32| [0, 1, 2].map(|k| (o[k] + u[k] * i + v[k] * j) as f32);
        for i in 0..nu {
            for j in 0..nv {
                let (a, b, c, d) = (p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1));
                corners.extend_from_slice(&[a, b, c, a, c, d]);
            }
        }
    }

    /// Closed cube `[0, n]^3` made of unit squares.
    pub(crate) fn lattice_box(n: i32) -> Mesh {
        let mut c = Vec::new();
        let (x, y, z) = ([1, 0, 0], [0, 1, 0], [0, 0, 1]);
        add_rect(&mut c, [0, 0, n], x, y, n, n);
        add_rect(&mut c, [0, 0, 0], y, x, n, n);
        add_rect(&mut c, [n, 0, 0], y, z, n, n);
        add_rect(&mut c, [0, 0, 0], z, y, n, n);
        add_rect(&mut c, [0, n, 0], z, x, n, n);
        add_rect(&mut c, [0, 0, 0], x, z, n, n);
        Mesh::from_corners(&c)
    }

    /// Closed L-shaped block: the profile (0,0) (2n,0) (2n,n) (n,n) (n,2n)
    /// (0,2n) in xz, extruded along y over `[0, w]`. The edge along y at
    /// x = z = n is concave.
    pub(crate) fn lattice_l_block(n: i32, w: i32) -> Mesh {
        let mut c = Vec::new();
        let (x, y, z) = ([1, 0, 0], [0, 1, 0], [0, 0, 1]);
        add_rect(&mut c, [0, 0, 0], y, x, w, 2 * n); // bottom, -z
        add_rect(&mut c, [2 * n, 0, 0], y, z, w, n); // right, +x
        add_rect(&mut c, [n, 0, n], x, y, n, w); // step top, +z
        add_rect(&mut c, [n, 0, n], y, z, w, n); // inner wall, +x
        add_rect(&mut c, [0, 0, 2 * n], x, y, n, w); // top, +z
        add_rect(&mut c, [0, 0, 0], z, y, 2 * n, w); // left, -x
        for i in 0..2 * n {
            for k in 0..2 * n {
                if i < n || k < n {
                    add_rect(&mut c, [i, 0, k], x, z, 1, 1); // front cap, -y
                    add_rect(&mut c, [i, w, k], z, x, 1, 1); // back cap, +y
                }
            }
        }
        Mesh::from_corners(&c)
    }

    /// Cylinder boss (radius `r`, height `ht`, closed top) standing on an
    /// annular plate at z = 0 with outer radius `big_r`. The outer rim of the
    /// plate stays open.
    pub(crate) fn boss_on_plate(r: f32, big_r: f32, ht: f32, sectors: usize) -> Mesh {
        let step = std::f32::consts::TAU * r / sectors as f32;
        let rings = ((big_r - r) / step).ceil() as usize;
        let levels = (ht / step).ceil() as usize;
        let s = sectors;
        let dir = |j: usize| {
            let a = std::f32::consts::TAU * (j % s) as f32 / s as f32;
            (a.cos(), a.sin())
        };
        let mut pos: Vec<[f32; 3]> = Vec::new();
        // Plate rings (ring 0 = foot of the boss), then boss levels 1..=levels.
        for k in 0..=rings {
            let rr = r + (big_r - r) * k as f32 / rings as f32;
            for j in 0..s {
                let (cx, sy) = dir(j);
                pos.push([rr * cx, rr * sy, 0.0]);
            }
        }
        for l in 1..=levels {
            let zz = ht * l as f32 / levels as f32;
            for j in 0..s {
                let (cx, sy) = dir(j);
                pos.push([r * cx, r * sy, zz]);
            }
        }
        let center = pos.len() as u32;
        pos.push([0.0, 0.0, ht]);
        let plate = |k: usize, j: usize| (k * s + j % s) as u32;
        let wall = |l: usize, j: usize| {
            if l == 0 {
                plate(0, j)
            } else {
                ((rings + 1) * s + (l - 1) * s + j % s) as u32
            }
        };
        let mut idx = Vec::new();
        for k in 0..rings {
            for j in 0..s {
                let (a, b, c, d) = (
                    plate(k, j),
                    plate(k + 1, j),
                    plate(k + 1, j + 1),
                    plate(k, j + 1),
                );
                idx.extend_from_slice(&[a, b, c, a, c, d]);
            }
        }
        for l in 0..levels {
            for j in 0..s {
                let (a, b, c, d) = (
                    wall(l, j),
                    wall(l, j + 1),
                    wall(l + 1, j + 1),
                    wall(l + 1, j),
                );
                idx.extend_from_slice(&[a, b, c, a, c, d]);
            }
        }
        for j in 0..s {
            idx.extend_from_slice(&[center, wall(levels, j), wall(levels, j + 1)]);
        }
        Mesh::from_indexed(pos, idx)
    }

    /// Wavy height field over `[0, n]^2` (no primitive fits it).
    pub(crate) fn wavy_sheet(n: usize) -> Mesh {
        let mut pos = Vec::new();
        for j in 0..=n {
            for i in 0..=n {
                let (x, y) = (i as f32, j as f32);
                pos.push([x, y, 1.5 * (x * 0.45).sin() * (y * 0.35).cos()]);
            }
        }
        let mut idx = Vec::new();
        for j in 0..n {
            for i in 0..n {
                let a = (j * (n + 1) + i) as u32;
                let (b, c, d) = (a + 1, a + (n + 1) as u32, a + (n + 2) as u32);
                idx.extend_from_slice(&[a, b, d, a, d, c]);
            }
        }
        Mesh::from_indexed(pos, idx)
    }

    /// Flat plate (z = 0 for x <= 10) folding up along x = 10 into a wavy,
    /// freeform flank.
    pub(crate) fn bent_sheet() -> Mesh {
        let (nx, ny) = (20usize, 12usize);
        let mut pos = Vec::new();
        for j in 0..=ny {
            for i in 0..=nx {
                let (x, y) = (i as f32, j as f32);
                let d = (x - 10.0).max(0.0);
                pos.push([x, y, 1.2 * d + 0.05 * d * d * (y * 0.5).sin()]);
            }
        }
        let mut idx = Vec::new();
        for j in 0..ny {
            for i in 0..nx {
                let a = (j * (nx + 1) + i) as u32;
                let (b, c, d) = (a + 1, a + (nx + 1) as u32, a + (nx + 2) as u32);
                idx.extend_from_slice(&[a, b, d, a, d, c]);
            }
        }
        Mesh::from_indexed(pos, idx)
    }

    /// Removes the triangles whose centroid lies within `radius` of `center`
    /// (unused vertices are dropped).
    pub(crate) fn cut_hole(mesh: &Mesh, center: Vec3, radius: f32) -> Mesh {
        let mut remap = vec![u32::MAX; mesh.vertex_count()];
        let mut pos = Vec::new();
        let mut idx = Vec::new();
        for t in 0..mesh.triangle_count() {
            let [a, b, c] = mesh.triangle(t);
            if ((a + b + c) / 3.0).distance(center) <= radius {
                continue;
            }
            for &v in &mesh.indices[3 * t..3 * t + 3] {
                if remap[v as usize] == u32::MAX {
                    remap[v as usize] = pos.len() as u32;
                    pos.push(mesh.positions[v as usize]);
                }
                idx.push(remap[v as usize]);
            }
        }
        Mesh::from_indexed(pos, idx)
    }
}

#[cfg(test)]
mod tests {
    use super::fixtures::*;
    use super::*;
    use crate::geom::hole_detect::detect_holes;
    use crate::geom::repair::analyze_mesh;
    use crate::geom::segment::segment_faces;
    use crate::geom::topology::MeshTopology;

    /// Face groups with the app's default detection settings.
    fn segment(mesh: &Mesh) -> (Vec<FaceGroup>, Vec<i32>) {
        let topo = MeshTopology::build(mesh);
        segment_faces(mesh, &topo, 45.0, 2, 0.0015, 0.04)
    }

    fn hole_near(mesh: &Mesh, p: Vec3) -> HoleLoop {
        detect_holes(mesh)
            .into_iter()
            .min_by(|a, b| a.centroid.distance(p).total_cmp(&b.centroid.distance(p)))
            .expect("a hole")
    }

    /// Guided fill of the hole nearest to `p`: (result, filled mesh).
    fn guided_fill(mesh: &Mesh, p: Vec3) -> (GuidedFillResult, Mesh) {
        let (groups, ids) = segment(mesh);
        let hole = hole_near(mesh, p);
        let res = guided_hole_patch(mesh, &hole, &groups, &ids, &[], GuidedFillConfig::default())
            .expect("patch");
        let mut filled = mesh.clone();
        apply_patch(&mut filled, &res.patch);
        (res, filled)
    }

    fn plane(p: [f32; 3], n: Vec3) -> Surface {
        Surface::Plane {
            point: Vec3::from(p),
            normal: n,
        }
    }

    /// Largest distance of any patch vertex from the nearest of `surfs`.
    fn max_dev_to(patch: &MeshPatch, surfs: &[Surface]) -> f32 {
        patch
            .preview_positions
            .iter()
            .map(|p| {
                surfs
                    .iter()
                    .map(|s| s.distance(Vec3::from(*p)))
                    .fold(f32::MAX, f32::min)
            })
            .fold(0.0, f32::max)
    }

    /// Every new vertex lies on one of `surfs`, and every patch triangle lies
    /// entirely on one of them (no triangle straddles a rebuilt edge).
    fn assert_on_surfaces(res: &GuidedFillResult, surfs: &[Surface], tol: f32) -> f32 {
        let dev = max_dev_to(&res.patch, surfs);
        assert!(dev < tol, "patch deviates {dev} from the surfaces");
        let pos = |k: u32| Vec3::from(res.patch.preview_positions[k as usize]);
        for t in res.patch.preview_indices.chunks_exact(3) {
            assert!(
                surfs
                    .iter()
                    .any(|s| t.iter().all(|&k| s.distance(pos(k)) < tol)),
                "triangle straddles an edge: {:?}",
                t.iter().map(|&k| pos(k)).collect::<Vec<_>>()
            );
        }
        dev
    }

    /// Deviation of the standard (Liepa) fill from the same surfaces, for
    /// comparison in the test output.
    fn standard_dev(mesh: &Mesh, p: Vec3, surfs: &[Surface]) -> f32 {
        let hole = hole_near(mesh, p);
        let patch = generate_hole_patch(mesh, &hole, HoleFillConfig::default()).unwrap();
        max_dev_to(&patch, surfs)
    }

    fn new_points_on_all(res: &GuidedFillResult, surfs: &[Surface], tol: f32) -> Vec<Vec3> {
        res.patch
            .new_positions
            .iter()
            .map(|p| Vec3::from(*p))
            .filter(|&p| surfs.iter().all(|s| s.distance(p) < tol))
            .collect()
    }

    fn assert_closed_manifold(mesh: &Mesh) {
        let health = analyze_mesh(mesh);
        assert!(health.is_watertight, "not watertight");
        assert_eq!(health.non_manifold_edges, 0);
        assert_eq!(health.inconsistent_normals, 0);
    }

    #[test]
    fn hole_across_a_convex_edge_rebuilds_the_sharp_edge() {
        let at = Vec3::new(10.0, 5.0, 10.0);
        let mesh = cut_hole(&lattice_box(10), at, 2.6);
        let (res, filled) = guided_fill(&mesh, at);
        assert!(res.report.guided, "{}", res.report.note);
        assert_eq!(res.report.surfaces, vec![GroupKind::Plane; 2]);
        assert_eq!(res.report.creases, 1);
        let surfs = [
            plane([0.0, 0.0, 10.0], Vec3::Z),
            plane([10.0, 0.0, 0.0], Vec3::X),
        ];
        let dev = assert_on_surfaces(&res, &surfs, 1e-4);
        // The edge x = z = 10 is rebuilt across the whole hole.
        let edge = new_points_on_all(&res, &surfs, 1e-4);
        assert!(edge.len() >= 3, "only {} edge vertices", edge.len());
        let crease = &res.creases[0];
        let span = (crease[0].y - crease[crease.len() - 1].y).abs();
        assert!(span > 4.0, "edge spans only {span}");
        assert!(
            crease
                .iter()
                .all(|p| surfs.iter().all(|s| s.distance(*p) < 1e-4))
        );
        assert_closed_manifold(&filled);
        let std_dev = standard_dev(&mesh, at, &surfs);
        eprintln!("convex edge: guided max dev {dev:e}, standard fill {std_dev:.3}");
        assert!(std_dev > 0.1, "the standard fill rounds the edge off");
    }

    #[test]
    fn hole_across_a_concave_edge_rebuilds_the_sharp_edge() {
        let at = Vec3::new(6.0, 4.0, 6.0);
        let mesh = cut_hole(&lattice_l_block(6, 8), at, 2.2);
        let (res, filled) = guided_fill(&mesh, at);
        assert!(res.report.guided, "{}", res.report.note);
        assert_eq!(res.report.creases, 1);
        let surfs = [
            plane([0.0, 0.0, 6.0], Vec3::Z),
            plane([6.0, 0.0, 0.0], Vec3::X),
        ];
        let dev = assert_on_surfaces(&res, &surfs, 1e-4);
        assert!(new_points_on_all(&res, &surfs, 1e-4).len() >= 3);
        assert_closed_manifold(&filled);
        let std_dev = standard_dev(&mesh, at, &surfs);
        eprintln!("concave edge: guided max dev {dev:e}, standard fill {std_dev:.3}");
    }

    #[test]
    fn hole_at_a_corner_rebuilds_three_edges_and_the_corner() {
        let at = Vec3::splat(10.0);
        let mesh = cut_hole(&lattice_box(10), at, 3.2);
        let (res, filled) = guided_fill(&mesh, at);
        assert!(res.report.guided, "{}", res.report.note);
        assert_eq!(res.report.creases, 3);
        assert_eq!(res.report.corners, 1);
        let surfs = [
            plane([0.0, 0.0, 10.0], Vec3::Z),
            plane([10.0, 0.0, 0.0], Vec3::X),
            plane([0.0, 10.0, 0.0], Vec3::Y),
        ];
        assert_on_surfaces(&res, &surfs, 1e-4);
        let corner = new_points_on_all(&res, &surfs, 1e-4);
        assert_eq!(corner.len(), 1);
        assert!(corner[0].distance(at) < 1e-4);
        assert_closed_manifold(&filled);
    }

    #[test]
    fn hole_at_a_plane_to_cylinder_transition_follows_both() {
        let (r, at) = (4.0, Vec3::new(4.0, 0.0, 0.0));
        let mesh = cut_hole(&boss_on_plate(r, 12.0, 6.0, 64), at, 1.5);
        let (res, filled) = guided_fill(&mesh, at);
        assert!(res.report.guided, "{}", res.report.note);
        let mut kinds = res.report.surfaces.clone();
        kinds.sort_by_key(|k| *k as u8);
        assert_eq!(kinds, vec![GroupKind::Plane, GroupKind::Cylinder]);
        assert_eq!(res.report.creases, 1);
        assert!(
            res.report.max_deviation < 1e-4,
            "{}",
            res.report.max_deviation
        );
        let surfs = [
            plane([0.0, 0.0, 0.0], Vec3::Z),
            Surface::Cylinder {
                point: Vec3::ZERO,
                axis: Vec3::Z,
                radius: r,
            },
        ];
        let dev = assert_on_surfaces(&res, &surfs, 1e-3);
        // The foot circle of the boss is rebuilt as an arc.
        let arc = new_points_on_all(&res, &surfs, 1e-3);
        assert!(arc.len() >= 3, "only {} arc vertices", arc.len());
        let health = analyze_mesh(&filled);
        assert_eq!(health.non_manifold_edges, 0);
        assert_eq!(health.inconsistent_normals, 0);
        assert_eq!(detect_holes(&filled).len(), detect_holes(&mesh).len() - 1);
        let std_dev = standard_dev(&mesh, at, &surfs);
        eprintln!("plane/cylinder: guided max dev {dev:e}, standard fill {std_dev:.3}");
    }

    #[test]
    fn hole_inside_one_group_is_filled_on_its_primitive() {
        // Plane: a hole in the middle of the top face.
        let at = Vec3::new(5.0, 5.0, 10.0);
        let mesh = cut_hole(&lattice_box(10), at, 2.5);
        let (res, filled) = guided_fill(&mesh, at);
        assert!(res.report.guided, "{}", res.report.note);
        assert_eq!(res.report.surfaces, vec![GroupKind::Plane]);
        assert_eq!(res.report.creases, 0);
        assert!(!res.patch.new_positions.is_empty());
        assert_on_surfaces(&res, &[plane([0.0, 0.0, 10.0], Vec3::Z)], 1e-4);
        assert_closed_manifold(&filled);

        // Cylinder: a hole in the wall of the boss is filled on the cylinder.
        let at = Vec3::new(4.0, 0.0, 3.0);
        let mesh = cut_hole(&boss_on_plate(4.0, 12.0, 6.0, 64), at, 1.2);
        let (res, _) = guided_fill(&mesh, at);
        assert!(res.report.guided, "{}", res.report.note);
        assert_eq!(res.report.surfaces, vec![GroupKind::Cylinder]);
        let wall = Surface::Cylinder {
            point: Vec3::ZERO,
            axis: Vec3::Z,
            radius: 4.0,
        };
        assert_on_surfaces(&res, &[wall], 1e-3);
    }

    #[test]
    fn freeform_or_missing_groups_fall_back_to_the_standard_fill() {
        let at = Vec3::new(10.0, 10.0, 0.0);
        let mesh = cut_hole(&wavy_sheet(20), at, 3.0);
        let (groups, ids) = segment(&mesh);
        let hole = hole_near(&mesh, Vec3::new(10.0, 10.0, 1.0));
        let config = GuidedFillConfig::default();
        let standard = generate_hole_patch(&mesh, &hole, config.fallback).unwrap();
        let before = detect_holes(&mesh).len();

        let res = guided_hole_patch(&mesh, &hole, &groups, &ids, &[], config).unwrap();
        assert!(!res.report.guided);
        assert!(res.report.note.contains("no plane"), "{}", res.report.note);
        assert_eq!(res.patch.new_indices, standard.new_indices);
        assert_eq!(res.tri_groups.len(), standard.new_indices.len() / 3);
        let mut filled = mesh.clone();
        apply_patch(&mut filled, &res.patch);
        assert_eq!(detect_holes(&filled).len(), before - 1);

        // Without face groups the fill falls back as well.
        let res = guided_hole_patch(&mesh, &hole, &[], &[], &[], config).unwrap();
        assert!(!res.report.guided);
        assert!(
            res.report.note.contains("face groups"),
            "{}",
            res.report.note
        );
        assert_eq!(res.patch.new_indices, standard.new_indices);
    }

    #[test]
    fn fitted_circle_rebuilds_a_round_profile_the_face_groups_miss() {
        let (r, at) = (4.0, Vec3::new(4.0, 0.0, 0.0));
        let mesh = cut_hole(&boss_on_plate(r, 12.0, 6.0, 64), at, 1.5);
        let (mut groups, ids) = segment(&mesh);
        // A ragged scan: the boss wall is not recognized as a cylinder.
        for g in &mut groups {
            if g.kind == GroupKind::Cylinder {
                g.kind = GroupKind::Freeform;
            }
        }
        let hole = hole_near(&mesh, at);
        let config = GuidedFillConfig::default();
        let res = guided_hole_patch(&mesh, &hole, &groups, &ids, &[], config).unwrap();
        assert!(!res.report.surfaces.contains(&GroupKind::Cylinder));

        let wall = Surface::Cylinder {
            point: Vec3::new(0.0, 0.0, 2.0),
            axis: Vec3::Z,
            radius: r,
        };
        let guide = Guide {
            name: "Circle 1".to_string(),
            surface: wall,
            tol: 0.0,
        };
        let guides = std::slice::from_ref(&guide);
        let res = guided_hole_patch(&mesh, &hole, &groups, &ids, guides, config).unwrap();
        assert!(res.report.guided, "{}", res.report.note);
        let mut kinds = res.report.surfaces.clone();
        kinds.sort_by_key(|k| *k as u8);
        assert_eq!(kinds, vec![GroupKind::Plane, GroupKind::Cylinder]);
        assert_eq!(res.report.creases, 1);
        assert!(res.report.note.contains("Circle 1"), "{}", res.report.note);
        assert_on_surfaces(&res, &[plane([0.0, 0.0, 0.0], Vec3::Z), wall], 1e-3);
        let mut filled = mesh.clone();
        apply_patch(&mut filled, &res.patch);
        let health = analyze_mesh(&filled);
        assert_eq!(health.non_manifold_edges, 0);
        assert_eq!(health.inconsistent_normals, 0);
        assert_eq!(detect_holes(&filled).len(), detect_holes(&mesh).len() - 1);

        // Without any face groups a fitted circle alone rebuilds a wall hole.
        let at = Vec3::new(4.0, 0.0, 3.0);
        let mesh = cut_hole(&boss_on_plate(r, 12.0, 6.0, 64), at, 1.2);
        let hole = hole_near(&mesh, at);
        let res = guided_hole_patch(&mesh, &hole, &[], &[], guides, config).unwrap();
        assert!(res.report.guided, "{}", res.report.note);
        assert_eq!(res.report.surfaces, vec![GroupKind::Cylinder]);
        assert_on_surfaces(&res, &[wall], 1e-3);
    }

    #[test]
    fn batch_fill_keeps_the_group_map_valid() {
        let mut mesh = cut_hole(&lattice_box(10), Vec3::new(10.0, 5.0, 10.0), 2.6);
        mesh = cut_hole(&mesh, Vec3::new(5.0, 5.0, 0.0), 2.0);
        let (groups, ids) = segment(&mesh);
        let holes = detect_holes(&mesh);
        assert_eq!(holes.len(), 2);
        let mut calls = 0;
        let batch = fill_holes_guided(
            &mesh,
            &holes,
            &groups,
            &ids,
            &[],
            GuidedFillConfig::default(),
            |_, _| calls += 1,
        );
        assert_eq!(calls, 2);
        assert_eq!((batch.guided, batch.fallback, batch.failed), (2, 0, 0));
        assert_closed_manifold(&batch.mesh);
        let (groups, ids) = batch.groups.expect("groups carried over");
        assert_eq!(ids.len(), batch.mesh.triangle_count());
        // Patch triangles joined their groups.
        let first_new = mesh.triangle_count();
        assert!(ids[first_new..].iter().all(|&g| g >= 0));
        let total: usize = groups.iter().map(|g| g.tris.len()).sum();
        assert_eq!(total, ids.iter().filter(|&&g| g >= 0).count());
    }

    #[test]
    fn short_label_runs_merge_into_their_neighbours() {
        let mut lab = vec![0, 0, 0, 1, 0, 0, 2, 2, 2, 2, 3, 2];
        merge_short_runs(&mut lab, 2);
        assert_eq!(lab, vec![0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2]);
        let mut single = vec![5, 5, 6];
        merge_short_runs(&mut single, 2);
        assert_eq!(single, vec![5, 5, 5]);
    }

    #[test]
    fn hole_between_a_plane_and_a_freeform_keeps_the_plane_exact() {
        let at = Vec3::new(10.0, 6.0, 0.0);
        let mesh = cut_hole(&bent_sheet(), at, 2.3);
        let (res, filled) = guided_fill(&mesh, at);
        assert!(res.report.guided, "{}", res.report.note);
        let mut kinds = res.report.surfaces.clone();
        kinds.sort_by_key(|k| *k as u8);
        assert_eq!(kinds, vec![GroupKind::Plane, GroupKind::Freeform]);
        assert_eq!(res.report.creases, 1);
        // Triangles of the plate group lie exactly on z = 0, the freeform part
        // is faired.
        let pos = |k: u32| Vec3::from(res.patch.preview_positions[k as usize]);
        let (mut on_plate, mut free) = (0, 0);
        for (t, &gid) in res
            .patch
            .preview_indices
            .chunks_exact(3)
            .zip(&res.tri_groups)
        {
            if gid >= 0 {
                on_plate += 1;
                assert!(t.iter().all(|&k| pos(k).z.abs() < 1e-4));
            } else {
                free += 1;
            }
        }
        assert!(on_plate > 0 && free > 0, "{on_plate} / {free}");
        let health = analyze_mesh(&filled);
        assert_eq!(health.non_manifold_edges, 0);
        assert_eq!(health.inconsistent_normals, 0);
        assert_eq!(detect_holes(&filled).len(), detect_holes(&mesh).len() - 1);
    }

    #[test]
    fn freeform_regions_fall_back_to_a_3d_triangulation() {
        // A saddle-shaped polygon folds in every plane; the 3D path closes it.
        let poly: Vec<Vec3> = (0..12)
            .map(|i| {
                let a = std::f32::consts::TAU * i as f32 / 12.0;
                Vec3::new(a.cos() * 3.0, a.sin() * 3.0, 2.5 * (2.0 * a).cos())
            })
            .collect();
        let rm = triangulate_region(&poly, None, 0.8, 20).expect("triangulated");
        assert!(!rm.interior.is_empty());
        assert_eq!(rm.tris.len(), poly.len() + 2 * rm.interior.len() - 2);
    }

    /// Robustness check on a real CAD mesh: holes of 2.5 % of the size cut
    /// at 80 places of `examples/fandisk.obj` (run `examples/fetch.sh`,
    /// then `cargo test --release -- --ignored fandisk`). Every guided patch
    /// must close its hole manifold and consistently wound.
    #[test]
    #[ignore = "needs examples/fandisk.obj (examples/fetch.sh)"]
    fn fandisk_holes_rebuild_cleanly() {
        let path =
            std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/fandisk.obj"));
        let Ok(bytes) = std::fs::read(path) else {
            eprintln!("skipped: {} not found", path.display());
            return;
        };
        let mesh = crate::io::load_any(path, &bytes).unwrap();
        let diag = mesh.bbox().diagonal();
        let (mut guided, mut fallback) = (0, 0);
        let mut reasons: BTreeMap<String, usize> = BTreeMap::new();
        let mut max_dev = 0f32;
        for k in 0..80 {
            let at = Vec3::from(mesh.positions[(k * 7919) % mesh.vertex_count()]);
            let holey = cut_hole(&mesh, at, diag * 0.025);
            let before = detect_holes(&holey).len();
            let (res, filled) = guided_fill(&holey, at);
            if !res.report.guided {
                fallback += 1;
                *reasons.entry(res.report.note).or_default() += 1;
                continue;
            }
            guided += 1;
            max_dev = max_dev.max(res.report.max_deviation);
            let health = analyze_mesh(&filled);
            assert_eq!(
                health.non_manifold_edges, 0,
                "at {at:?}: {}",
                res.report.note
            );
            assert_eq!(
                health.inconsistent_normals, 0,
                "at {at:?}: {}",
                res.report.note
            );
            assert_eq!(detect_holes(&filled).len() + 1, before, "at {at:?}");
        }
        eprintln!(
            "fandisk: {guided} guided, {fallback} standard fill, max dev {max_dev:e} (diag {diag})"
        );
        for (reason, count) in reasons {
            eprintln!("  standard fill {count}x: {reason}");
        }
        assert!(guided >= 40, "only {guided} of 80 holes were rebuilt");
    }
}
