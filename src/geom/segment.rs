use crate::geom::fitting::{eigen_3x3, fit_circle_2d, fit_plane, plane_basis};
use crate::geom::topology::MeshTopology;
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Semantic classification of a face group.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GroupKind {
    Plane,
    Cylinder,
    Sphere,
    Freeform,
}

impl GroupKind {
    pub fn label(self) -> &'static str {
        match self {
            GroupKind::Plane => "Plane",
            GroupKind::Cylinder => "Cylinder",
            GroupKind::Sphere => "Sphere",
            GroupKind::Freeform => "Freeform",
        }
    }
}

/// Type colors for "by type" coloring mode.
/// Must stay in sync with `type_color` in src/render/mod.rs.
pub const KIND_COLORS: [[f32; 3]; 4] = [
    [0.30, 0.55, 0.98], // Plane
    [0.30, 0.82, 0.42], // Cylinder
    [0.98, 0.66, 0.28], // Sphere
    [0.78, 0.45, 0.62], // Freeform
];

/// Distinct hue per group id.
/// Must stay in sync with `group_color` in src/render/mod.rs.
pub fn group_hue_color(id: i32) -> [f32; 3] {
    let h = (id as f32 * 0.618_034 + 0.04).fract();
    let (s, v) = (0.62f32, 1.0f32);
    let c = v * s;
    let hp = h * 6.0;
    let x = c * (1.0 - (hp.rem_euclid(2.0) - 1.0).abs());
    let m = v - c;
    let i = (hp.floor() as i32) % 6;
    match i {
        0 => [c + m, x + m, m],
        1 => [x + m, c + m, m],
        2 => [m, c + m, x + m],
        3 => [m, x + m, c + m],
        4 => [x + m, m, c + m],
        _ => [c + m, m, x + m],
    }
}

/// A connected patch of triangles bounded by creases (region-growing result),
/// including a primitive classification of its surface.
#[derive(Clone, Debug)]
pub struct FaceGroup {
    pub id: i32,
    pub kind: GroupKind,
    pub tris: Vec<u32>,
    pub center: Vec3,
    pub area: f32,
    /// Plane normal or cylinder axis direction.
    pub normal: Vec3,
    /// Point on plane, point on axis, or sphere center.
    pub point: Vec3,
    pub radius: f32,
    /// Fit residual RMS in mm (plane / cylinder / sphere fit).
    pub rms: f32,
}

/// Ring caps for the neighbourhood searches, so very dense meshes stay
/// affordable (the crease threshold is scaled by the distance actually
/// reached when a cap truncates a neighbourhood).
const SMOOTH_MAX_RINGS: u32 = 4;
const CREASE_MAX_RINGS: u32 = 12;
/// Crease detection runs at the feature size, then half and a quarter of it
/// on the faces still unassigned, so narrow features get their own core.
const SCALE_LEVELS: u32 = 3;
/// The crease test looks at the outer part of the neighbourhood (from this
/// fraction of the reached distance outwards) and flags the face when more
/// than `CREASE_RIM_SHARE` of that rim area bends away by over half the
/// crease angle. A face in the middle of a fillet sees about two thirds of
/// the rim bent (both neighbouring surfaces), a flat face right next to a
/// fillet or a sharp edge at most half.
const RIM_START: f32 = 0.7;
const CREASE_RIM_SHARE: f32 = 0.45;

/// Segments the mesh into face groups.
///
/// Region growing over the triangle dual graph, robust to scan noise and to
/// fillets: normals are smoothed over a small neighbourhood, a face is a
/// *crease face* when the smoothed normal changes by more than `angle_deg`
/// across a neighbourhood of diameter `2 * feature_frac * bbox diagonal`
/// around it, evaluated at three scales (so a fillet whose normal
/// turns quickly is a boundary while a large cylinder is not), regions grow
/// through non-crease faces only, and the crease bands are then given back
/// to the region whose anchor normal they match. Faces still left over
/// (sharp-edged strips narrower than the feature size, fillets) are grouped
/// with a plain dihedral-angle flood fill. On coarse meshes, where the
/// feature size is smaller than one triangle, this reduces to the classic
/// crease-angle region growing.
///
/// Regions smaller than `min_tris` triangles are discarded (left ungrouped).
/// `fit_tol` is the classification tolerance as a fraction of the bbox
/// diagonal.
///
/// Returns the groups (sorted by size, largest first) and a per-triangle group
/// id map (-1 = ungrouped).
pub fn segment_faces(
    mesh: &Mesh,
    topo: &MeshTopology,
    angle_deg: f32,
    min_tris: usize,
    fit_tol: f32,
    feature_frac: f32,
) -> (Vec<FaceGroup>, Vec<i32>) {
    let nt = topo.triangle_count();
    let mut group_ids = vec![-1i32; nt];
    if nt == 0 {
        return (Vec::new(), group_ids);
    }
    let angle = angle_deg.to_radians();
    let cos_thresh = angle.cos();
    let diag = mesh.bbox().diagonal().max(1e-9);
    let feature = feature_frac.max(0.0) * diag;

    let centers: Vec<Vec3> = (0..nt).map(|t| topo.centroids[t]).collect();
    let areas: Vec<f32> = (0..nt)
        .map(|t| {
            let [a, b, c] = mesh.triangle(t);
            (b - a).cross(c - a).length() * 0.5
        })
        .collect();

    // --- Smoothed normals (area weighted over a quarter feature size, at
    // least the edge neighbours; never across a crease) ----------------------
    let smooth_radius = feature * 0.25;
    // Only faces within half the crease angle are averaged, so a ridge just
    // below the crease angle keeps its full dihedral after smoothing.
    let smooth_cos = (angle * 0.5).cos();
    let smoothed: Vec<Vec3> = (0..nt)
        .into_par_iter()
        .map_init(
            || Scratch::new(nt),
            |scratch, t| {
                let own = topo.face_normals[t];
                scratch.neighbourhood(
                    topo,
                    &centers,
                    t as u32,
                    smooth_radius,
                    SMOOTH_MAX_RINGS,
                    true,
                    |f| topo.face_normals[f as usize].dot(own) >= smooth_cos,
                );
                let mut acc = Vec3::ZERO;
                for &(f, _) in &scratch.found {
                    acc += topo.face_normals[f as usize] * areas[f as usize];
                }
                let n = acc.normalize_or_zero();
                if n.length_squared() > 0.5 { n } else { own }
            },
        )
        .collect();

    let mut region_of = vec![-1i32; nt];
    let mut regions: Vec<Vec<u32>> = Vec::new();
    // Absorption anchor per face: the smoothed normal and centre of the core
    // face a chain started from (inherited along the chain).
    let mut anchor: Vec<(Vec3, Vec3)> = smoothed
        .iter()
        .zip(centers.iter())
        .map(|(n, c)| (*n, *c))
        .collect();
    // A band face must also lie close to the anchor's tangent plane, so an
    // offset surface (a step, a raised rim) is not absorbed just because its
    // normal matches.
    let absorb_dist = feature * 0.15;
    let fits_anchor = |f: usize, a: &(Vec3, Vec3)| {
        smoothed[f].dot(a.0) >= (angle * 0.35).cos()
            && (centers[f] - a.1).dot(a.0).abs() <= absorb_dist
    };

    // --- Multi-scale core growing --------------------------------------------
    // Faces near a fillet are crease faces at the full feature size even when
    // they lie on a flat surface (the neighbourhood reaches into the fillet);
    // the surrounding region takes them back in the absorption step. Narrow
    // features that are entirely crease at one scale get their own core at
    // the next smaller scale.
    for level in 0..SCALE_LEVELS {
        let scale = feature / (1u32 << level) as f32;
        if scale <= 0.0 || region_of.iter().all(|&r| r >= 0) {
            break;
        }
        let crease: Vec<bool> = (0..nt)
            .into_par_iter()
            .map_init(
                || Scratch::new(nt),
                |scratch, t| {
                    if region_of[t] >= 0 {
                        return false;
                    }
                    let st = smoothed[t];
                    // Only through faces that are still unassigned, so a
                    // narrow feature between two finished regions is judged
                    // on its own.
                    let truncated = scratch.neighbourhood(
                        topo,
                        &centers,
                        t as u32,
                        scale,
                        CREASE_MAX_RINGS,
                        false,
                        |f| region_of[f as usize] < 0,
                    );
                    let max_dist = scratch.found.iter().fold(0.0f32, |m, &(_, d)| m.max(d));
                    // The bend is measured from the centre to the rim, i.e.
                    // over half the neighbourhood diameter, so half the crease
                    // angle is the limit. A truncated neighbourhood tests a
                    // shorter distance: scale the allowed bend accordingly.
                    let s = if truncated {
                        (max_dist / scale).clamp(0.2, 1.0)
                    } else {
                        1.0
                    };
                    let bent_cos = (angle * 0.5 * s).cos();
                    // A face is a crease face when a substantial share of
                    // the rim of its neighbourhood bends away from it: a
                    // fillet face sees both neighbouring surfaces bent away
                    // (most of the rim), a flat face next to a fillet or a
                    // sharp edge sees at most half of it, and a thin feature
                    // (emboss wall, small step) covers only a sliver.
                    let rim_from = max_dist * RIM_START;
                    let mut bent = 0.0f32;
                    let mut total = 0.0f32;
                    for &(f, d) in &scratch.found {
                        if d < rim_from {
                            continue;
                        }
                        let a = areas[f as usize];
                        total += a;
                        if st.dot(smoothed[f as usize]) < bent_cos {
                            bent += a;
                        }
                    }
                    total > 0.0 && bent > total * CREASE_RIM_SHARE
                },
            )
            .collect();

        // Cores: flood fill through unassigned non-crease faces.
        let mut stack: Vec<u32> = Vec::new();
        for seed in 0..nt {
            if region_of[seed] >= 0 || crease[seed] {
                continue;
            }
            let rid = regions.len() as i32;
            let mut tris = Vec::new();
            region_of[seed] = rid;
            stack.push(seed as u32);
            while let Some(t) = stack.pop() {
                tris.push(t);
                let na = smoothed[t as usize];
                for &nb in &topo.neighbors[t as usize] {
                    if nb == u32::MAX {
                        continue;
                    }
                    let nbi = nb as usize;
                    if region_of[nbi] >= 0 || crease[nbi] {
                        continue;
                    }
                    // Smoothed and raw dihedral below the crease angle.
                    if na.dot(smoothed[nbi]) >= cos_thresh
                        && topo.face_normals[t as usize].dot(topo.face_normals[nbi]) >= cos_thresh
                    {
                        region_of[nbi] = rid;
                        stack.push(nb);
                    }
                }
            }
            regions.push(tris);
        }

        // Give crease bands back to matching regions, best match first
        // (smallest angle to the anchor normal of the core face the chain
        // started from), so bands between two regions split where the
        // surface actually turns instead of by visiting order.
        let mut heap: BinaryHeap<Absorb> = BinaryHeap::new();
        for t in 0..nt {
            if region_of[t] < 0 {
                continue;
            }
            for &nb in &topo.neighbors[t] {
                if nb != u32::MAX
                    && region_of[nb as usize] < 0
                    && fits_anchor(nb as usize, &anchor[t])
                {
                    heap.push(Absorb {
                        dot: smoothed[nb as usize].dot(anchor[t].0),
                        face: nb,
                        from: t as u32,
                    });
                }
            }
        }
        while let Some(Absorb { face, from, .. }) = heap.pop() {
            let f = face as usize;
            if region_of[f] >= 0 {
                continue;
            }
            let rid = region_of[from as usize];
            region_of[f] = rid;
            regions[rid as usize].push(face);
            anchor[f] = anchor[from as usize];
            for &nb in &topo.neighbors[f] {
                if nb != u32::MAX
                    && region_of[nb as usize] < 0
                    && fits_anchor(nb as usize, &anchor[f])
                {
                    heap.push(Absorb {
                        dot: smoothed[nb as usize].dot(anchor[f].0),
                        face: nb,
                        from: face,
                    });
                }
            }
        }
    }

    // --- Leftovers: classic crease-angle flood fill -----------------------
    let mut stack: Vec<u32> = Vec::new();
    for seed in 0..nt {
        if region_of[seed] >= 0 {
            continue;
        }
        let rid = regions.len() as i32;
        let mut tris = Vec::new();
        region_of[seed] = rid;
        stack.push(seed as u32);
        while let Some(t) = stack.pop() {
            tris.push(t);
            let na = topo.face_normals[t as usize];
            for &nb in &topo.neighbors[t as usize] {
                if nb == u32::MAX {
                    continue;
                }
                let nbi = nb as usize;
                if region_of[nbi] >= 0 {
                    continue;
                }
                if na.dot(topo.face_normals[nbi]) >= cos_thresh {
                    region_of[nbi] = rid;
                    stack.push(nb);
                }
            }
        }
        regions.push(tris);
    }

    // --- Reunite regions split by an absorbed band ------------------------
    // Two cores of the same surface can be separated by a crease band (a
    // groove, an emboss) that the absorption then fills from both sides.
    // Regions touching along a boundary where the faces lie on each other's
    // anchor planes are merged (union-find over the shared edges).
    {
        let mut parent: Vec<usize> = (0..regions.len()).collect();
        fn find(p: &mut [usize], mut x: usize) -> usize {
            while p[x] != x {
                p[x] = p[p[x]];
                x = p[x];
            }
            x
        }
        // (a, b) -> (edges passing, edges total)
        let mut pairs: HashMap<(i32, i32), (u32, u32)> = HashMap::new();
        for t in 0..nt {
            let ra = region_of[t];
            if ra < 0 {
                continue;
            }
            for &nb in &topo.neighbors[t] {
                if nb == u32::MAX {
                    continue;
                }
                let rb = region_of[nb as usize];
                if rb < 0 || rb <= ra {
                    continue;
                }
                let ok =
                    fits_anchor(nb as usize, &anchor[t]) && fits_anchor(t, &anchor[nb as usize]);
                let e = pairs.entry((ra, rb)).or_insert((0, 0));
                e.0 += ok as u32;
                e.1 += 1;
            }
        }
        let mut merges: Vec<(i32, i32)> = pairs
            .iter()
            .filter(|(_, (ok, total))| ok * 10 >= total * 8)
            .map(|(&k, _)| k)
            .collect();
        merges.sort_unstable();
        for (a, b) in merges {
            let (ra, rb) = (find(&mut parent, a as usize), find(&mut parent, b as usize));
            if ra != rb {
                parent[rb.max(ra)] = rb.min(ra);
            }
        }
        for r in 0..regions.len() {
            let root = find(&mut parent, r);
            if root != r {
                let tris = std::mem::take(&mut regions[r]);
                for &t in &tris {
                    region_of[t as usize] = root as i32;
                }
                regions[root].extend(tris);
            }
        }
    }

    // Tiny islands (noise pockets inside crease bands) join the neighbouring
    // region they share the most edges with.
    let tiny = min_tris.max(nt / 2000);
    if tiny > 1 {
        merge_tiny_regions(topo, &mut regions, &mut region_of, tiny);
    }

    // Keep regions above the size limit, largest first so colors stay stable
    // while the crease angle slider is dragged.
    let mut keep: Vec<usize> = (0..regions.len())
        .filter(|&r| regions[r].len() >= min_tris.max(1))
        .collect();
    keep.sort_by(|&a, &b| regions[b].len().cmp(&regions[a].len()).then(a.cmp(&b)));

    let tol = (fit_tol.max(1e-6) * diag).max(1e-9);

    for (new_id, &r) in keep.iter().enumerate() {
        for &t in &regions[r] {
            group_ids[t as usize] = new_id as i32;
        }
    }
    // Primitive classification of the regions is independent per region.
    let groups: Vec<FaceGroup> = keep
        .par_iter()
        .enumerate()
        .map(|(new_id, &r)| classify_region(mesh, &smoothed, &regions[r], new_id as i32, tol, diag))
        .collect();
    (groups, group_ids)
}

/// Merges regions with fewer than `tiny` triangles into the adjacent region
/// sharing the most edges (smallest first, so chains of islands collapse
/// into their surroundings rather than into each other).
fn merge_tiny_regions(
    topo: &MeshTopology,
    regions: &mut [Vec<u32>],
    region_of: &mut [i32],
    tiny: usize,
) {
    let mut order: Vec<usize> = (0..regions.len())
        .filter(|&r| regions[r].len() < tiny)
        .collect();
    order.sort_by_key(|&r| regions[r].len());
    let mut shared: HashMap<i32, usize> = HashMap::new();
    for r in order {
        if regions[r].is_empty() {
            continue;
        }
        shared.clear();
        for &t in &regions[r] {
            for &nb in &topo.neighbors[t as usize] {
                if nb == u32::MAX {
                    continue;
                }
                let other = region_of[nb as usize];
                if other >= 0 && other as usize != r {
                    *shared.entry(other).or_insert(0) += 1;
                }
            }
        }
        let Some((&target, _)) = shared.iter().max_by(|a, b| a.1.cmp(b.1).then(b.0.cmp(a.0)))
        else {
            continue;
        };
        let tris = std::mem::take(&mut regions[r]);
        for &t in &tris {
            region_of[t as usize] = target;
        }
        regions[target as usize].extend(tris);
    }
}

/// Candidate for the crease-band absorption, ordered by best normal match.
struct Absorb {
    dot: f32,
    face: u32,
    from: u32,
}

impl PartialEq for Absorb {
    fn eq(&self, other: &Self) -> bool {
        self.dot == other.dot && self.face == other.face
    }
}
impl Eq for Absorb {}
impl PartialOrd for Absorb {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Absorb {
    fn cmp(&self, other: &Self) -> Ordering {
        self.dot
            .total_cmp(&other.dot)
            .then_with(|| other.face.cmp(&self.face))
    }
}

/// Per-thread scratch for breadth-first neighbourhood queries on the dual
/// graph (a stamp array instead of a hash set per query).
struct Scratch {
    stamp: Vec<u32>,
    frontier: Vec<u32>,
    next: Vec<u32>,
    /// Faces found by the last query with their centre distance.
    found: Vec<(u32, f32)>,
}

impl Scratch {
    fn new(nt: usize) -> Self {
        Scratch {
            stamp: vec![u32::MAX; nt],
            frontier: Vec::new(),
            next: Vec::new(),
            found: Vec::new(),
        }
    }

    /// Collects the faces whose centre lies within `radius` of face `t`'s
    /// centre, reachable through such faces that satisfy `pass`, up to
    /// `max_rings` rings. With `first_ring` the edge neighbours are included
    /// even when they lie further away (coarse meshes). Returns true when the
    /// ring cap stopped the search early.
    #[allow(clippy::too_many_arguments)]
    fn neighbourhood(
        &mut self,
        topo: &MeshTopology,
        centers: &[Vec3],
        t: u32,
        radius: f32,
        max_rings: u32,
        first_ring: bool,
        pass: impl Fn(u32) -> bool,
    ) -> bool {
        self.found.clear();
        self.frontier.clear();
        self.stamp[t as usize] = t;
        self.found.push((t, 0.0));
        self.frontier.push(t);
        let c0 = centers[t as usize];
        let r2 = radius * radius;
        for ring in 0..max_rings {
            self.next.clear();
            let take_all = first_ring && ring == 0;
            for &f in &self.frontier {
                for &nb in &topo.neighbors[f as usize] {
                    if nb == u32::MAX || self.stamp[nb as usize] == t {
                        continue;
                    }
                    self.stamp[nb as usize] = t;
                    if !pass(nb) {
                        continue;
                    }
                    let d2 = (centers[nb as usize] - c0).length_squared();
                    if d2 <= r2 || take_all {
                        self.found.push((nb, d2.sqrt()));
                        self.next.push(nb);
                    }
                }
            }
            if self.next.is_empty() {
                return false;
            }
            std::mem::swap(&mut self.frontier, &mut self.next);
        }
        !self.frontier.is_empty()
    }
}

/// Classifies one region as plane / cylinder / sphere / freeform and fits its
/// primitive parameters. `normals` are the (smoothed) face normals used for
/// the cylinder axis estimate.
fn classify_region(
    mesh: &Mesh,
    normals: &[Vec3],
    tris: &[u32],
    id: i32,
    tol: f32,
    diag: f32,
) -> FaceGroup {
    let mut pts: Vec<[f32; 3]> = Vec::with_capacity(tris.len() * 3);
    let mut area = 0.0f32;
    let mut centroid = Vec3::ZERO;
    let mut ncov = [[0.0f64; 3]; 3];
    for &t in tris {
        let [a, b, c] = mesh.triangle(t as usize);
        let n = normals[t as usize];
        let ta = (b - a).cross(c - a).length() * 0.5;
        area += ta;
        centroid += (a + b + c) * (1.0 / 3.0);
        let w = ta as f64;
        let nx = [n.x as f64, n.y as f64, n.z as f64];
        for i in 0..3 {
            for j in 0..3 {
                ncov[i][j] += w * nx[i] * nx[j];
            }
        }
        pts.push(a.to_array());
        pts.push(b.to_array());
        pts.push(c.to_array());
    }
    let center = centroid / tris.len().max(1) as f32;

    let mut group = FaceGroup {
        id,
        kind: GroupKind::Freeform,
        tris: tris.to_vec(),
        center,
        area,
        normal: Vec3::ZERO,
        point: center,
        radius: 0.0,
        rms: 0.0,
    };

    // --- Plane test --------------------------------------------------------
    // Scans carry spikes and small embossed marks: judge flatness on the
    // best 95 % of the points (trimmed RMS and 90th percentile) instead of
    // the worst point, after refitting the plane to those inliers.
    if let Some(first) = fit_plane(&pts) {
        let dev = |plane: &crate::geom::fitting::PlaneFit, p: &[f32; 3]| {
            (Vec3::from(*p) - plane.point).dot(plane.normal).abs()
        };
        let mut devs: Vec<f32> = pts.iter().map(|p| dev(&first, p)).collect();
        let keep = ((devs.len() as f32 * 0.95) as usize).clamp(3, devs.len());
        devs.select_nth_unstable_by(keep - 1, |a, b| a.total_cmp(b));
        let cut = devs[keep - 1];
        let inliers: Vec<[f32; 3]> = pts
            .iter()
            .copied()
            .filter(|p| dev(&first, p) <= cut)
            .collect();
        if let Some(plane) = fit_plane(&inliers) {
            let mut d2: Vec<f32> = inliers.iter().map(|p| dev(&plane, p)).collect();
            let rms = (d2.iter().map(|d| d * d).sum::<f32>() / d2.len().max(1) as f32).sqrt();
            let k = d2.len().saturating_sub(1);
            let p90 = *d2.select_nth_unstable_by(k, |a, b| a.total_cmp(b)).1;
            if rms <= tol && p90 <= tol * 3.0 {
                group.kind = GroupKind::Plane;
                group.normal = plane.normal;
                group.point = plane.point;
                group.rms = rms;
                return group;
            }
        }
    }

    // --- Cylinder test -----------------------------------------------------
    // On a cylinder the face normals are perpendicular to the axis, so the
    // area-weighted normal covariance has rank ~2: the eigenvector of the
    // smallest eigenvalue is the axis direction. A plane (rank ~1) and a
    // sphere (rank ~3) are rejected by the eigenvalue ratios.
    let eig = eigen_3x3(ncov);
    let (l0, e0) = eig[0];
    let (l1, _) = eig[1];
    let (l2, _) = eig[2];
    let axis = Vec3::new(e0[0] as f32, e0[1] as f32, e0[2] as f32).normalize_or_zero();
    let normals_planar = l1 <= l2 * 1e-4 + 1e-12;
    // Scan noise lifts the smallest eigenvalue; the radial residual below is
    // the decisive test, so the ratio only has to reject spheres.
    if !normals_planar && l0 <= l1 * 0.3 && axis.length_squared() > 0.5 {
        let (u, v) = plane_basis(axis);
        let mut xy: Vec<(f64, f64)> = Vec::with_capacity(pts.len());
        for p in &pts {
            let d = Vec3::from(*p) - center;
            xy.push((d.dot(u) as f64, d.dot(v) as f64));
        }
        if let Some((cx, cy, r)) = fit_circle_2d(&xy) {
            let radius = r as f32;
            let axis_point = center + u * cx as f32 + v * cy as f32;
            let mut ss = 0.0f64;
            for p in &pts {
                let d = Vec3::from(*p) - axis_point;
                let radial = (d - axis * d.dot(axis)).length() - radius;
                ss += (radial * radial) as f64;
            }
            let rms = (ss / pts.len().max(1) as f64).sqrt() as f32;
            if rms <= tol && radius > 1e-6 && radius < diag * 0.75 {
                group.kind = GroupKind::Cylinder;
                group.normal = axis;
                group.point = axis_point;
                group.radius = radius;
                group.rms = rms;
                return group;
            }
        }
    }

    // --- Sphere test -------------------------------------------------------
    if let Some((center_s, radius)) = fit_sphere(&pts, center) {
        let mut ss = 0.0f64;
        for p in &pts {
            let d = (Vec3::from(*p) - center_s).length() - radius;
            ss += (d * d) as f64;
        }
        let rms = (ss / pts.len().max(1) as f64).sqrt() as f32;
        if rms <= tol {
            group.kind = GroupKind::Sphere;
            group.normal = (center_s - center).normalize_or_zero();
            group.point = center_s;
            group.radius = radius;
            group.rms = rms;
            return group;
        }
    }

    group
}

/// Algebraic (Kasa) sphere fit around a seed center. Returns (center, radius).
fn fit_sphere(pts: &[[f32; 3]], c0: Vec3) -> Option<(Vec3, f32)> {
    let mut m = [[0.0f64; 4]; 4];
    let mut b = [0.0f64; 4];
    for p in pts {
        let d = Vec3::from(*p) - c0;
        let phi = [d.x as f64, d.y as f64, d.z as f64, 1.0];
        let q = d.length_squared() as f64;
        for i in 0..4 {
            for j in 0..4 {
                m[i][j] += phi[i] * phi[j];
            }
            b[i] -= phi[i] * q;
        }
    }
    let sol = solve_4x4(m, b)?;
    let a = Vec3::new(sol[0] as f32, sol[1] as f32, sol[2] as f32);
    let d_coef = sol[3] as f32;
    let center = c0 - a * 0.5;
    let r2 = a.length_squared() * 0.25 - d_coef;
    if r2 <= 1e-12 {
        return None;
    }
    Some((center, r2.sqrt()))
}

fn solve_4x4(a: [[f64; 4]; 4], b: [f64; 4]) -> Option<[f64; 4]> {
    let mut m = a;
    let mut r = b;
    for col in 0..4 {
        let mut piv = col;
        for row in col + 1..4 {
            if m[row][col].abs() > m[piv][col].abs() {
                piv = row;
            }
        }
        if m[piv][col].abs() < 1e-14 {
            return None;
        }
        m.swap(col, piv);
        r.swap(col, piv);
        let mcol = m[col];
        for row in 0..4 {
            if row == col {
                continue;
            }
            let f = m[row][col] / mcol[col];
            for k in col..4 {
                m[row][k] -= f * mcol[k];
            }
            r[row] -= f * r[col];
        }
    }
    Some([
        r[0] / m[0][0],
        r[1] / m[1][1],
        r[2] / m[2][2],
        r[3] / m[3][3],
    ])
}
