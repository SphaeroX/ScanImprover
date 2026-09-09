use crate::geom::fitting::{eigen_3x3, fit_circle_2d, fit_plane, plane_basis};
use crate::geom::topology::MeshTopology;
use crate::mesh::Mesh;
use glam::Vec3;

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

/// Segments the mesh into face groups via region growing over the triangle
/// dual graph, stopping at edges whose dihedral angle exceeds `angle_deg`.
/// Regions smaller than `min_tris` triangles are discarded (left ungrouped).
/// `fit_tol` is the classification tolerance as a fraction of the bbox diagonal.
///
/// Returns the groups (sorted by size, largest first) and a per-triangle group
/// id map (-1 = ungrouped).
pub fn segment_faces(
    mesh: &Mesh,
    topo: &MeshTopology,
    angle_deg: f32,
    min_tris: usize,
    fit_tol: f32,
) -> (Vec<FaceGroup>, Vec<i32>) {
    let nt = topo.triangle_count();
    let mut group_ids = vec![-1i32; nt];
    if nt == 0 {
        return (Vec::new(), group_ids);
    }
    let cos_thresh = angle_deg.to_radians().cos();

    // Region growing: flood fill across edges with small dihedral angle.
    let mut region_of = vec![-1i32; nt];
    let mut regions: Vec<Vec<u32>> = Vec::new();
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
                let nbn = topo.face_normals[nbi];
                // angle between the face normals <= threshold <=> dot >= cos(threshold)
                if na.dot(nbn) >= cos_thresh {
                    region_of[nbi] = rid;
                    stack.push(nb);
                }
            }
        }
        regions.push(tris);
    }

    // Keep regions above the size limit, largest first so colors stay stable
    // while the crease angle slider is dragged.
    let mut keep: Vec<usize> = (0..regions.len())
        .filter(|&r| regions[r].len() >= min_tris.max(1))
        .collect();
    keep.sort_by(|&a, &b| regions[b].len().cmp(&regions[a].len()).then(a.cmp(&b)));

    let diag = mesh.bbox().diagonal().max(1e-9);
    let tol = (fit_tol.max(1e-6) * diag).max(1e-9);

    let mut groups = Vec::with_capacity(keep.len());
    for (new_id, &r) in keep.iter().enumerate() {
        let id = new_id as i32;
        for &t in &regions[r] {
            group_ids[t as usize] = id;
        }
        groups.push(classify_region(mesh, topo, &regions[r], id, tol, diag));
    }
    (groups, group_ids)
}

/// Classifies one region as plane / cylinder / sphere / freeform and fits its
/// primitive parameters.
fn classify_region(
    mesh: &Mesh,
    topo: &MeshTopology,
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
        let n = topo.face_normals[t as usize];
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
    if let Some(plane) = fit_plane(&pts) {
        let rms = plane.rms.sqrt();
        if rms <= tol && plane.max_dev <= tol * 4.0 {
            group.kind = GroupKind::Plane;
            group.normal = plane.normal;
            group.point = plane.point;
            group.rms = rms;
            return group;
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
    if !normals_planar && l0 <= l1 * 0.05 && axis.length_squared() > 0.5 {
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
