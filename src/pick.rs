//! Ray picking and brush queries against the displayed mesh.

use crate::camera::Camera;
use crate::geom::bvh::Bvh;
use crate::geom::topology::MeshTopology;
use crate::mesh::Mesh;
use glam::Vec3;

pub struct Hit {
    pub pos: Vec3,
    pub tri: u32,
}

#[cfg_attr(not(test), allow(dead_code))]
pub fn ray_pick(bvh: &Bvh, cam: &Camera, sx: f32, sy: f32, w: f32, h: f32) -> Option<Hit> {
    ray_pick_filtered(bvh, cam, sx, sy, w, h, |_| false)
}

pub fn ray_pick_filtered(
    bvh: &Bvh,
    cam: &Camera,
    sx: f32,
    sy: f32,
    w: f32,
    h: f32,
    is_hidden: impl Fn(u32) -> bool,
) -> Option<Hit> {
    let (ro, rd) = cam.screen_ray(sx, sy, w, h);
    if rd.length_squared() < 0.5 {
        return None;
    }
    let (t, tri) = bvh.ray_cast_filtered(ro, rd, f32::INFINITY, is_hidden)?;
    Some(Hit {
        pos: ro + rd * t,
        tri,
    })
}

/// Brush radius in world units at the hit depth.
fn brush_world_radius(cam: &Camera, hit: &Hit, radius_px: f32, viewport_h_px: f32) -> f32 {
    let depth = (hit.pos - cam.eye()).length();
    radius_px * cam.world_per_pixel_at(depth, viewport_h_px)
}

/// True when triangle `t` faces the eye and has a corner inside the sphere.
#[inline]
fn tri_in_brush(mesh: &Mesh, t: u32, eye: Vec3, center: Vec3, r2: f32) -> bool {
    let [a, b, c] = mesh.triangle(t as usize);
    let n = (b - a).cross(c - a);
    if n.length_squared() < 1e-20 {
        return false;
    }
    let centroid = (a + b + c) * (1.0 / 3.0);
    if n.dot(eye - centroid) <= 0.0 {
        return false;
    }
    (a - center).length_squared() <= r2
        || (b - center).length_squared() <= r2
        || (c - center).length_squared() <= r2
}

#[allow(dead_code)]
pub fn query_brush_triangles(
    mesh: &Mesh,
    bvh: &Bvh,
    cam: &Camera,
    hit: &Hit,
    radius_px: f32,
    viewport_h_px: f32,
) -> (f32, Vec<u32>) {
    query_brush_triangles_filtered(mesh, bvh, cam, hit, radius_px, viewport_h_px, |_| false)
}

/// All visible, front-facing triangles with a corner inside the brush
/// sphere (irrespective of connectivity).
pub fn query_brush_triangles_filtered(
    mesh: &Mesh,
    bvh: &Bvh,
    cam: &Camera,
    hit: &Hit,
    radius_px: f32,
    viewport_h_px: f32,
    is_hidden: impl Fn(u32) -> bool,
) -> (f32, Vec<u32>) {
    let eye = cam.eye();
    let r = brush_world_radius(cam, hit, radius_px, viewport_h_px);
    let r2 = r * r;

    let mut candidates = Vec::new();
    bvh.query_sphere_filtered(hit.pos, r, &mut candidates, &is_hidden);
    candidates.sort_unstable();
    candidates.dedup();

    let mut result = Vec::with_capacity(candidates.len() + 1);
    if !is_hidden(hit.tri) {
        result.push(hit.tri);
    }
    for t in candidates {
        if t == hit.tri || is_hidden(t) {
            continue;
        }
        if tri_in_brush(mesh, t, eye, hit.pos, r2) {
            result.push(t);
        }
    }
    (r, result)
}

/// Triangles inside the brush sphere that are edge-connected to the hit
/// triangle through other triangles inside the sphere. This keeps the brush
/// from bleeding onto separate surfaces behind thin walls or across gaps.
#[allow(clippy::too_many_arguments)]
pub fn query_brush_connected(
    mesh: &Mesh,
    bvh: &Bvh,
    topo: &MeshTopology,
    cam: &Camera,
    hit: &Hit,
    radius_px: f32,
    viewport_h_px: f32,
    is_hidden: impl Fn(u32) -> bool,
) -> (f32, Vec<u32>) {
    if topo.triangle_count() != mesh.triangle_count() {
        return query_brush_triangles_filtered(
            mesh,
            bvh,
            cam,
            hit,
            radius_px,
            viewport_h_px,
            is_hidden,
        );
    }
    let eye = cam.eye();
    let r = brush_world_radius(cam, hit, radius_px, viewport_h_px);
    let r2 = r * r;
    if is_hidden(hit.tri) {
        return (r, Vec::new());
    }
    // Flood over the dual graph, restricted to the brush sphere.
    let mut result = vec![hit.tri];
    let mut visited: std::collections::HashSet<u32> = std::collections::HashSet::new();
    visited.insert(hit.tri);
    let mut stack = vec![hit.tri];
    while let Some(t) = stack.pop() {
        for &nb in &topo.neighbors[t as usize] {
            if nb == u32::MAX || visited.contains(&nb) {
                continue;
            }
            visited.insert(nb);
            if is_hidden(nb) || !tri_in_brush(mesh, nb, eye, hit.pos, r2) {
                continue;
            }
            result.push(nb);
            stack.push(nb);
        }
    }
    (r, result)
}

#[cfg_attr(not(test), allow(dead_code))]
#[allow(clippy::too_many_arguments)]
pub fn brush(
    mesh: &Mesh,
    bvh: &Bvh,
    cam: &Camera,
    hit: &Hit,
    radius_px: f32,
    viewport_h_px: f32,
    add: bool,
    sel: &mut [u8],
) {
    brush_filtered(
        mesh,
        bvh,
        cam,
        hit,
        radius_px,
        viewport_h_px,
        add,
        sel,
        |_| false,
    );
}

#[allow(clippy::too_many_arguments)]
#[cfg_attr(not(test), allow(dead_code))]
pub fn brush_filtered(
    mesh: &Mesh,
    bvh: &Bvh,
    cam: &Camera,
    hit: &Hit,
    radius_px: f32,
    viewport_h_px: f32,
    add: bool,
    sel: &mut [u8],
    is_hidden: impl Fn(u32) -> bool,
) {
    let (_, tris) =
        query_brush_triangles_filtered(mesh, bvh, cam, hit, radius_px, viewport_h_px, is_hidden);
    let value = if add { 1u8 } else { 0u8 };
    for t in tris {
        if (t as usize) < sel.len() {
            sel[t as usize] = value;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::ViewDir;

    /// Two parallel unit quads, 0.05 apart along Z, facing +Z.
    fn double_layer() -> Mesh {
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        for (layer, z) in [0.0f32, -0.05].iter().enumerate() {
            let base = (layer * 4) as u32;
            positions.extend_from_slice(&[
                [0.0, 0.0, *z],
                [1.0, 0.0, *z],
                [1.0, 1.0, *z],
                [0.0, 1.0, *z],
            ]);
            indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        }
        Mesh::from_indexed(positions, indices)
    }

    #[test]
    fn connected_brush_ignores_detached_layer_behind() {
        let m = double_layer();
        let bvh = Bvh::new(&m.positions, &m.indices);
        let topo = MeshTopology::build(&m);
        let mut cam = Camera::default();
        cam.target = Vec3::new(0.5, 0.5, 0.0);
        cam.distance = 5.0;
        cam.aspect = 1.0;
        cam.set_view(ViewDir::Front);
        let (ro, rd) = cam.screen_ray(400.0, 400.0, 800.0, 800.0);
        let (t, tri) = bvh.ray_cast(ro, rd, f32::INFINITY).unwrap();
        let hit = Hit {
            pos: ro + rd * t,
            tri,
        };
        let (_, plain) =
            query_brush_triangles_filtered(&m, &bvh, &cam, &hit, 400.0, 800.0, |_| false);
        let (_, connected) =
            query_brush_connected(&m, &bvh, &topo, &cam, &hit, 400.0, 800.0, |_| false);
        assert!(
            plain.iter().any(|&t| t >= 2),
            "plain brush reaches the rear layer"
        );
        assert!(
            connected.iter().all(|&t| t < 2),
            "connected brush stays on the front layer"
        );
        assert_eq!(connected.len(), 2);
    }
}
