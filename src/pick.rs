use crate::camera::Camera;
use crate::geom::bvh::Bvh;
use crate::mesh::Mesh;
use glam::Vec3;

pub struct Hit {
    pub pos: Vec3,
    #[allow(dead_code)]
    pub tri: u32,
}

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
    let (t, tri) = bvh.ray_cast_filtered(ro, rd, cam.far, is_hidden)?;
    Some(Hit {
        pos: ro + rd * t,
        tri,
    })
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
    let depth = (hit.pos - eye).length();
    let r = radius_px * cam.world_per_pixel_at(depth, viewport_h_px);
    let r2 = r * r;
    let hit_pos = hit.pos;
    let positions = &mesh.positions;
    let indices = &mesh.indices;

    let mut candidates = Vec::new();
    bvh.query_sphere_filtered(hit_pos, r, &mut candidates, &is_hidden);
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
        let t_usize = t as usize;
        let i0 = indices[3 * t_usize] as usize;
        let i1 = indices[3 * t_usize + 1] as usize;
        let i2 = indices[3 * t_usize + 2] as usize;
        let a = Vec3::from(positions[i0]);
        let b = Vec3::from(positions[i1]);
        let c = Vec3::from(positions[i2]);
        let centroid = (a + b + c) * (1.0 / 3.0);
        let n = (b - a).cross(c - a);
        if n.length_squared() < 1e-20 {
            continue;
        }
        if n.normalize().dot(eye - centroid) <= 0.0 {
            continue;
        }
        if (a - hit_pos).length_squared() <= r2
            || (b - hit_pos).length_squared() <= r2
            || (c - hit_pos).length_squared() <= r2
        {
            result.push(t);
        }
    }
    (r, result)
}

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
    brush_filtered(mesh, bvh, cam, hit, radius_px, viewport_h_px, add, sel, |_| false);
}

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
    let (_, tris) = query_brush_triangles_filtered(mesh, bvh, cam, hit, radius_px, viewport_h_px, is_hidden);
    let value = if add { 1u8 } else { 0u8 };
    for t in tris {
        if (t as usize) < sel.len() {
            sel[t as usize] = value;
        }
    }
}
