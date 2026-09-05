use crate::camera::Camera;
use crate::geom::bvh::Bvh;
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;

pub struct Hit {
    pub pos: Vec3,
    #[allow(dead_code)]
    pub tri: u32,
}

pub fn ray_pick(
    bvh: &Bvh,
    cam: &Camera,
    sx: f32,
    sy: f32,
    w: f32,
    h: f32,
) -> Option<Hit> {
    let (ro, rd) = cam.screen_ray(sx, sy, w, h);
    if rd.length_squared() < 0.5 {
        return None;
    }
    let (t, tri) = bvh.ray_cast(ro, rd, cam.far)?;
    Some(Hit {
        pos: ro + rd * t,
        tri,
    })
}

pub fn brush(
    mesh: &Mesh,
    cam: &Camera,
    hit: &Hit,
    radius_px: f32,
    viewport_h_px: f32,
    add: bool,
    sel: &mut [u8],
) {
    let eye = cam.eye();
    let depth = (hit.pos - eye).length();
    let r = radius_px * cam.world_per_pixel_at(depth, viewport_h_px);
    let r2 = r * r;
    let nt = mesh.triangle_count();
    let positions = &mesh.positions;
    let indices = &mesh.indices;
    let hit_pos = hit.pos;
    sel[hit.tri as usize] = if add { 1u8 } else { 0u8 };
    let found: Vec<Vec<u32>> = (0..nt)
        .into_par_iter()
        .chunks(2048)
        .map(|chunk| {
            let mut local = Vec::new();
            for t in chunk {
                let i0 = indices[3 * t] as usize;
                let i1 = indices[3 * t + 1] as usize;
                let i2 = indices[3 * t + 2] as usize;
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
                    local.push(t as u32);
                }
            }
            local
        })
        .collect();
    let value = if add { 1u8 } else { 0u8 };
    for chunk in found {
        for t in chunk {
            sel[t as usize] = value;
        }
    }
}
