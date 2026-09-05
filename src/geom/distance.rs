use crate::geom::bvh::Bvh;
use rayon::prelude::*;

#[derive(Clone, Copy, Debug, Default)]
pub struct Deviation {
    pub max_dev: f32,
    pub rms: f32,
    pub count: usize,
}

pub fn deviation(points: &[[f32; 3]], bvh: &Bvh) -> Deviation {
    if points.is_empty() {
        return Deviation::default();
    }
    let (max_dev, sum_sq, count) = points
        .par_chunks(8192)
        .map(|chunk| {
            let mut mx = 0.0f32;
            let mut ss = 0.0f64;
            for p in chunk {
                let d = bvh.closest_distance(glam::Vec3::from(*p));
                if d.is_finite() {
                    mx = mx.max(d);
                    ss += (d * d) as f64;
                }
            }
            (mx, ss, chunk.len())
        })
        .reduce(
            || (0.0f32, 0.0f64, 0usize),
            |a, b| (a.0.max(b.0), a.1 + b.1, a.2 + b.2),
        );
    Deviation {
        max_dev,
        rms: (sum_sq / count.max(1) as f64) as f32,
        count,
    }
}

pub fn per_triangle_max(points: &[[f32; 3]], bvh: &Bvh, ntri: usize) -> Vec<f32> {
    let hits: Vec<(u32, f32)> = points
        .par_iter()
        .map(|p| {
            let (_, tri, d) = bvh.closest_point(glam::Vec3::from(*p));
            (tri, d)
        })
        .collect();
    let mut per_tri = vec![0.0f32; ntri];
    for (t, d) in hits {
        if d.is_finite() && d > per_tri[t as usize] {
            per_tri[t as usize] = d;
        }
    }
    per_tri
}

pub fn per_vertex_max(per_tri: &[f32], indices: &[u32], nverts: usize) -> Vec<f32> {
    let mut per_vert = vec![0.0f32; nverts];
    for (t, &d) in per_tri.iter().enumerate() {
        if d <= 0.0 {
            continue;
        }
        for k in 0..3 {
            let v = indices[3 * t + k] as usize;
            if d > per_vert[v] {
                per_vert[v] = d;
            }
        }
    }
    per_vert
}
