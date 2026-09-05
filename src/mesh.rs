use crate::rng::Rng;
use glam::{Quat, Vec3};
use rayon::prelude::*;
use std::collections::HashMap;

#[derive(Clone, Default)]
pub struct Mesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    pub fn extent(&self) -> Vec3 {
        self.max - self.min
    }

    pub fn diagonal(&self) -> f32 {
        self.extent().length()
    }
}

impl Mesh {
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    pub fn from_indexed(positions: Vec<[f32; 3]>, indices: Vec<u32>) -> Mesh {
        let mut m = Mesh {
            positions,
            normals: Vec::new(),
            indices,
        };
        m.recompute_normals();
        m
    }

    pub fn from_corners(raw: &[[f32; 3]]) -> Mesh {
        let (positions, indices) = weld_corners(raw);
        Mesh::from_indexed(positions, indices)
    }

    pub fn recompute_normals(&mut self) {
        let nv = self.positions.len();
        if nv == 0 {
            self.normals.clear();
            return;
        }
        let nt = self.indices.len() / 3;
        let mut offsets = vec![0u32; nv + 1];
        for &i in &self.indices {
            offsets[i as usize + 1] += 1;
        }
        for v in 0..nv {
            offsets[v + 1] += offsets[v];
        }
        let mut fill = vec![0u32; nv];
        let mut tris_of_vert = vec![0u32; self.indices.len()];
        for t in 0..nt {
            for k in 0..3 {
                let v = self.indices[3 * t + k] as usize;
                tris_of_vert[offsets[v] as usize + fill[v] as usize] = t as u32;
                fill[v] += 1;
            }
        }
        let positions = &self.positions;
        let indices = &self.indices;
        self.normals = (0..nv)
            .into_par_iter()
            .map(|v| {
                let mut n = Vec3::ZERO;
                for k in offsets[v]..offsets[v + 1] {
                    let t = tris_of_vert[k as usize];
                    let a = Vec3::from(positions[indices[3 * t as usize] as usize]);
                    let b = Vec3::from(positions[indices[3 * t as usize + 1] as usize]);
                    let c = Vec3::from(positions[indices[3 * t as usize + 2] as usize]);
                    n += (b - a).cross(c - a);
                }
                if n.length_squared() > 1e-20 {
                    n.normalize()
                } else {
                    Vec3::Y
                }
                .to_array()
            })
            .collect();
    }

    pub fn transform(&mut self, rotation: Quat, translation: Vec3) {
        self.positions
            .par_iter_mut()
            .for_each(|p: &mut [f32; 3]| {
                let v = rotation * Vec3::from(*p) + translation;
                *p = v.to_array();
            });
        self.normals
            .par_iter_mut()
            .for_each(|n: &mut [f32; 3]| {
                let v = rotation * Vec3::from(*n);
                *n = v.to_array();
            });
    }

    pub fn bbox(&self) -> Aabb {
        if self.positions.is_empty() {
            return Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            };
        }
        self.positions
            .par_iter()
            .fold(
                || (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
                |(mn, mx), p| {
                    let p = Vec3::from(*p);
                    (mn.min(p), mx.max(p))
                },
            )
            .reduce(
                || (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
                |a, b| (a.0.min(b.0), a.1.max(b.1)),
            )
            .into()
    }

    #[allow(dead_code)]
    pub fn surface_area(&self) -> f32 {
        let positions = &self.positions;
        let indices = &self.indices;
        (0..self.triangle_count())
            .into_par_iter()
            .map(|t| {
                let a = Vec3::from(positions[indices[3 * t] as usize]);
                let b = Vec3::from(positions[indices[3 * t + 1] as usize]);
                let c = Vec3::from(positions[indices[3 * t + 2] as usize]);
                (b - a).cross(c - a).length() * 0.5
            })
            .sum()
    }

    pub fn triangle(&self, t: usize) -> [Vec3; 3] {
        let i0 = self.indices[3 * t] as usize;
        let i1 = self.indices[3 * t + 1] as usize;
        let i2 = self.indices[3 * t + 2] as usize;
        [
            Vec3::from(self.positions[i0]),
            Vec3::from(self.positions[i1]),
            Vec3::from(self.positions[i2]),
        ]
    }

    #[allow(dead_code)]
    pub fn face_normal(&self, t: usize) -> Vec3 {
        let [a, b, c] = self.triangle(t);
        (b - a).cross(c - a)
    }

    pub fn sample_surface(&self, n: usize, rng: &mut Rng) -> Vec<Vec3> {
        let nt = self.triangle_count();
        if nt == 0 || n == 0 {
            return Vec::new();
        }
        let mut cum = Vec::with_capacity(nt + 1);
        cum.push(0.0f64);
        for t in 0..nt {
            let [a, b, c] = self.triangle(t);
            let area = (b - a).cross(c - a).length() as f64 * 0.5;
            cum.push(cum[t] + area.max(1e-20));
        }
        let total = cum[nt];
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            let r = rng.f64() * total;
            let t = match cum.partition_point(|x| *x < r) {
                i if i >= nt => nt - 1,
                i => i.saturating_sub(1).max(0),
            };
            let [a, b, c] = self.triangle(t);
            let mut u = rng.f32();
            let mut v = rng.f32();
            if u + v > 1.0 {
                u = 1.0 - u;
                v = 1.0 - v;
            }
            out.push(a + (b - a) * u + (c - a) * v);
        }
        out
    }
}

impl From<(Vec3, Vec3)> for Aabb {
    fn from((min, max): (Vec3, Vec3)) -> Self {
        Aabb { min, max }
    }
}

fn key_of(p: &[f32; 3]) -> [u8; 12] {
    let mut k = [0u8; 12];
    for (i, v) in p.iter().enumerate() {
        let bits = if v.to_bits() == 0x8000_0000 {
            0.0f32.to_bits()
        } else {
            v.to_bits()
        };
        k[4 * i..4 * i + 4].copy_from_slice(&bits.to_le_bytes());
    }
    k
}

pub fn weld_corners(raw: &[[f32; 3]]) -> (Vec<[f32; 3]>, Vec<u32>) {
    let mut map: HashMap<[u8; 12], u32> = HashMap::with_capacity(raw.len() / 2);
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(raw.len() / 2);
    let mut indices: Vec<u32> = Vec::with_capacity(raw.len());
    for p in raw {
        let k = key_of(p);
        let idx = match map.get(&k) {
            Some(&i) => i,
            None => {
                let i = positions.len() as u32;
                positions.push(*p);
                map.insert(k, i);
                i
            }
        };
        indices.push(idx);
    }
    (positions, indices)
}
