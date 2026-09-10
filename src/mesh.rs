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
        self.positions.par_iter_mut().for_each(|p: &mut [f32; 3]| {
            let v = rotation * Vec3::from(*p) + translation;
            *p = v.to_array();
        });
        self.normals.par_iter_mut().for_each(|n: &mut [f32; 3]| {
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

    #[cfg_attr(not(test), allow(dead_code))]
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
                i => i.saturating_sub(1),
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

    /// Splits this mesh into two meshes:
    /// - first: kept triangles (where `sel[t] == 0`)
    /// - second: hidden / extracted triangles (where `sel[t] > 0`)
    ///
    /// Only referenced vertices are kept in each mesh, with remapped indices and preserved normals.
    pub fn split_by_selection(&self, sel: &[u8]) -> (Mesh, Mesh) {
        let nt = self.triangle_count();
        let nv = self.vertex_count();
        if sel.len() != nt {
            return (self.clone(), Mesh::default());
        }

        let mut kept_remap = vec![u32::MAX; nv];
        let mut hidden_remap = vec![u32::MAX; nv];

        let mut kept_pos = Vec::new();
        let mut kept_nrm = Vec::new();
        let mut kept_idx = Vec::new();

        let mut hidden_pos = Vec::new();
        let mut hidden_nrm = Vec::new();
        let mut hidden_idx = Vec::new();

        let has_normals = self.normals.len() == nv;

        for t in 0..nt {
            let is_hidden = sel[t] > 0;
            for k in 0..3 {
                let old_v = self.indices[3 * t + k] as usize;
                if is_hidden {
                    let new_v = if hidden_remap[old_v] != u32::MAX {
                        hidden_remap[old_v]
                    } else {
                        let idx = hidden_pos.len() as u32;
                        hidden_remap[old_v] = idx;
                        hidden_pos.push(self.positions[old_v]);
                        if has_normals {
                            hidden_nrm.push(self.normals[old_v]);
                        }
                        idx
                    };
                    hidden_idx.push(new_v);
                } else {
                    let new_v = if kept_remap[old_v] != u32::MAX {
                        kept_remap[old_v]
                    } else {
                        let idx = kept_pos.len() as u32;
                        kept_remap[old_v] = idx;
                        kept_pos.push(self.positions[old_v]);
                        if has_normals {
                            kept_nrm.push(self.normals[old_v]);
                        }
                        idx
                    };
                    kept_idx.push(new_v);
                }
            }
        }

        let mut kept_mesh = Mesh {
            positions: kept_pos,
            normals: kept_nrm,
            indices: kept_idx,
        };
        if !has_normals && !kept_mesh.positions.is_empty() {
            kept_mesh.recompute_normals();
        }

        let mut hidden_mesh = Mesh {
            positions: hidden_pos,
            normals: hidden_nrm,
            indices: hidden_idx,
        };
        if !has_normals && !hidden_mesh.positions.is_empty() {
            hidden_mesh.recompute_normals();
        }

        (kept_mesh, hidden_mesh)
    }

    /// Combines this mesh with another mesh into a single mesh.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn combine(&self, other: &Mesh) -> Mesh {
        if self.positions.is_empty() {
            return other.clone();
        }
        if other.positions.is_empty() {
            return self.clone();
        }

        let nv_self = self.positions.len();
        let mut positions = self.positions.clone();
        positions.extend_from_slice(&other.positions);

        let mut normals = self.normals.clone();
        if normals.len() == nv_self && other.normals.len() == other.positions.len() {
            normals.extend_from_slice(&other.normals);
        } else {
            normals.clear();
        }

        let offset = nv_self as u32;
        let mut indices = self.indices.clone();
        indices.reserve(other.indices.len());
        for &idx in &other.indices {
            indices.push(idx + offset);
        }

        let mut m = Mesh {
            positions,
            normals,
            indices,
        };
        if m.normals.is_empty() && !m.positions.is_empty() {
            m.recompute_normals();
        }
        m
    }
}

impl From<(Vec3, Vec3)> for Aabb {
    fn from((min, max): (Vec3, Vec3)) -> Self {
        Aabb { min, max }
    }
}

/// Bit-exact position key (negative zero folded onto zero).
#[inline]
fn key_of(p: &[f32; 3]) -> [u32; 3] {
    let norm = |v: f32| if v == 0.0 { 0 } else { v.to_bits() };
    [norm(p[0]), norm(p[1]), norm(p[2])]
}

/// Welds bit-identical corner positions into shared vertices.
///
/// Vertices are numbered in order of first appearance, exactly like a
/// sequential hash-map weld would number them, but the work is done with a
/// parallel sort so large STL files load several times faster.
pub fn weld_corners(raw: &[[f32; 3]]) -> (Vec<[f32; 3]>, Vec<u32>) {
    let n = raw.len();
    if n < 4096 {
        return weld_corners_sequential(raw);
    }
    // Sort corner indices by position key; equal keys become contiguous runs.
    let mut order: Vec<u32> = (0..n as u32).collect();
    order.par_sort_unstable_by_key(|&i| key_of(&raw[i as usize]));

    // Representative (= smallest corner index) of every run, scattered to
    // each corner of the run.
    let mut rep = vec![0u32; n];
    {
        let rep_cells: Vec<std::sync::atomic::AtomicU32> = (0..n)
            .map(|_| std::sync::atomic::AtomicU32::new(0))
            .collect();
        // Run starts: positions where the key differs from the previous one.
        let starts: Vec<usize> = (0..n)
            .into_par_iter()
            .filter(|&k| {
                k == 0 || key_of(&raw[order[k] as usize]) != key_of(&raw[order[k - 1] as usize])
            })
            .collect();
        starts.par_iter().enumerate().for_each(|(s, &start)| {
            let end = starts.get(s + 1).copied().unwrap_or(n);
            let run = &order[start..end];
            let r = run.iter().copied().min().unwrap_or(run[0]);
            for &i in run {
                rep_cells[i as usize].store(r, std::sync::atomic::Ordering::Relaxed);
            }
        });
        rep.par_iter_mut()
            .zip(rep_cells.par_iter())
            .for_each(|(dst, cell)| *dst = cell.load(std::sync::atomic::Ordering::Relaxed));
    }

    // Number unique vertices by first appearance (rep[i] == i marks a first
    // occurrence), then map every corner through its representative.
    let mut new_index = vec![u32::MAX; n];
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(n / 3);
    for i in 0..n {
        if rep[i] as usize == i {
            new_index[i] = positions.len() as u32;
            positions.push(raw[i]);
        }
    }
    let indices: Vec<u32> = rep.par_iter().map(|&r| new_index[r as usize]).collect();
    (positions, indices)
}

fn weld_corners_sequential(raw: &[[f32; 3]]) -> (Vec<[f32; 3]>, Vec<u32>) {
    let mut map: HashMap<[u32; 3], u32> = HashMap::with_capacity(raw.len() / 2);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parallel_weld_matches_sequential_numbering() {
        let mut rng = Rng::new(3);
        // Many repeated positions drawn from a small set, above the parallel threshold.
        let pool: Vec<[f32; 3]> = (0..500)
            .map(|_| [rng.f32() * 10.0, rng.f32() * 10.0, rng.f32() * 10.0])
            .collect();
        let raw: Vec<[f32; 3]> = (0..9000)
            .map(|_| pool[(rng.next_u64() % 500) as usize])
            .collect();
        let (pa, ia) = weld_corners(&raw);
        let (pb, ib) = weld_corners_sequential(&raw);
        assert_eq!(pa, pb);
        assert_eq!(ia, ib);
        assert!(pa.len() <= 500);
    }

    #[test]
    fn negative_zero_welds_with_zero() {
        let raw = vec![[0.0, 0.0, 0.0], [-0.0, 0.0, -0.0], [1.0, 0.0, 0.0]];
        let (p, i) = weld_corners_sequential(&raw);
        assert_eq!(p.len(), 2);
        assert_eq!(i, vec![0, 0, 1]);
    }
}
