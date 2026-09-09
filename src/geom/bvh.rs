use glam::Vec3;

pub struct Bvh {
    node_min: Vec<[f32; 3]>,
    node_max: Vec<[f32; 3]>,
    node_left: Vec<u32>,
    node_right: Vec<u32>,
    node_count: Vec<u32>,
    tri_order: Vec<u32>,
    verts: Vec<[f32; 3]>,
    tris: Vec<[u32; 3]>,
}

impl Bvh {
    pub fn new(positions: &[[f32; 3]], indices: &[u32]) -> Bvh {
        let nt = indices.len() / 3;
        let mut bvh = Bvh {
            node_min: Vec::with_capacity(nt * 2),
            node_max: Vec::with_capacity(nt * 2),
            node_left: Vec::with_capacity(nt * 2),
            node_right: Vec::with_capacity(nt * 2),
            node_count: Vec::with_capacity(nt * 2),
            tri_order: (0..nt as u32).collect(),
            verts: positions.to_vec(),
            tris: indices
                .chunks_exact(3)
                .map(|c| [c[0], c[1], c[2]])
                .collect(),
        };
        if nt == 0 {
            return bvh;
        }
        let centroids: Vec<[f32; 3]> = bvh
            .tris
            .iter()
            .map(|t| {
                let a = Vec3::from(bvh.verts[t[0] as usize]);
                let b = Vec3::from(bvh.verts[t[1] as usize]);
                let c = Vec3::from(bvh.verts[t[2] as usize]);
                ((a + b + c) * (1.0 / 3.0)).to_array()
            })
            .collect();
        let mut builder = Builder {
            bvh: &mut bvh,
            centroids: &centroids,
        };
        builder.build(0, nt);
        bvh
    }

    #[allow(dead_code)]
    pub fn triangle_count(&self) -> usize {
        self.tris.len()
    }

    pub fn tri_verts(&self, t: u32) -> [Vec3; 3] {
        let [a, b, c] = self.tris[t as usize];
        [
            Vec3::from(self.verts[a as usize]),
            Vec3::from(self.verts[b as usize]),
            Vec3::from(self.verts[c as usize]),
        ]
    }

    pub fn face_normal(&self, t: u32) -> Vec3 {
        let [a, b, c] = self.tri_verts(t);
        (b - a).cross(c - a).normalize_or_zero()
    }

    pub fn closest_point(&self, p: Vec3) -> (Vec3, u32, f32) {
        if self.tris.is_empty() {
            return (p, 0, f32::INFINITY);
        }
        let mut stack: [(u32, f32); 128] = [(0, 0.0); 128];
        let mut sp = 0usize;
        stack[sp] = (0, 0.0);
        sp += 1;
        let mut best_d2 = f32::MAX;
        let mut best_point = p;
        let mut best_tri = 0u32;
        while sp > 0 {
            sp -= 1;
            let (node, nd2) = stack[sp];
            if nd2 >= best_d2 {
                continue;
            }
            let n = node as usize;
            let cnt = self.node_count[n];
            if cnt > 0 {
                let first = self.node_left[n] as usize;
                for k in 0..cnt as usize {
                    let t = self.tri_order[first + k];
                    let [a, b, c] = self.tri_verts(t);
                    let (q, d2) = closest_point_triangle(p, a, b, c);
                    if d2 < best_d2 {
                        best_d2 = d2;
                        best_point = q;
                        best_tri = t;
                    }
                }
            } else {
                let l = self.node_left[n];
                let r = self.node_right[n];
                let ld2 =
                    point_aabb_dist_sq(p, &self.node_min[l as usize], &self.node_max[l as usize]);
                let rd2 =
                    point_aabb_dist_sq(p, &self.node_min[r as usize], &self.node_max[r as usize]);
                if ld2 < best_d2 {
                    stack[sp] = (l, ld2);
                    sp += 1;
                }
                if rd2 < best_d2 {
                    stack[sp] = (r, rd2);
                    sp += 1;
                }
            }
        }
        (best_point, best_tri, best_d2.sqrt())
    }

    pub fn closest_distance(&self, p: Vec3) -> f32 {
        self.closest_point(p).2
    }

    pub fn ray_cast(&self, ro: Vec3, rd: Vec3, tmax: f32) -> Option<(f32, u32)> {
        self.ray_cast_filtered(ro, rd, tmax, |_| false)
    }

    pub fn ray_cast_filtered(
        &self,
        ro: Vec3,
        rd: Vec3,
        tmax: f32,
        is_hidden: impl Fn(u32) -> bool,
    ) -> Option<(f32, u32)> {
        if self.tris.is_empty() {
            return None;
        }
        let mut stack: [(u32, f32); 128] = [(0, 0.0); 128];
        let mut sp = 0usize;
        stack[sp] = (0, 0.0);
        sp += 1;
        let mut best_t = tmax;
        let mut best_tri = 0u32;
        let mut hit = false;
        while sp > 0 {
            sp -= 1;
            let (node, tmin) = stack[sp];
            if tmin > best_t {
                continue;
            }
            let n = node as usize;
            let cnt = self.node_count[n];
            if cnt > 0 {
                let first = self.node_left[n] as usize;
                for k in 0..cnt as usize {
                    let t = self.tri_order[first + k];
                    if is_hidden(t) {
                        continue;
                    }
                    let [a, b, c] = self.tri_verts(t);
                    if let Some(th) = ray_triangle(ro, rd, a, b, c) {
                        if th < best_t {
                            best_t = th;
                            best_tri = t;
                            hit = true;
                        }
                    }
                }
            } else {
                let l = self.node_left[n];
                let r = self.node_right[n];
                if let Some(tl) = ray_aabb(
                    ro,
                    rd,
                    &self.node_min[l as usize],
                    &self.node_max[l as usize],
                ) {
                    stack[sp] = (l, tl);
                    sp += 1;
                }
                if let Some(tr) = ray_aabb(
                    ro,
                    rd,
                    &self.node_min[r as usize],
                    &self.node_max[r as usize],
                ) {
                    stack[sp] = (r, tr);
                    sp += 1;
                }
            }
        }
        if hit { Some((best_t, best_tri)) } else { None }
    }

    #[allow(dead_code)]
    pub fn query_sphere(&self, center: Vec3, radius: f32, out: &mut Vec<u32>) {
        self.query_sphere_filtered(center, radius, out, |_| false);
    }

    pub fn query_sphere_filtered(
        &self,
        center: Vec3,
        radius: f32,
        out: &mut Vec<u32>,
        is_hidden: impl Fn(u32) -> bool,
    ) {
        if self.tris.is_empty() {
            return;
        }
        let r2 = radius * radius;
        let mut stack: [u32; 256] = [0; 256];
        let mut sp = 0usize;
        stack[sp] = 0;
        sp += 1;
        while sp > 0 {
            sp -= 1;
            let node = stack[sp];
            let n = node as usize;
            let d2 = point_aabb_dist_sq(center, &self.node_min[n], &self.node_max[n]);
            if d2 > r2 {
                continue;
            }
            let cnt = self.node_count[n];
            if cnt > 0 {
                let first = self.node_left[n] as usize;
                for k in 0..cnt as usize {
                    let t = self.tri_order[first + k];
                    if is_hidden(t) {
                        continue;
                    }
                    let [a, b, c] = self.tri_verts(t);
                    if (a - center).length_squared() <= r2
                        || (b - center).length_squared() <= r2
                        || (c - center).length_squared() <= r2
                        || closest_point_triangle(center, a, b, c).1 <= r2
                    {
                        out.push(t);
                    }
                }
            } else {
                let l = self.node_left[n];
                let r = self.node_right[n];
                if sp + 2 < stack.len() {
                    stack[sp] = l;
                    sp += 1;
                    stack[sp] = r;
                    sp += 1;
                }
            }
        }
    }
}

struct Builder<'a> {
    bvh: &'a mut Bvh,
    centroids: &'a [[f32; 3]],
}

impl<'a> Builder<'a> {
    fn build(&mut self, first: usize, count: usize) {
        let idx = self.bvh.node_min.len() as u32;
        self.bvh.node_min.push([0.0; 3]);
        self.bvh.node_max.push([0.0; 3]);
        self.bvh.node_left.push(0);
        self.bvh.node_right.push(0);
        self.bvh.node_count.push(0);

        let mut bmin = [f32::MAX; 3];
        let mut bmax = [f32::MIN; 3];
        let mut cmin = [f32::MAX; 3];
        let mut cmax = [f32::MIN; 3];
        for k in 0..count {
            let t = self.bvh.tri_order[first + k] as usize;
            for &v in &self.bvh.tris[t] {
                let p = self.bvh.verts[v as usize];
                for a in 0..3 {
                    bmin[a] = bmin[a].min(p[a]);
                    bmax[a] = bmax[a].max(p[a]);
                }
            }
            let c = self.centroids[t];
            for a in 0..3 {
                cmin[a] = cmin[a].min(c[a]);
                cmax[a] = cmax[a].max(c[a]);
            }
        }
        let n = idx as usize;
        self.bvh.node_min[n] = bmin;
        self.bvh.node_max[n] = bmax;

        if count <= 4 {
            self.bvh.node_count[n] = count as u32;
            self.bvh.node_left[n] = first as u32;
            return;
        }

        let extent = [cmax[0] - cmin[0], cmax[1] - cmin[1], cmax[2] - cmin[2]];
        let mut axis = 0;
        if extent[1] > extent[axis] {
            axis = 1;
        }
        if extent[2] > extent[axis] {
            axis = 2;
        }
        let lc = if extent[axis] <= 1e-12 {
            count / 2
        } else {
            let mid_c = (cmin[axis] + cmax[axis]) * 0.5;
            let mut i = first;
            let mut j = first + count - 1;
            while i <= j {
                let ci = self.centroids[self.bvh.tri_order[i] as usize][axis];
                if ci < mid_c {
                    i += 1;
                } else {
                    self.bvh.tri_order.swap(i, j);
                    j -= 1;
                }
            }
            (i - first).clamp(1, count - 1)
        };
        self.build(first, lc);
        let right = self.bvh.node_min.len() as u32;
        self.build(first + lc, count - lc);
        self.bvh.node_left[n] = idx + 1;
        self.bvh.node_right[n] = right;
        self.bvh.node_count[n] = 0;
    }
}

fn point_aabb_dist_sq(p: Vec3, mn: &[f32; 3], mx: &[f32; 3]) -> f32 {
    let mn = Vec3::from(*mn);
    let mx = Vec3::from(*mx);
    let d = mn - p;
    let e = p - mx;
    let dx = d.x.max(e.x).max(0.0);
    let dy = d.y.max(e.y).max(0.0);
    let dz = d.z.max(e.z).max(0.0);
    dx * dx + dy * dy + dz * dz
}

pub fn closest_point_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> (Vec3, f32) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, ap.length_squared());
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, bp.length_squared());
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let q = a + ab * v;
        let dq = p - q;
        return (q, dq.length_squared());
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, cp.length_squared());
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let q = a + ac * w;
        let dq = p - q;
        return (q, dq.length_squared());
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let q = b + (c - b) * w;
        let dq = p - q;
        return (q, dq.length_squared());
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let q = a + ab * v + ac * w;
    let dq = p - q;
    (q, dq.length_squared())
}

fn ray_aabb(ro: Vec3, rd: Vec3, mn: &[f32; 3], mx: &[f32; 3]) -> Option<f32> {
    let mn = Vec3::from(*mn);
    let mx = Vec3::from(*mx);
    let mut tmin = 0.0f32;
    let mut tmax = f32::INFINITY;
    for a in 0..3 {
        let (t0, t1) = if rd[a] >= 0.0 {
            ((mn[a] - ro[a]) / rd[a], (mx[a] - ro[a]) / rd[a])
        } else {
            ((mx[a] - ro[a]) / rd[a], (mn[a] - ro[a]) / rd[a])
        };
        tmin = tmin.max(t0.min(t1));
        tmax = tmax.min(t0.max(t1));
        if tmax < tmin {
            return None;
        }
    }
    Some(tmin)
}

pub fn ray_triangle(ro: Vec3, rd: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    let e1 = b - a;
    let e2 = c - a;
    let pvec = rd.cross(e2);
    let det = e1.dot(pvec);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let tvec = ro - a;
    let u = tvec.dot(pvec) * inv_det;
    if u < 0.0 || u > 1.0 {
        return None;
    }
    let qvec = tvec.cross(e1);
    let v = rd.dot(qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(qvec) * inv_det;
    if t > 1e-6 { Some(t) } else { None }
}
