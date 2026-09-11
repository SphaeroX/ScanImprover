//! Multi-resolution graph hierarchy for the field solvers.
//!
//! Level 0 is the vertex graph of the work mesh. Every coarser level pairs
//! neighbouring vertices greedily (Instant Meshes `downsample_graph`), so
//! each level has roughly half the vertices of the one below. Fields are
//! solved on the coarsest level first and refined level by level.

use super::field::align_to;
use glam::Vec3;
use rayon::prelude::*;

/// No constraint.
pub(super) const FREE: u8 = 0;
/// The orientation follows `dir` and a lattice line runs through `pt`.
pub(super) const LINE: u8 = 1;
/// The orientation follows `dir` and `pt` is a lattice point (feature corner).
pub(super) const POINT: u8 = 2;

/// Stop coarsening below this many vertices.
const COARSEST_VERTS: usize = 64;
const MAX_LEVELS: usize = 32;

/// Hard feature constraint of one vertex.
#[derive(Clone, Copy, Default)]
pub(super) struct Constraint {
    /// Unit tangent direction the orientation field must follow (zero:
    /// orientation free, only allowed for POINT).
    pub dir: Vec3,
    /// Point the position lattice must pass through.
    pub pt: Vec3,
    pub kind: u8,
}

/// One level of the hierarchy: a weighted vertex graph in CSR form.
pub(super) struct Level {
    pub p: Vec<Vec3>,
    pub n: Vec<Vec3>,
    pub area: Vec<f32>,
    pub off: Vec<u32>,
    pub nbr: Vec<u32>,
    pub w: Vec<f32>,
    /// Graph colouring: vertices of one phase are pairwise non-adjacent and
    /// can be updated in parallel (Gauss–Seidel per colour).
    pub phases: Vec<Vec<u32>>,
    pub cons: Vec<Constraint>,
    /// Vertex of the next coarser level each vertex was merged into (empty
    /// on the coarsest level).
    pub parent: Vec<u32>,
}

impl Level {
    pub fn len(&self) -> usize {
        self.p.len()
    }

    #[inline]
    pub fn neighbors(&self, i: usize) -> impl Iterator<Item = (usize, f32)> + '_ {
        let r = self.off[i] as usize..self.off[i + 1] as usize;
        self.nbr[r.clone()]
            .iter()
            .zip(&self.w[r])
            .map(|(&j, &w)| (j as usize, w))
    }

    /// Creates a level from its vertices and directed, weighted edges
    /// (both directions must be present; duplicates are summed).
    pub fn new(
        p: Vec<Vec3>,
        n: Vec<Vec3>,
        area: Vec<f32>,
        cons: Vec<Constraint>,
        pairs: Vec<(u32, u32, f32)>,
    ) -> Level {
        let (off, nbr, w) = build_csr(p.len(), pairs);
        let phases = color_phases(&off, &nbr);
        Level {
            p,
            n,
            area,
            off,
            nbr,
            w,
            phases,
            cons,
            parent: Vec::new(),
        }
    }
}

/// Sorts directed edges into CSR form, merging duplicates and dropping loops.
fn build_csr(n: usize, mut pairs: Vec<(u32, u32, f32)>) -> (Vec<u32>, Vec<u32>, Vec<f32>) {
    pairs.par_sort_unstable_by_key(|&(a, b, _)| ((a as u64) << 32) | b as u64);
    let mut off = vec![0u32; n + 1];
    let mut nbr = Vec::with_capacity(pairs.len());
    let mut w: Vec<f32> = Vec::with_capacity(pairs.len());
    let mut last = None;
    for (a, b, wt) in pairs {
        if a == b {
            continue;
        }
        if last == Some((a, b)) {
            if let Some(x) = w.last_mut() {
                *x += wt;
            }
            continue;
        }
        last = Some((a, b));
        nbr.push(b);
        w.push(wt);
        off[a as usize + 1] += 1;
    }
    for i in 0..n {
        off[i + 1] += off[i];
    }
    (off, nbr, w)
}

/// Greedy graph colouring, returned as the vertex list of each colour.
fn color_phases(off: &[u32], nbr: &[u32]) -> Vec<Vec<u32>> {
    let n = off.len() - 1;
    let mut color = vec![u32::MAX; n];
    // stamp[c] == i: colour c is taken by a neighbour of vertex i.
    let mut stamp: Vec<usize> = Vec::new();
    let mut phases: Vec<Vec<u32>> = Vec::new();
    for i in 0..n {
        for &j in &nbr[off[i] as usize..off[i + 1] as usize] {
            let c = color[j as usize];
            if c != u32::MAX {
                stamp[c as usize] = i;
            }
        }
        let c = match (0..stamp.len()).find(|&c| stamp[c] != i) {
            Some(c) => c,
            None => {
                stamp.push(usize::MAX);
                phases.push(Vec::new());
                stamp.len() - 1
            }
        };
        color[i] = c as u32;
        phases[c].push(i as u32);
    }
    phases
}

/// Builds the coarser levels on top of `level0`.
pub(super) fn build(level0: Level) -> Vec<Level> {
    let mut levels = vec![level0];
    while levels.len() < MAX_LEVELS {
        let fine = &levels[levels.len() - 1];
        if fine.len() <= COARSEST_VERTS {
            break;
        }
        let (parent, coarse) = coarsen(fine);
        if coarse.len() as f32 > 0.9 * fine.len() as f32 {
            // Nothing left to pair (isolated vertices only).
            break;
        }
        let last = levels.len() - 1;
        levels[last].parent = parent;
        levels.push(coarse);
    }
    levels
}

/// Pairs vertices along edges, preferring similar normals and unbalanced
/// areas (score `(n_i·n_j)·max(A_i/A_j, A_j/A_i)` as in Instant Meshes).
fn coarsen(fine: &Level) -> (Vec<u32>, Level) {
    let n = fine.len();
    let mut cand: Vec<(f32, u32, u32)> = (0..n)
        .into_par_iter()
        .flat_map_iter(|i| {
            fine.neighbors(i)
                .filter(move |&(j, _)| j > i)
                .map(move |(j, _)| {
                    let dp = fine.n[i].dot(fine.n[j]);
                    let (ai, aj) = (fine.area[i].max(1e-30), fine.area[j].max(1e-30));
                    let ratio = if ai > aj { ai / aj } else { aj / ai };
                    (dp * ratio, i as u32, j as u32)
                })
        })
        .collect();
    cand.par_sort_unstable_by(|a, b| b.0.total_cmp(&a.0));

    let mut parent = vec![u32::MAX; n];
    let mut next = 0u32;
    for &(_, i, j) in &cand {
        let (i, j) = (i as usize, j as usize);
        if parent[i] == u32::MAX && parent[j] == u32::MAX {
            parent[i] = next;
            parent[j] = next;
            next += 1;
        }
    }
    for p in parent.iter_mut() {
        if *p == u32::MAX {
            *p = next;
            next += 1;
        }
    }
    let nc = next as usize;

    let mut area = vec![0.0f32; nc];
    let mut psum = vec![Vec3::ZERO; nc];
    let mut pavg = vec![Vec3::ZERO; nc];
    let mut count = vec![0u32; nc];
    let mut nsum = vec![Vec3::ZERO; nc];
    let mut nfirst = vec![Vec3::ZERO; nc];
    let mut cons = vec![Constraint::default(); nc];
    let mut cons_count = vec![0u32; nc];
    for i in 0..n {
        let c = parent[i] as usize;
        let a = fine.area[i];
        area[c] += a;
        psum[c] += fine.p[i] * a;
        pavg[c] += fine.p[i];
        nsum[c] += fine.n[i] * a;
        if count[c] == 0 {
            nfirst[c] = fine.n[i];
        }
        count[c] += 1;
        let fc = fine.cons[i];
        if fc.kind != FREE {
            let cc = &mut cons[c];
            if cc.kind == FREE {
                *cc = fc;
                cons_count[c] = 1;
            } else {
                if cc.dir == Vec3::ZERO {
                    cc.dir = fc.dir;
                } else if fc.dir != Vec3::ZERO {
                    cc.dir += align_to(cc.dir.normalize_or_zero(), fc.dir, fine.n[i]);
                }
                if fc.kind > cc.kind {
                    cc.kind = fc.kind;
                    cc.pt = fc.pt;
                    cons_count[c] = 1;
                } else if fc.kind == cc.kind {
                    let k = cons_count[c] as f32;
                    cc.pt = (cc.pt * k + fc.pt) / (k + 1.0);
                    cons_count[c] += 1;
                }
            }
        }
    }
    let p: Vec<Vec3> = (0..nc)
        .map(|c| {
            if area[c] > 0.0 {
                psum[c] / area[c]
            } else {
                pavg[c] / count[c] as f32
            }
        })
        .collect();
    let nrm: Vec<Vec3> = (0..nc)
        .map(|c| {
            let v = nsum[c].normalize_or_zero();
            if v == Vec3::ZERO { nfirst[c] } else { v }
        })
        .collect();
    for c in 0..nc {
        let cc = &mut cons[c];
        if cc.kind != FREE {
            // Points may leave the orientation free, lines need a direction.
            cc.dir = (cc.dir - nrm[c] * nrm[c].dot(cc.dir)).normalize_or_zero();
            if cc.dir == Vec3::ZERO && cc.kind == LINE {
                *cc = Constraint::default();
            }
        }
    }

    let pairs: Vec<(u32, u32, f32)> = (0..n)
        .into_par_iter()
        .flat_map_iter(|i| {
            let pi = parent[i];
            let parent = &parent;
            fine.neighbors(i).filter_map(move |(j, w)| {
                let pj = parent[j];
                (pi != pj).then_some((pi, pj, w))
            })
        })
        .collect();
    (parent, Level::new(p, nrm, area, cons, pairs))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn path_level(n: usize) -> Level {
        let p: Vec<Vec3> = (0..n).map(|i| Vec3::new(i as f32, 0.0, 0.0)).collect();
        let nrm = vec![Vec3::Z; n];
        let mut pairs = Vec::new();
        for i in 0..n - 1 {
            pairs.push((i as u32, i as u32 + 1, 1.0));
            pairs.push((i as u32 + 1, i as u32, 1.0));
        }
        Level::new(p, nrm, vec![1.0; n], vec![Constraint::default(); n], pairs)
    }

    #[test]
    fn colouring_separates_neighbours() {
        let lv = path_level(50);
        assert_eq!(lv.phases.len(), 2);
        for phase in &lv.phases {
            for &i in phase {
                for (j, _) in lv.neighbors(i as usize) {
                    assert!(!phase.contains(&(j as u32)));
                }
            }
        }
    }

    #[test]
    fn hierarchy_halves_each_level() {
        let levels = build(path_level(1000));
        assert!(levels.len() >= 4);
        for pair in levels.windows(2) {
            let (fine, coarse) = (&pair[0], &pair[1]);
            assert_eq!(fine.parent.len(), fine.len());
            assert!(fine.parent.iter().all(|&c| (c as usize) < coarse.len()));
            assert!(
                coarse.len() * 10 < fine.len() * 8,
                "{} -> {}",
                fine.len(),
                coarse.len()
            );
            let total: f32 = coarse.area.iter().sum();
            assert!((total - 1000.0).abs() < 1e-2, "area is preserved");
        }
    }
}
