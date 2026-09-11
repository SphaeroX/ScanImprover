//! Relationship inference ("regularisation") between the fitted surfaces.
//!
//! Scanned parts are designed with exact relations that the independent
//! fits only reproduce approximately. Plane normals and axes that are
//! parallel within the snap angle are merged into one direction, direction
//! sets that are perpendicular within the snap angle are made exactly
//! perpendicular, surfaces of revolution whose axes (nearly) coincide share
//! one axis, and spheres centred on such an axis are moved onto it. Every
//! changed surface is refitted with the snapped parameters held fixed.

use super::surface::{Freeze, Surface, refit};
use glam::DVec3;

/// One fitted surface with the points it was fitted to.
pub(super) struct FitSlot {
    pub surface: Surface,
    pub pts: Vec<DVec3>,
    /// Surface area of the group (larger groups dominate merged parameters).
    pub weight: f64,
}

/// What the relationship inference changed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Relations {
    /// Direction sets with at least two members (parallel planes / axes).
    pub parallel_sets: usize,
    /// Surfaces whose direction was changed by the snapping.
    pub snapped: usize,
    /// Pairs of direction sets made exactly perpendicular.
    pub perpendicular: usize,
    /// Sets of at least two surfaces of revolution sharing one axis.
    pub coaxial_sets: usize,
    /// Spheres moved onto an axis.
    pub centered_spheres: usize,
}

/// A set of parallel directions: (weighted direction sum, members as
/// (slot, sign), total weight).
type DirSet = (DVec3, Vec<(usize, f64)>, f64);

fn direction(s: &Surface) -> Option<DVec3> {
    match s {
        Surface::Plane { normal, .. } => Some(*normal),
        Surface::Cylinder { axis, .. } | Surface::Cone { axis, .. } => Some(*axis),
        _ => None,
    }
}

fn with_direction(s: &Surface, d: DVec3) -> Surface {
    match s.clone() {
        Surface::Plane { origin, .. } => Surface::Plane { origin, normal: d },
        Surface::Cylinder { origin, radius, .. } => Surface::Cylinder {
            origin,
            axis: d,
            radius,
        },
        Surface::Cone {
            origin,
            radius,
            half_angle,
            ..
        } => Surface::Cone {
            origin,
            axis: d,
            radius,
            half_angle,
        },
        other => other,
    }
}

/// Snaps the relations in place. `snap_angle` in radians (0 disables),
/// `coax_tol` is the largest axis distance for a shared axis.
pub(super) fn regularize(slots: &mut [FitSlot], snap_angle: f64, coax_tol: f64) -> Relations {
    let mut rel = Relations::default();
    if snap_angle <= 0.0 {
        return rel;
    }
    let cos_par = snap_angle.cos();
    let sin_perp = snap_angle.sin();

    // --- Parallel direction sets (largest surfaces first) ------------------
    let mut order: Vec<usize> = (0..slots.len())
        .filter(|&i| direction(&slots[i].surface).is_some())
        .collect();
    order.sort_by(|&a, &b| slots[b].weight.total_cmp(&slots[a].weight).then(a.cmp(&b)));
    let mut sets: Vec<DirSet> = Vec::new();
    for &i in &order {
        let d = direction(&slots[i].surface).unwrap();
        let w = slots[i].weight.max(1e-12);
        let best = sets
            .iter()
            .enumerate()
            .map(|(k, s)| (k, s.0.normalize().dot(d)))
            .filter(|(_, c)| c.abs() >= cos_par)
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()));
        match best {
            Some((k, c)) => {
                let sign = c.signum();
                sets[k].0 += d * sign * w;
                sets[k].1.push((i, sign));
                sets[k].2 += w;
            }
            None => sets.push((d * w, vec![(i, 1.0)], w)),
        }
    }
    sets.sort_by(|a, b| b.2.total_cmp(&a.2));

    // --- Perpendicular sets --------------------------------------------------
    let mut fixed: Vec<DVec3> = Vec::new();
    for set in &sets {
        let mut d = set.0.normalize();
        let near: Vec<DVec3> = fixed
            .iter()
            .copied()
            .filter(|f| f.dot(d).abs() <= sin_perp)
            .collect();
        // Orthonormal basis of the directions d must be perpendicular to.
        let mut basis: Vec<DVec3> = Vec::new();
        for f in &near {
            let mut q = *f;
            for b in &basis {
                q -= *b * q.dot(*b);
            }
            if q.length() > 1e-6 {
                basis.push(q.normalize());
            }
        }
        if basis.len() <= 2 && !basis.is_empty() {
            let mut e = d;
            for b in &basis {
                e -= *b * e.dot(*b);
            }
            if e.length() > 1e-9 {
                d = e.normalize();
                rel.perpendicular += near.len();
            }
        }
        fixed.push(d);
    }

    // --- Apply directions, refit with the direction held -------------------
    for (set, dir) in sets.iter().zip(&fixed) {
        if set.1.len() >= 2 {
            rel.parallel_sets += 1;
        }
        for &(i, sign) in &set.1 {
            let d = *dir * sign;
            let old = direction(&slots[i].surface).unwrap();
            if old.dot(d) < 1.0 - 1e-15 {
                rel.snapped += 1;
                let s = with_direction(&slots[i].surface, d);
                slots[i].surface = refit(
                    &s,
                    &slots[i].pts,
                    Freeze {
                        dir: true,
                        pos: false,
                    },
                );
            }
        }
    }

    // --- Shared axes -----------------------------------------------------------
    // Axis lines in a direction set are compared by their trace in the plane
    // perpendicular to the direction through the world origin.
    let mut axes: Vec<(DVec3, DVec3)> = Vec::new();
    for (set, dir) in sets.iter().zip(&fixed) {
        let revs: Vec<usize> = set
            .1
            .iter()
            .map(|m| m.0)
            .filter(|&i| slots[i].surface.axis().is_some())
            .collect();
        // (trace sum, members, weight)
        let mut coax: Vec<(DVec3, Vec<usize>, f64)> = Vec::new();
        for i in revs {
            let (o, _) = slots[i].surface.axis().unwrap();
            let q = o - *dir * o.dot(*dir);
            let w = slots[i].weight.max(1e-12);
            match coax
                .iter_mut()
                .find(|c| (c.0 / c.2 - q).length() <= coax_tol)
            {
                Some(c) => {
                    c.0 += q * w;
                    c.1.push(i);
                    c.2 += w;
                }
                None => coax.push((q * w, vec![i], w)),
            }
        }
        for (sum, members, w) in coax {
            let q = sum / w;
            axes.push((q, *dir));
            if members.len() < 2 {
                continue;
            }
            rel.coaxial_sets += 1;
            for i in members {
                let (o, a) = slots[i].surface.axis().unwrap();
                let moved = q + *dir * o.dot(*dir);
                let s = match slots[i].surface.clone() {
                    Surface::Cylinder { radius, .. } => Surface::Cylinder {
                        origin: moved,
                        axis: a,
                        radius,
                    },
                    Surface::Cone {
                        radius, half_angle, ..
                    } => Surface::Cone {
                        origin: moved,
                        axis: a,
                        radius,
                        half_angle,
                    },
                    other => other,
                };
                slots[i].surface = refit(
                    &s,
                    &slots[i].pts,
                    Freeze {
                        dir: true,
                        pos: true,
                    },
                );
            }
        }
    }
    for slot in slots.iter_mut() {
        let Surface::Sphere { center, radius } = slot.surface else {
            continue;
        };
        let near = axes
            .iter()
            .map(|&(q, d)| {
                let on = q + d * (center - q).dot(d);
                (on, (center - on).length())
            })
            .filter(|&(_, dist)| dist > 0.0 && dist <= coax_tol)
            .min_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((on, _)) = near {
            rel.centered_spheres += 1;
            let s = Surface::Sphere { center: on, radius };
            slot.surface = refit(
                &s,
                &slot.pts,
                Freeze {
                    dir: true,
                    pos: true,
                },
            );
        }
    }
    rel
}
