//! Builders for the line / fill primitives drawn over the mesh (bounding
//! box, origin triad, ground grid, fitted planes and circles, markers).

use crate::camera::UpAxis;
use crate::geom::fitting::plane_basis;
use crate::mesh::Aabb;
use crate::render::LineVertex;
use glam::Vec3;

pub(crate) type Line = LineVertex;

pub(crate) fn push_line(out: &mut Vec<Line>, a: Vec3, b: Vec3, c: [f32; 4]) {
    out.push([a.x, a.y, a.z, c[0], c[1], c[2], c[3]]);
    out.push([b.x, b.y, b.z, c[0], c[1], c[2], c[3]]);
}

pub(crate) fn push_tri(out: &mut Vec<Line>, a: Vec3, b: Vec3, c: Vec3, col: [f32; 4]) {
    out.push([a.x, a.y, a.z, col[0], col[1], col[2], col[3]]);
    out.push([b.x, b.y, b.z, col[0], col[1], col[2], col[3]]);
    out.push([c.x, c.y, c.z, col[0], col[1], col[2], col[3]]);
}

pub(crate) fn bbox_lines(bb: &Aabb, c: [f32; 4]) -> Vec<Line> {
    let (min, max) = (bb.min, bb.max);
    let p = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
    let mut l = Vec::with_capacity(24);
    let edges = [
        (p(min.x, min.y, min.z), p(max.x, min.y, min.z)),
        (p(min.x, min.y, min.z), p(min.x, max.y, min.z)),
        (p(min.x, min.y, min.z), p(min.x, min.y, max.z)),
        (p(max.x, max.y, max.z), p(min.x, max.y, max.z)),
        (p(max.x, max.y, max.z), p(max.x, min.y, max.z)),
        (p(max.x, max.y, max.z), p(max.x, max.y, min.z)),
        (p(min.x, min.y, max.z), p(max.x, min.y, max.z)),
        (p(min.x, min.y, max.z), p(min.x, max.y, max.z)),
        (p(max.x, min.y, min.z), p(max.x, max.y, min.z)),
        (p(max.x, min.y, min.z), p(max.x, min.y, max.z)),
        (p(min.x, max.y, min.z), p(max.x, max.y, min.z)),
        (p(min.x, max.y, min.z), p(min.x, max.y, max.z)),
    ];
    for (a, b) in edges {
        push_line(&mut l, a, b, c);
    }
    l
}

pub(crate) const AXIS_X_COLOR: [f32; 4] = [0.93, 0.33, 0.36, 1.0];
pub(crate) const AXIS_Y_COLOR: [f32; 4] = [0.45, 0.85, 0.40, 1.0];
pub(crate) const AXIS_Z_COLOR: [f32; 4] = [0.36, 0.58, 1.0, 1.0];

pub(crate) fn triad_lines(len: f32) -> Vec<Line> {
    let mut l = Vec::with_capacity(6);
    push_line(&mut l, Vec3::ZERO, Vec3::X * len, AXIS_X_COLOR);
    push_line(&mut l, Vec3::ZERO, Vec3::Y * len, AXIS_Y_COLOR);
    push_line(&mut l, Vec3::ZERO, Vec3::Z * len, AXIS_Z_COLOR);
    l
}

pub(crate) fn marker_lines(p: Vec3, size: f32, c: [f32; 4]) -> Vec<Line> {
    let mut l = Vec::with_capacity(6);
    push_line(&mut l, p - Vec3::X * size, p + Vec3::X * size, c);
    push_line(&mut l, p - Vec3::Y * size, p + Vec3::Y * size, c);
    push_line(&mut l, p - Vec3::Z * size, p + Vec3::Z * size, c);
    l
}

pub(crate) fn plane_grid_lines(p: Vec3, n: Vec3, half: f32, div: u32, c: [f32; 4]) -> Vec<Line> {
    let (u, v) = plane_basis(n);
    let mut l = Vec::with_capacity((div as usize + 1) * 4);
    for i in 0..=div {
        let f = -half + (2.0 * half) * (i as f32 / div as f32);
        push_line(&mut l, p + u * f + v * (-half), p + u * f + v * half, c);
        push_line(&mut l, p + u * (-half) + v * f, p + u * half + v * f, c);
    }
    l
}

pub(crate) fn plane_fill(p: Vec3, n: Vec3, half: f32, c: [f32; 4]) -> Vec<Line> {
    let (u, v) = plane_basis(n);
    let a = p + u * (-half) + v * (-half);
    let b = p + u * half + v * (-half);
    let d = p + u * (-half) + v * half;
    let e = p + u * half + v * half;
    let mut out = Vec::with_capacity(6);
    push_tri(&mut out, a, b, e, c);
    push_tri(&mut out, a, e, d, c);
    out
}

pub(crate) fn circle_lines(c: Vec3, n: Vec3, r: f32, col: [f32; 4]) -> Vec<Line> {
    circle_lines_segments(c, n, r, col, 96, true)
}

pub(crate) fn circle_lines_segments(
    c: Vec3,
    n: Vec3,
    r: f32,
    col: [f32; 4],
    segs: usize,
    with_axis: bool,
) -> Vec<Line> {
    let (u, v) = plane_basis(n);
    let mut l = Vec::with_capacity(segs * 2 + 2);
    for i in 0..segs {
        let a0 = std::f32::consts::TAU * (i as f32 / segs as f32);
        let a1 = std::f32::consts::TAU * ((i + 1) as f32 / segs as f32);
        let p0 = c + u * (r * a0.cos()) + v * (r * a0.sin());
        let p1 = c + u * (r * a1.cos()) + v * (r * a1.sin());
        push_line(&mut l, p0, p1, col);
    }
    if with_axis {
        push_line(&mut l, c - n * (r * 1.4), c + n * (r * 1.4), col);
    }
    l
}

/// "Nice" grid step (1, 2 or 5 times a power of ten) so that roughly
/// `target_cells` cells span `extent`.
pub(crate) fn nice_step(extent: f32, target_cells: f32) -> f32 {
    let raw = (extent / target_cells.max(1.0)).max(1e-6);
    let mag = 10f32.powf(raw.log10().floor());
    let norm = raw / mag;
    let nice = if norm < 1.5 {
        1.0
    } else if norm < 3.5 {
        2.0
    } else if norm < 7.5 {
        5.0
    } else {
        10.0
    };
    nice * mag
}

/// Ground grid on the plane through the origin perpendicular to the up
/// axis. Every 10th line is emphasised and the two axes running through the
/// origin are colored; lines fade towards the rim.
pub(crate) fn ground_grid_lines(
    up: UpAxis,
    half_extent: f32,
    step: f32,
    minor: [f32; 4],
    major: [f32; 4],
) -> Vec<Line> {
    let n = ((half_extent / step).ceil() as i32).clamp(1, 400);
    let half = n as f32 * step;
    let (ax_a, ax_b, col_a, col_b) = match up {
        UpAxis::Y => (Vec3::X, Vec3::Z, AXIS_X_COLOR, AXIS_Z_COLOR),
        UpAxis::Z => (Vec3::X, Vec3::Y, AXIS_X_COLOR, AXIS_Y_COLOR),
    };
    let mut l = Vec::with_capacity((n as usize * 2 + 1) * 8);
    // Each line is split in two halves so the alpha can fade towards the rim.
    for i in -n..=n {
        let f = i as f32 * step;
        let is_major = i % 10 == 0;
        let base = if is_major { major } else { minor };
        for (dir, other, axis_col) in [(ax_a, ax_b, col_b), (ax_b, ax_a, col_a)] {
            let p = dir * f;
            let col = if i == 0 {
                [axis_col[0], axis_col[1], axis_col[2], 0.55]
            } else {
                base
            };
            let faint = [col[0], col[1], col[2], 0.0];
            let mid = p;
            let end_a = p - other * half;
            let end_b = p + other * half;
            l.push([
                end_a.x, end_a.y, end_a.z, faint[0], faint[1], faint[2], faint[3],
            ]);
            l.push([mid.x, mid.y, mid.z, col[0], col[1], col[2], col[3]]);
            l.push([mid.x, mid.y, mid.z, col[0], col[1], col[2], col[3]]);
            l.push([
                end_b.x, end_b.y, end_b.z, faint[0], faint[1], faint[2], faint[3],
            ]);
        }
    }
    l
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nice_step_picks_1_2_5_series() {
        assert!((nice_step(100.0, 10.0) - 10.0).abs() < 1e-6);
        assert!((nice_step(23.0, 10.0) - 2.0).abs() < 1e-6);
        assert!((nice_step(0.47, 10.0) - 0.05).abs() < 1e-6);
    }

    #[test]
    fn ground_grid_lies_in_plane() {
        for up in [UpAxis::Y, UpAxis::Z] {
            let lines = ground_grid_lines(up, 50.0, 5.0, [0.5; 4], [0.6; 4]);
            assert!(!lines.is_empty());
            let axis = up.vector();
            for v in &lines {
                let p = Vec3::new(v[0], v[1], v[2]);
                assert!(p.dot(axis).abs() < 1e-5);
            }
        }
    }
}
