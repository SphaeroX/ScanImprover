use crate::geom::fitting::{fit_plane, plane_basis};
use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::HashMap;
use std::sync::Arc;

/// A freeform surface fitted onto a selection, kept as a persistent object
/// (like fitted planes / circles). Stores the (sub-sampled) source points so
/// the surface can be rebuilt when the user changes the overshoot or the
/// extension mode.
#[derive(Clone)]
pub struct FittedFreeform {
    pub id: u64,
    pub name: String,
    pub visible: bool,
    pub color: [f32; 4],
    /// Sub-sampled selection points the surface was fitted through.
    pub source_points: Arc<Vec<[f32; 3]>>,
    pub params: FreeformParams,
    /// Tessellated surface; `None` while the (async) fit is running.
    pub surface: Option<Mesh>,
    /// World-space boundary edges of the surface (for the viewport outline).
    pub boundary: Vec<([f32; 3], [f32; 3])>,
    pub rms: f32,
    pub max_dev: f32,
    pub fold_ratio: f32,
    pub point_count: usize,
    /// Set when parameters changed while a fit job was already running.
    pub refit_pending: bool,
    /// Whether the deviation heatmap is rendered on this surface.
    pub heat_on: bool,
    /// Max deviation threshold in mm (where the heatmap turns full red).
    pub heat_max: f32,
    /// Color gradient style used for the heatmap.
    pub heat_gradient: FreeformGradient,
    /// Per-vertex deviation distances (in mm) from the fitted surface to the scan mesh.
    pub heat: Option<Vec<f32>>,
    /// Cached percentage of vertices within the heat_max tolerance.
    pub in_tolerance_pct: f32,
}

/// Color gradient style used for the freeform deviation heatmap.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FreeformGradient {
    /// Green (0.0 mm / close to scan) -> Yellow -> Orange -> Red (>= max deviation). CAD inspection style.
    TrafficLight,
    /// Blue (0.0 mm) -> Cyan -> Green -> Yellow -> Red (>= max deviation). Spectrum style.
    Spectrum,
}

impl FreeformGradient {
    #[allow(dead_code)]
    pub fn label(self) -> &'static str {
        match self {
            FreeformGradient::TrafficLight => "Traffic Light (Green → Red)",
            FreeformGradient::Spectrum => "Spectrum (Blue → Red)",
        }
    }
}

/// How the surface continues into the overshoot region beyond the selection.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FreeformExtend {
    /// Continue the local slope only (tangent extension).
    Slope,
    /// Continue the local slope *and* curvature (organic extension).
    Curvature,
}

impl FreeformExtend {
    pub fn label(self) -> &'static str {
        match self {
            FreeformExtend::Slope => "Slope",
            FreeformExtend::Curvature => "Curvature",
        }
    }
}

/// Fitting parameters of a freeform surface.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct FreeformParams {
    /// How far the surface extends beyond the selection boundary, in mm.
    pub overshoot_mm: f32,
    /// Extension behavior past the boundary (slope vs. curvature).
    pub extend: FreeformExtend,
    /// Grid resolution: number of cells along the longer side of the fit.
    pub resolution: u32,
    /// Laplacian smoothing passes over the fit grid (0 = raw fit).
    pub smoothness: u32,
}

impl FreeformParams {
    pub fn default_for(extent: f32) -> FreeformParams {
        FreeformParams {
            overshoot_mm: (extent * 0.03).clamp(0.2, 10.0),
            // Slope is the safe default: curvature continuation can spike on
            // noisy or thin selections (it is still available per surface).
            extend: FreeformExtend::Slope,
            resolution: 120,
            smoothness: 2,
        }
    }
}

/// Result of a freeform fit.
pub struct FreeformFitData {
    /// The tessellated surface, oriented along the fitted PCA normal.
    pub surface: Mesh,
    /// RMS deviation of the source points from the surface, in mm.
    pub rms: f32,
    /// Max deviation of the source points from the surface, in mm.
    pub max_dev: f32,
    /// Fold heuristic in 0..1: large values mean the selection wraps around
    /// (is not single-valued over the fit plane) and the fit loses accuracy.
    pub fold_ratio: f32,
    /// Number of source points used.
    pub point_count: usize,
}

/// Number of nearest neighbors used for the moving least squares fit.
const MLS_K: usize = 24;
/// Minimum points required for a fit.
const MIN_POINTS: usize = 12;
/// Interior holes up to this multiple of the median point spacing are bridged.
const GAP_FILL_SPACING: f32 = 3.0;
const MIN_RES: u32 = 16;
const MAX_RES: u32 = 320;

/// Evaluated fit grid in the plane frame: node heights, validity and frame.
/// Shared between the display fit (trimmed mesh) and the export grid
/// (full rectangle for the CAD B-spline surface).
struct FitGrid {
    origin: Vec3,
    n: Vec3,
    u: Vec3,
    v: Vec3,
    x0: f32,
    y0: f32,
    gw: f32,
    gh: f32,
    nx: usize,
    ny: usize,
    /// Node heights, row-major: `heights[j * (nx + 1) + i]`.
    heights: Vec<f32>,
    valid: Vec<bool>,
    spacing: f32,
    h_range: f32,
}

/// Context passed into the per-node MLS evaluation.
struct EvalCtx {
    h_range: f32,
    extent: f32,
    h_limit: f32,
}

/// Projects the points, evaluates the MLS height field at every grid node and
/// applies the configured smoothing passes. With `all_valid` the grid covers
/// the full (overshoot) rectangle — used for the CAD export patch.
fn build_fit_grid(
    points: &[[f32; 3]],
    params: &FreeformParams,
    all_valid: bool,
) -> Result<(FitGrid, Vec<(f32, f32)>, Vec<f32>, Hash2D), String> {
    if points.len() < MIN_POINTS {
        return Err(format!(
            "Selection too small for a freeform surface (needs >= {MIN_POINTS} points)."
        ));
    }
    let plane =
        fit_plane(points).ok_or_else(|| "Degenerate selection: no fit plane.".to_string())?;
    let n = plane.normal.normalize();
    let (u, v) = plane_basis(n);
    let origin = plane.point;

    // Project into the plane frame: (x, y) in-plane, h along the normal.
    let mut xy: Vec<(f32, f32)> = Vec::with_capacity(points.len());
    let mut hh: Vec<f32> = Vec::with_capacity(points.len());
    let mut h_min = f32::MAX;
    let mut h_max = f32::MIN;
    for p in points {
        let d = Vec3::from(*p) - origin;
        let x = d.dot(u);
        let y = d.dot(v);
        let h = d.dot(n);
        xy.push((x, y));
        hh.push(h);
        h_min = h_min.min(h);
        h_max = h_max.max(h);
    }
    let h_range = (h_max - h_min).max(1e-9);

    let (mut min_x, mut max_x) = (f32::MAX, f32::MIN);
    let (mut min_y, mut max_y) = (f32::MAX, f32::MIN);
    for &(x, y) in &xy {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    let extent_x = max_x - min_x;
    let extent_y = max_y - min_y;
    let extent = extent_x.max(extent_y);
    if extent < 1e-9 {
        return Err("Degenerate selection: zero in-plane extent.".to_string());
    }

    // Median spacing of distinct nearby 2D points (vertex scale of the scan).
    let spacing = median_spacing(&xy, extent).ok_or_else(|| {
        "Could not measure point spacing; selection may be too sparse.".to_string()
    })?;

    // Hash grid for 2D neighborhood queries.
    let hash = Hash2D::new(&xy, spacing * 2.0);

    let ov = params.overshoot_mm.max(0.0);
    let res = params.resolution.clamp(MIN_RES, MAX_RES) as usize;
    let (nx, ny) = if extent_x >= extent_y {
        let ny = ((extent_y / extent).max(1e-3) * res as f32)
            .round()
            .max(4.0) as usize;
        (res.max(4), ny)
    } else {
        let nx = ((extent_x / extent).max(1e-3) * res as f32)
            .round()
            .max(4.0) as usize;
        (nx, res.max(4))
    };

    let x0 = min_x - ov;
    let x1 = max_x + ov;
    let y0 = min_y - ov;
    let y1 = max_y + ov;
    let gw = (x1 - x0) / nx as f32;
    let gh = (y1 - y0) / ny as f32;

    // Every grid node lies within sqrt(2)*ov of the data bbox, so a ring
    // search capped at this many rings always finds data.
    let max_rings = (((ov * std::f32::consts::SQRT_2) / (spacing * 2.0)).ceil() as i32 + 8).min(200);
    let valid_dist = (GAP_FILL_SPACING * spacing).max(ov);

    // Evaluate the MLS height field at every grid node.
    let n_nodes = (nx + 1) * (ny + 1);
    let quad = params.extend == FreeformExtend::Curvature;
    let ctx = EvalCtx {
        h_range,
        extent,
        h_limit: 2.0 * h_range + 2.0 * ov + 1e-6,
    };

    let nodes: Vec<(f32, bool)> = (0..n_nodes)
        .into_par_iter()
        .map_init(
            || Vec::with_capacity(MLS_K * 6),
            |buf, idx| {
                let i = idx % (nx + 1);
                let j = idx / (nx + 1);
                let x = x0 + i as f32 * gw;
                let y = y0 + j as f32 * gh;
                buf.clear();
                let d_min2 = hash.query_k(x, y, MLS_K, max_rings, buf);
                let valid = all_valid || (d_min2 <= valid_dist * valid_dist && !buf.is_empty());
                let h = if buf.is_empty() {
                    0.0
                } else {
                    eval_node(x, y, buf, &xy, &hh, quad, &ctx)
                };
                (h, valid)
            },
        )
        .collect();
    let mut heights = vec![0.0f32; n_nodes];
    let mut valid = vec![false; n_nodes];
    for (idx, (h, val)) in nodes.iter().enumerate() {
        heights[idx] = *h;
        valid[idx] = *val;
    }

    // QuickSurface-style smoothing: Laplacian passes over the fit grid.
    for _ in 0..params.smoothness {
        smooth_pass(&mut heights, &valid, nx, ny);
    }

    Ok((
        FitGrid {
            origin,
            n,
            u,
            v,
            x0,
            y0,
            gw,
            gh,
            nx,
            ny,
            heights,
            valid,
            spacing,
            h_range,
        },
        xy,
        hh,
        hash,
    ))
}

/// One Laplacian smoothing pass over the height grid: each valid node moves
/// halfway toward the average of its valid neighbors. Invalid nodes stay put
/// and are skipped as neighbors, so boundaries are preserved.
fn smooth_pass(heights: &mut [f32], valid: &[bool], nx: usize, ny: usize) {
    let w = nx + 1;
    let mut next = heights.to_vec();
    for j in 0..=ny {
        for i in 0..=nx {
            let idx = j * w + i;
            if !valid[idx] {
                continue;
            }
            let mut sum = 0.0f32;
            let mut cnt = 0usize;
            if i > 0 && valid[idx - 1] {
                sum += heights[idx - 1];
                cnt += 1;
            }
            if i < nx && valid[idx + 1] {
                sum += heights[idx + 1];
                cnt += 1;
            }
            if j > 0 && valid[idx - w] {
                sum += heights[idx - w];
                cnt += 1;
            }
            if j < ny && valid[idx + w] {
                sum += heights[idx + w];
                cnt += 1;
            }
            if cnt > 0 {
                next[idx] = heights[idx] + 0.5 * (sum / cnt as f32 - heights[idx]);
            }
        }
    }
    heights.copy_from_slice(&next);
}

/// Fits a smooth freeform surface through the given points (e.g. the corners
/// of a selected triangle patch) using a moving least squares height field
/// over the best-fitting (PCA) plane.
///
/// The surface covers the selection plus an `overshoot_mm` margin on all
/// sides. Past the selection boundary the surface continues either the local
/// slope (tangent) or the local curvature of the data, which makes the
/// patch easy to trim against neighboring geometry in CAD (QuickSurface
/// style overshoot).
pub fn fit_freeform(points: &[[f32; 3]], params: &FreeformParams) -> Result<FreeformFitData, String> {
    let (grid, xy, hh, hash) = build_fit_grid(points, params, false)?;
    let FitGrid {
        origin,
        n,
        u,
        v,
        x0,
        y0,
        gw,
        gh,
        nx,
        ny,
        heights,
        valid,
        spacing,
        h_range,
    } = grid;
    let n_nodes = (nx + 1) * (ny + 1);

    // Triangulate cells whose four corners are all valid.
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(n_nodes);
    let mut indices: Vec<u32> = Vec::new();
    let mut node_idx = vec![u32::MAX; n_nodes];
    for idx in 0..n_nodes {
        if valid[idx] {
            node_idx[idx] = positions.len() as u32;
            let i = idx % (nx + 1);
            let j = idx / (nx + 1);
            let p = origin
                + u * (x0 + i as f32 * gw)
                + v * (y0 + j as f32 * gh)
                + n * heights[idx];
            positions.push(p.to_array());
        }
    }
    for j in 0..ny {
        for i in 0..nx {
            let a = j * (nx + 1) + i;
            let b = a + 1;
            let c = a + (nx + 1);
            let d = c + 1;
            if valid[a] && valid[b] && valid[c] && valid[d] {
                // CCW in (x, y) => triangle normal along +n (since u x v = n).
                indices.push(node_idx[a]);
                indices.push(node_idx[b]);
                indices.push(node_idx[d]);
                indices.push(node_idx[a]);
                indices.push(node_idx[d]);
                indices.push(node_idx[c]);
            }
        }
    }
    if indices.is_empty() {
        return Err(
            "Freeform fit produced no surface. Try a larger selection or coarser resolution."
                .to_string(),
        );
    }
    let surface = Mesh::from_indexed(positions, indices);

    // Deviation stats: bilinearly interpolate the grid at each source point.
    let grid_for_stats: Vec<(f32, f32, f32, bool)> = (0..n_nodes)
        .map(|idx| {
            let i = idx % (nx + 1);
            let j = idx / (nx + 1);
            (
                x0 + i as f32 * gw,
                y0 + j as f32 * gh,
                heights[idx],
                valid[idx],
            )
        })
        .collect();
    let (rms, max_dev) = deviation_stats(&xy, &hh, &grid_for_stats, nx, ny, x0, y0, gw, gh);
    let fold_ratio = fold_heuristic(&xy, &hh, &hash, spacing, h_range);

    Ok(FreeformFitData {
        surface,
        rms,
        max_dev,
        fold_ratio,
        point_count: points.len(),
    })
}

/// The full-rectangle export grid (all nodes valid) in the fit plane frame.
/// Used to build the 4-sided B-spline patch for CAD (STEP) export.
pub struct FreeformGrid {
    pub nx: usize,
    pub ny: usize,
    /// Node heights, row-major: `heights[j * (nx + 1) + i]`.
    pub heights: Vec<f32>,
    pub origin: Vec3,
    pub u: Vec3,
    pub v: Vec3,
    pub normal: Vec3,
    /// Plane coordinates of node (0, 0) and node spacing.
    pub x0: f32,
    pub y0: f32,
    pub gw: f32,
    pub gh: f32,
}

/// Evaluates the freeform fit over the full overshoot rectangle (all grid
/// nodes, including the trimmed corners), with the same smoothing settings.
/// This is the 4-sided patch QuickSurface-style CAD workflows expect.
pub fn fit_freeform_grid(
    points: &[[f32; 3]],
    params: &FreeformParams,
) -> Result<FreeformGrid, String> {
    let (grid, _xy, _hh, _hash) = build_fit_grid(points, params, true)?;
    Ok(FreeformGrid {
        nx: grid.nx,
        ny: grid.ny,
        heights: grid.heights,
        origin: grid.origin,
        u: grid.u,
        v: grid.v,
        normal: grid.n,
        x0: grid.x0,
        y0: grid.y0,
        gw: grid.gw,
        gh: grid.gh,
    })
}

/// Weighted polynomial evaluation at one grid node. `neigh` holds
/// (dist², point index) sorted by distance.
///
/// Anti-spike strategy (QuickSurface-like robustness): the quadratic fit is
/// only accepted when its deviation from the linear fit stays within a
/// geometrically plausible bend for the extrapolation distance; otherwise the
/// node falls back to the linear (slope) continuation. Degenerate
/// neighborhoods (thin scan strips, collinear points) produce wild
/// quadratic extrapolations, but sane linear ones — so spikes are clamped
/// while genuine curvature passes through.
fn eval_node(
    qx: f32,
    qy: f32,
    neigh: &[(f32, usize)],
    xy: &[(f32, f32)],
    hh: &[f32],
    quadratic: bool,
    ctx: &EvalCtx,
) -> f32 {
    if neigh.is_empty() {
        return 0.0;
    }
    let d_min = neigh[0].0.sqrt();
    let d_k = neigh.last().map(|t| t.0.sqrt()).unwrap_or(0.0);
    let sigma = (0.65 * d_k).max(1e-7);
    let inv_2s2 = 1.0f64 / (2.0 * sigma as f64 * sigma as f64);

    // Weighted mean as the ultimate fallback.
    let (mut w_sum, mut wh_sum) = (0.0f64, 0.0f64);
    for &(d2, i) in neigh {
        let w = (-(d2 as f64) * inv_2s2).exp();
        w_sum += w;
        wh_sum += w * hh[i] as f64;
    }
    let h_mean = (wh_sum / w_sum.max(1e-12)) as f32;

    // Plausible bend budget: curvature strong enough to produce the whole
    // height range over half the extent, acting over the extrapolation
    // distance (interior nodes use a fraction of the neighborhood radius).
    let clamp_scale = d_min.max(0.35 * d_k).max(1e-9);
    let max_bend =
        8.0 * ctx.h_range * (clamp_scale / ctx.extent).powi(2) + 0.01 * ctx.h_range + 1e-6;

    let build = |order: usize| -> Option<f64> {
        let mut a = [[0.0f64; 6]; 6];
        let mut b = [0.0f64; 6];
        for &(d2, i) in neigh {
            let w = (-(d2 as f64) * inv_2s2).exp();
            let dx = (xy[i].0 - qx) as f64;
            let dy = (xy[i].1 - qy) as f64;
            let phi = [1.0, dx, dy, dx * dx, dx * dy, dy * dy];
            for m in 0..order {
                b[m] += w * phi[m] * hh[i] as f64;
                for k in m..order {
                    a[m][k] += w * phi[m] * phi[k];
                }
            }
        }
        for m in 0..order {
            for k in 0..m {
                a[m][k] = a[k][m];
            }
            if m > 0 {
                a[m][m] += a[m][m].abs() * 1e-9 + 1e-12;
            }
        }
        solve_dense(&mut a, &mut b, order).map(|x| x[0])
    };

    let sane = |h: f64| h.is_finite() && h.abs() <= ctx.h_limit as f64;

    let lin = build(3).filter(|h| sane(*h));

    if quadratic && neigh.len() >= 8 {
        if let Some(hq) = build(6).filter(|h| sane(*h)) {
            let base = lin.unwrap_or(h_mean as f64);
            if (hq - base).abs() <= max_bend as f64 {
                return hq as f32;
            }
        }
    }
    if let Some(hl) = lin {
        return hl as f32;
    }
    h_mean.clamp(-ctx.h_limit, ctx.h_limit)
}

/// Gaussian elimination with partial pivoting on the leading `n`x`n` block.
fn solve_dense(a: &mut [[f64; 6]; 6], b: &mut [f64; 6], n: usize) -> Option<[f64; 6]> {
    for col in 0..n {
        let mut piv = col;
        for row in col + 1..n {
            if a[row][col].abs() > a[piv][col].abs() {
                piv = row;
            }
        }
        if a[piv][col].abs() < 1e-14 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        let mcol = a[col];
        for row in 0..n {
            if row == col {
                continue;
            }
            let f = a[row][col] / mcol[col];
            if f == 0.0 {
                continue;
            }
            for k in col..n {
                a[row][k] -= f * mcol[k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0f64; 6];
    for i in 0..n {
        if a[i][i].abs() < 1e-14 {
            return None;
        }
        x[i] = b[i] / a[i][i];
    }
    Some(x)
}

/// Uniform hash grid over 2D points for k-nearest-neighbor queries.
struct Hash2D {
    inv_cell: f32,
    cols: i32,
    rows: i32,
    min: (f32, f32),
    buckets: Vec<Vec<u32>>,
    pts: Vec<(f32, f32)>,
}

impl Hash2D {
    fn new(pts: &[(f32, f32)], cell: f32) -> Hash2D {
        let cell = cell.max(1e-9);
        let (mut min_x, mut min_y) = (f32::MAX, f32::MAX);
        let (mut max_x, mut max_y) = (f32::MIN, f32::MIN);
        for &(x, y) in pts {
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        }
        let span = (max_x - min_x).max(max_y - min_y).max(cell);
        let cols = (span / cell).ceil().max(1.0) as i32 + 1;
        let rows = cols;
        let mut buckets = vec![Vec::new(); (cols as usize) * (rows as usize)];
        for (i, &(x, y)) in pts.iter().enumerate() {
            let cx = (((x - min_x) / cell).floor() as i32).clamp(0, cols - 1);
            let cy = (((y - min_y) / cell).floor() as i32).clamp(0, rows - 1);
            buckets[(cy * cols + cx) as usize].push(i as u32);
        }
        Hash2D {
            inv_cell: 1.0 / cell,
            cols,
            rows,
            min: (min_x, min_y),
            buckets,
            pts: pts.to_vec(),
        }
    }

    /// Expanding square-ring search. Fills `out` with (dist², point index)
    /// pairs, the `k` nearest found, sorted by distance. Returns the squared
    /// distance to the nearest point seen (f32::MAX if none).
    fn query_k(&self, x: f32, y: f32, k: usize, max_rings: i32, out: &mut Vec<(f32, usize)>) -> f32 {
        let cx = (((x - self.min.0) * self.inv_cell).floor() as i32)
            .clamp(-1, self.cols);
        let cy = (((y - self.min.1) * self.inv_cell).floor() as i32)
            .clamp(-1, self.rows);
        let mut d_min2 = f32::MAX;
        let cap = max_rings.min(200);
        let mut r = 0i32;
        while r <= cap {
            if r == 0 {
                self.scan_cell(cx, cy, x, y, out, &mut d_min2);
            } else {
                // Perimeter of the square ring at Chebyshev distance r.
                for dx in -r..=r {
                    self.scan_cell(cx + dx, cy - r, x, y, out, &mut d_min2);
                    self.scan_cell(cx + dx, cy + r, x, y, out, &mut d_min2);
                }
                for dy in -r + 1..=r - 1 {
                    self.scan_cell(cx - r, cy + dy, x, y, out, &mut d_min2);
                    self.scan_cell(cx + r, cy + dy, x, y, out, &mut d_min2);
                }
            }
            // One extra ring after reaching k keeps the k-NN (and d_min)
            // accurate enough despite the Chebyshev/Euclid mismatch.
            if out.len() >= k && r >= 2 {
                break;
            }
            r += 1;
        }
        out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        out.truncate(k);
        d_min2
    }

    fn scan_cell(
        &self,
        cx: i32,
        cy: i32,
        x: f32,
        y: f32,
        out: &mut Vec<(f32, usize)>,
        d_min2: &mut f32,
    ) {
        if cx < 0 || cy < 0 || cx >= self.cols || cy >= self.rows {
            return;
        }
        for &pi in &self.buckets[(cy * self.cols + cx) as usize] {
            let (px, py) = self.pts[pi as usize];
            let dx = px - x;
            let dy = py - y;
            let d2 = dx * dx + dy * dy;
            *d_min2 = d_min2.min(d2);
            out.push((d2, pi as usize));
        }
    }
}

/// Median distance from a sample of points to the nearest *distinct* other
/// point in 2D. Shared triangle corners (identical positions) are skipped so
/// welded duplicate vertices don't collapse the estimate to zero.
fn median_spacing(xy: &[(f32, f32)], extent: f32) -> Option<f32> {
    let hash = Hash2D::new(xy, extent.max(1e-9) / 64.0);
    let step = (xy.len() / 1024).max(1);
    let eps2 = (extent * 1e-7).max(1e-12);
    let eps2 = eps2 * eps2;
    let mut ds: Vec<f32> = xy
        .iter()
        .step_by(step)
        .filter_map(|&(x, y)| {
            let mut buf = Vec::with_capacity(256);
            let _ = hash.query_k(x, y, 16, 3, &mut buf);
            buf.iter()
                .find(|&&(d2, _)| d2 > eps2)
                .map(|&(d2, _)| d2.sqrt())
        })
        .collect();
    if ds.is_empty() {
        return None;
    }
    ds.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Some(ds[ds.len() / 2].max(1e-9))
}

/// RMS / max deviation of the source points against the bilinearly
/// interpolated surface grid.
fn deviation_stats(
    xy: &[(f32, f32)],
    hh: &[f32],
    nodes: &[(f32, f32, f32, bool)],
    nx: usize,
    ny: usize,
    x0: f32,
    y0: f32,
    gw: f32,
    gh: f32,
) -> (f32, f32) {
    let step = (xy.len() / 4096).max(1);
    let mut ss = 0.0f64;
    let mut max_dev = 0.0f32;
    let mut count = 0usize;
    for (i, &(x, y)) in xy.iter().enumerate().step_by(step) {
        let fx = (x - x0) / gw;
        let fy = (y - y0) / gh;
        let cx = (fx.floor() as i64).clamp(0, nx as i64 - 1) as usize;
        let cy = (fy.floor() as i64).clamp(0, ny as i64 - 1) as usize;
        let tx = (fx - cx as f32).clamp(0.0, 1.0);
        let ty = (fy - cy as f32).clamp(0.0, 1.0);
        let a = cy * (nx + 1) + cx;
        let b = a + 1;
        let c = a + (nx + 1);
        let d = c + 1;
        if !nodes[a].3 || !nodes[b].3 || !nodes[c].3 || !nodes[d].3 {
            continue;
        }
        let h00 = nodes[a].2;
        let h10 = nodes[b].2;
        let h01 = nodes[c].2;
        let h11 = nodes[d].2;
        let h = h00 * (1.0 - tx) * (1.0 - ty)
            + h10 * tx * (1.0 - ty)
            + h01 * (1.0 - tx) * ty
            + h11 * tx * ty;
        let dev = hh[i] - h;
        ss += (dev * dev) as f64;
        max_dev = max_dev.max(dev.abs());
        count += 1;
    }
    let rms = if count > 0 {
        ((ss / count as f64) as f32).sqrt()
    } else {
        0.0
    };
    (rms, max_dev)
}

/// Heuristic for selections that wrap around (are not single-valued over the
/// fit plane): samples points and measures the height spread of their close
/// 2D neighborhoods relative to the overall height range.
fn fold_heuristic(
    xy: &[(f32, f32)],
    hh: &[f32],
    hash: &Hash2D,
    spacing: f32,
    h_range: f32,
) -> f32 {
    let step = (xy.len() / 512).max(1);
    let radius = spacing * 2.5;
    let mut spreads: Vec<f32> = xy
        .iter()
        .step_by(step)
        .filter_map(|&(x, y)| {
            let mut buf = Vec::with_capacity(MLS_K * 6);
            let _ = hash.query_k(x, y, 24, 4, &mut buf);
            if buf.len() < 3 {
                return None;
            }
            let mut mn = f32::MAX;
            let mut mx = f32::MIN;
            for &(d2, j) in buf.iter() {
                if d2.sqrt() <= radius {
                    mn = mn.min(hh[j]);
                    mx = mx.max(hh[j]);
                }
            }
            if mx < mn {
                None
            } else {
                Some(mx - mn)
            }
        })
        .collect();
    if spreads.is_empty() {
        return 0.0;
    }
    spreads.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = spreads[spreads.len() / 2];
    (med / h_range.max(1e-9)).min(1.0)
}

/// Boundary edges (edges belonging to exactly one triangle) of a mesh, as
/// world-space line segments. Used to outline freeform patches in the
/// viewport.
pub fn boundary_segments(mesh: &Mesh) -> Vec<([f32; 3], [f32; 3])> {
    let mut count: HashMap<(u32, u32), u32> =
        HashMap::with_capacity(mesh.indices.len() * 3 / 2);
    for t in 0..mesh.triangle_count() {
        let i0 = mesh.indices[3 * t];
        let i1 = mesh.indices[3 * t + 1];
        let i2 = mesh.indices[3 * t + 2];
        for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            *count.entry(key).or_insert(0) += 1;
        }
    }
    let mut out = Vec::new();
    for t in 0..mesh.triangle_count() {
        let i0 = mesh.indices[3 * t];
        let i1 = mesh.indices[3 * t + 1];
        let i2 = mesh.indices[3 * t + 2];
        for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            if count[&key] == 1 {
                out.push((mesh.positions[a as usize], mesh.positions[b as usize]));
            }
        }
    }
    out
}

/// Computes per-vertex distance (in mm) from the fitted freeform surface to the scan mesh BVH.
/// Returns (heat_distances, mean_deviation, max_deviation).
pub fn compute_freeform_deviation(surface: &Mesh, bvh: &crate::geom::bvh::Bvh) -> (Vec<f32>, f32, f32) {
    if surface.positions.is_empty() {
        return (Vec::new(), 0.0, 0.0);
    }
    let heat: Vec<f32> = surface
        .positions
        .par_iter()
        .map(|&p| bvh.closest_distance(Vec3::from(p)))
        .collect();

    let mut max_dev = 0.0f32;
    let mut sum_dev = 0.0f64;
    for &d in &heat {
        if d > max_dev {
            max_dev = d;
        }
        sum_dev += d as f64;
    }
    let mean_dev = (sum_dev / heat.len() as f64) as f32;
    (heat, mean_dev, max_dev)
}

/// Calculates the percentage of points in `heat` that are <= `tolerance`.
pub fn calculate_in_tolerance_pct(heat: &[f32], tolerance: f32) -> f32 {
    if heat.is_empty() || tolerance <= 0.0 {
        return 0.0;
    }
    let within = heat.iter().filter(|&&d| d <= tolerance).count();
    (within as f32 / heat.len() as f32) * 100.0
}

/// Maps a distance measurement (mm) to an RGBA color based on the selected gradient and max deviation threshold.
pub fn freeform_vertex_color(
    dist: f32,
    heat_max: f32,
    gradient: FreeformGradient,
    alpha: f32,
) -> [f32; 4] {
    let t = if heat_max > 1e-7 {
        (dist / heat_max).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let rgb = match gradient {
        FreeformGradient::TrafficLight => {
            // Green (0.0) -> Yellow (0.5) -> Red (1.0)
            if t < 0.5 {
                let f = t / 0.5;
                [
                    0.1 + f * 0.9,
                    0.9 - f * 0.05,
                    0.2 - f * 0.2,
                ]
            } else {
                let f = (t - 0.5) / 0.5;
                [
                    1.0,
                    0.85 - f * 0.80,
                    f * 0.05,
                ]
            }
        }
        FreeformGradient::Spectrum => {
            // Blue (0.0) -> Cyan (0.25) -> Green (0.50) -> Yellow (0.75) -> Red (1.0)
            if t < 0.25 {
                let f = t / 0.25;
                [
                    0.0,
                    0.2 + f * 0.7,
                    1.0 - f * 0.1,
                ]
            } else if t < 0.50 {
                let f = (t - 0.25) / 0.25;
                [
                    f * 0.1,
                    0.9,
                    0.9 - f * 0.7,
                ]
            } else if t < 0.75 {
                let f = (t - 0.50) / 0.25;
                [
                    0.1 + f * 0.9,
                    0.9 - f * 0.05,
                    0.2 - f * 0.2,
                ]
            } else {
                let f = (t - 0.75) / 0.25;
                [
                    1.0,
                    0.85 - f * 0.80,
                    0.05 * (1.0 - f),
                ]
            }
        }
    };

    [rgb[0], rgb[1], rgb[2], alpha]
}

// ---------------------------------------------------------------------------
// B-spline conversion (for CAD / STEP export)
// ---------------------------------------------------------------------------

/// A bicubic (clamped, non-rational) B-spline control net interpolating the
/// export grid. `cps[j][i]`: `j` = v row (0..=ny), `i` = u column (0..=nx).
#[derive(Clone)]
pub struct BicubicNet {
    pub deg_u: usize,
    pub deg_v: usize,
    pub cps: Vec<Vec<Vec3>>,
    /// Full knot vectors (with repeats), clamped: [0,0,0,0, ..., 1,1,1,1].
    pub u_knots_full: Vec<f64>,
    pub v_knots_full: Vec<f64>,
    pub nx: usize,
    pub ny: usize,
}

/// Global bicubic interpolation of the (rectangle) fit grid: the resulting
/// surface passes exactly through every grid node (Piegl & Tiller, global
/// surface interpolation). Control points equal grid points in count.
pub fn bicubic_control_net(grid: &FreeformGrid) -> BicubicNet {
    let (nx, ny) = (grid.nx, grid.ny);
    let deg = 3usize;

    // World-space grid points.
    let mut pts = vec![vec![Vec3::ZERO; nx + 1]; ny + 1];
    for j in 0..=ny {
        for i in 0..=nx {
            pts[j][i] = grid.origin
                + grid.u * (grid.x0 + i as f32 * grid.gw)
                + grid.v * (grid.y0 + j as f32 * grid.gh)
                + grid.normal * grid.heights[j * (nx + 1) + i];
        }
    }

    let u_knots = interpolation_knots(nx, deg);
    let v_knots = interpolation_knots(ny, deg);

    // Interpolation matrices along u and v (uniform parameters).
    let mat_u = interpolation_matrix(&u_knots, nx, deg);
    let mat_v = interpolation_matrix(&v_knots, ny, deg);
    let lu_u = Lu::factorize(&mat_u);
    let lu_v = Lu::factorize(&mat_v);

    // Pass 1: interpolate each grid row (v fixed) along u.
    let mut d = vec![vec![Vec3::ZERO; nx + 1]; ny + 1];
    for j in 0..=ny {
        for c in 0..3 {
            let rhs: Vec<f64> = (0..=nx).map(|i| pts[j][i][c] as f64).collect();
            let sol = lu_solve(&lu_u, &rhs);
            for i in 0..=nx {
                d[j][i][c] = sol[i] as f32;
            }
        }
    }
    // Pass 2: interpolate each column (u fixed) along v.
    let mut cps = vec![vec![Vec3::ZERO; nx + 1]; ny + 1];
    for i in 0..=nx {
        for c in 0..3 {
            let rhs: Vec<f64> = (0..=ny).map(|j| d[j][i][c] as f64).collect();
            let sol = lu_solve(&lu_v, &rhs);
            for j in 0..=ny {
                cps[j][i][c] = sol[j] as f32;
            }
        }
    }

    BicubicNet {
        deg_u: deg,
        deg_v: deg,
        cps,
        u_knots_full: u_knots,
        v_knots_full: v_knots,
        nx,
        ny,
    }
}

/// Evaluates the bicubic surface at parameter (u, v) in 0..=1. Used for
/// verification (the surface must reproduce the grid nodes).
#[allow(dead_code)]
pub fn eval_bicubic(net: &BicubicNet, u: f64, v: f64) -> Vec3 {
    let (nu, nv) = (net.nx + 1, net.ny + 1);
    let bu = basis_all(&net.u_knots_full, nu, net.deg_u, u.clamp(0.0, 1.0));
    let bv = basis_all(&net.v_knots_full, nv, net.deg_v, v.clamp(0.0, 1.0));
    let mut p = Vec3::ZERO;
    for j in 0..nv {
        if bv[j] == 0.0 {
            continue;
        }
        for i in 0..nu {
            if bu[i] == 0.0 {
                continue;
            }
            p += net.cps[j][i] * (bu[i] * bv[j]) as f32;
        }
    }
    p
}

/// Clamped cubic knot vector for global interpolation of `n`+1 points with
/// uniform parameters (interior knots by the averaging method, NURBS Book).
fn interpolation_knots(n: usize, deg: usize) -> Vec<f64> {
    let len = n + deg + 2;
    let mut knots = vec![0.0f64; len];
    for k in 0..=deg {
        knots[k] = 0.0;
        knots[len - 1 - k] = 1.0;
    }
    // Interior knots U[j + deg] = avg(u_bar[j], ..., u_bar[j + deg - 1]),
    // u_bar_k = k / n.
    for j in 1..=(n - deg) {
        let sum: f64 = (0..deg).map(|k| (j + k) as f64 / n as f64).sum();
        knots[j + deg] = sum / deg as f64;
    }
    knots
}

/// Dense (n+1)x(n+1) collocation matrix A[k][i] = N_i,deg(u_k = k/n).
fn interpolation_matrix(knots: &[f64], n: usize, deg: usize) -> Vec<Vec<f64>> {
    let mut a = vec![vec![0.0f64; n + 1]; n + 1];
    for k in 0..=n {
        let t = k as f64 / n as f64;
        let row = basis_all(knots, n + 1, deg, t);
        a[k] = row;
    }
    a
}

/// All basis functions N_i,deg(t) for i in 0..n_ctrl (most are zero).
fn basis_all(knots: &[f64], n_ctrl: usize, deg: usize, t: f64) -> Vec<f64> {
    let mut out = vec![0.0f64; n_ctrl];
    if t <= 0.0 {
        out[0] = 1.0;
        return out;
    }
    if t >= 1.0 {
        out[n_ctrl - 1] = 1.0;
        return out;
    }
    // Find span s: knots[s] <= t < knots[s+1].
    let mut s = 0usize;
    for k in 0..(knots.len() - 1) {
        if knots[k] <= t && t < knots[k + 1] {
            s = k;
            break;
        }
    }
    if s + 1 < deg {
        // t is inside the clamped end; treat as span deg.
        s = deg;
    }
    // Cox-de Boor recursion restricted to the span.
    let mut n = vec![0.0f64; deg + 1];
    let mut left = vec![0.0f64; deg + 1];
    let mut right = vec![0.0f64; deg + 1];
    n[0] = 1.0;
    for j in 1..=deg {
        left[j] = t - knots[s + 1 - j];
        right[j] = knots[s + j] - t;
        let mut saved = 0.0f64;
        for r in 0..j {
            let temp = n[r] / (right[r + 1] + left[j - r]).max(1e-15);
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    // Scatter: N_{s-deg+r} = n[r].
    let start = s.saturating_sub(deg);
    for r in 0..=deg {
        let idx = start + r;
        if idx < n_ctrl {
            out[idx] = n[r];
        }
    }
    out
}

/// LU factorization with partial pivoting (dense, small systems).
struct Lu {
    a: Vec<Vec<f64>>,
    piv: Vec<usize>,
}

impl Lu {
    fn factorize(a: &[Vec<f64>]) -> Lu {
        let n = a.len();
        let mut m = a.to_vec();
        let mut piv = (0..n).collect::<Vec<usize>>();
        for col in 0..n {
            let mut best = col;
            for row in col + 1..n {
                if m[row][col].abs() > m[best][col].abs() {
                    best = row;
                }
            }
            if best != col {
                m.swap(col, best);
                piv.swap(col, best);
            }
            let diag = m[col][col];
            if diag.abs() < 1e-14 {
                m[col][col] = 1e-14;
            }
            for row in col + 1..n {
                let f = m[row][col] / m[col][col];
                m[row][col] = f;
                for k in col + 1..n {
                    m[row][k] -= f * m[col][k];
                }
            }
        }
        Lu { a: m, piv }
    }
}

/// Solves the LU-factored system for one right-hand side.
fn lu_solve(lu: &Lu, rhs: &[f64]) -> Vec<f64> {
    let n = lu.a.len();
    let mut x = vec![0.0f64; n];
    // Apply permutation.
    for i in 0..n {
        x[i] = rhs[lu.piv[i]];
    }
    // Forward substitution (unit lower).
    for i in 0..n {
        for k in 0..i {
            x[i] -= lu.a[i][k] * x[k];
        }
    }
    // Back substitution.
    for i in (0..n).rev() {
        for k in i + 1..n {
            x[i] -= lu.a[i][k] * x[k];
        }
        x[i] /= lu.a[i][i];
    }
    x
}
