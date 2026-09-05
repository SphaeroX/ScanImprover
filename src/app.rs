use crate::camera::{Camera, ViewDir};
use crate::geom::bvh::Bvh;
use crate::geom::distance::Deviation;
use crate::geom::fitting::{fit_circle, fit_plane, plane_basis, CircleFit, PlaneFit};
use crate::geom::symmetry::SymPlane;
use crate::io;
use crate::mesh::{Aabb, Mesh};
use crate::pick;
use crate::render::GpuState;
use crate::worker::{JobResult, Worker};
use eframe::egui;
use glam::{Quat, Vec3};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    Orbit,
    BrushAdd,
    BrushErase,
    SymPickLine,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum DecMode {
    Fixed,
    Accuracy,
    Deviation,
}

#[derive(Clone)]
struct Snapshot {
    current: Arc<Mesh>,
    original: Arc<Mesh>,
    sel: Arc<Vec<u8>>,
    sym: Option<SymState>,
    plane: Option<PlaneFit>,
    circle: Option<CircleFit>,
}

#[derive(Clone, Copy)]
struct SymState {
    plane: SymPlane,
    rms: f64,
    show: bool,
}

pub struct App {
    gpu: Option<Arc<Mutex<GpuState>>>,
    camera: Camera,
    current: Option<Arc<Mesh>>,
    original: Option<Arc<Mesh>>,
    preview: Option<Arc<Mesh>>,
    preview_error: f32,
    sel: Arc<Vec<u8>>,
    sel_count: usize,
    bvh: Option<Arc<Bvh>>,
    bbox: Aabb,
    worker: Worker,
    mode: Mode,
    brush_radius: f32,
    dec_mode: DecMode,
    dec_ratio: f32,
    dec_error_mm: f32,
    dec_target_acc: f32,
    dec_target_mm: f32,
    dec_lock_border: bool,
    dec_auto_preview: bool,
    dec_job: Option<u64>,
    dev_job: Option<u64>,
    sym_job: Option<u64>,
    bvh_job: Option<u64>,
    bvh_job_mesh: Option<Arc<Mesh>>,
    deviation: Option<Deviation>,
    heat: Option<Arc<Vec<f32>>>,
    heat_on: bool,
    heat_max: f32,
    sym: Option<SymState>,
    sym_pick: Vec<Vec3>,
    plane: Option<PlaneFit>,
    show_plane: bool,
    circle: Option<CircleFit>,
    show_circle: bool,
    show_wireframe: bool,
    show_bbox: bool,
    show_triad: bool,
    undo: Vec<Snapshot>,
    status: String,
    file_path: Option<PathBuf>,
    mesh_dirty: bool,
    aux_dirty: bool,
    wire_dirty: bool,
}

impl App {
    pub fn new() -> App {
        let mut app = App {
            gpu: None,
            camera: Camera::default(),
            current: None,
            original: None,
            preview: None,
            preview_error: 0.0,
            sel: Arc::new(Vec::new()),
            sel_count: 0,
            bvh: None,
            bbox: Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            },
            worker: Worker::new(),
            mode: Mode::Orbit,
            brush_radius: 25.0,
            dec_mode: DecMode::Fixed,
            dec_ratio: 0.1,
            dec_error_mm: 0.02,
            dec_target_acc: 99.5,
            dec_target_mm: 0.05,
            dec_lock_border: false,
            dec_auto_preview: true,
            dec_job: None,
            dev_job: None,
            sym_job: None,
            bvh_job: None,
            bvh_job_mesh: None,
            deviation: None,
            heat: None,
            heat_on: false,
            heat_max: 1.0,
            sym: None,
            sym_pick: Vec::new(),
            plane: None,
            show_plane: true,
            circle: None,
            show_circle: true,
            show_wireframe: false,
            show_bbox: true,
            show_triad: true,
            undo: Vec::new(),
            status: "Open a mesh file to begin (STL, PLY or OBJ). Units are assumed to be mm."
                .to_string(),
            file_path: None,
            mesh_dirty: false,
            aux_dirty: false,
            wire_dirty: false,
        };
        if let Some(arg) = std::env::args().nth(1) {
            let path = PathBuf::from(arg);
            if path.exists() {
                app.load_file(path);
            }
        }
        app
    }

    fn display(&self) -> Option<&Arc<Mesh>> {
        self.preview.as_ref().or(self.current.as_ref())
    }

    fn orig_mesh(&self) -> Option<&Arc<Mesh>> {
        self.original.as_ref()
    }

    fn push_snapshot(&mut self) {
        if let (Some(current), Some(original)) = (&self.current, &self.original) {
            self.undo.push(Snapshot {
                current: current.clone(),
                original: original.clone(),
                sel: self.sel.clone(),
                sym: self.sym,
                plane: self.plane,
                circle: self.circle,
            });
            if self.undo.len() > 25 {
                self.undo.remove(0);
            }
        }
    }

    fn undo(&mut self) {
        if let Some(s) = self.undo.pop() {
            self.current = Some(s.current);
            self.original = Some(s.original);
            self.sel = s.sel;
            self.preview = None;
            self.sym = s.sym;
            self.plane = s.plane;
            self.circle = s.circle;
            self.deviation = None;
            self.heat = None;
            self.bvh = None;
            self.mesh_dirty = true;
            self.aux_dirty = true;
            self.recount_sel();
            self.sync_bbox();
            self.status = "Undo applied.".to_string();
        }
    }

    fn recount_sel(&mut self) {
        self.sel_count = self.sel.iter().filter(|&&v| v > 0).count();
    }

    fn sync_bbox(&mut self) {
        if let Some(m) = self.display() {
            self.bbox = m.bbox();
        }
    }

    fn load_file(&mut self, path: PathBuf) {
        match std::fs::read(&path) {
            Ok(bytes) => match io::load_any(&path, &bytes) {
                Ok(mesh) => {
                    let tris = mesh.triangle_count();
                    let verts = mesh.vertex_count();
                    let m = Arc::new(mesh);
                    self.bbox = m.bbox();
                    self.camera.aspect = 1.5;
                    self.camera.fit(&self.bbox);
                    self.current = Some(m.clone());
                    self.original = Some(m);
                    self.preview = None;
                    self.sel = Arc::new(vec![0u8; tris]);
                    self.sel_count = 0;
                    self.bvh = None;
                    self.deviation = None;
                    self.heat = None;
                    self.sym = None;
                    self.sym_pick.clear();
                    self.plane = None;
                    self.circle = None;
                    self.undo.clear();
                    self.mesh_dirty = true;
                    self.aux_dirty = true;
                    self.wire_dirty = true;
                    self.file_path = Some(path);
                    self.status = format!(
                        "Loaded: {} triangles, {} vertices (welded). Units assumed mm.",
                        tris, verts
                    );
                }
                Err(e) => self.status = format!("Load failed: {e}"),
            },
            Err(e) => self.status = format!("Could not read file: {e}"),
        }
    }

    fn apply_transform(&mut self, rot: Quat, trans: Vec3) {
        self.push_snapshot();
        let mut new_current = None;
        let mut new_original = None;
        let mut new_preview = None;
        if let Some(m) = &self.current {
            let mut m2 = (**m).clone();
            m2.transform(rot, trans);
            new_current = Some(Arc::new(m2));
        }
        if let Some(m) = &self.original {
            let mut m2 = (**m).clone();
            m2.transform(rot, trans);
            new_original = Some(Arc::new(m2));
        }
        if let Some(m) = &self.preview {
            let mut m2 = (**m).clone();
            m2.transform(rot, trans);
            new_preview = Some(Arc::new(m2));
        }
        self.current = new_current;
        self.original = new_original;
        self.preview = new_preview;
        if let Some(s) = &mut self.sym {
            s.plane.point = rot * s.plane.point + trans;
            s.plane.normal = (rot * s.plane.normal).normalize();
        }
        if let Some(p) = &mut self.plane {
            p.point = rot * p.point + trans;
            p.normal = (rot * p.normal).normalize();
        }
        if let Some(c) = &mut self.circle {
            c.center = rot * c.center + trans;
            c.normal = (rot * c.normal).normalize();
        }
        for p in &mut self.sym_pick {
            *p = rot * *p + trans;
        }
        self.bvh = None;
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.wire_dirty = true;
        self.sync_bbox();
    }

    fn ensure_bvh(&mut self) -> Option<Arc<Bvh>> {
        if let Some(b) = &self.bvh {
            return Some(b.clone());
        }
        let m = self.display()?.clone();
        if m.vertex_count() < 400_000 {
            let bvh = Arc::new(Bvh::new(&m.positions, &m.indices));
            self.bvh = Some(bvh.clone());
            Some(bvh)
        } else {
            if self.bvh_job.is_none() {
                self.bvh_job_mesh = Some(m.clone());
                self.bvh_job = Some(self.worker.submit_bvh_build(m));
            }
            self.status = "Preparing picking (building spatial index)…".to_string();
            None
        }
    }

    fn maintain_bvh(&mut self) {
        if self.bvh.is_some() || self.bvh_job.is_some() {
            return;
        }
        if self.mode == Mode::Orbit {
            return;
        }
        if let Some(m) = self.display().cloned() {
            if m.vertex_count() >= 400_000 {
                self.bvh_job_mesh = Some(m.clone());
                self.bvh_job = Some(self.worker.submit_bvh_build(m));
            }
        }
    }

    fn set_preview(&mut self, mesh: Arc<Mesh>, error: f32) {
        self.preview = Some(mesh);
        self.preview_error = error;
        self.sel = Arc::new(Vec::new());
        self.sel_count = 0;
        self.bvh = None;
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.wire_dirty = true;
        self.sync_bbox();
    }

    fn schedule_decimate(&mut self) {
        if let Some(m) = self.orig_mesh().cloned() {
            let id = self
                .worker
                .submit_decimate(m, self.dec_ratio, self.dec_error_mm, self.dec_lock_border);
            self.dec_job = Some(id);
            self.status = "Simplifying preview…".to_string();
        }
    }

    fn run_auto_decimate(&mut self) {
        if self.dec_job.is_some() {
            return;
        }
        if let Some(m) = self.orig_mesh().cloned() {
            let target = self.dec_target_mm.max(1e-6);
            let id = self
                .worker
                .submit_decimate_auto(m, target, self.dec_lock_border);
            self.dec_job = Some(id);
            self.status = format!(
                "Auto-decimating to {:.4} mm max deviation…",
                self.dec_target_mm
            );
        }
    }

    fn handle_worker(&mut self, ctx: &egui::Context) {
        let results = self.worker.poll();
        for res in results {
            match res {
                JobResult::Decimated { id, mesh, error } => {
                    if self.dec_job == Some(id) {
                        self.set_preview(mesh, error);
                        self.dec_job = None;
                        if let (Some(orig), Some(prev)) = (
                            self.orig_mesh().cloned(),
                            self.preview.clone(),
                        ) {
                            self.dev_job = Some(self.worker.submit_deviation(orig, prev));
                        }
                    }
                }
                JobResult::DecimatedAuto {
                    id,
                    mesh,
                    error,
                    dev,
                    heat,
                    iterations,
                } => {
                    if self.dec_job == Some(id) {
                        self.dec_job = None;
                        let tris = mesh.triangle_count();
                        self.set_preview(mesh, error);
                        self.deviation = Some(dev);
                        self.heat_max = dev.max_dev.max(1e-6);
                        self.heat = Some(heat);
                        self.aux_dirty = true;
                        self.status = format!(
                            "Auto decimation done: {} passes, {} triangles remain, max deviation {:.4} mm",
                            iterations, tris, dev.max_dev
                        );
                    }
                }
                JobResult::Failed { id, message } => {
                    if self.dec_job == Some(id) {
                        self.dec_job = None;
                    }
                    self.status = message;
                }
                JobResult::Deviation { id, dev, heat } => {
                    if self.dev_job == Some(id) {
                        self.deviation = Some(dev);
                        self.heat_max = dev.max_dev.max(1e-6);
                        self.heat = Some(heat);
                        self.aux_dirty = true;
                        self.dev_job = None;
                    }
                }
                JobResult::Symmetry { id, plane } => {
                    if self.sym_job == Some(id) {
                        self.sym_job = None;
                        self.sym_pick.clear();
                        match plane {
                            Some((p, rms)) => {
                                self.sym = Some(SymState {
                                    plane: p,
                                    rms,
                                    show: true,
                                });
                                self.status =
                                    format!("Symmetry plane found. RMS deviation: {rms:.4} mm");
                            }
                            None => self.status = "No symmetry detected.".to_string(),
                        }
                    }
                }
                JobResult::BvhReady { id, bvh } => {
                    if self.bvh_job == Some(id) {
                        self.bvh_job = None;
                        let matches = match (self.display(), &self.bvh_job_mesh) {
                            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                            _ => false,
                        };
                        self.bvh_job_mesh = None;
                        if matches {
                            self.bvh = Some(bvh);
                        }
                    }
                }
            }
        }
        let _ = ctx;
    }

    fn schedule_sym_auto(&mut self) {
        if let Some(m) = self.display().cloned() {
            self.sym_job = Some(self.worker.submit_symmetry(m, None));
            self.status = "Detecting symmetry plane…".to_string();
        }
    }

    fn schedule_sym_refine(&mut self, init: SymPlane) {
        if let Some(m) = self.display().cloned() {
            self.sym_job = Some(self.worker.submit_symmetry(m, Some(init)));
            self.status = "Optimizing symmetry plane…".to_string();
        }
    }

    fn rotate_normal_to_axis(&mut self, axis: Vec3) {
        if let Some(n) = self.sym.map(|s| s.plane.normal).or(self.plane.map(|p| p.normal)) {
            let q = rotation_between(n, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned normal to {}.", axis_name(axis));
        }
    }

    fn circle_axis_to(&mut self, axis: Vec3) {
        if let Some(c) = self.circle {
            let q = rotation_between(c.normal, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned circle axis to {}.", axis_name(axis));
        }
    }

    fn origin_on_plane(&mut self) {
        if let Some(p) = self.plane {
            let d = p.point.dot(p.normal);
            self.apply_transform(Quat::IDENTITY, -p.normal * d);
            self.status = "Origin moved onto the fitted plane.".to_string();
        }
    }

    fn origin_at_symmetry(&mut self) {
        if let Some(s) = self.sym {
            let d = s.plane.point.dot(s.plane.normal);
            self.apply_transform(Quat::IDENTITY, -s.plane.normal * d);
            self.status = "Mesh translated so the symmetry plane passes through the origin."
                .to_string();
        }
    }

    fn origin_at_circle_center(&mut self) {
        if let Some(c) = self.circle {
            self.apply_transform(Quat::IDENTITY, -c.center);
            self.status = "Origin moved to the circle center.".to_string();
        }
    }

    fn center_axes(&mut self, x: bool, y: bool, z: bool) {
        let c = self.bbox.center();
        let mut t = Vec3::ZERO;
        if x {
            t.x = -c.x;
        }
        if y {
            t.y = -c.y;
        }
        if z {
            t.z = -c.z;
        }
        if t != Vec3::ZERO {
            self.apply_transform(Quat::IDENTITY, t);
            let mut axes = Vec::new();
            if x {
                axes.push("X");
            }
            if y {
                axes.push("Y");
            }
            if z {
                axes.push("Z");
            }
            self.status = format!("Centered {} axis origin.", axes.join(", "));
        }
    }

    fn fit_plane_from_selection(&mut self) {
        let points = self.selection_points();
        match points.map(|p| fit_plane(&p)) {
            Some(Some(f)) => {
                self.plane = Some(f);
                self.show_plane = true;
                self.status = format!(
                    "Plane fitted: RMS {:.4} mm, max {:.4} mm.",
                    f.rms.sqrt(),
                    f.max_dev
                );
            }
            Some(None) => self.status = "Not enough points to fit a plane.".to_string(),
            None => self.status = "Select faces first.".to_string(),
        }
    }

    fn fit_circle_from_selection(&mut self) {
        let points = self.selection_points();
        match points.map(|p| fit_circle(&p)) {
            Some(Some(c)) => {
                self.circle = Some(c);
                self.show_circle = true;
                self.status = format!(
                    "Circle fitted: R = {:.4} mm, radial RMS {:.4} mm.",
                    c.radius,
                    c.radial_rms.sqrt()
                );
            }
            Some(None) => self.status = "Circle fit failed on selection.".to_string(),
            None => self.status = "Select faces first.".to_string(),
        }
    }

    fn selection_points(&self) -> Option<Vec<[f32; 3]>> {
        let m = self.display()?;
        let sel = &*self.sel;
        if sel.len() != m.triangle_count() {
            return None;
        }
        let mut pts = Vec::new();
        for t in 0..m.triangle_count() {
            if sel[t] > 0 {
                for k in 0..3 {
                    pts.push(m.positions[m.indices[3 * t + k] as usize]);
                }
            }
        }
        if pts.is_empty() {
            None
        } else {
            Some(pts)
        }
    }

    fn clear_selection(&mut self) {
        if let Some(m) = self.display() {
            if self.sel.len() == m.triangle_count() {
                let sel = Arc::make_mut(&mut self.sel);
                sel.fill(0);
            }
            self.sel_count = 0;
            self.aux_dirty = true;
        }
    }

    fn apply_preview(&mut self) {
        if let Some(p) = self.preview.clone() {
            self.push_snapshot();
            self.current = Some(p.clone());
            self.original = Some(p);
            self.preview = None;
            self.deviation = None;
            self.heat = None;
            self.sel = Arc::new(vec![0u8; self.current.as_ref().unwrap().triangle_count()]);
            self.sel_count = 0;
            self.bvh = None;
            self.mesh_dirty = true;
            self.aux_dirty = true;
            self.wire_dirty = true;
            self.status = "Decimation applied. New mesh is the reference.".to_string();
        }
    }

    fn discard_preview(&mut self) {
        self.preview = None;
        self.deviation = None;
        self.heat = None;
        self.sel = Arc::new(Vec::new());
        self.sel_count = 0;
        self.bvh = None;
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.wire_dirty = true;
        self.sync_bbox();
        self.status = "Preview discarded.".to_string();
    }

    fn reset_mesh(&mut self) {
        if let Some(orig) = self.original.clone() {
            self.push_snapshot();
            let tris = orig.triangle_count();
            self.current = Some(orig);
            self.preview = None;
            self.sel = Arc::new(vec![0u8; tris]);
            self.sel_count = 0;
            self.bvh = None;
            self.deviation = None;
            self.heat = None;
            self.mesh_dirty = true;
            self.aux_dirty = true;
            self.wire_dirty = true;
            self.sync_bbox();
            self.status = "Reset to original mesh.".to_string();
        }
    }
}

fn rotation_between(from: Vec3, to: Vec3) -> Quat {
    let f = from.normalize_or_zero();
    let t = to.normalize_or_zero();
    if f.length_squared() < 1e-12 {
        return Quat::IDENTITY;
    }
    if f.dot(t) < -0.999_999 {
        let perp = if f.x.abs() < 0.9 {
            Vec3::X
        } else {
            Vec3::Y
        };
        let axis = f.cross(perp).normalize_or_zero();
        return Quat::from_axis_angle(axis, std::f32::consts::PI);
    }
    Quat::from_rotation_arc(f, t)
}

fn axis_name(a: Vec3) -> &'static str {
    if a == Vec3::X {
        "X"
    } else if a == Vec3::Y {
        "Y"
    } else if a == Vec3::Z {
        "Z"
    } else {
        "axis"
    }
}

type Line = [f32; 7];

fn push_line(out: &mut Vec<Line>, a: Vec3, b: Vec3, c: [f32; 4]) {
    out.push([a.x, a.y, a.z, c[0], c[1], c[2], c[3]]);
    out.push([b.x, b.y, b.z, c[0], c[1], c[2], c[3]]);
}

fn bbox_lines(bb: &Aabb, c: [f32; 4]) -> Vec<Line> {
    let min = bb.min;
    let max = bb.max;
    let p = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
    let mut l = Vec::new();
    let cs = [
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
    for (a, b) in cs {
        push_line(&mut l, a, b, c);
    }
    l
}

fn triad_lines(len: f32) -> Vec<Line> {
    let mut l = Vec::new();
    push_line(&mut l, Vec3::ZERO, Vec3::X * len, [1.0, 0.3, 0.3, 1.0]);
    push_line(&mut l, Vec3::ZERO, Vec3::Y * len, [0.3, 1.0, 0.4, 1.0]);
    push_line(&mut l, Vec3::ZERO, Vec3::Z * len, [0.35, 0.55, 1.0, 1.0]);
    l
}

fn marker_lines(p: Vec3, size: f32, c: [f32; 4]) -> Vec<Line> {
    let mut l = Vec::new();
    push_line(&mut l, p - Vec3::X * size, p + Vec3::X * size, c);
    push_line(&mut l, p - Vec3::Y * size, p + Vec3::Y * size, c);
    push_line(&mut l, p - Vec3::Z * size, p + Vec3::Z * size, c);
    l
}

fn plane_grid_lines(p: Vec3, n: Vec3, half: f32, div: u32, c: [f32; 4]) -> Vec<Line> {
    let (u, v) = plane_basis(n);
    let mut l = Vec::new();
    for i in 0..=div {
        let f = -half + (2.0 * half) * (i as f32 / div as f32);
        push_line(
            &mut l,
            p + u * f + v * (-half),
            p + u * f + v * half,
            c,
        );
        push_line(
            &mut l,
            p + u * (-half) + v * f,
            p + u * half + v * f,
            c,
        );
    }
    l
}

fn plane_fill(p: Vec3, n: Vec3, half: f32, c: [f32; 4]) -> Vec<Line> {
    let (u, v) = plane_basis(n);
    let a = p + u * (-half) + v * (-half);
    let b = p + u * half + v * (-half);
    let d = p + u * (-half) + v * half;
    let e = p + u * half + v * half;
    let mut out = Vec::with_capacity(6);
    let mut push = |x: Vec3, y: Vec3, z: Vec3| {
        out.push([x.x, x.y, x.z, c[0], c[1], c[2], c[3]]);
        out.push([y.x, y.y, y.z, c[0], c[1], c[2], c[3]]);
        out.push([z.x, z.y, z.z, c[0], c[1], c[2], c[3]]);
    };
    push(a, b, e);
    push(a, e, d);
    out
}

fn circle_lines(c: Vec3, n: Vec3, r: f32, col: [f32; 4]) -> Vec<Line> {
    let (u, v) = plane_basis(n);
    let segs = 96;
    let mut l = Vec::new();
    for i in 0..segs {
        let a0 = 2.0 * std::f32::consts::PI * (i as f32 / segs as f32);
        let a1 = 2.0 * std::f32::consts::PI * ((i + 1) as f32 / segs as f32);
        let p0 = c + u * (r * a0.cos()) + v * (r * a0.sin());
        let p1 = c + u * (r * a1.cos()) + v * (r * a1.sin());
        push_line(&mut l, p0, p1, col);
    }
    push_line(&mut l, c - n * (r * 1.4), c + n * (r * 1.4), col);
    l
}

impl App {
    fn top_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Open…").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("Mesh files", &["stl", "ply", "obj"])
                    .pick_file()
                {
                    self.load_file(p);
                }
            }
            ui.separator();
            let export = |ui: &mut egui::Ui, app: &mut App, ext: &str| {
                if ui.button(format!("Export {ext}")).clicked() {
                    if app.display().is_some() {
                        if let Some(p) = rfd::FileDialog::new()
                            .add_filter(ext, &[ext])
                            .set_file_name(app.file_path.as_ref().and_then(|p| p.file_stem()).and_then(|s| s.to_str()).map(|s| format!("{s}.{ext}")).unwrap_or(format!("mesh.{ext}")))
                            .save_file()
                        {
                            let m = app.display().unwrap().clone();
                            match io::save_any(&p, &m) {
                                Ok(()) => app.status = format!("Exported {}.", p.display()),
                                Err(e) => app.status = format!("Export failed: {e}"),
                            }
                        }
                    } else {
                        app.status = "Nothing to export.".to_string();
                    }
                }
            };
            export(ui, self, "stl");
            export(ui, self, "ply");
            export(ui, self, "obj");
            ui.separator();
            ui.checkbox(&mut self.show_wireframe, "Wireframe");
            ui.checkbox(&mut self.show_bbox, "BBox");
            ui.checkbox(&mut self.show_triad, "Origin");
            ui.separator();
            ui.label("View:");
            if ui.small_button("X").clicked() {
                self.camera.set_view(ViewDir::X);
            }
            if ui.small_button("Y").clicked() {
                self.camera.set_view(ViewDir::Y);
            }
            if ui.small_button("Z").clicked() {
                self.camera.set_view(ViewDir::Z);
            }
            if ui.small_button("Iso").clicked() {
                self.camera.set_view(ViewDir::Iso);
            }
            if ui.small_button("Fit").clicked() {
                self.camera.fit(&self.bbox);
            }
        });
    }

    fn active_tool_section(&mut self, ui: &mut egui::Ui) {
        let (name, desc, accent) = match self.mode {
            Mode::Orbit => (
                "ORBIT",
                "LMB drag = orbit · MMB = pan · wheel = zoom".to_string(),
                (110, 140, 255),
            ),
            Mode::BrushAdd => (
                "SELECT",
                "LMB drag = paint-select faces under the brush".to_string(),
                (255, 150, 40),
            ),
            Mode::BrushErase => (
                "ERASE",
                "LMB drag = erase faces from the selection".to_string(),
                (255, 70, 70),
            ),
            Mode::SymPickLine => (
                "PICK LINE",
                format!(
                    "Click point {} of 2 on the mesh · Esc = cancel",
                    self.sym_pick.len() + 1
                ),
                (40, 220, 255),
            ),
        };
        let bg = egui::Color32::from_rgba_unmultiplied(accent.0, accent.1, accent.2, 30);
        let fg = egui::Color32::from_rgb(accent.0, accent.1, accent.2);
        egui::Frame::new()
            .fill(bg)
            .corner_radius(6.0)
            .inner_margin(egui::Margin::symmetric(10, 8))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.label(egui::RichText::new(name).strong().size(19.0).color(fg));
                ui.label(desc);
            });
        ui.add_space(6.0);
        let mut new_mode = None;
        ui.horizontal(|ui| {
            if ui.button("Orbit").clicked() {
                new_mode = Some(Mode::Orbit);
            }
            if ui.button("Select").clicked() {
                new_mode = Some(Mode::BrushAdd);
            }
            if ui.button("Erase").clicked() {
                new_mode = Some(Mode::BrushErase);
            }
            if ui.button("Pick Line").clicked() {
                new_mode = Some(Mode::SymPickLine);
            }
        });
        if let Some(m) = new_mode {
            if m != self.mode {
                self.sym_pick.clear();
                self.mode = m;
            }
        }
        ui.add_space(4.0);
        ui.separator();
        ui.add_space(4.0);
    }

    fn tool_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            self.active_tool_section(ui);
            ui.heading("Decimation");
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.dec_mode, DecMode::Fixed, "Fixed mm");
                ui.selectable_value(&mut self.dec_mode, DecMode::Accuracy, "Auto %");
                ui.selectable_value(&mut self.dec_mode, DecMode::Deviation, "Auto mm");
            });
            ui.checkbox(&mut self.dec_lock_border, "Lock open borders");
            let mut schedule_manual = false;
            match self.dec_mode {
                DecMode::Fixed => {
                    let err_resp = ui.add(
                        egui::Slider::new(&mut self.dec_error_mm, 0.0001..=1.0)
                            .logarithmic(true)
                            .text("Error tolerance (mm)"),
                    );
                    let ratio_resp = ui.add(
                        egui::Slider::new(&mut self.dec_ratio, 0.002..=1.0)
                            .text("Target triangle ratio"),
                    );
                    ui.checkbox(&mut self.dec_auto_preview, "Auto preview on change");
                    ui.horizontal(|ui| {
                        if ui.button("Preview now").clicked() {
                            schedule_manual = true;
                        }
                        if self.dec_job.is_some() {
                            ui.add(egui::Spinner::new());
                            ui.label("working…");
                        }
                    });
                    if err_resp.drag_stopped() || ratio_resp.drag_stopped() {
                        if self.dec_auto_preview && self.dec_job.is_none() {
                            schedule_manual = true;
                        }
                    }
                }
                DecMode::Accuracy | DecMode::Deviation => {
                    let diag = self.bbox.diagonal().max(1e-9);
                    if self.dec_mode == DecMode::Accuracy {
                        let mut acc = self.dec_target_acc;
                        let resp = ui
                            .add(
                                egui::DragValue::new(&mut acc)
                                    .range(90.0..=99.9999)
                                    .speed(0.02)
                                    .prefix("target ≥ ")
                                    .suffix(" %")
                                    .fixed_decimals(3),
                            );
                        if resp.changed() {
                            self.dec_target_acc = acc;
                            self.dec_target_mm = ((1.0 - acc / 100.0) * diag).max(1e-6);
                        }
                    } else {
                        let mut mm = self.dec_target_mm;
                        let resp = ui.add(
                            egui::DragValue::new(&mut mm)
                                .range(0.0001..=10.0)
                                .speed(0.01)
                                .prefix("target ≤ ")
                                .suffix(" mm")
                                .fixed_decimals(4),
                        );
                        if resp.changed() {
                            self.dec_target_mm = mm;
                            self.dec_target_acc = 100.0 * (1.0 - mm / diag);
                        }
                    }
                    ui.label(format!(
                        "Goal: max deviation {:.4} mm = {:.4} % accuracy",
                        self.dec_target_mm, self.dec_target_acc
                    ));
                    ui.horizontal(|ui| {
                        if ui.button("Auto decimate").clicked() {
                            self.run_auto_decimate();
                        }
                        if self.dec_job.is_some() {
                            ui.add(egui::Spinner::new());
                            ui.label("working…");
                        }
                    });
                }
            }
            if self.preview.is_some() {
                ui.horizontal(|ui| {
                    if ui
                        .button("✔ Apply decimation")
                        .clicked()
                    {
                        self.apply_preview();
                    }
                    if ui.button("✖ Discard").clicked() {
                        self.discard_preview();
                    }
                });
            }
            if let (Some(orig), Some(prev)) = (self.orig_mesh(), self.preview.as_ref()) {
                ui.add_space(4.0);
                ui.label(format!(
                    "Triangles: {} → {} ({:.1}%)",
                    orig.triangle_count(),
                    prev.triangle_count(),
                    100.0 * prev.triangle_count() as f32 / orig.triangle_count().max(1) as f32
                ));
                ui.label(format!("Simplifier error estimate: {:.4} mm", self.preview_error));
                if let Some(dev) = &self.deviation {
                    if self.dev_job.is_some() && self.dec_mode == DecMode::Fixed {
                        ui.label("Measuring deviation…");
                    } else {
                        let diag = self.bbox.diagonal();
                        let acc = 100.0 * (1.0 - dev.max_dev / diag.max(1e-9));
                        ui.label(format!("Max deviation: {:.4} mm", dev.max_dev));
                        ui.label(format!("RMS deviation: {:.4} mm", dev.rms));
                        ui.label(format!("Contour match: {:.5}%", acc));
                        ui.label(format!("Measured on {} source points", dev.count));
                    }
                }
                ui.checkbox(&mut self.heat_on, "Deviation heatmap");
            }
            if schedule_manual && self.dec_job.is_none() {
                self.schedule_decimate();
            }

            ui.separator();
            ui.heading("Symmetry plane");
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui.button("Auto-detect").clicked() {
                    self.schedule_sym_auto();
                }
                let pick_label = if self.mode == Mode::SymPickLine {
                    "Pick line: ON"
                } else {
                    "Pick line"
                };
                if ui.button(pick_label).clicked() {
                    self.sym_pick.clear();
                    self.mode = if self.mode == Mode::SymPickLine {
                        Mode::Orbit
                    } else {
                        Mode::SymPickLine
                    };
                }
                if self.sym_job.is_some() {
                    ui.add(egui::Spinner::new());
                }
            });
            let sym_copy = self.sym;
            if sym_copy.is_some() {
                let mut show = sym_copy.unwrap().show;
                if ui.checkbox(&mut show, "Show plane").changed() {
                    if let Some(s) = &mut self.sym {
                        s.show = show;
                    }
                }
                let rms = sym_copy.unwrap().rms;
                if rms.is_finite() {
                    ui.label(format!("RMS: {:.4} mm", rms));
                } else {
                    ui.label("Optimizing plane…");
                }
                let mut refine = false;
                let mut to_origin = false;
                let mut align_axis = None;
                ui.horizontal(|ui| {
                    if ui.button("Optimize").clicked() {
                        refine = true;
                    }
                    if ui.button("Origin → plane").clicked() {
                        to_origin = true;
                    }
                });
                ui.label("Align symmetry to:");
                ui.horizontal(|ui| {
                    if ui.button("X = 0").clicked() {
                        align_axis = Some(Vec3::X);
                    }
                    if ui.button("Y = 0").clicked() {
                        align_axis = Some(Vec3::Y);
                    }
                    if ui.button("Z = 0").clicked() {
                        align_axis = Some(Vec3::Z);
                    }
                });
                let sp = sym_copy.unwrap().plane;
                if refine {
                    self.schedule_sym_refine(sp);
                }
                if to_origin {
                    self.origin_at_symmetry();
                }
                if let Some(axis) = align_axis {
                    let q = rotation_between(sp.normal, axis);
                    self.align_sym_full(axis, q);
                }
            }

            ui.separator();
            ui.heading("Face selection");
            ui.add_space(4.0);
            ui.add(
                egui::Slider::new(&mut self.brush_radius, 2.0..=150.0).text("Brush radius (px)"),
            );
            ui.horizontal(|ui| {
                if ui.button("Clear").clicked() {
                    self.clear_selection();
                }
                if ui.button("Invert").clicked() {
                    if let Some(m) = self.display() {
                        if self.sel.len() == m.triangle_count() {
                            let sel = Arc::make_mut(&mut self.sel);
                            for v in sel.iter_mut() {
                                *v = if *v > 0 { 0 } else { 1 };
                            }
                            self.recount_sel();
                            self.aux_dirty = true;
                        }
                    }
                }
                ui.label(format!("{} faces", self.sel_count));
            });
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.button("Fit plane").clicked() {
                    self.fit_plane_from_selection();
                }
                if ui.button("Fit circle").clicked() {
                    self.fit_circle_from_selection();
                }
            });
            let plane_copy = self.plane;
            if plane_copy.is_some() {
                let p = plane_copy.unwrap();
                ui.add_space(4.0);
                ui.checkbox(&mut self.show_plane, "Show plane");
                ui.label(format!(
                    "Plane  N ({:.3}, {:.3}, {:.3})  off {:.3} mm",
                    p.normal.x,
                    p.normal.y,
                    p.normal.z,
                    p.point.dot(p.normal)
                ));
                ui.label(format!(
                    "RMS {:.4} mm · max {:.4} mm",
                    p.rms.sqrt(),
                    p.max_dev
                ));
                let mut axis = None;
                let mut origin = false;
                ui.horizontal(|ui| {
                    if ui.button("N → X").clicked() {
                        axis = Some(Vec3::X);
                    }
                    if ui.button("N → Y").clicked() {
                        axis = Some(Vec3::Y);
                    }
                    if ui.button("N → Z").clicked() {
                        axis = Some(Vec3::Z);
                    }
                    if ui.button("Origin on plane").clicked() {
                        origin = true;
                    }
                });
                if let Some(a) = axis {
                    self.rotate_normal_to_axis(a);
                }
                if origin {
                    self.origin_on_plane();
                }
            }
            let circle_copy = self.circle;
            if circle_copy.is_some() {
                let c = circle_copy.unwrap();
                ui.add_space(4.0);
                ui.checkbox(&mut self.show_circle, "Show circle");
                ui.label(format!(
                    "Circle R = {:.4} mm, center ({:.2}, {:.2}, {:.2})",
                    c.radius, c.center.x, c.center.y, c.center.z
                ));
                ui.label(format!(
                    "Radial RMS {:.4} mm · plane RMS {:.4} mm · max {:.4} mm",
                    c.radial_rms.sqrt(),
                    c.plane_rms.sqrt(),
                    c.radial_max
                ));
                let mut axis = None;
                let mut origin = false;
                ui.horizontal(|ui| {
                    if ui.button("Axis → X").clicked() {
                        axis = Some(Vec3::X);
                    }
                    if ui.button("Axis → Y").clicked() {
                        axis = Some(Vec3::Y);
                    }
                    if ui.button("Axis → Z").clicked() {
                        axis = Some(Vec3::Z);
                    }
                    if ui.button("Origin at center").clicked() {
                        origin = true;
                    }
                });
                if let Some(a) = axis {
                    self.circle_axis_to(a);
                }
                if origin {
                    self.origin_at_circle_center();
                }
            }

            ui.separator();
            ui.heading("Coordinate system");
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui.button("Center X").clicked() {
                    self.center_axes(true, false, false);
                }
                if ui.button("Center Y").clicked() {
                    self.center_axes(false, true, false);
                }
                if ui.button("Center Z").clicked() {
                    self.center_axes(false, false, true);
                }
                if ui.button("Center all").clicked() {
                    self.center_axes(true, true, true);
                }
            });
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.button("Undo").clicked() {
                    self.undo();
                }
                if ui.button("Reset mesh").clicked() {
                    self.reset_mesh();
                }
                ui.label(format!("{} undo steps", self.undo.len()));
            });
        });
    }

    fn align_sym_full(&mut self, axis: Vec3, q: Quat) {
        self.apply_transform(q, Vec3::ZERO);
        if let Some(s) = &self.sym {
            let d = s.plane.point.dot(s.plane.normal);
            self.apply_transform(Quat::IDENTITY, -s.plane.normal * d);
        }
        self.status = format!(
            "Symmetry plane aligned: mesh rotated so the plane is {} = 0 and passes through the origin.",
            axis_name(axis)
        );
    }

    fn status_bar(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let hint: String = match self.mode {
                Mode::Orbit => "LMB drag = free orbit · MMB drag = pan · Wheel = zoom · 1/2/3/4 = views".to_string(),
                Mode::BrushAdd => "LMB drag over mesh to paint-select front faces".to_string(),
                Mode::BrushErase => "LMB drag to erase selection".to_string(),
                Mode::SymPickLine => format!(
                    "Symmetry line: click point {} of 2 on the mesh · Esc = cancel",
                    self.sym_pick.len() + 1
                ),
            };
            ui.label(hint);
        });
    }

    fn viewport(&mut self, ui: &mut egui::Ui) {
        let rect = ui.available_rect_before_wrap();
        let (alloc, response) =
            ui.allocate_exact_size(rect.size(), egui::Sense::click_and_drag());
        let rect = alloc;
        let ppp = ui.ctx().pixels_per_point();
        let vp_w = (rect.width() * ppp).round().max(1.0) as u32;
        let vp_h = (rect.height() * ppp).round().max(1.0) as u32;
        self.camera.aspect = rect.width() / rect.height().max(1.0);

        let ctrl_z = ui
            .ctx()
            .input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Z));
        if ctrl_z {
            self.undo();
        }
        let esc = ui.ctx().input(|i| i.key_pressed(egui::Key::Escape));
        if esc {
            self.mode = Mode::Orbit;
            self.sym_pick.clear();
        }

        if response.dragged_by(egui::PointerButton::Middle) {
            let d = response.drag_delta();
            self.camera.pan_drag(d.x, d.y, rect.height());
        }
        if response.dragged_by(egui::PointerButton::Primary) {
            if self.mode == Mode::Orbit {
                let d = response.drag_delta();
                self.camera.rotate(d.x, d.y);
            }
        }
        if response.hovered() {
            let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.zoom(0.95f32.powf(scroll / 60.0));
            }
        }

        if (self.mode == Mode::BrushAdd || self.mode == Mode::BrushErase)
            && (response.dragged_by(egui::PointerButton::Primary) || response.is_pointer_button_down_on())
        {
            if let Some(mesh) = self.display().cloned() {
                if let Some(bvh) = self.ensure_bvh() {
                    if let Some(pos) = response.interact_pointer_pos() {
                        let sx = pos.x - rect.min.x;
                        let sy = pos.y - rect.min.y;
                        if let Some(hit) =
                            pick::ray_pick(&bvh, &self.camera, sx, sy, rect.width(), rect.height())
                        {
                            let mut sel = self.sel.clone();
                            let sel_slice: &mut Vec<u8> = Arc::make_mut(&mut sel);
                            pick::brush(
                                &mesh,
                                &self.camera,
                                &hit,
                                self.brush_radius,
                                rect.height(),
                                self.mode == Mode::BrushAdd,
                                sel_slice,
                            );
                            self.sel = sel;
                            self.recount_sel();
                            self.aux_dirty = true;
                        }
                    }
                }
            }
        }

        let k1 = ui.ctx().input(|i| i.key_pressed(egui::Key::Num1));
        let k2 = ui.ctx().input(|i| i.key_pressed(egui::Key::Num2));
        let k3 = ui.ctx().input(|i| i.key_pressed(egui::Key::Num3));
        let k4 = ui.ctx().input(|i| i.key_pressed(egui::Key::Num4));
        if k1 {
            self.camera.set_view(crate::camera::ViewDir::X);
        }
        if k2 {
            self.camera.set_view(crate::camera::ViewDir::Y);
        }
        if k3 {
            self.camera.set_view(crate::camera::ViewDir::Z);
        }
        if k4 {
            self.camera.set_view(crate::camera::ViewDir::Iso);
        }

        let mut hover_pos_3d: Option<Vec3> = None;
        if self.mode == Mode::SymPickLine && response.hovered() {
            if let Some(pos) = response.hover_pos() {
                let sx = pos.x - rect.min.x;
                let sy = pos.y - rect.min.y;
                if let Some(bvh) = self.ensure_bvh() {
                    hover_pos_3d = pick::ray_pick(
                        &bvh,
                        &self.camera,
                        sx,
                        sy,
                        rect.width(),
                        rect.height(),
                    )
                    .map(|h| h.pos);
                }
            }
        }

        if self.mode == Mode::SymPickLine
            && response.hovered()
            && ui.input(|i| {
                i.pointer.button_pressed(egui::PointerButton::Primary)
                    || i.pointer.button_pressed(egui::PointerButton::Secondary)
            })
        {
            if let Some(bvh) = self.ensure_bvh() {
                if let Some(pos) = response.interact_pointer_pos() {
                    let sx = pos.x - rect.min.x;
                    let sy = pos.y - rect.min.y;
                    if let Some(hit) =
                        pick::ray_pick(&bvh, &self.camera, sx, sy, rect.width(), rect.height())
                    {
                        self.sym_pick.push(hit.pos);
                        if self.sym_pick.len() == 2 {
                            let a = self.sym_pick[0];
                            let b = self.sym_pick[1];
                            let l = b - a;
                            let fwd = -self.camera.back();
                            let mut n = l.cross(fwd);
                            if n.length_squared() < 1e-8 {
                                n = l.cross(Vec3::Y);
                            }
                            if n.length_squared() < 1e-8 {
                                n = l.cross(Vec3::X);
                            }
                            let n = n.normalize_or_zero();
                            let init = SymPlane {
                                normal: n,
                                point: (a + b) * 0.5,
                            };
                            self.sym = Some(SymState {
                                plane: init,
                                rms: f64::INFINITY,
                                show: true,
                            });
                            self.schedule_sym_refine(init);
                            self.mode = Mode::Orbit;
                        }
                    }
                }
            }
        }

        if matches!(self.mode, Mode::BrushAdd | Mode::BrushErase) && response.hovered() {
            if let Some(pos) = response.hover_pos() {
                let (stroke_col, fill_col) = if self.mode == Mode::BrushAdd {
                    (
                        egui::Color32::from_rgba_unmultiplied(255, 150, 40, 230),
                        egui::Color32::from_rgba_unmultiplied(255, 150, 40, 24),
                    )
                } else {
                    (
                        egui::Color32::from_rgba_unmultiplied(255, 70, 70, 230),
                        egui::Color32::from_rgba_unmultiplied(255, 70, 70, 24),
                    )
                };
                ui.painter()
                    .circle_filled(pos, self.brush_radius, fill_col);
                ui.painter().circle_stroke(
                    pos,
                    self.brush_radius,
                    egui::Stroke::new(2.0, stroke_col),
                );
            }
        }

        let diag = self.bbox.diagonal();
        let mut depth_lines: Vec<Line> = Vec::new();
        let mut overlay_lines: Vec<Line> = Vec::new();
        let mut fills: Vec<Line> = Vec::new();
        if self.display().is_some() {
            if self.show_bbox {
                depth_lines.extend(bbox_lines(&self.bbox, [0.45, 0.47, 0.52, 1.0]));
            }
            if self.show_triad {
                overlay_lines.extend(triad_lines(diag * 0.12));
            }
            if let Some(s) = &self.sym {
                if s.show {
                    let half = diag * 0.6;
                    let col = [0.15, 0.85, 0.95, 0.9];
                    depth_lines.extend(plane_grid_lines(
                        s.plane.point,
                        s.plane.normal,
                        half,
                        10,
                        col,
                    ));
                    fills.extend(plane_fill(
                        s.plane.point,
                        s.plane.normal,
                        half,
                        [0.15, 0.85, 0.95, 0.10],
                    ));
                }
            }
            if let Some(p) = &self.plane {
                if self.show_plane {
                    let half = diag * 0.35;
                    depth_lines.extend(plane_grid_lines(p.point, p.normal, half, 6, [1.0, 0.6, 0.1, 0.9]));
                    fills.extend(plane_fill(p.point, p.normal, half, [1.0, 0.6, 0.1, 0.12]));
                }
            }
            if let Some(c) = &self.circle {
                if self.show_circle {
                    depth_lines.extend(circle_lines(c.center, c.normal, c.radius, [1.0, 0.2, 0.8, 1.0]));
                }
            }
            if self.mode == Mode::SymPickLine {
                if let Some(h) = hover_pos_3d {
                    overlay_lines.extend(marker_lines(h, diag * 0.02, [0.2, 0.9, 1.0, 1.0]));
                    overlay_lines
                        .extend(circle_lines(h, self.camera.back(), diag * 0.008, [0.2, 0.9, 1.0, 1.0]));
                    if let Some(&a) = self.sym_pick.first() {
                        push_line(&mut overlay_lines, a, h, [1.0, 1.0, 0.3, 1.0]);
                    }
                }
            }
            for &p in &self.sym_pick {
                overlay_lines.extend(marker_lines(p, diag * 0.02, [1.0, 1.0, 1.0, 1.0]));
                overlay_lines
                    .extend(circle_lines(p, self.camera.back(), diag * 0.008, [1.0, 1.0, 1.0, 1.0]));
            }
            if self.sym_pick.len() == 2 {
                push_line(
                    &mut overlay_lines,
                    self.sym_pick[0],
                    self.sym_pick[1],
                    [1.0, 1.0, 0.3, 1.0],
                );
            }
        }

        if self.display().is_none() {
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Open a mesh file (STL / PLY / OBJ) to get started",
                egui::FontId::proportional(20.0),
                egui::Color32::from_gray(150),
            );
        }

        if let Some(gpu_arc) = self.gpu.clone() {
            {
                let mut gpu = gpu_arc.lock().unwrap();
                gpu.set_viewport_px(vp_w, vp_h);
                if self.mesh_dirty {
                    if let Some(m) = self.display() {
                        gpu.upload_mesh(m);
                        self.mesh_dirty = false;
                        self.wire_dirty = true;
                    }
                }
                if self.wire_dirty && self.show_wireframe {
                    if let Some(m) = self.display() {
                        if !gpu.upload_wireframe(m, 800_000) {
                            self.status =
                                "Wireframe disabled: mesh too dense (> 800k triangles).".to_string();
                        }
                        self.wire_dirty = false;
                    }
                }
                if self.aux_dirty {
                    if let Some(m) = self.display() {
                        let nv = m.vertex_count();
                        let mut aux = vec![[0.0f32; 2]; nv];
                        if let Some(heat) = &self.heat {
                            if heat.len() == nv && self.heat_on {
                                for (a, h) in aux.iter_mut().zip(heat.iter()) {
                                    a[1] = *h;
                                }
                            }
                        }
                        if self.sel.len() == m.triangle_count() {
                            for t in 0..m.triangle_count() {
                                if self.sel[t] > 0 {
                                    for k in 0..3 {
                                        let v = m.indices[3 * t + k] as usize;
                                        aux[v][0] = 1.0;
                                    }
                                }
                            }
                        }
                        gpu.upload_aux(&aux);
                        self.aux_dirty = false;
                    }
                }
                gpu.set_frame(
                    self.camera.view_proj(),
                    self.camera.eye(),
                    self.heat_on && self.heat.is_some(),
                    if self.heat_on && self.heat_max > 0.0 {
                        1.0 / self.heat_max
                    } else {
                        0.0
                    },
                    true,
                    self.show_wireframe,
                );
                gpu.write_lines_depth(&depth_lines);
                gpu.write_lines_overlay(&overlay_lines);
                gpu.write_fills(&fills);
            }
            ui.painter()
                .add(crate::render::make_callback(gpu_arc, rect));
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        if self.gpu.is_none() {
            if let Some(rs) = frame.wgpu_render_state() {
                self.gpu = Some(Arc::new(Mutex::new(GpuState::new(
                    rs.device.clone(),
                    rs.queue.clone(),
                    rs.target_format,
                ))));
            }
        }
        let ctx = ui.ctx().clone();
        self.maintain_bvh();
        self.handle_worker(&ctx);
        egui::Panel::top("topbar").show(ui, |ui| {
            self.top_bar(ui);
        });
        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(self.status.clone());
                ui.separator();
                self.status_bar(ui);
            });
        });
        egui::Panel::left("tools")
            .default_size(320.0)
            .min_size(280.0)
            .max_size(440.0)
            .show(ui, |ui| {
                self.tool_panel(ui);
            });
        egui::CentralPanel::default().show(ui, |ui| {
            self.viewport(ui);
        });
    }
}
