use crate::camera::{Camera, ViewDir};
use crate::geom::bvh::Bvh;
use crate::geom::distance::Deviation;
use crate::geom::fitting::{
    CircleFit, FittedCircle, FittedPlane, PlaneFit, fit_circle, fit_plane, plane_basis,
};
use crate::geom::freeform::{FittedFreeform, FreeformParams};
use crate::geom::segment::{FaceGroup, GroupKind};
use crate::geom::symmetry::SymPlane;
use crate::io;
use crate::mesh::{Aabb, Mesh};
use crate::pick;
use crate::render::GpuState;
use crate::worker::{JobResult, SymmetryJobKind, Worker};
use eframe::egui;
use glam::{Quat, Vec3};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

pub(crate) const PLANE_COLORS: [[f32; 4]; 5] = [
    [1.0, 0.65, 0.1, 0.9],   // Warm Orange / Gold
    [0.15, 0.85, 0.95, 0.9], // Cyan
    [0.35, 0.9, 0.45, 0.9],  // Emerald Green
    [0.85, 0.35, 0.95, 0.9], // Purple
    [1.0, 0.3, 0.4, 0.9],    // Coral Red
];

pub(crate) const CIRCLE_COLORS: [[f32; 4]; 4] = [
    [1.0, 0.25, 0.8, 1.0], // Magenta / Pink
    [0.2, 0.8, 1.0, 1.0],  // Sky Blue
    [1.0, 0.85, 0.2, 1.0], // Amber
    [0.4, 1.0, 0.5, 1.0],  // Mint
];

pub(crate) const FREEFORM_COLORS: [[f32; 4]; 4] = [
    [0.55, 0.45, 1.0, 0.9], // Violet
    [0.35, 1.0, 0.55, 0.9], // Spring Green
    [1.0, 0.5, 0.75, 0.9],  // Pink
    [0.95, 0.75, 0.25, 0.9], // Gold
];

/// Maximum number of source points stored per freeform (fit input is
/// sub-sampled to this so parameter changes can re-fit quickly).
const FREEFORM_MAX_POINTS: usize = 96_000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Mode {
    Orbit,
    SymPickLine,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum DecMode {
    Fixed,
    Accuracy,
    Deviation,
}

/// Filter for the face group list.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum GroupFilter {
    All,
    Plane,
    Cylinder,
    Sphere,
    Freeform,
}

/// True if a group of this kind passes the active list filter.
pub(crate) fn group_matches_filter(kind: GroupKind, filter: GroupFilter) -> bool {
    match filter {
        GroupFilter::All => true,
        GroupFilter::Plane => kind == GroupKind::Plane,
        GroupFilter::Cylinder => kind == GroupKind::Cylinder,
        GroupFilter::Sphere => kind == GroupKind::Sphere,
        GroupFilter::Freeform => kind == GroupKind::Freeform,
    }
}

#[derive(Clone)]
pub struct HiddenRegion {
    pub id: u64,
    pub name: String,
    pub visible: bool,
    pub faces: Vec<u32>,
}

impl HiddenRegion {
    pub fn triangle_count(&self) -> usize {
        self.faces.len()
    }
}

#[derive(Clone)]
pub(crate) struct Snapshot {
    pub(crate) current: Arc<Mesh>,
    pub(crate) original: Arc<Mesh>,
    pub(crate) sel: Arc<Vec<u8>>,
    pub(crate) sym: Option<SymState>,
    pub(crate) plane: Option<PlaneFit>,
    pub(crate) circle: Option<CircleFit>,
    pub(crate) planes: Vec<FittedPlane>,
    pub(crate) circles: Vec<FittedCircle>,
    pub(crate) freeforms: Vec<FittedFreeform>,
    pub(crate) selected_plane_id: Option<u64>,
    pub(crate) selected_circle_id: Option<u64>,
    pub(crate) selected_freeform_id: Option<u64>,
    pub(crate) hidden_regions: Vec<HiddenRegion>,
    pub(crate) base_mesh: Option<Arc<Mesh>>,
}

#[derive(Clone, Copy)]
pub(crate) struct SymState {
    pub(crate) plane: SymPlane,
    pub(crate) rms: f64,
    pub(crate) show: bool,
}

pub struct App {
    pub(crate) gpu: Option<Arc<Mutex<GpuState>>>,
    pub(crate) camera: Camera,
    pub(crate) current: Option<Arc<Mesh>>,
    pub(crate) original: Option<Arc<Mesh>>,
    pub(crate) preview: Option<Arc<Mesh>>,
    pub(crate) preview_error: f32,
    pub(crate) sel: Arc<Vec<u8>>,
    pub(crate) sel_count: usize,
    pub(crate) bvh: Option<Arc<Bvh>>,
    pub(crate) bbox: Aabb,
    pub(crate) worker: Worker,
    pub(crate) mode: Mode,
    pub(crate) brush_radius: f32,
    pub(crate) dec_mode: DecMode,
    pub(crate) dec_ratio: f32,
    pub(crate) dec_error_mm: f32,
    pub(crate) dec_target_acc: f32,
    pub(crate) dec_target_mm: f32,
    pub(crate) dec_lock_border: bool,
    pub(crate) dec_auto_preview: bool,
    pub(crate) dec_job: Option<u64>,
    pub(crate) dev_job: Option<u64>,
    pub(crate) sym_job: Option<u64>,
    pub(crate) bvh_job: Option<u64>,
    pub(crate) bvh_job_mesh: Option<Arc<Mesh>>,
    pub(crate) deviation: Option<Deviation>,
    pub(crate) heat: Option<Arc<Vec<f32>>>,
    pub(crate) heat_on: bool,
    pub(crate) heat_max: f32,
    pub(crate) sym: Option<SymState>,
    pub(crate) sym_pick: Vec<Vec3>,
    pub(crate) sym_exclude_selection: bool,
    pub(crate) sym_exclude_holes: bool,
    pub(crate) plane: Option<PlaneFit>,
    pub(crate) show_plane: bool,
    pub(crate) planes: Vec<FittedPlane>,
    pub(crate) selected_plane_id: Option<u64>,
    pub(crate) circle: Option<CircleFit>,
    pub(crate) show_circle: bool,
    pub(crate) circles: Vec<FittedCircle>,
    pub(crate) selected_circle_id: Option<u64>,
    pub(crate) freeforms: Vec<FittedFreeform>,
    pub(crate) selected_freeform_id: Option<u64>,
    /// (job id, freeform id) of the freeform fit currently running.
    pub(crate) freeform_job: Option<(u64, u64)>,
    pub(crate) next_obj_id: u64,
    pub(crate) hidden_regions: Vec<HiddenRegion>,
    pub(crate) hidden_mask: Vec<bool>,
    pub(crate) base_mesh: Option<Arc<Mesh>>,
    pub(crate) show_mesh: bool,
    pub(crate) show_object_browser: bool,
    pub(crate) show_wireframe: bool,
    pub(crate) show_bbox: bool,
    pub(crate) show_triad: bool,
    pub(crate) undo: Vec<Snapshot>,
    pub(crate) redo: Vec<Snapshot>,
    pub(crate) status: String,
    pub(crate) file_path: Option<PathBuf>,
    pub(crate) mesh_dirty: bool,
    pub(crate) aux_dirty: bool,
    pub(crate) wire_dirty: bool,
    pub(crate) expand_angle_deg: f32,
    pub(crate) topology: Option<Arc<crate::geom::topology::MeshTopology>>,
    pub(crate) face_groups: Vec<FaceGroup>,
    pub(crate) group_ids: Vec<i32>,
    pub(crate) groups_show: bool,
    pub(crate) groups_by_type: bool,
    pub(crate) group_angle_deg: f32,
    pub(crate) group_min_tris: f32,
    pub(crate) group_fit_tol: f32,
    pub(crate) groups_filter: GroupFilter,
    pub(crate) selected_group: Option<i32>,
    pub(crate) hover_group: Option<i32>,
    pub(crate) group_sel_additive: bool,
    pub(crate) hover_hit: Option<pick::Hit>,
    pub(crate) hover_tris: Vec<u32>,
    pub(crate) hover_radius_world: f32,
    pub(crate) hover_is_erase: bool,
    pub(crate) wheel_accum: f32,
    pub(crate) suppress_sel_drag: bool,
    pub(crate) active_section: Option<crate::ui::ToolSection>,
    pub(crate) align_slots: crate::geom::alignment::AlignmentSlots,
    pub(crate) repair_holes: Vec<crate::geom::hole_detect::HoleLoop>,
    pub(crate) repair_selected_hole: Option<usize>,
    pub(crate) repair_config: crate::geom::hole_fill::HoleFillConfig,
    pub(crate) repair_preview_active: bool,
    pub(crate) repair_preview_patch: Option<crate::geom::hole_fill::MeshPatch>,
    pub(crate) repair_health: Option<crate::geom::repair::MeshHealthReport>,
    pub(crate) repair_refine_to_references: bool,
    pub(crate) repair_solve_status: Option<String>,
    pub(crate) repair_circle_mode: crate::geom::hole_solver::CircleGuideMode,
    pub(crate) bridge_config: crate::geom::bridge::BridgeConfig,
    pub(crate) bridge_preview_active: bool,
    pub(crate) bridge_preview_patch: Option<crate::geom::hole_fill::MeshPatch>,
    pub(crate) bridge_status: Option<String>,
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
            sym_exclude_selection: true,
            sym_exclude_holes: true,
            plane: None,
            show_plane: true,
            planes: Vec::new(),
            selected_plane_id: None,
            circle: None,
            show_circle: true,
            circles: Vec::new(),
            selected_circle_id: None,
            freeforms: Vec::new(),
            selected_freeform_id: None,
            freeform_job: None,
            next_obj_id: 1,
            hidden_regions: Vec::new(),
            hidden_mask: Vec::new(),
            base_mesh: None,
            show_mesh: true,
            show_object_browser: true,
            show_wireframe: false,
            show_bbox: true,
            show_triad: true,
            undo: Vec::new(),
            redo: Vec::new(),
            status: "Open a mesh file to begin (STL, PLY or OBJ). Units are assumed to be mm."
                .to_string(),
            file_path: None,
            mesh_dirty: false,
            aux_dirty: false,
            wire_dirty: false,
            expand_angle_deg: 45.0,
            topology: None,
            face_groups: Vec::new(),
            group_ids: Vec::new(),
            groups_show: true,
            groups_by_type: false,
            group_angle_deg: 45.0,
            group_min_tris: 2.0,
            group_fit_tol: 0.0015,
            groups_filter: GroupFilter::All,
            selected_group: None,
            hover_group: None,
            group_sel_additive: true,
            hover_hit: None,
            hover_tris: Vec::new(),
            hover_radius_world: 0.0,
            hover_is_erase: false,
            wheel_accum: 0.0,
            suppress_sel_drag: false,
            active_section: None,
            align_slots: crate::geom::alignment::AlignmentSlots::default(),
            repair_holes: Vec::new(),
            repair_selected_hole: None,
            repair_config: crate::geom::hole_fill::HoleFillConfig::default(),
            repair_preview_active: true,
            repair_preview_patch: None,
            repair_health: None,
            repair_refine_to_references: false,
            repair_solve_status: None,
            repair_circle_mode: crate::geom::hole_solver::CircleGuideMode::DiskAndRim,
            bridge_config: crate::geom::bridge::BridgeConfig::default(),
            bridge_preview_active: false,
            bridge_preview_patch: None,
            bridge_status: None,
        };
        if let Some(arg) = std::env::args().nth(1) {
            let path = PathBuf::from(arg);
            if path.exists() {
                app.load_file(path);
            }
        }
        app
    }

    pub(crate) fn display(&self) -> Option<&Arc<Mesh>> {
        self.preview.as_ref().or(self.current.as_ref())
    }

    pub(crate) fn orig_mesh(&self) -> Option<&Arc<Mesh>> {
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
                planes: self.planes.clone(),
                circles: self.circles.clone(),
                freeforms: self.freeforms.clone(),
                selected_plane_id: self.selected_plane_id,
                selected_circle_id: self.selected_circle_id,
                selected_freeform_id: self.selected_freeform_id,
                hidden_regions: self.hidden_regions.clone(),
                base_mesh: self.base_mesh.clone(),
            });
            if self.undo.len() > 25 {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
    }

    pub(crate) fn undo(&mut self) {
        if let Some(s) = self.undo.pop() {
            if let (Some(current), Some(original)) = (&self.current, &self.original) {
                self.redo.push(Snapshot {
                    current: current.clone(),
                    original: original.clone(),
                    sel: self.sel.clone(),
                    sym: self.sym,
                    plane: self.plane,
                    circle: self.circle,
                    planes: self.planes.clone(),
                    circles: self.circles.clone(),
                    freeforms: self.freeforms.clone(),
                    selected_plane_id: self.selected_plane_id,
                    selected_circle_id: self.selected_circle_id,
                    selected_freeform_id: self.selected_freeform_id,
                    hidden_regions: self.hidden_regions.clone(),
                    base_mesh: self.base_mesh.clone(),
                });
                if self.redo.len() > 25 {
                    self.redo.remove(0);
                }
            }
            self.current = Some(s.current);
            self.original = Some(s.original);
            self.sel = s.sel;
            self.preview = None;
            self.sym = s.sym;
            self.plane = s.plane;
            self.circle = s.circle;
            self.planes = s.planes;
            self.circles = s.circles;
            self.freeforms = s.freeforms;
            self.selected_plane_id = s.selected_plane_id;
            self.selected_circle_id = s.selected_circle_id;
            self.selected_freeform_id = s.selected_freeform_id;
            self.hidden_regions = s.hidden_regions;
            self.base_mesh = s.base_mesh;
            self.update_hidden_mask();
            self.freeform_job = None;
            self.deviation = None;
            self.heat = None;
            self.bvh = None;
            self.topology = None;
            self.invalidate_face_groups();
            self.hover_hit = None;
            self.hover_tris.clear();
            self.repair_holes.clear();
            self.repair_selected_hole = None;
            self.repair_preview_patch = None;
            self.bridge_preview_patch = None;
            self.bridge_status = None;
            self.repair_health = None;
            self.mesh_dirty = true;
            self.aux_dirty = true;
            self.recount_sel();
            self.sync_bbox();
            self.status = format!("Undone. ({} remaining)", self.undo.len());
        }
    }

    pub(crate) fn redo(&mut self) {
        if let Some(s) = self.redo.pop() {
            if let (Some(current), Some(original)) = (&self.current, &self.original) {
                self.undo.push(Snapshot {
                    current: current.clone(),
                    original: original.clone(),
                    sel: self.sel.clone(),
                    sym: self.sym,
                    plane: self.plane,
                    circle: self.circle,
                    planes: self.planes.clone(),
                    circles: self.circles.clone(),
                    freeforms: self.freeforms.clone(),
                    selected_plane_id: self.selected_plane_id,
                    selected_circle_id: self.selected_circle_id,
                    selected_freeform_id: self.selected_freeform_id,
                    hidden_regions: self.hidden_regions.clone(),
                    base_mesh: self.base_mesh.clone(),
                });
                if self.undo.len() > 25 {
                    self.undo.remove(0);
                }
            }
            self.current = Some(s.current);
            self.original = Some(s.original);
            self.sel = s.sel;
            self.preview = None;
            self.sym = s.sym;
            self.plane = s.plane;
            self.circle = s.circle;
            self.planes = s.planes;
            self.circles = s.circles;
            self.freeforms = s.freeforms;
            self.selected_plane_id = s.selected_plane_id;
            self.selected_circle_id = s.selected_circle_id;
            self.selected_freeform_id = s.selected_freeform_id;
            self.hidden_regions = s.hidden_regions;
            self.base_mesh = s.base_mesh;
            self.update_hidden_mask();
            self.freeform_job = None;
            self.deviation = None;
            self.heat = None;
            self.bvh = None;
            self.topology = None;
            self.invalidate_face_groups();
            self.hover_hit = None;
            self.hover_tris.clear();
            self.repair_holes.clear();
            self.repair_selected_hole = None;
            self.repair_preview_patch = None;
            self.bridge_preview_patch = None;
            self.bridge_status = None;
            self.repair_health = None;
            self.mesh_dirty = true;
            self.aux_dirty = true;
            self.recount_sel();
            self.sync_bbox();
            self.status = format!("Redone. ({} remaining)", self.redo.len());
        }
    }

    pub(crate) fn recount_sel(&mut self) {
        self.sel_count = self.sel.iter().filter(|&&v| v > 0).count();
        if self.sel_count == 0 {
            self.bridge_preview_active = false;
        }
        self.update_bridge_preview();
    }

    fn sync_bbox(&mut self) {
        if let Some(m) = self.display() {
            self.bbox = m.bbox();
        }
    }

    pub(crate) fn load_file(&mut self, path: PathBuf) {
        match std::fs::read(&path) {
            Ok(bytes) => match io::load_any(&path, &bytes) {
                Ok(mut mesh) => {
                    let center = mesh.bbox().center();
                    if center.length_squared() > 1e-10 {
                        mesh.transform(Quat::IDENTITY, -center);
                    }
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
                    self.topology = None;
                    self.invalidate_face_groups();
                    self.hover_hit = None;
                    self.hover_tris.clear();
                    self.deviation = None;
                    self.heat = None;
                    self.sym = None;
                    self.sym_pick.clear();
                    self.plane = None;
                    self.circle = None;
                    self.planes.clear();
                    self.circles.clear();
                    self.freeforms.clear();
                    self.selected_plane_id = None;
                    self.selected_circle_id = None;
                    self.selected_freeform_id = None;
                    self.freeform_job = None;
                    self.next_obj_id = 1;
                    self.repair_holes.clear();
                    self.repair_selected_hole = None;
                    self.repair_preview_patch = None;
                    self.repair_health = None;
                    self.hidden_regions.clear();
                    self.base_mesh = None;
                    self.undo.clear();
                    self.redo.clear();
                    self.mesh_dirty = true;
                    self.aux_dirty = true;
                    self.wire_dirty = true;
                    self.file_path = Some(path);
                    self.status = format!(
                        "Loaded: {} triangles, {} vertices (welded). Centered at global origin.",
                        tris, verts
                    );
                }
                Err(e) => self.status = format!("Load failed: {e}"),
            },
            Err(e) => self.status = format!("Could not read file: {e}"),
        }
    }

    pub(crate) fn apply_transform(&mut self, rot: Quat, trans: Vec3) {
        self.push_snapshot();
        let mut new_current = None;
        let mut new_original = None;
        let mut new_preview = None;
        let mut new_base = None;
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
        if let Some(m) = &self.base_mesh {
            let mut m2 = (**m).clone();
            m2.transform(rot, trans);
            new_base = Some(Arc::new(m2));
        }
        self.current = new_current;
        self.original = new_original;
        self.preview = new_preview;
        self.base_mesh = new_base;
        if let Some(s) = &mut self.sym {
            s.plane.point = rot * s.plane.point + trans;
            s.plane.normal = (rot * s.plane.normal).normalize();
        }
        for p in &mut self.planes {
            p.fit.point = rot * p.fit.point + trans;
            p.fit.normal = (rot * p.fit.normal).normalize();
        }
        for c in &mut self.circles {
            c.fit.center = rot * c.fit.center + trans;
            c.fit.normal = (rot * c.fit.normal).normalize();
        }
        for f in &mut self.freeforms {
            if let Some(surf) = &mut f.surface {
                surf.transform(rot, trans);
            }
            for p in Arc::make_mut(&mut f.source_points).iter_mut() {
                *p = (rot * Vec3::from(*p) + trans).to_array();
            }
            for (a, b) in &mut f.boundary {
                *a = (rot * Vec3::from(*a) + trans).to_array();
                *b = (rot * Vec3::from(*b) + trans).to_array();
            }
        }
        for g in &mut self.face_groups {
            g.center = rot * g.center + trans;
            g.point = rot * g.point + trans;
            g.normal = (rot * g.normal).normalize_or_zero();
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
        self.topology = None;
        self.hover_hit = None;
        self.hover_tris.clear();
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.wire_dirty = true;
        self.sync_bbox();
    }

    pub(crate) fn ensure_bvh(&mut self) -> Option<Arc<Bvh>> {
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
        self.topology = None;
        self.invalidate_face_groups();
        self.hover_hit = None;
        self.hover_tris.clear();
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.wire_dirty = true;
        self.sync_bbox();
    }

    pub(crate) fn schedule_decimate(&mut self) {
        if let Some(m) = self.orig_mesh().cloned() {
            let id = self.worker.submit_decimate(
                m,
                self.dec_ratio,
                self.dec_error_mm,
                self.dec_lock_border,
            );
            self.dec_job = Some(id);
            self.status = "Simplifying preview…".to_string();
        }
    }

    pub(crate) fn run_auto_decimate(&mut self) {
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

    pub(crate) fn handle_worker(&mut self, ctx: &egui::Context) {
        let results = self.worker.poll();
        for res in results {
            match res {
                JobResult::Decimated { id, mesh, error } => {
                    if self.dec_job == Some(id) {
                        self.set_preview(mesh, error);
                        self.dec_job = None;
                        if let (Some(orig), Some(prev)) =
                            (self.orig_mesh().cloned(), self.preview.clone())
                        {
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
                    if self.freeform_job.map(|(jid, _)| jid) == Some(id) {
                        let (_, fid) = self.freeform_job.unwrap();
                        self.freeform_job = None;
                        if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == fid) {
                            f.refit_pending = false;
                        }
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
                JobResult::FreeformFit {
                    id,
                    freeform_id,
                    data,
                } => {
                    if self.freeform_job.map(|(jid, _)| jid) == Some(id) {
                        self.freeform_job = None;
                        match data {
                            Ok(d) => {
                                let boundary =
                                    crate::geom::freeform::boundary_segments(&d.surface);
                                let (rms, max_dev, fold, count) =
                                    (d.rms, d.max_dev, d.fold_ratio, d.point_count);
                                let tris = d.surface.triangle_count();
                                if let Some(f) = self
                                    .freeforms
                                    .iter_mut()
                                    .find(|f| f.id == freeform_id)
                                {
                                    let (heat, in_tol) = if let Some(bvh) = &self.bvh {
                                        let (h, _, _) = crate::geom::freeform::compute_freeform_deviation(&d.surface, bvh);
                                        let in_tol = crate::geom::freeform::calculate_in_tolerance_pct(&h, f.heat_max);
                                        (Some(h), in_tol)
                                    } else {
                                        (None, 100.0)
                                    };
                                    f.surface = Some(d.surface);
                                    f.boundary = boundary;
                                    f.rms = rms;
                                    f.max_dev = max_dev;
                                    f.fold_ratio = fold;
                                    f.point_count = count;
                                    f.heat = heat;
                                    f.in_tolerance_pct = in_tol;
                                    let ov = f.params.overshoot_mm;
                                    self.status = format!(
                                        "Freeform surface fitted: {tris} triangles, RMS {rms:.4} mm, overshoot {ov:.2} mm."
                                    );
                                    if fold > 0.15 {
                                        self.status.push_str(
                                            " Note: the selection appears to wrap around; \
                                             the fit may be inaccurate there.",
                                        );
                                    }
                                }
                            }
                            Err(message) => {
                                self.status = format!("Freeform fit failed: {message}");
                                if let Some(f) = self
                                    .freeforms
                                    .iter_mut()
                                    .find(|f| f.id == freeform_id)
                                {
                                    f.refit_pending = false;
                                }
                            }
                        }
                        // Chained refits: the next freeform with pending
                        // parameter changes gets the worker next.
                        if let Some(next_id) = self
                            .freeforms
                            .iter()
                            .find(|f| f.refit_pending)
                            .map(|f| f.id)
                        {
                            if let Some(f) = self
                                .freeforms
                                .iter_mut()
                                .find(|f| f.id == next_id)
                            {
                                f.refit_pending = false;
                            }
                            self.submit_freeform_fit(next_id);
                        }
                    }
                }
            }
        }
        let _ = ctx;
    }

    pub(crate) fn schedule_sym_auto(&mut self) {
        if let Some(m) = self.display().cloned() {
            let mask = if self.sym_exclude_selection && self.sel_count > 0 {
                Some(self.sel.clone())
            } else {
                None
            };
            self.sym_job = Some(self.worker.submit_symmetry(
                m,
                SymmetryJobKind::Auto,
                mask,
                self.sym_exclude_holes,
            ));
            self.status = "Detecting symmetry plane…".to_string();
        }
    }

    pub(crate) fn schedule_sym_from_line(&mut self, a: Vec3, b: Vec3) {
        if let Some(m) = self.display().cloned() {
            let mask = if self.sym_exclude_selection && self.sel_count > 0 {
                Some(self.sel.clone())
            } else {
                None
            };
            self.sym_job = Some(self.worker.submit_symmetry(
                m,
                SymmetryJobKind::FromLine { a, b },
                mask,
                self.sym_exclude_holes,
            ));
            self.status = "Calculating and optimizing symmetry plane from line…".to_string();
        }
    }

    pub(crate) fn schedule_sym_refine(&mut self, init: SymPlane) {
        if let Some(m) = self.display().cloned() {
            let mask = if self.sym_exclude_selection && self.sel_count > 0 {
                Some(self.sel.clone())
            } else {
                None
            };
            self.sym_job = Some(self.worker.submit_symmetry(
                m,
                SymmetryJobKind::Refine(init),
                mask,
                self.sym_exclude_holes,
            ));
            self.status = "Optimizing symmetry plane…".to_string();
        }
    }

    pub(crate) fn rotate_normal_to_axis(&mut self, axis: Vec3) {
        if let Some(n) = self
            .sym
            .map(|s| s.plane.normal)
            .or(self.plane.map(|p| p.normal))
        {
            let q = rotation_between(n, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned normal to {}.", axis_name(axis));
        }
    }

    pub(crate) fn circle_axis_to(&mut self, axis: Vec3) {
        if let Some(c) = self.circle {
            let q = rotation_between(c.normal, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned circle axis to {}.", axis_name(axis));
        }
    }

    pub(crate) fn origin_on_plane(&mut self) {
        if let Some(p) = self.plane {
            let d = p.point.dot(p.normal);
            self.apply_transform(Quat::IDENTITY, -p.normal * d);
            self.status = "Origin moved onto the fitted plane.".to_string();
        }
    }

    pub(crate) fn origin_at_symmetry(&mut self) {
        if let Some(s) = self.sym {
            let d = s.plane.point.dot(s.plane.normal);
            self.apply_transform(Quat::IDENTITY, -s.plane.normal * d);
            self.status =
                "Mesh translated so the symmetry plane passes through the origin.".to_string();
        }
    }

    pub(crate) fn origin_at_circle_center(&mut self) {
        if let Some(c) = self.circle {
            self.apply_transform(Quat::IDENTITY, -c.center);
            self.status = "Origin moved to the circle center.".to_string();
        }
    }

    pub(crate) fn center_axes(&mut self, x: bool, y: bool, z: bool) {
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

    pub(crate) fn fit_plane_from_selection(&mut self) {
        let points = self.selection_points();
        match points.map(|p| fit_plane(&p)) {
            Some(Some(f)) => {
                self.push_snapshot();
                let id = self.next_obj_id;
                self.next_obj_id += 1;
                let col_idx = self.planes.len() % PLANE_COLORS.len();
                let color = PLANE_COLORS[col_idx];
                let name = format!("Plane {}", self.planes.len() + 1);
                self.planes.push(FittedPlane {
                    id,
                    name,
                    fit: f,
                    visible: true,
                    color,
                });
                self.selected_plane_id = Some(id);
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

    pub(crate) fn fit_circle_from_selection(&mut self) {
        let points = self.selection_points();
        match points.map(|p| fit_circle(&p)) {
            Some(Some(c)) => {
                self.push_snapshot();
                let id = self.next_obj_id;
                self.next_obj_id += 1;
                let col_idx = self.circles.len() % CIRCLE_COLORS.len();
                let color = CIRCLE_COLORS[col_idx];
                let name = format!("Circle {}", self.circles.len() + 1);
                self.circles.push(FittedCircle {
                    id,
                    name,
                    fit: c,
                    visible: true,
                    color,
                });
                self.selected_circle_id = Some(id);
                self.circle = Some(c);
                self.show_circle = true;
                self.status = if c.cylinder {
                    format!(
                        "Circle fitted as cylinder cross-section: R = {:.4} mm (D = {:.4} mm), radial RMS {:.4} mm.",
                        c.radius,
                        c.radius * 2.0,
                        c.radial_rms.sqrt()
                    )
                } else {
                    format!(
                        "Circle fitted: R = {:.4} mm, radial RMS {:.4} mm.",
                        c.radius,
                        c.radial_rms.sqrt()
                    )
                };
            }
            Some(None) => self.status = "Circle fit failed on selection.".to_string(),
            None => self.status = "Select faces first.".to_string(),
        }
    }

    /// Sub-sampled selection points for the freeform fitter.
    fn selection_points_capped(&self, max_points: usize) -> Option<Vec<[f32; 3]>> {
        let pts = self.selection_points()?;
        if pts.len() > max_points {
            let stride = pts.len().div_ceil(max_points);
            Some(pts.iter().step_by(stride).copied().collect())
        } else {
            Some(pts)
        }
    }

    /// Fits a freeform surface onto the current face selection.
    pub(crate) fn fit_freeform_from_selection(&mut self) {
        let Some(points) = self.selection_points_capped(FREEFORM_MAX_POINTS) else {
            self.status = "Select faces first.".to_string();
            return;
        };
        if points.len() < 12 {
            self.status = "Selection too small for a freeform surface.".to_string();
            return;
        }
        let mut min = [f32::MAX; 3];
        let mut max = [f32::MIN; 3];
        for p in &points {
            for k in 0..3 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        let extent = (max[0] - min[0])
            .max(max[1] - min[1])
            .max(max[2] - min[2])
            .max(1e-6);

        self.push_snapshot();
        let id = self.next_obj_id;
        self.next_obj_id += 1;
        let col_idx = self.freeforms.len() % FREEFORM_COLORS.len();
        let color = FREEFORM_COLORS[col_idx];
        let name = format!("Freeform {}", self.freeforms.len() + 1);
        self.freeforms.push(FittedFreeform {
            id,
            name,
            visible: true,
            color,
            source_points: Arc::new(points),
            params: FreeformParams::default_for(extent),
            surface: None,
            boundary: Vec::new(),
            rms: 0.0,
            max_dev: 0.0,
            fold_ratio: 0.0,
            point_count: 0,
            refit_pending: false,
            heat_on: false,
            heat_max: 0.20,
            heat_gradient: crate::geom::freeform::FreeformGradient::TrafficLight,
            heat: None,
            in_tolerance_pct: 100.0,
        });
        self.selected_freeform_id = Some(id);
        self.status = "Fitting freeform surface…".to_string();
        self.submit_freeform_fit(id);
    }

    fn submit_freeform_fit(&mut self, freeform_id: u64) {
        if self.freeform_job.is_some() {
            // One fit at a time; queue this one until the running fit lands.
            if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id) {
                f.refit_pending = true;
            }
            return;
        }
        let Some(f) = self.freeforms.iter().find(|f| f.id == freeform_id) else {
            return;
        };
        let job = self
            .worker
            .submit_freeform_fit(freeform_id, f.source_points.clone(), f.params);
        self.freeform_job = Some((job, freeform_id));
    }

    /// Re-fits a freeform after a parameter change. If a fit is already
    /// running, the re-fit is queued and executed once it finishes.
    pub(crate) fn schedule_freeform_refit(&mut self, freeform_id: u64) {
        self.submit_freeform_fit(freeform_id);
    }

    pub(crate) fn set_freeform_params(&mut self, freeform_id: u64, params: FreeformParams) {
        if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id) {
            if f.params != params {
                f.params = params;
                self.schedule_freeform_refit(freeform_id);
            }
        }
    }

    pub(crate) fn select_freeform(&mut self, id: u64) {
        self.selected_freeform_id = Some(id);
    }

    pub(crate) fn delete_freeform(&mut self, id: u64) {
        self.push_snapshot();
        self.freeforms.retain(|f| f.id != id);
        if self.selected_freeform_id == Some(id) {
            self.selected_freeform_id = self.freeforms.last().map(|f| f.id);
        }
        if self.freeform_job.map(|(_, fid)| fid) == Some(id) {
            self.freeform_job = None;
        }
        self.status = "Freeform surface deleted.".to_string();
    }

    pub(crate) fn export_freeform_id(&mut self, freeform_id: u64) {
        if let Some(f) = self.freeforms.iter().find(|f| f.id == freeform_id) {
            match crate::export::export_freeform_dialog(f) {
                Ok(msg) => self.status = msg,
                Err(e) => self.status = format!("Export failed: {e}"),
            }
        }
    }

    pub(crate) fn ensure_freeform_heat(&mut self, freeform_id: u64) {
        if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id) {
            if f.heat.is_none() {
                if let Some(surf) = &f.surface {
                    if let Some(bvh) = &self.bvh {
                        let (heat, _, _) = crate::geom::freeform::compute_freeform_deviation(surf, bvh);
                        let in_tol = crate::geom::freeform::calculate_in_tolerance_pct(&heat, f.heat_max);
                        f.heat = Some(heat);
                        f.in_tolerance_pct = in_tol;
                    }
                }
            }
        }
    }

    pub(crate) fn select_plane(&mut self, id: u64) {
        self.selected_plane_id = Some(id);
        if let Some(p) = self.planes.iter().find(|p| p.id == id) {
            self.plane = Some(p.fit);
            self.show_plane = p.visible;
        }
    }

    pub(crate) fn delete_plane(&mut self, id: u64) {
        self.push_snapshot();
        self.planes.retain(|p| p.id != id);
        if self.selected_plane_id == Some(id) {
            self.selected_plane_id = self.planes.last().map(|p| p.id);
            self.plane = self.planes.last().map(|p| p.fit);
        }
        if self.align_slots.x == Some(crate::geom::alignment::FeatureRef::Plane(id)) {
            self.align_slots.x = None;
        }
        if self.align_slots.y == Some(crate::geom::alignment::FeatureRef::Plane(id)) {
            self.align_slots.y = None;
        }
        if self.align_slots.z == Some(crate::geom::alignment::FeatureRef::Plane(id)) {
            self.align_slots.z = None;
        }
        if matches!(self.align_slots.origin, crate::geom::alignment::OriginRef::Plane(pid) if pid == id)
        {
            self.align_slots.origin = crate::geom::alignment::OriginRef::FromAssignedFeatures;
        }
        self.status = "Plane deleted.".to_string();
    }

    pub(crate) fn rotate_plane_normal_to_axis(&mut self, plane_id: u64, axis: Vec3) {
        if let Some(p) = self.planes.iter().find(|p| p.id == plane_id) {
            let normal = p.fit.normal;
            let q = rotation_between(normal, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned normal to {}.", axis_name(axis));
        }
    }

    pub(crate) fn origin_on_plane_id(&mut self, plane_id: u64) {
        if let Some(p) = self.planes.iter().find(|p| p.id == plane_id) {
            let normal = p.fit.normal;
            let d = p.fit.point.dot(normal);
            self.apply_transform(Quat::IDENTITY, -normal * d);
            self.status = "Origin moved onto the fitted plane.".to_string();
        }
    }

    pub(crate) fn select_circle(&mut self, id: u64) {
        self.selected_circle_id = Some(id);
        if let Some(c) = self.circles.iter().find(|c| c.id == id) {
            self.circle = Some(c.fit);
            self.show_circle = c.visible;
        }
    }

    pub(crate) fn delete_circle(&mut self, id: u64) {
        self.push_snapshot();
        self.circles.retain(|c| c.id != id);
        if self.selected_circle_id == Some(id) {
            self.selected_circle_id = self.circles.last().map(|c| c.id);
            self.circle = self.circles.last().map(|c| c.fit);
        }
        if self.align_slots.x == Some(crate::geom::alignment::FeatureRef::Circle(id)) {
            self.align_slots.x = None;
        }
        if self.align_slots.y == Some(crate::geom::alignment::FeatureRef::Circle(id)) {
            self.align_slots.y = None;
        }
        if self.align_slots.z == Some(crate::geom::alignment::FeatureRef::Circle(id)) {
            self.align_slots.z = None;
        }
        if matches!(self.align_slots.origin, crate::geom::alignment::OriginRef::CircleCenter(cid) if cid == id)
        {
            self.align_slots.origin = crate::geom::alignment::OriginRef::FromAssignedFeatures;
        }
        self.status = "Circle deleted.".to_string();
    }

    pub(crate) fn rotate_circle_axis_to(&mut self, circle_id: u64, axis: Vec3) {
        if let Some(c) = self.circles.iter().find(|c| c.id == circle_id) {
            let normal = c.fit.normal;
            let q = rotation_between(normal, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned circle axis to {}.", axis_name(axis));
        }
    }

    pub(crate) fn origin_at_circle_id(&mut self, circle_id: u64) {
        if let Some(c) = self.circles.iter().find(|c| c.id == circle_id) {
            let center = c.fit.center;
            self.apply_transform(Quat::IDENTITY, -center);
            self.status = "Origin moved to the circle center.".to_string();
        }
    }

    pub(crate) fn export_plane_id(&mut self, plane_id: u64) {
        if let Some(p) = self.planes.iter().find(|p| p.id == plane_id) {
            let size = (self.bbox.diagonal() * 0.4).clamp(20.0, 500.0);
            match crate::export::export_plane_dialog(p, size) {
                Ok(msg) => self.status = msg,
                Err(e) => self.status = format!("Export failed: {e}"),
            }
        }
    }

    pub(crate) fn export_circle_id(&mut self, circle_id: u64) {
        if let Some(c) = self.circles.iter().find(|c| c.id == circle_id) {
            match crate::export::export_circle_dialog(c) {
                Ok(msg) => self.status = msg,
                Err(e) => self.status = format!("Export failed: {e}"),
            }
        }
    }

    pub(crate) fn export_all_references(&mut self) {
        let size = (self.bbox.diagonal() * 0.4).clamp(20.0, 500.0);
        match crate::export::export_all_references_dialog(&self.planes, &self.circles, size) {
            Ok(msg) => self.status = msg,
            Err(e) => self.status = format!("Export failed: {e}"),
        }
    }

    pub(crate) fn feature_name(&self, feat: crate::geom::alignment::FeatureRef) -> String {
        <Self as crate::geom::alignment::AlignmentGeometrySource>::get_feature_name(self, feat)
    }

    pub(crate) fn feature_assigned_axis(
        &self,
        feat: crate::geom::alignment::FeatureRef,
    ) -> Option<crate::geom::alignment::AxisChoice> {
        if self.align_slots.x == Some(feat) {
            Some(crate::geom::alignment::AxisChoice::X)
        } else if self.align_slots.y == Some(feat) {
            Some(crate::geom::alignment::AxisChoice::Y)
        } else if self.align_slots.z == Some(feat) {
            Some(crate::geom::alignment::AxisChoice::Z)
        } else {
            None
        }
    }

    pub(crate) fn is_feature_origin(&self, feat: crate::geom::alignment::FeatureRef) -> bool {
        match (feat, self.align_slots.origin) {
            (
                crate::geom::alignment::FeatureRef::SymmetryPlane,
                crate::geom::alignment::OriginRef::SymmetryPlane,
            ) => true,
            (
                crate::geom::alignment::FeatureRef::Plane(id),
                crate::geom::alignment::OriginRef::Plane(oid),
            ) => id == oid,
            (
                crate::geom::alignment::FeatureRef::Circle(id),
                crate::geom::alignment::OriginRef::CircleCenter(oid),
            ) => id == oid,
            _ => false,
        }
    }

    pub(crate) fn toggle_assign_feature(
        &mut self,
        feat: crate::geom::alignment::FeatureRef,
        axis: crate::geom::alignment::AxisChoice,
    ) {
        let is_current = match axis {
            crate::geom::alignment::AxisChoice::X => self.align_slots.x == Some(feat),
            crate::geom::alignment::AxisChoice::Y => self.align_slots.y == Some(feat),
            crate::geom::alignment::AxisChoice::Z => self.align_slots.z == Some(feat),
        };

        // Remove this feature from any other slots first
        if self.align_slots.x == Some(feat) {
            self.align_slots.x = None;
        }
        if self.align_slots.y == Some(feat) {
            self.align_slots.y = None;
        }
        if self.align_slots.z == Some(feat) {
            self.align_slots.z = None;
        }

        if !is_current {
            match axis {
                crate::geom::alignment::AxisChoice::X => self.align_slots.x = Some(feat),
                crate::geom::alignment::AxisChoice::Y => self.align_slots.y = Some(feat),
                crate::geom::alignment::AxisChoice::Z => self.align_slots.z = Some(feat),
            }
        }
    }

    pub(crate) fn toggle_origin_feature(&mut self, feat: crate::geom::alignment::FeatureRef) {
        if self.is_feature_origin(feat) {
            self.align_slots.origin = crate::geom::alignment::OriginRef::FromAssignedFeatures;
        } else {
            self.align_slots.origin = match feat {
                crate::geom::alignment::FeatureRef::SymmetryPlane => {
                    crate::geom::alignment::OriginRef::SymmetryPlane
                }
                crate::geom::alignment::FeatureRef::Plane(id) => {
                    crate::geom::alignment::OriginRef::Plane(id)
                }
                crate::geom::alignment::FeatureRef::Circle(id) => {
                    crate::geom::alignment::OriginRef::CircleCenter(id)
                }
            };
        }
    }

    pub(crate) fn clear_alignment_slots(&mut self) {
        self.align_slots = crate::geom::alignment::AlignmentSlots::default();
        self.status = "Cleared feature alignment slots.".to_string();
    }

    pub(crate) fn align_to_features(&mut self) -> bool {
        match crate::geom::alignment::compute_alignment(&self.align_slots, self) {
            Ok(t) => {
                self.apply_transform(t.rotation, t.translation);
                self.status = t.summary;
                true
            }
            Err(e) => {
                self.status = format!("Alignment failed: {e}");
                false
            }
        }
    }

    pub(crate) fn auto_assign_alignment_from_selection(&mut self) {
        let mut assigned_any = false;

        if self.sym.is_some() && self.align_slots.x.is_none() {
            self.align_slots.x = Some(crate::geom::alignment::FeatureRef::SymmetryPlane);
            assigned_any = true;
        }

        // Circle: prefer selected_circle_id, or first circle
        let target_circle = self
            .selected_circle_id
            .or_else(|| self.circles.first().map(|c| c.id));
        if let Some(cid) = target_circle {
            if self.align_slots.z.is_none() {
                self.align_slots.z = Some(crate::geom::alignment::FeatureRef::Circle(cid));
                assigned_any = true;
            }
            if matches!(
                self.align_slots.origin,
                crate::geom::alignment::OriginRef::FromAssignedFeatures
            ) {
                self.align_slots.origin = crate::geom::alignment::OriginRef::CircleCenter(cid);
            }
        }

        // Plane: prefer selected_plane_id, or first plane
        let target_plane = self
            .selected_plane_id
            .or_else(|| self.planes.first().map(|p| p.id));
        if let Some(pid) = target_plane {
            if self.align_slots.y.is_none() {
                self.align_slots.y = Some(crate::geom::alignment::FeatureRef::Plane(pid));
                assigned_any = true;
            } else if self.align_slots.x.is_none() {
                self.align_slots.x = Some(crate::geom::alignment::FeatureRef::Plane(pid));
                assigned_any = true;
            }
        }

        if assigned_any {
            self.status = "Auto-assigned features to alignment slots.".to_string();
        } else {
            self.status = "No unassigned features available to auto-assign.".to_string();
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
        if pts.is_empty() { None } else { Some(pts) }
    }

    pub(crate) fn clear_selection(&mut self) {
        if let Some(m) = self.display() {
            if self.sel.len() == m.triangle_count() {
                let sel = Arc::make_mut(&mut self.sel);
                sel.fill(0);
            }
            self.sel_count = 0;
            self.aux_dirty = true;
            self.bridge_preview_patch = None;
            self.bridge_status = None;
            self.bridge_preview_active = false;
        }
    }

    pub(crate) fn update_hidden_mask(&mut self) {
        let nt = self.current.as_ref().map(|m| m.triangle_count()).unwrap_or(0);
        let mut mask = vec![false; nt];
        for hr in &self.hidden_regions {
            if !hr.visible {
                for &f in &hr.faces {
                    if (f as usize) < nt {
                        mask[f as usize] = true;
                    }
                }
            }
        }
        self.hidden_mask = mask;
    }

    pub(crate) fn is_face_hidden(&self, tri: u32) -> bool {
        let t = tri as usize;
        t < self.hidden_mask.len() && self.hidden_mask[t]
    }

    #[allow(dead_code)]
    pub(crate) fn sync_visible_mesh(&mut self) {
        self.update_hidden_mask();
        self.mesh_dirty = true;
        self.wire_dirty = true;
        self.aux_dirty = true;
    }

    pub(crate) fn hide_selection(&mut self) {
        if self.sel_count == 0 {
            return;
        }
        let Some(m) = self.current.clone() else { return; };
        let nt = m.triangle_count();
        if self.sel.len() != nt {
            return;
        }

        let mut selected_faces = Vec::with_capacity(self.sel_count);
        for t in 0..nt {
            if self.sel[t] > 0 {
                selected_faces.push(t as u32);
            }
        }

        if selected_faces.is_empty() {
            return;
        }

        let mut prospective_hidden = 0usize;
        for t in 0..nt {
            if self.is_face_hidden(t as u32) || self.sel[t] > 0 {
                prospective_hidden += 1;
            }
        }
        if prospective_hidden >= nt {
            self.status = "Cannot hide all faces of the mesh.".to_string();
            return;
        }

        self.push_snapshot();

        let hidden_tris = selected_faces.len();
        let id = self.next_obj_id;
        self.next_obj_id += 1;
        let name = format!("Hidden Region {}", self.hidden_regions.len() + 1);

        self.hidden_regions.push(HiddenRegion {
            id,
            name: name.clone(),
            visible: false,
            faces: selected_faces,
        });

        self.sel = Arc::new(vec![0u8; nt]);
        self.sel_count = 0;
        self.hover_hit = None;
        self.hover_tris.clear();
        self.update_hidden_mask();
        self.mesh_dirty = true;
        self.wire_dirty = true;
        self.aux_dirty = true;

        self.status = format!("Hidden {hidden_tris} faces into '{name}'.");
    }

    pub(crate) fn toggle_hidden_region_visibility(&mut self, id: u64) {
        let idx = self.hidden_regions.iter().position(|r| r.id == id);
        if let Some(i) = idx {
            self.push_snapshot();
            let hr = &mut self.hidden_regions[i];
            hr.visible = !hr.visible;
            let vis = hr.visible;
            let name = hr.name.clone();
            self.update_hidden_mask();
            self.mesh_dirty = true;
            self.wire_dirty = true;
            self.aux_dirty = true;
            self.status = if vis {
                format!("Showing '{name}'.")
            } else {
                format!("Hiding '{name}'.")
            };
        }
    }

    pub(crate) fn restore_hidden_region(&mut self, id: u64) {
        let idx = self.hidden_regions.iter().position(|r| r.id == id);
        if let Some(i) = idx {
            self.push_snapshot();
            let hr = self.hidden_regions.remove(i);
            self.update_hidden_mask();
            self.mesh_dirty = true;
            self.wire_dirty = true;
            self.aux_dirty = true;
            self.status = format!("Restored '{}' back into the mesh.", hr.name);
        }
    }

    pub(crate) fn restore_all_hidden_regions(&mut self) {
        if self.hidden_regions.is_empty() {
            return;
        }
        self.push_snapshot();
        self.hidden_regions.clear();
        self.update_hidden_mask();
        self.mesh_dirty = true;
        self.wire_dirty = true;
        self.aux_dirty = true;
        self.status = "All hidden regions restored to mesh.".to_string();
    }

    pub(crate) fn delete_hidden_region(&mut self, id: u64) {
        let idx = self.hidden_regions.iter().position(|r| r.id == id);
        if let Some(i) = idx {
            self.push_snapshot();
            let hr = self.hidden_regions.remove(i);
            let Some(m) = self.current.clone() else { return; };
            let mut del_sel = vec![0u8; m.triangle_count()];
            for &f in &hr.faces {
                if (f as usize) < del_sel.len() {
                    del_sel[f as usize] = 1;
                }
            }
            let (kept_mesh, _) = m.split_by_selection(&del_sel);
            self.set_mesh_modified(kept_mesh, format!("Deleted '{}'.", hr.name));
            self.hidden_regions.clear();
            self.update_hidden_mask();
        }
    }

    pub(crate) fn ensure_topology(&mut self) -> Option<Arc<crate::geom::topology::MeshTopology>> {
        if let Some(t) = &self.topology {
            return Some(t.clone());
        }
        let m = self.display()?.clone();
        let topo = Arc::new(crate::geom::topology::MeshTopology::build(&m));
        self.topology = Some(topo.clone());
        Some(topo)
    }

    pub(crate) fn grow_selection(&mut self) {
        if self.sel_count == 0 {
            return;
        }
        if let Some(topo) = self.ensure_topology() {
            self.push_snapshot();
            let angle_rad = self.expand_angle_deg.to_radians();
            let new_sel = crate::geom::topology::grow_selection(&topo, &self.sel, angle_rad);
            self.sel = Arc::new(new_sel);
            self.recount_sel();
            self.aux_dirty = true;
            self.status = format!(
                "Selection expanded (crease threshold: {:.1}°): {} faces",
                self.expand_angle_deg, self.sel_count
            );
        }
    }

    pub(crate) fn shrink_selection(&mut self) {
        if self.sel_count == 0 {
            return;
        }
        if let Some(topo) = self.ensure_topology() {
            self.push_snapshot();
            let new_sel = crate::geom::topology::shrink_selection(&topo, &self.sel);
            self.sel = Arc::new(new_sel);
            self.recount_sel();
            self.aux_dirty = true;
            self.status = format!("Selection shrunk: {} faces", self.sel_count);
        }
    }

    /// Drops all face group data (e.g. because the mesh topology changed).
    pub(crate) fn invalidate_face_groups(&mut self) {
        self.face_groups.clear();
        self.group_ids.clear();
        self.selected_group = None;
        self.hover_group = None;
    }

    pub(crate) fn detect_face_groups(&mut self) {
        let Some(m) = self.display().cloned() else { return; };
        let Some(topo) = self.ensure_topology() else { return; };
        let min_tris = self.group_min_tris.round().max(1.0) as usize;
        let (groups, ids) = crate::geom::segment::segment_faces(
            &m,
            &topo,
            self.group_angle_deg,
            min_tris,
            self.group_fit_tol,
        );
        let planes = groups.iter().filter(|g| g.kind == GroupKind::Plane).count();
        let cylinders = groups.iter().filter(|g| g.kind == GroupKind::Cylinder).count();
        let spheres = groups.iter().filter(|g| g.kind == GroupKind::Sphere).count();
        let other = groups.len() - planes - cylinders - spheres;
        self.selected_group = None;
        self.hover_group = None;
        if groups.is_empty() {
            self.status =
                "No face groups found. Try a larger crease angle or smaller min faces."
                    .to_string();
        } else {
            self.status = format!(
                "{} face groups: {} planes, {} cylinders, {} spheres, {} other",
                groups.len(),
                planes,
                cylinders,
                spheres,
                other
            );
        }
        self.face_groups = groups;
        self.group_ids = ids;
        self.groups_show = true;
        self.aux_dirty = true;
    }

    pub(crate) fn clear_face_groups(&mut self) {
        self.invalidate_face_groups();
        self.aux_dirty = true;
        self.status = "Face groups cleared.".to_string();
    }

    /// Selects the faces of a group. With `additive` the group's faces are
    /// merged into the current selection instead of replacing it.
    pub(crate) fn select_group_faces(&mut self, id: i32, additive: bool) {
        let Some(m) = self.display() else { return; };
        if self.group_ids.len() != m.triangle_count() {
            return;
        }
        if !self.face_groups.iter().any(|g| g.id == id) {
            return;
        }
        let tris = m.triangle_count();
        let prev = if additive && self.sel.len() == tris {
            self.sel_count
        } else {
            0
        };
        self.push_snapshot();
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else { return; };
        let kind = g.kind;
        let mut sel = if additive && self.sel.len() == tris {
            (*self.sel).clone()
        } else {
            vec![0u8; tris]
        };
        for &t in &g.tris {
            sel[t as usize] = 1;
        }
        self.sel = Arc::new(sel);
        self.recount_sel();
        self.aux_dirty = true;
        self.status = if additive && prev > 0 {
            format!(
                "Group #{} ({}): +{} faces added, {} total selected.",
                id + 1,
                kind.label(),
                self.sel_count - prev,
                self.sel_count
            )
        } else {
            format!(
                "Group #{} ({}): {} faces selected.",
                id + 1,
                kind.label(),
                self.sel_count
            )
        };
    }

    /// Selects the group's faces and pushes a fitted plane into the objects list.
    pub(crate) fn fit_group_plane(&mut self, id: i32) {
        self.select_group_faces(id, false);
        self.fit_plane_from_selection();
    }

    /// Selects the group's faces and fits a freeform surface onto them.
    pub(crate) fn fit_group_freeform(&mut self, id: i32) {
        self.select_group_faces(id, false);
        self.fit_freeform_from_selection();
    }

    pub(crate) fn align_group_axis(&mut self, id: i32, axis: Vec3) {
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else { return; };
        let q = rotation_between(g.normal, axis);
        let kind = g.kind;
        self.apply_transform(q, Vec3::ZERO);
        self.status = format!(
            "Group #{} ({}) aligned to {}.",
            id + 1,
            kind.label(),
            axis_name(axis)
        );
    }

    pub(crate) fn origin_on_group_plane(&mut self, id: i32) {
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else { return; };
        let d = g.point.dot(g.normal);
        self.apply_transform(Quat::IDENTITY, -g.normal * d);
        self.status = "Origin moved onto the group plane.".to_string();
    }

    pub(crate) fn origin_on_group_axis(&mut self, id: i32) {
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else { return; };
        let p = g.point - g.normal * g.point.dot(g.normal);
        self.apply_transform(Quat::IDENTITY, -p);
        self.status = "Origin moved onto the cylinder axis.".to_string();
    }

    /// Double-click pick: selects the complete face group under the cursor.
    /// If no face group exists there (or none were detected), falls back to a
    /// Meshmixer-style region select grown from the clicked triangle using the
    /// brush crease angle.
    pub(crate) fn select_region_under(&mut self, tri: u32) {
        let Some(m) = self.display() else { return; };
        let tris = m.triangle_count();
        if (tri as usize) < tris && self.group_ids.len() == tris {
            let gid = self.group_ids[tri as usize];
            if gid >= 0 && self.face_groups.iter().any(|g| g.id == gid) {
                self.select_group_faces(gid, self.group_sel_additive);
                return;
            }
        }
        if let Some(topo) = self.ensure_topology() {
            let region =
                crate::geom::topology::flood_select_from(&topo, tri, self.expand_angle_deg.to_radians());
            if region.is_empty() {
                return;
            }
            self.push_snapshot();
            let additive = self.group_sel_additive && self.sel.len() == tris;
            let mut sel = if additive {
                (*self.sel).clone()
            } else {
                vec![0u8; tris]
            };
            let mut added = 0usize;
            for &t in &region {
                if sel[t as usize] == 0 {
                    sel[t as usize] = 1;
                    added += 1;
                }
            }
            self.sel = Arc::new(sel);
            self.recount_sel();
            self.aux_dirty = true;
            self.status = if additive {
                format!(
                    "Region added (crease angle {:.1}°): +{} faces, {} total selected",
                    self.expand_angle_deg,
                    added,
                    self.sel_count
                )
            } else {
                format!(
                    "Region selected (crease angle {:.1}°): {} faces",
                    self.expand_angle_deg,
                    added
                )
            };
        }
    }

    pub(crate) fn apply_preview(&mut self) {
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
            self.topology = None;
            self.invalidate_face_groups();
            self.hover_hit = None;
            self.hover_tris.clear();
            self.mesh_dirty = true;
            self.aux_dirty = true;
            self.wire_dirty = true;
            self.status = "Decimation applied. New mesh is the reference.".to_string();
        }
    }

    pub(crate) fn discard_preview(&mut self) {
        self.preview = None;
        self.deviation = None;
        self.heat = None;
        self.sel = Arc::new(Vec::new());
        self.sel_count = 0;
        self.bvh = None;
        self.topology = None;
        self.invalidate_face_groups();
        self.hover_hit = None;
        self.hover_tris.clear();
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.wire_dirty = true;
        self.sync_bbox();
        self.status = "Preview discarded.".to_string();
    }

    pub(crate) fn reset_mesh(&mut self) {
        if let Some(orig) = self.original.clone() {
            self.push_snapshot();
            let tris = orig.triangle_count();
            self.current = Some(orig);
            self.preview = None;
            self.sel = Arc::new(vec![0u8; tris]);
            self.sel_count = 0;
            self.bvh = None;
            self.topology = None;
            self.invalidate_face_groups();
            self.hover_hit = None;
            self.hover_tris.clear();
            self.deviation = None;
            self.heat = None;
            self.hidden_regions.clear();
            self.update_hidden_mask();
            self.mesh_dirty = true;
            self.aux_dirty = true;
            self.wire_dirty = true;
            self.sync_bbox();
            self.status = "Reset to original mesh.".to_string();
        }
    }

    pub(crate) fn set_mesh_modified(&mut self, next_mesh: Mesh, status_msg: String) {
        let tris = next_mesh.triangle_count();
        let m = Arc::new(next_mesh);
        self.bbox = m.bbox();
        self.current = Some(m.clone());
        self.preview = None;
        self.sel = Arc::new(vec![0u8; tris]);
        self.sel_count = 0;
        self.bvh = None;
        self.topology = None;
        self.invalidate_face_groups();
        self.hover_hit = None;
        self.hover_tris.clear();
        self.deviation = None;
        self.heat = None;
        self.hidden_regions.clear();
        self.update_hidden_mask();
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.wire_dirty = true;
        self.status = status_msg;
        self.refresh_repair();
    }

    pub(crate) fn refresh_repair(&mut self) {
        if let Some(m) = self.display().cloned() {
            let holes = crate::geom::hole_detect::detect_holes(&m);
            let health = crate::geom::repair::analyze_mesh(&m);
            self.repair_holes = holes;
            self.repair_health = Some(health);
            if let Some(sel) = self.repair_selected_hole {
                if sel >= self.repair_holes.len() {
                    self.repair_selected_hole = None;
                }
            }
            self.update_hole_preview();
        }
    }

    pub(crate) fn set_hole_fill_config(&mut self, config: crate::geom::hole_fill::HoleFillConfig) {
        self.repair_config = config;
        self.update_hole_preview();
    }

    pub(crate) fn set_hole_preview_active(&mut self, active: bool) {
        self.repair_preview_active = active;
        self.update_hole_preview();
    }

    pub(crate) fn select_hole(&mut self, idx: Option<usize>) {
        self.repair_selected_hole = idx;
        self.update_hole_preview();
    }

    pub(crate) fn focus_selected_hole(&mut self) {
        if let Some(idx) = self.repair_selected_hole {
            if let Some(hole) = self.repair_holes.get(idx) {
                self.camera.fit(&hole.bbox);
            }
        }
    }

    pub(crate) fn active_reference_geometries(&self) -> Vec<crate::geom::hole_solver::ReferenceGeometry> {
        let mut refs = Vec::new();
        for p in &self.planes {
            if p.visible {
                refs.push(crate::geom::hole_solver::ReferenceGeometry::from_plane(p));
            }
        }
        for c in &self.circles {
            if c.visible {
                refs.push(crate::geom::hole_solver::ReferenceGeometry::from_circle(c, self.repair_circle_mode));
            }
        }
        if refs.is_empty() {
            if let Some(p) = self.plane {
                refs.push(crate::geom::hole_solver::ReferenceGeometry::Plane {
                    id: 0,
                    name: "Active Plane".to_string(),
                    point: p.point,
                    normal: p.normal,
                });
            }
            if let Some(c) = self.circle {
                refs.push(crate::geom::hole_solver::ReferenceGeometry::Circle {
                    id: 0,
                    name: "Active Circle".to_string(),
                    center: c.center,
                    normal: c.normal,
                    radius: c.radius,
                    mode: self.repair_circle_mode,
                });
            }
        }
        refs
    }

    pub(crate) fn set_hole_circle_mode(&mut self, mode: crate::geom::hole_solver::CircleGuideMode) {
        self.repair_circle_mode = mode;
        self.update_hole_preview();
    }

    pub(crate) fn solve_best_hole_fill(&mut self) {
        let refs = self.active_reference_geometries();
        if refs.is_empty() {
            self.status = "No fitted planes or circles available to guide hole fill.".to_string();
            self.repair_solve_status = Some("No planes/circles available to guide solver.".to_string());
            return;
        }

        let m_opt = self.display().cloned();
        let hole_opt = self
            .repair_selected_hole
            .and_then(|idx| self.repair_holes.get(idx).cloned())
            .or_else(|| self.repair_holes.first().cloned());

        if let (Some(m), Some(hole)) = (m_opt, hole_opt) {
            match crate::geom::hole_solver::solve_best_hole_config(&m, &hole, &refs) {
                Ok(result) => {
                    self.repair_config = result.best_config;
                    self.repair_solve_status = Some(format!(
                        "✓ Solved: {} (bulge: {:.2}, RMS: {:.3} mm)",
                        result.best_method.display_name(),
                        result.best_config.bulge,
                        result.rms_error
                    ));
                    self.status = format!(
                        "Solver selected {} (bulge: {:.2}, dir: {}, tested {} configs).",
                        result.best_method.display_name(),
                        result.best_config.bulge,
                        result.best_config.direction_mode.display_name(),
                        result.tested_count
                    );
                    self.update_hole_preview();
                }
                Err(e) => {
                    self.status = format!("Solver error: {e}");
                    self.repair_solve_status = Some(format!("Solver error: {e}"));
                }
            }
        } else {
            self.status = "No holes detected to solve.".to_string();
            self.repair_solve_status = Some("No holes detected".to_string());
        }
    }

    pub(crate) fn set_hole_refine_to_references(&mut self, refine: bool) {
        self.repair_refine_to_references = refine;
        self.update_hole_preview();
    }

    pub(crate) fn update_hole_preview(&mut self) {
        if !self.repair_preview_active {
            self.repair_preview_patch = None;
            return;
        }
        let m_opt = self.display().cloned();
        let hole_opt = self
            .repair_selected_hole
            .and_then(|idx| self.repair_holes.get(idx).cloned());

        if let (Some(m), Some(hole)) = (m_opt, hole_opt) {
            match crate::geom::hole_fill::generate_hole_patch(&m, &hole, self.repair_config) {
                Ok(mut patch) => {
                    if self.repair_refine_to_references {
                        let refs = self.active_reference_geometries();
                        crate::geom::hole_solver::refine_patch_to_references(&mut patch, &m, &hole, &refs);
                    }
                    self.repair_preview_patch = Some(patch);
                }
                Err(e) => {
                    self.status = format!("Preview error: {e}");
                    self.repair_preview_patch = None;
                }
            }
        } else {
            self.repair_preview_patch = None;
        }
    }

    pub(crate) fn fill_selected_hole(&mut self) {
        if let Some(idx) = self.repair_selected_hole {
            if let Some(hole) = self.repair_holes.get(idx).cloned() {
                if let Some(curr) = self.current.clone() {
                    match crate::geom::hole_fill::generate_hole_patch(&curr, &hole, self.repair_config) {
                        Ok(mut patch) => {
                            if self.repair_refine_to_references {
                                let refs = self.active_reference_geometries();
                                crate::geom::hole_solver::refine_patch_to_references(&mut patch, &curr, &hole, &refs);
                            }
                            self.push_snapshot();
                            let mut next_mesh = (*curr).clone();
                            crate::geom::hole_fill::apply_patch(&mut next_mesh, &patch);
                            let method_name = self.repair_config.method.display_name();
                            let refined_suffix = if self.repair_refine_to_references {
                                " [CAD-refined]"
                            } else {
                                ""
                            };
                            self.set_mesh_modified(
                                next_mesh,
                                format!("Hole #{} filled using {}{}.", hole.id, method_name, refined_suffix),
                            );
                        }
                        Err(e) => self.status = format!("Failed to fill hole: {e}"),
                    }
                }
            }
        }
    }

    pub(crate) fn fill_all_holes(&mut self) {
        if self.repair_holes.is_empty() {
            self.status = "No holes to fill.".to_string();
            return;
        }
        if let Some(curr) = self.current.clone() {
            let refs = if self.repair_refine_to_references {
                self.active_reference_geometries()
            } else {
                Vec::new()
            };

            let mut working = (*curr).clone();
            let mut filled_count = 0;
            for hole in &self.repair_holes {
                match crate::geom::hole_fill::generate_hole_patch(&working, hole, self.repair_config) {
                    Ok(mut patch) => {
                        if self.repair_refine_to_references && !refs.is_empty() {
                            crate::geom::hole_solver::refine_patch_to_references(&mut patch, &working, hole, &refs);
                        }
                        crate::geom::hole_fill::apply_patch(&mut working, &patch);
                        filled_count += 1;
                    }
                    Err(_) => {}
                }
            }

            if filled_count > 0 {
                self.push_snapshot();
                let method_name = self.repair_config.method.display_name();
                self.set_mesh_modified(
                    working,
                    format!("Filled {}/{} holes using {}.", filled_count, self.repair_holes.len(), method_name),
                );
            } else {
                self.status = "Failed to fill holes.".to_string();
            }
        }
    }

    pub(crate) fn set_bridge_config(&mut self, config: crate::geom::bridge::BridgeConfig) {
        self.bridge_config = config;
        self.update_bridge_preview();
    }

    pub(crate) fn set_bridge_preview_active(&mut self, active: bool) {
        self.bridge_preview_active = active;
        self.update_bridge_preview();
    }

    pub(crate) fn update_bridge_preview(&mut self) {
        if !self.bridge_preview_active || self.sel_count == 0 {
            self.bridge_preview_patch = None;
            self.bridge_status = None;
            return;
        }

        if let Some(curr) = self.current.clone() {
            let topo = self.topology.clone();
            match crate::geom::bridge::detect_selection_clusters(&curr, &self.sel, topo.as_deref()) {
                Ok((cluster_a, cluster_b)) => {
                    match crate::geom::bridge::generate_bridge_patch(&curr, &cluster_a, &cluster_b, self.bridge_config) {
                        Ok(patch) => {
                            self.bridge_status = Some(format!(
                                "Bridge ready: A ({} v) ↔ B ({} v), {} segments",
                                cluster_a.boundary_chain.len(),
                                cluster_b.boundary_chain.len(),
                                self.bridge_config.segments
                            ));
                            self.bridge_preview_patch = Some(patch);
                        }
                        Err(e) => {
                            self.bridge_status = Some(format!("Bridge error: {e}"));
                            self.bridge_preview_patch = None;
                        }
                    }
                }
                Err(e) => {
                    self.bridge_status = Some(e);
                    self.bridge_preview_patch = None;
                    self.bridge_preview_active = false;
                }
            }
        }
    }

    pub(crate) fn apply_bridge(&mut self) {
        if let Some(curr) = self.current.clone() {
            let topo = self.topology.clone();
            match crate::geom::bridge::detect_selection_clusters(&curr, &self.sel, topo.as_deref()) {
                Ok((cluster_a, cluster_b)) => {
                    match crate::geom::bridge::generate_bridge_patch(&curr, &cluster_a, &cluster_b, self.bridge_config) {
                        Ok(patch) => {
                            self.push_snapshot();
                            let mut next_mesh = (*curr).clone();
                            crate::geom::bridge::apply_bridge_patch(&mut next_mesh, &patch);
                            self.bridge_preview_patch = None;
                            self.bridge_status = None;
                            self.bridge_preview_active = false;
                            let segs = self.bridge_config.segments;
                            let added_tris = patch.new_indices.len() / 3;
                            self.set_mesh_modified(
                                next_mesh,
                                format!("Bridge applied ({} segments, {} triangles added).", segs, added_tris),
                            );
                        }
                        Err(e) => self.status = format!("Bridge failed: {e}"),
                    }
                }
                Err(e) => self.status = format!("Cannot bridge: {e}"),
            }
        }
    }

    pub(crate) fn auto_repair(&mut self) {
        if let Some(curr) = self.current.clone() {
            self.push_snapshot();
            let (next_mesh, summary) = crate::geom::repair::auto_repair_mesh(&curr);
            self.set_mesh_modified(next_mesh, summary);
        }
    }

    pub(crate) fn unify_normals_action(&mut self) {
        if let Some(curr) = self.current.clone() {
            self.push_snapshot();
            let next_mesh = crate::geom::repair::unify_normals(&curr);
            self.set_mesh_modified(next_mesh, "Unified triangle normals across all shared edges.".to_string());
        }
    }

    pub(crate) fn remove_small_components_action(&mut self) {
        if let Some(curr) = self.current.clone() {
            self.push_snapshot();
            let next_mesh = crate::geom::repair::remove_small_components(&curr, false, 0.005);
            self.set_mesh_modified(next_mesh, "Removed small floating components (< 0.5% faces).".to_string());
        }
    }

    pub(crate) fn remove_degenerate_faces_action(&mut self) {
        if let Some(curr) = self.current.clone() {
            self.push_snapshot();
            let next_mesh = crate::geom::repair::remove_degenerate_faces(&curr);
            self.set_mesh_modified(next_mesh, "Removed degenerate and zero-area faces.".to_string());
        }
    }
}

pub(crate) fn rotation_between(from: Vec3, to: Vec3) -> Quat {
    let f = from.normalize_or_zero();
    let t = to.normalize_or_zero();
    if f.length_squared() < 1e-12 {
        return Quat::IDENTITY;
    }
    if f.dot(t) < -0.999_999 {
        let perp = if f.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let axis = f.cross(perp).normalize_or_zero();
        return Quat::from_axis_angle(axis, std::f32::consts::PI);
    }
    Quat::from_rotation_arc(f, t)
}

pub(crate) fn axis_name(a: Vec3) -> &'static str {
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
        push_line(&mut l, p + u * f + v * (-half), p + u * f + v * half, c);
        push_line(&mut l, p + u * (-half) + v * f, p + u * half + v * f, c);
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
            let can_undo = !self.undo.is_empty();
            let can_redo = !self.redo.is_empty();
            let undo_label = if can_undo {
                format!("⮌ Undo ({})", self.undo.len())
            } else {
                "⮌ Undo".to_string()
            };
            let redo_label = if can_redo {
                format!("⮎ Redo ({})", self.redo.len())
            } else {
                "⮎ Redo".to_string()
            };
            if ui
                .add_enabled(can_undo, egui::Button::new(undo_label))
                .on_hover_text("Undo last action (Ctrl+Z)")
                .clicked()
            {
                self.undo();
            }
            if ui
                .add_enabled(can_redo, egui::Button::new(redo_label))
                .on_hover_text("Redo last undone action (Ctrl+Y or Ctrl+Shift+Z)")
                .clicked()
            {
                self.redo();
            }
            ui.separator();
            let export = |ui: &mut egui::Ui, app: &mut App, ext: &str| {
                if ui.button(format!("Export {ext}")).clicked() {
                    if app.display().is_some() {
                        if let Some(p) = rfd::FileDialog::new()
                            .add_filter(ext, &[ext])
                            .set_file_name(
                                app.file_path
                                    .as_ref()
                                    .and_then(|p| p.file_stem())
                                    .and_then(|s| s.to_str())
                                    .map(|s| format!("{s}.{ext}"))
                                    .unwrap_or(format!("mesh.{ext}")),
                            )
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
            ui.checkbox(&mut self.show_object_browser, "Objects");
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

    pub(crate) fn align_sym_full(&mut self, axis: Vec3, q: Quat) {
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
                Mode::Orbit => "LMB drag = select · Shift+LMB = erase · Double-click = select group · Alt+Wheel = brush size · Ctrl+Wheel = expand/shrink · RMB drag = orbit · MMB drag = pan · Wheel = zoom".to_string(),
                Mode::SymPickLine => {
                    if self.sym_pick.len() >= 2 {
                        "Symmetry line ready · Esc = reset".to_string()
                    } else {
                        format!(
                            "Symmetry line: click point {} of 2 on the mesh · Esc = cancel",
                            self.sym_pick.len() + 1
                        )
                    }
                }
            };
            ui.label(hint);
        });
    }

    fn viewport(&mut self, ui: &mut egui::Ui) {
        let rect = ui.available_rect_before_wrap();
        let (alloc, response) = ui.allocate_exact_size(rect.size(), egui::Sense::click_and_drag());
        let rect = alloc;
        let ppp = ui.ctx().pixels_per_point();
        let vp_w = (rect.width() * ppp).round().max(1.0) as u32;
        let vp_h = (rect.height() * ppp).round().max(1.0) as u32;
        self.camera.aspect = rect.width() / rect.height().max(1.0);

        let ctrl_z = ui
            .ctx()
            .input(|i| i.modifiers.ctrl && !i.modifiers.shift && i.key_pressed(egui::Key::Z));
        if ctrl_z {
            self.undo();
        }
        let ctrl_redo = ui.ctx().input(|i| {
            (i.modifiers.ctrl && i.key_pressed(egui::Key::Y))
                || (i.modifiers.ctrl && i.modifiers.shift && i.key_pressed(egui::Key::Z))
        });
        if ctrl_redo {
            self.redo();
        }
        let esc = ui.ctx().input(|i| i.key_pressed(egui::Key::Escape));
        if esc {
            self.mode = Mode::Orbit;
            self.sym_pick.clear();
        }
        let key_h = ui.ctx().input(|i| !i.modifiers.ctrl && !i.modifiers.alt && i.key_pressed(egui::Key::H));
        if key_h && self.sel_count > 0 {
            self.hide_selection();
        }

        // Navigation: MMB drag = Pan
        // Navigation flags
        let rmb_down = ui.input(|i| i.pointer.button_down(egui::PointerButton::Secondary));
        let mmb_down = ui.input(|i| i.pointer.button_down(egui::PointerButton::Middle));
        let is_navigating = rmb_down
            || mmb_down
            || response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle);

        // Navigation: MMB drag = Pan
        if response.dragged_by(egui::PointerButton::Middle) {
            let d = response.drag_delta();
            self.camera.pan_drag(d.x, d.y, rect.height());
        }

        // Navigation: RMB drag = Rotate / Orbit (Meshmixer style)
        if response.dragged_by(egui::PointerButton::Secondary) {
            let d = response.drag_delta();
            self.camera.rotate(d.x, d.y);
        }

        // Mouse Wheel: Alt + Wheel = Change Brush Size, Ctrl + Wheel = Grow / Shrink selection (Meshmixer style), Wheel = Zoom
        if response.hovered() && !is_navigating {
            let ctrl = ui.ctx().input(|i| i.modifiers.ctrl);
            let alt = ui.ctx().input(|i| i.modifiers.alt);
            if alt {
                let mut wheel_events: Vec<(egui::MouseWheelUnit, egui::Vec2)> = Vec::new();
                let mut zoom_delta = 1.0f32;
                ui.ctx().input(|i| {
                    zoom_delta = i.zoom_delta();
                    for event in &i.events {
                        if let egui::Event::MouseWheel { unit, delta, .. } = event {
                            wheel_events.push((*unit, *delta));
                        }
                    }
                });

                let mut triggered = false;
                for (unit, delta) in wheel_events {
                    match unit {
                        egui::MouseWheelUnit::Line => {
                            let steps = delta.y;
                            if steps.abs() > 0.0 {
                                self.brush_radius =
                                    (self.brush_radius + steps * 3.0).clamp(2.0, 150.0);
                                triggered = true;
                            }
                        }
                        egui::MouseWheelUnit::Point | egui::MouseWheelUnit::Page => {
                            self.brush_radius =
                                (self.brush_radius + delta.y * 0.25).clamp(2.0, 150.0);
                            triggered = true;
                        }
                    }
                }
                if !triggered {
                    if zoom_delta > 1.01 {
                        self.brush_radius = (self.brush_radius + 3.0).clamp(2.0, 150.0);
                    } else if zoom_delta < 0.99 {
                        self.brush_radius = (self.brush_radius - 3.0).clamp(2.0, 150.0);
                    }
                }
            } else if ctrl {
                let mut wheel_events: Vec<(egui::MouseWheelUnit, egui::Vec2)> = Vec::new();
                let mut zoom_delta = 1.0f32;
                ui.ctx().input(|i| {
                    zoom_delta = i.zoom_delta();
                    for event in &i.events {
                        if let egui::Event::MouseWheel { unit, delta, .. } = event {
                            wheel_events.push((*unit, *delta));
                        }
                    }
                });

                let mut triggered = false;
                for (unit, delta) in wheel_events {
                    match unit {
                        egui::MouseWheelUnit::Line => {
                            let steps = delta.y.round() as i32;
                            if steps > 0 {
                                for _ in 0..steps {
                                    self.grow_selection();
                                }
                                triggered = true;
                            } else if steps < 0 {
                                for _ in 0..(-steps) {
                                    self.shrink_selection();
                                }
                                triggered = true;
                            }
                        }
                        egui::MouseWheelUnit::Point | egui::MouseWheelUnit::Page => {
                            self.wheel_accum += delta.y;
                            while self.wheel_accum >= 10.0 {
                                self.grow_selection();
                                self.wheel_accum -= 10.0;
                                triggered = true;
                            }
                            while self.wheel_accum <= -10.0 {
                                self.shrink_selection();
                                self.wheel_accum += 10.0;
                                triggered = true;
                            }
                        }
                    }
                }
                if !triggered {
                    if zoom_delta > 1.01 {
                        self.grow_selection();
                    } else if zoom_delta < 0.99 {
                        self.shrink_selection();
                    }
                }
            } else {
                self.wheel_accum = 0.0;
                let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.camera.zoom(0.95f32.powf(scroll / 60.0));
                }
            }
        }

        // Keyboard shortcuts for grow/shrink as in Meshmixer (Period/Plus to grow, Comma/Minus to shrink)
        if ui
            .ctx()
            .input(|i| i.key_pressed(egui::Key::Period) || i.key_pressed(egui::Key::Plus))
        {
            self.grow_selection();
        }
        if ui
            .ctx()
            .input(|i| i.key_pressed(egui::Key::Comma) || i.key_pressed(egui::Key::Minus))
        {
            self.shrink_selection();
        }

        // Selection Tool: LMB drag = Select, Shift + LMB = Erase
        let shift = ui.ctx().input(|i| i.modifiers.shift);
        let is_add = !shift;

        let lmb_down = ui.input(|i| i.pointer.button_down(egui::PointerButton::Primary));
        let lmb_drag = response.dragged_by(egui::PointerButton::Primary);

        if !lmb_down {
            self.suppress_sel_drag = false;
        }

        if self.mode != Mode::SymPickLine && !is_navigating && !self.suppress_sel_drag {
            if response.drag_started_by(egui::PointerButton::Primary)
                || (response.hovered()
                    && ui.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary)))
            {
                self.push_snapshot();
            }

            if lmb_drag || (response.hovered() && lmb_down) {
                if let Some(mesh) = self.display().cloned() {
                    if let Some(bvh) = self.ensure_bvh() {
                        if let Some(pos) = response.interact_pointer_pos() {
                            let sx = pos.x - rect.min.x;
                            let sy = pos.y - rect.min.y;
                            let is_hidden = |t: u32| {
                                let tu = t as usize;
                                tu < self.hidden_mask.len() && self.hidden_mask[tu]
                            };
                            if let Some(hit) = pick::ray_pick_filtered(
                                &bvh,
                                &self.camera,
                                sx,
                                sy,
                                rect.width(),
                                rect.height(),
                                &is_hidden,
                            ) {
                                let mut sel = self.sel.clone();
                                let sel_slice: &mut Vec<u8> = Arc::make_mut(&mut sel);
                                pick::brush_filtered(
                                    &mesh,
                                    &bvh,
                                    &self.camera,
                                    &hit,
                                    self.brush_radius,
                                    rect.height(),
                                    is_add,
                                    sel_slice,
                                    &is_hidden,
                                );
                                self.sel = sel;
                                self.recount_sel();
                                self.aux_dirty = true;
                                if is_add && self.sel_count > 0 {
                                    self.active_section = Some(crate::ui::ToolSection::Selection);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Double-click: select the complete face group under the cursor
        // (Meshmixer style). Falls back to a crease-angle grown region when
        // no face group exists there.
        if self.mode != Mode::SymPickLine
            && !is_navigating
            && !self.suppress_sel_drag
            && response.double_clicked()
        {
            if let Some(pos) = response.hover_pos() {
                let sx = pos.x - rect.min.x;
                let sy = pos.y - rect.min.y;
                if let Some(bvh) = self.ensure_bvh() {
                    let is_hidden = |t: u32| {
                        let tu = t as usize;
                        tu < self.hidden_mask.len() && self.hidden_mask[tu]
                    };
                    if let Some(hit) = pick::ray_pick_filtered(
                        &bvh,
                        &self.camera,
                        sx,
                        sy,
                        rect.width(),
                        rect.height(),
                        &is_hidden,
                    ) {
                        self.select_region_under(hit.tri);
                    }
                }
            }
        }

        // Real-time hover preview computation (only when not navigating and not painting)
        let mut new_hover_hit = None;
        let mut new_hover_tris = Vec::new();
        let mut new_hover_r = 0.0f32;

        if self.mode != Mode::SymPickLine && response.hovered() && !is_navigating && !lmb_down {
            if let Some(pos) = response.hover_pos() {
                let sx = pos.x - rect.min.x;
                let sy = pos.y - rect.min.y;
                if let (Some(mesh), Some(bvh)) = (self.display().cloned(), self.ensure_bvh()) {
                    let is_hidden = |t: u32| {
                        let tu = t as usize;
                        tu < self.hidden_mask.len() && self.hidden_mask[tu]
                    };
                    if let Some(hit) = pick::ray_pick_filtered(
                        &bvh,
                        &self.camera,
                        sx,
                        sy,
                        rect.width(),
                        rect.height(),
                        &is_hidden,
                    ) {
                        let (r, tris) = pick::query_brush_triangles_filtered(
                            &mesh,
                            &bvh,
                            &self.camera,
                            &hit,
                            self.brush_radius,
                            rect.height(),
                            &is_hidden,
                        );
                        new_hover_hit = Some(hit);
                        new_hover_tris = tris;
                        new_hover_r = r;
                    }
                }
            }
        }

        let hover_changed = self.hover_tris != new_hover_tris
            || self.hover_is_erase != shift
            || (self.hover_radius_world - new_hover_r).abs() > 1e-4;
        self.hover_hit = new_hover_hit;
        self.hover_tris = new_hover_tris;
        self.hover_radius_world = new_hover_r;
        self.hover_is_erase = shift;
        if hover_changed {
            self.aux_dirty = true;
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
                    let is_hidden = |t: u32| {
                        let tu = t as usize;
                        tu < self.hidden_mask.len() && self.hidden_mask[tu]
                    };
                    hover_pos_3d = pick::ray_pick_filtered(
                        &bvh,
                        &self.camera,
                        sx,
                        sy,
                        rect.width(),
                        rect.height(),
                        &is_hidden,
                    )
                    .map(|h| h.pos);
                }
            }
        }

        if self.mode == Mode::SymPickLine
            && response.hovered()
            && !is_navigating
            && ui.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
        {
            self.suppress_sel_drag = true;
            if let Some(pos) = hover_pos_3d {
                if self.sym_pick.len() >= 2 {
                    self.sym_pick.clear();
                }
                self.sym_pick.push(pos);
                if self.sym_pick.len() == 1 {
                    self.status = "Point 1 placed. Click point 2 on the mesh.".to_string();
                } else if self.sym_pick.len() == 2 {
                    let a = self.sym_pick[0];
                    let b = self.sym_pick[1];
                    self.schedule_sym_from_line(a, b);
                    self.mode = Mode::Orbit;
                }
            }
        }

        if self.mode != Mode::SymPickLine && response.hovered() && !is_navigating {
            if let Some(pos) = response.hover_pos() {
                let (stroke_col, fill_col) = if self.hover_is_erase {
                    (
                        egui::Color32::from_rgba_unmultiplied(255, 70, 70, 230),
                        egui::Color32::from_rgba_unmultiplied(255, 70, 70, 20),
                    )
                } else {
                    (
                        egui::Color32::from_rgba_unmultiplied(255, 150, 40, 230),
                        egui::Color32::from_rgba_unmultiplied(255, 150, 40, 20),
                    )
                };
                ui.painter().circle_filled(pos, self.brush_radius, fill_col);
                ui.painter().circle_stroke(
                    pos,
                    self.brush_radius,
                    egui::Stroke::new(1.5, stroke_col),
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
            for p in &self.planes {
                if p.visible {
                    let half = diag * 0.35;
                    let is_sel = self.selected_plane_id == Some(p.id);
                    let mut col = p.color;
                    let mut fill_col = [col[0], col[1], col[2], 0.12];
                    if is_sel {
                        col = [col[0], col[1], col[2], 1.0];
                        fill_col = [col[0], col[1], col[2], 0.22];
                        overlay_lines.extend(plane_grid_lines(
                            p.fit.point,
                            p.fit.normal,
                            half,
                            2,
                            [1.0, 1.0, 1.0, 0.7],
                        ));
                    }
                    depth_lines.extend(plane_grid_lines(p.fit.point, p.fit.normal, half, 6, col));
                    fills.extend(plane_fill(p.fit.point, p.fit.normal, half, fill_col));
                }
            }
            if self.planes.is_empty() {
                if let Some(p) = &self.plane {
                    if self.show_plane {
                        let half = diag * 0.35;
                        depth_lines.extend(plane_grid_lines(
                            p.point,
                            p.normal,
                            half,
                            6,
                            [1.0, 0.6, 0.1, 0.9],
                        ));
                        fills.extend(plane_fill(p.point, p.normal, half, [1.0, 0.6, 0.1, 0.12]));
                    }
                }
            }
            for c in &self.circles {
                if c.visible {
                    let is_sel = self.selected_circle_id == Some(c.id);
                    let mut col = c.color;
                    if is_sel {
                        col = [1.0, 1.0, 0.3, 1.0];
                    }
                    depth_lines.extend(circle_lines(c.fit.center, c.fit.normal, c.fit.radius, col));
                }
            }
            if self.circles.is_empty() {
                if let Some(c) = &self.circle {
                    if self.show_circle {
                        depth_lines.extend(circle_lines(
                            c.center,
                            c.normal,
                            c.radius,
                            [1.0, 0.2, 0.8, 1.0],
                        ));
                    }
                }
            }
            for f in &self.freeforms {
                if !f.visible {
                    continue;
                }
                let Some(surf) = &f.surface else {
                    continue;
                };
                let is_sel = self.selected_freeform_id == Some(f.id);
                let col = f.color;
                let fill_col = if is_sel {
                    [col[0], col[1], col[2], 0.30]
                } else {
                    [col[0], col[1], col[2], 0.16]
                };
                let edge_col = if is_sel {
                    [1.0, 1.0, 0.3, 1.0]
                } else {
                    [col[0], col[1], col[2], 0.9]
                };
                let heatmap_active = f.heat_on
                    && f.heat
                        .as_ref()
                        .map(|h| h.len() == surf.vertex_count())
                        .unwrap_or(false);
                let heat_alpha = if is_sel { 0.85 } else { 0.65 };
                for chunk in surf.indices.chunks_exact(3) {
                    for k in 0..3 {
                        let idx = chunk[k] as usize;
                        let p = surf.positions[idx];
                        let c = if heatmap_active {
                            let dist = f.heat.as_ref().unwrap()[idx];
                            crate::geom::freeform::freeform_vertex_color(
                                dist,
                                f.heat_max,
                                f.heat_gradient,
                                heat_alpha,
                            )
                        } else {
                            fill_col
                        };
                        fills.push([
                            p[0], p[1], p[2], c[0], c[1], c[2], c[3],
                        ]);
                    }
                }
                for (a, b) in &f.boundary {
                    push_line(
                        &mut depth_lines,
                        Vec3::from(*a),
                        Vec3::from(*b),
                        edge_col,
                    );
                }
            }
            if self.mode == Mode::SymPickLine {
                if let Some(h) = hover_pos_3d {
                    overlay_lines.extend(marker_lines(h, diag * 0.02, [0.2, 0.9, 1.0, 1.0]));
                    overlay_lines.extend(circle_lines(
                        h,
                        self.camera.back(),
                        diag * 0.008,
                        [0.2, 0.9, 1.0, 1.0],
                    ));
                    if let Some(&a) = self.sym_pick.first() {
                        push_line(&mut overlay_lines, a, h, [1.0, 1.0, 0.3, 1.0]);
                    }
                }
            }
            for &p in &self.sym_pick {
                overlay_lines.extend(marker_lines(p, diag * 0.02, [1.0, 1.0, 1.0, 1.0]));
                overlay_lines.extend(circle_lines(
                    p,
                    self.camera.back(),
                    diag * 0.008,
                    [1.0, 1.0, 1.0, 1.0],
                ));
            }
            if self.sym_pick.len() == 2 {
                push_line(
                    &mut overlay_lines,
                    self.sym_pick[0],
                    self.sym_pick[1],
                    [1.0, 1.0, 0.3, 1.0],
                );
            }
            if self.active_section == Some(crate::ui::ToolSection::Repair) || !self.repair_holes.is_empty() {
                if let Some(m) = self.display() {
                    let pos = &m.positions;
                    for (h_idx, hole) in self.repair_holes.iter().enumerate() {
                        let is_sel = self.repair_selected_hole == Some(h_idx);
                        let edge_col = if is_sel {
                            [0.1, 0.95, 1.0, 1.0]
                        } else {
                            [1.0, 0.65, 0.2, 0.75]
                        };
                        let n_verts = hole.vertices.len();
                        for i in 0..n_verts {
                            let p0 = Vec3::from(pos[hole.vertices[i] as usize]);
                            let p1 = Vec3::from(pos[hole.vertices[(i + 1) % n_verts] as usize]);
                            if is_sel {
                                push_line(&mut overlay_lines, p0, p1, edge_col);
                            } else {
                                push_line(&mut depth_lines, p0, p1, edge_col);
                            }
                        }
                        if is_sel {
                            overlay_lines.extend(marker_lines(hole.centroid, diag * 0.015, [0.1, 0.95, 1.0, 1.0]));
                        }
                    }
                }
            }
            if self.repair_preview_active {
                if let Some(patch) = &self.repair_preview_patch {
                    let fill_col = [0.15, 0.85, 0.95, 0.35];
                    let wire_col = [0.2, 0.95, 1.0, 0.9];
                    for chunk in patch.preview_indices.chunks_exact(3) {
                        let p0 = Vec3::from(patch.preview_positions[chunk[0] as usize]);
                        let p1 = Vec3::from(patch.preview_positions[chunk[1] as usize]);
                        let p2 = Vec3::from(patch.preview_positions[chunk[2] as usize]);

                        fills.push([p0.x, p0.y, p0.z, fill_col[0], fill_col[1], fill_col[2], fill_col[3]]);
                        fills.push([p1.x, p1.y, p1.z, fill_col[0], fill_col[1], fill_col[2], fill_col[3]]);
                        fills.push([p2.x, p2.y, p2.z, fill_col[0], fill_col[1], fill_col[2], fill_col[3]]);

                        push_line(&mut overlay_lines, p0, p1, wire_col);
                        push_line(&mut overlay_lines, p1, p2, wire_col);
                        push_line(&mut overlay_lines, p2, p0, wire_col);
                    }
                }
            }
            if self.bridge_preview_active {
                if let Some(patch) = &self.bridge_preview_patch {
                    let fill_col = [0.15, 0.90, 0.65, 0.40];
                    let wire_col = [0.20, 1.0, 0.75, 0.95];
                    for chunk in patch.preview_indices.chunks_exact(3) {
                        let p0 = Vec3::from(patch.preview_positions[chunk[0] as usize]);
                        let p1 = Vec3::from(patch.preview_positions[chunk[1] as usize]);
                        let p2 = Vec3::from(patch.preview_positions[chunk[2] as usize]);

                        fills.push([p0.x, p0.y, p0.z, fill_col[0], fill_col[1], fill_col[2], fill_col[3]]);
                        fills.push([p1.x, p1.y, p1.z, fill_col[0], fill_col[1], fill_col[2], fill_col[3]]);
                        fills.push([p2.x, p2.y, p2.z, fill_col[0], fill_col[1], fill_col[2], fill_col[3]]);

                        push_line(&mut overlay_lines, p0, p1, wire_col);
                        push_line(&mut overlay_lines, p1, p2, wire_col);
                        push_line(&mut overlay_lines, p2, p0, wire_col);
                    }
                }
            }
            if let Some(hit) = &self.hover_hit {
                if !is_navigating {
                    if let Some(m) = self.display() {
                        let r = self.hover_radius_world;
                        if r > 0.0 && (hit.tri as usize) < m.triangle_count() {
                            let i0 = m.indices[3 * hit.tri as usize] as usize;
                            let i1 = m.indices[3 * hit.tri as usize + 1] as usize;
                            let i2 = m.indices[3 * hit.tri as usize + 2] as usize;
                            let a = Vec3::from(m.positions[i0]);
                            let b = Vec3::from(m.positions[i1]);
                            let c = Vec3::from(m.positions[i2]);
                            let mut n = (b - a).cross(c - a);
                            if n.length_squared() > 1e-12 {
                                n = n.normalize();
                            } else {
                                n = self.camera.back();
                            }
                            if n.dot(self.camera.eye() - hit.pos) < 0.0 {
                                n = -n;
                            }
                            let col = if self.hover_is_erase {
                                [1.0, 0.25, 0.25, 0.95]
                            } else {
                                [1.0, 0.65, 0.15, 0.95]
                            };
                            let center = hit.pos + n * (r * 0.005);
                            overlay_lines.extend(circle_lines(center, n, r, col));
                        }
                    }
                }
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
                        gpu.upload_mesh(m, Some(&self.hidden_mask));
                        self.mesh_dirty = false;
                        self.wire_dirty = true;
                    }
                }
                if self.wire_dirty && self.show_wireframe {
                    if let Some(m) = self.display() {
                        if !gpu.upload_wireframe(m, 800_000, Some(&self.hidden_mask)) {
                            self.status = "Wireframe disabled: mesh too dense (> 800k triangles)."
                                .to_string();
                        }
                        self.wire_dirty = false;
                    }
                }
                if self.aux_dirty {
                    if let Some(m) = self.display() {
                        let nv = m.vertex_count();
                        // aux per vertex: x = selection/hover, y = deviation heat,
                        // z = face group color code (id, type code or -1/-2), w = group id.
                        let mut aux = vec![[0.0f32, 0.0, -1.0, -1.0]; nv];
                        if let Some(heat) = &self.heat {
                            if heat.len() == nv && self.heat_on {
                                for (a, h) in aux.iter_mut().zip(heat.iter()) {
                                    a[1] = *h;
                                }
                            }
                        }
                        if self.groups_show
                            && !self.face_groups.is_empty()
                            && self.group_ids.len() == m.triangle_count()
                        {
                            let by_type = self.groups_by_type;
                            let filter = self.groups_filter;
                            // -1 = no group yet, -2 = boundary between two groups.
                            let mut vgid = vec![-1i32; nv];
                            for t in 0..m.triangle_count() {
                                let g = self.group_ids[t];
                                if g < 0 {
                                    continue;
                                }
                                for k in 0..3 {
                                    let v = m.indices[3 * t + k] as usize;
                                    match vgid[v] {
                                        -1 => vgid[v] = g,
                                        cur if cur == g => {}
                                        _ => vgid[v] = -2,
                                    }
                                }
                            }
                            for (v, a) in aux.iter_mut().enumerate() {
                                let vg = vgid[v];
                                if vg >= 0 {
                                    let kind = self.face_groups[vg as usize].kind;
                                    if group_matches_filter(kind, filter) {
                                        a[2] = if by_type {
                                            -(3.0 + kind as i32 as f32)
                                        } else {
                                            vg as f32
                                        };
                                        a[3] = vg as f32;
                                    }
                                } else if vg == -2 {
                                    a[2] = -2.0;
                                    a[3] = -1.0;
                                }
                            }
                        }
                        if self.sel.len() == m.triangle_count() {
                            for t in 0..m.triangle_count() {
                                if self.is_face_hidden(t as u32) {
                                    continue;
                                }
                                if self.sel[t] > 0 {
                                    for k in 0..3 {
                                        let v = m.indices[3 * t + k] as usize;
                                        aux[v][0] = 1.0;
                                    }
                                }
                            }
                            if !self.hover_tris.is_empty() {
                                let is_erase = self.hover_is_erase;
                                for &t in &self.hover_tris {
                                    let t_usize = t as usize;
                                    if t_usize < m.triangle_count() {
                                        let is_sel = self.sel[t_usize] > 0;
                                        for k in 0..3 {
                                            let v = m.indices[3 * t_usize + k] as usize;
                                            if is_erase {
                                                if is_sel {
                                                    aux[v][0] = -0.5;
                                                }
                                            } else {
                                                if !is_sel {
                                                    aux[v][0] = 0.5;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        gpu.upload_aux(&aux);
                        self.aux_dirty = false;
                    }
                }
                let (light1, light2) = self.camera.light_directions();
                let groups_on = self.groups_show
                    && !self.face_groups.is_empty()
                    && self
                        .display()
                        .map(|m| self.group_ids.len() == m.triangle_count())
                        .unwrap_or(false);
                let hover_gid = self.hover_group.map(|g| g as f32).unwrap_or(-1.0);
                gpu.set_frame(
                    self.camera.view_proj(),
                    self.camera.eye(),
                    light1,
                    light2,
                    self.heat_on && self.heat.is_some(),
                    if self.heat_on && self.heat_max > 0.0 {
                        1.0 / self.heat_max
                    } else {
                        0.0
                    },
                    groups_on,
                    hover_gid,
                    self.show_mesh,
                    self.show_wireframe,
                );
                gpu.write_lines_depth(&depth_lines);
                gpu.write_lines_overlay(&overlay_lines);
                gpu.write_fills(&fills);
            }
            ui.painter()
                .add(crate::render::make_callback(gpu_arc, rect));
        }

        // Viewport Overlays: Selection HUD and Object Browser
        crate::ui::selection_hud::render_selection_hud(self, ui, rect);
        crate::ui::object_browser::render_object_browser(self, ui, rect);
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
                crate::ui::render_left_panel(self, ui);
            });
        egui::CentralPanel::default().show(ui, |ui| {
            self.viewport(ui);
        });
    }
}

impl crate::geom::alignment::AlignmentGeometrySource for App {
    fn get_feature_direction_and_point(
        &self,
        feat: crate::geom::alignment::FeatureRef,
    ) -> Option<(Vec3, Vec3)> {
        match feat {
            crate::geom::alignment::FeatureRef::SymmetryPlane => {
                self.sym.map(|s| (s.plane.normal, s.plane.point))
            }
            crate::geom::alignment::FeatureRef::Plane(id) => self
                .planes
                .iter()
                .find(|p| p.id == id)
                .map(|p| (p.fit.normal, p.fit.point)),
            crate::geom::alignment::FeatureRef::Circle(id) => self
                .circles
                .iter()
                .find(|c| c.id == id)
                .map(|c| (c.fit.normal, c.fit.center)),
        }
    }

    fn get_feature_name(&self, feat: crate::geom::alignment::FeatureRef) -> String {
        match feat {
            crate::geom::alignment::FeatureRef::SymmetryPlane => "Symmetry plane".to_string(),
            crate::geom::alignment::FeatureRef::Plane(id) => self
                .planes
                .iter()
                .find(|p| p.id == id)
                .map(|p| p.name.clone())
                .unwrap_or_else(|| format!("Plane {id}")),
            crate::geom::alignment::FeatureRef::Circle(id) => self
                .circles
                .iter()
                .find(|c| c.id == id)
                .map(|c| c.name.clone())
                .unwrap_or_else(|| format!("Circle {id}")),
        }
    }

    fn get_bbox_center(&self) -> Vec3 {
        self.bbox.center()
    }
}
