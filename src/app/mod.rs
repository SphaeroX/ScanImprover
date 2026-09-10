//! Application state and top-level frame loop.
//!
//! The `App` struct owns every piece of document state (meshes, selection,
//! fitted features, undo history) plus the view / tool settings. Its
//! behaviour is split over sub-modules by concern:
//!
//! * [`history`]   – undo / redo snapshots
//! * [`session`]   – loading, transforms, background jobs, decimation preview
//! * [`features`]  – symmetry plane, fitted planes / circles / freeforms, alignment
//! * [`selection`] – face selection, hidden regions, face groups
//! * [`repair`]    – hole detection / filling, contour bridges, repair tools
//! * [`viewport`]  – 3D viewport input handling and overlay generation
//! * [`overlay`]   – line / fill primitive builders for the viewport

mod features;
mod history;
pub(crate) mod overlay;
mod repair;
mod selection;
mod session;
mod tasks;
#[cfg(test)]
mod tests;
mod viewport;

pub(crate) use history::Snapshot;
pub(crate) use tasks::MeshEditKind;

use crate::camera::Camera;
use crate::geom::alignment::{AlignmentSlots, AxisChoice, FeatureRef, OriginRef};
use crate::geom::bridge::{BridgeConfig, SelectionCluster};
use crate::geom::bvh::Bvh;
use crate::geom::distance::Deviation;
use crate::geom::fitting::{CircleFit, FittedCircle, FittedPlane, PlaneFit};
use crate::geom::freeform::FittedFreeform;
use crate::geom::hole_detect::HoleLoop;
use crate::geom::hole_fill::{HoleFillConfig, MeshPatch};
use crate::geom::hole_solver::CircleGuideMode;
use crate::geom::repair::MeshHealthReport;
use crate::geom::segment::{FaceGroup, GroupKind};
use crate::geom::symmetry::SymPlane;
use crate::geom::topology::MeshTopology;
use crate::mesh::{Aabb, Mesh};
use crate::pick;
use crate::ui::ToolSection;
use crate::worker::Worker;
use glam::{Quat, Vec3};
use std::path::PathBuf;
use std::sync::Arc;

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
    [0.55, 0.45, 1.0, 0.9],  // Violet
    [0.35, 1.0, 0.55, 0.9],  // Spring Green
    [1.0, 0.5, 0.75, 0.9],   // Pink
    [0.95, 0.75, 0.25, 0.9], // Gold
];

/// Maximum number of source points stored per freeform (fit input is
/// sub-sampled to this so parameter changes can re-fit quickly).
pub(crate) const FREEFORM_MAX_POINTS: usize = 96_000;

/// Meshes with at least this many vertices build their spatial index and
/// topology on the worker thread instead of blocking the UI.
pub(crate) const ASYNC_INDEX_MIN_VERTS: usize = 150_000;

/// Meshes with at least this many triangles run analysis, segmentation and
/// repair operations on the worker thread instead of blocking the UI.
pub(crate) const ASYNC_MIN_TRIS: usize = 200_000;

/// Maximum triangle count for which the wireframe overlay is generated.
pub(crate) const WIREFRAME_MAX_TRIS: usize = 800_000;

/// Number of undo steps kept.
pub(crate) const HISTORY_LIMIT: usize = 25;

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

/// A set of faces temporarily hidden from the viewport. Hiding is
/// non-destructive: the faces stay in the mesh and are only skipped when
/// rendering and picking.
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

#[derive(Clone, Copy)]
pub(crate) struct SymState {
    pub(crate) plane: SymPlane,
    pub(crate) rms: f64,
    pub(crate) show: bool,
}

/// Per-frame viewport description handed to the Bevy scene.
pub(crate) struct FrameOutput {
    /// Central panel rectangle in logical points.
    pub(crate) viewport_rect: egui::Rect,
    pub(crate) depth_lines: Vec<overlay::Line>,
    pub(crate) overlay_lines: Vec<overlay::Line>,
    pub(crate) fills: Vec<overlay::Line>,
    pub(crate) show_mesh: bool,
    pub(crate) show_wireframe: bool,
}

impl Default for FrameOutput {
    fn default() -> Self {
        FrameOutput {
            viewport_rect: egui::Rect::ZERO,
            depth_lines: Vec::new(),
            overlay_lines: Vec::new(),
            fills: Vec::new(),
            show_mesh: true,
            show_wireframe: false,
        }
    }
}

/// Cached result of the bridge cluster detection for the current selection.
pub(crate) struct BridgeClusterCache {
    sel_generation: u64,
    mesh_generation: u64,
    result: Result<(SelectionCluster, SelectionCluster), String>,
}

#[derive(bevy::prelude::Resource)]
pub struct App {
    pub(crate) camera: Camera,
    /// What the viewport should show this frame (consumed by the Bevy scene).
    pub(crate) frame: FrameOutput,

    // --- Document -----------------------------------------------------------
    pub(crate) current: Option<Arc<Mesh>>,
    pub(crate) original: Option<Arc<Mesh>>,
    pub(crate) preview: Option<Arc<Mesh>>,
    pub(crate) preview_error: f32,
    pub(crate) base_mesh: Option<Arc<Mesh>>,
    pub(crate) file_path: Option<PathBuf>,
    pub(crate) bbox: Aabb,
    /// Incremented whenever the displayed mesh object changes.
    pub(crate) mesh_generation: u64,

    // --- Selection ----------------------------------------------------------
    pub(crate) sel: Arc<Vec<u8>>,
    pub(crate) sel_count: usize,
    /// Incremented whenever the selection content changes.
    pub(crate) sel_generation: u64,
    pub(crate) brush_radius: f32,
    /// Restrict the brush to faces connected to the face under the cursor.
    pub(crate) brush_connected: bool,
    pub(crate) expand_angle_deg: f32,
    pub(crate) hidden_regions: Vec<HiddenRegion>,
    pub(crate) hidden_mask: Vec<bool>,

    // --- Spatial caches and background jobs ---------------------------------
    pub(crate) bvh: Option<Arc<Bvh>>,
    pub(crate) topology: Option<Arc<MeshTopology>>,
    pub(crate) worker: Worker,
    pub(crate) dec_job: Option<u64>,
    pub(crate) dev_job: Option<u64>,
    pub(crate) sym_job: Option<u64>,
    pub(crate) bvh_job: Option<u64>,
    pub(crate) bvh_job_mesh: Option<Arc<Mesh>>,
    pub(crate) topo_job: Option<u64>,
    pub(crate) topo_job_mesh: Option<Arc<Mesh>>,
    /// (job id, freeform id) of the freeform fit currently running.
    pub(crate) freeform_job: Option<(u64, u64)>,
    pub(crate) load_job: Option<u64>,
    pub(crate) groups_job: Option<u64>,
    pub(crate) groups_rerun_pending: bool,
    /// (job id, mesh generation the analysis belongs to)
    pub(crate) analysis_job: Option<(u64, u64)>,
    /// (job id, mesh generation the edit was started from)
    pub(crate) edit_job: Option<(u64, u64)>,
    pub(crate) solve_job: Option<u64>,
    pub(crate) export_job: Option<u64>,
    /// Triangle count from which analysis / repair operations run on the
    /// worker thread (see [`ASYNC_MIN_TRIS`]; tests lower it).
    pub(crate) async_min_tris: usize,

    // --- Decimation ---------------------------------------------------------
    pub(crate) dec_mode: DecMode,
    pub(crate) dec_ratio: f32,
    pub(crate) dec_error_mm: f32,
    pub(crate) dec_target_acc: f32,
    pub(crate) dec_target_mm: f32,
    pub(crate) dec_lock_border: bool,
    pub(crate) dec_auto_preview: bool,
    pub(crate) deviation: Option<Deviation>,
    pub(crate) heat: Option<Arc<Vec<f32>>>,
    pub(crate) heat_on: bool,
    pub(crate) heat_max: f32,

    // --- Symmetry -----------------------------------------------------------
    pub(crate) mode: Mode,
    pub(crate) sym: Option<SymState>,
    pub(crate) sym_pick: Vec<Vec3>,
    pub(crate) sym_exclude_selection: bool,
    pub(crate) sym_exclude_holes: bool,

    // --- Fitted features ----------------------------------------------------
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
    pub(crate) next_obj_id: u64,
    pub(crate) align_slots: AlignmentSlots,

    // --- Face groups --------------------------------------------------------
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

    // --- Repair -------------------------------------------------------------
    pub(crate) repair_holes: Vec<HoleLoop>,
    pub(crate) repair_selected_hole: Option<usize>,
    pub(crate) repair_config: HoleFillConfig,
    pub(crate) repair_preview_active: bool,
    pub(crate) repair_preview_patch: Option<MeshPatch>,
    pub(crate) repair_health: Option<MeshHealthReport>,
    pub(crate) repair_refine_to_references: bool,
    pub(crate) repair_solve_status: Option<String>,
    pub(crate) repair_circle_mode: CircleGuideMode,
    pub(crate) bridge_config: BridgeConfig,
    pub(crate) bridge_preview_active: bool,
    pub(crate) bridge_preview_patch: Option<MeshPatch>,
    pub(crate) bridge_status: Option<String>,
    pub(crate) bridge_cluster_cache: Option<BridgeClusterCache>,

    // --- View / UI state ----------------------------------------------------
    pub(crate) show_mesh: bool,
    pub(crate) show_object_browser: bool,
    pub(crate) show_wireframe: bool,
    pub(crate) show_bbox: bool,
    pub(crate) show_triad: bool,
    pub(crate) show_grid: bool,
    pub(crate) theme_mode: crate::ui::theme::ThemeMode,
    pub(crate) undo: Vec<Snapshot>,
    pub(crate) redo: Vec<Snapshot>,
    pub(crate) status: String,
    pub(crate) mesh_dirty: bool,
    pub(crate) aux_dirty: bool,
    pub(crate) sel_dirty: bool,
    pub(crate) wire_dirty: bool,
    /// Most recently opened tool section.
    pub(crate) active_section: Option<ToolSection>,
    /// Sections currently expanded in the tool panel (several may be open).
    pub(crate) open_sections: Vec<ToolSection>,
    pub(crate) left_panel_width: f32,
    pub(crate) right_panel_width: f32,

    // --- Viewport interaction -----------------------------------------------
    pub(crate) hover_hit: Option<pick::Hit>,
    pub(crate) hover_tris: Vec<u32>,
    pub(crate) hover_radius_world: f32,
    pub(crate) hover_is_erase: bool,
    pub(crate) wheel_accum: f32,
    pub(crate) suppress_sel_drag: bool,
    /// Set on a primary press; the first brush change of the stroke records
    /// an undo snapshot and clears it.
    pub(crate) stroke_snapshot_pending: bool,
    /// World point the current orbit drag rotates around.
    pub(crate) orbit_pivot: Option<Vec3>,
    /// Depth (distance from the eye) of the point grabbed by the current pan.
    pub(crate) pan_depth: Option<f32>,
}

impl App {
    pub fn new() -> App {
        let mut app = App {
            camera: Camera::default(),
            frame: FrameOutput::default(),
            current: None,
            original: None,
            preview: None,
            preview_error: 0.0,
            base_mesh: None,
            file_path: None,
            bbox: Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            },
            mesh_generation: 0,
            sel: Arc::new(Vec::new()),
            sel_count: 0,
            sel_generation: 0,
            brush_radius: 25.0,
            brush_connected: true,
            expand_angle_deg: 45.0,
            hidden_regions: Vec::new(),
            hidden_mask: Vec::new(),
            bvh: None,
            topology: None,
            worker: Worker::new(),
            dec_job: None,
            dev_job: None,
            sym_job: None,
            bvh_job: None,
            bvh_job_mesh: None,
            topo_job: None,
            topo_job_mesh: None,
            freeform_job: None,
            load_job: None,
            groups_job: None,
            groups_rerun_pending: false,
            analysis_job: None,
            edit_job: None,
            solve_job: None,
            export_job: None,
            async_min_tris: ASYNC_MIN_TRIS,
            dec_mode: DecMode::Fixed,
            dec_ratio: 0.1,
            dec_error_mm: 0.02,
            dec_target_acc: 99.5,
            dec_target_mm: 0.05,
            dec_lock_border: false,
            dec_auto_preview: true,
            deviation: None,
            heat: None,
            heat_on: false,
            heat_max: 1.0,
            mode: Mode::Orbit,
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
            next_obj_id: 1,
            align_slots: AlignmentSlots::default(),
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
            repair_holes: Vec::new(),
            repair_selected_hole: None,
            repair_config: HoleFillConfig::default(),
            repair_preview_active: true,
            repair_preview_patch: None,
            repair_health: None,
            repair_refine_to_references: false,
            repair_solve_status: None,
            repair_circle_mode: CircleGuideMode::DiskAndRim,
            bridge_config: BridgeConfig::default(),
            bridge_preview_active: false,
            bridge_preview_patch: None,
            bridge_status: None,
            bridge_cluster_cache: None,
            show_mesh: true,
            show_object_browser: true,
            show_wireframe: false,
            show_bbox: true,
            show_triad: true,
            show_grid: true,
            theme_mode: crate::ui::theme::ThemeMode::Dark,
            undo: Vec::new(),
            redo: Vec::new(),
            status: "Open a mesh file to begin (STL, PLY or OBJ). Units are assumed to be mm."
                .to_string(),
            mesh_dirty: false,
            aux_dirty: false,
            sel_dirty: false,
            wire_dirty: false,
            active_section: None,
            open_sections: Vec::new(),
            left_panel_width: 330.0,
            right_panel_width: 300.0,
            hover_hit: None,
            hover_tris: Vec::new(),
            hover_radius_world: 0.0,
            hover_is_erase: false,
            wheel_accum: 0.0,
            suppress_sel_drag: false,
            stroke_snapshot_pending: false,
            orbit_pivot: None,
            pan_depth: None,
        };
        if !cfg!(test) {
            app.apply_settings(&crate::settings::Settings::load());
        }
        if let Some(arg) = std::env::args().nth(1) {
            let path = PathBuf::from(arg);
            if path.exists() {
                app.load_file(path);
            }
        }
        app
    }

    pub(crate) fn apply_settings(&mut self, s: &crate::settings::Settings) {
        self.left_panel_width = s.left_panel_width;
        self.right_panel_width = s.right_panel_width;
        self.theme_mode = s.theme;
        self.camera.set_up_axis(s.up_axis);
        self.show_grid = s.show_grid;
        self.show_bbox = s.show_bbox;
        self.show_triad = s.show_triad;
        self.show_wireframe = s.show_wireframe;
        self.show_object_browser = s.show_object_browser;
        self.brush_connected = s.brush_connected;
        self.brush_radius = s.brush_radius;
        self.open_sections = s.open_sections.clone();
        self.active_section = self.open_sections.last().copied();
    }

    pub(crate) fn settings(&self) -> crate::settings::Settings {
        crate::settings::Settings {
            left_panel_width: self.left_panel_width,
            right_panel_width: self.right_panel_width,
            theme: self.theme_mode,
            up_axis: self.camera.up_axis,
            show_grid: self.show_grid,
            show_bbox: self.show_bbox,
            show_triad: self.show_triad,
            show_wireframe: self.show_wireframe,
            show_object_browser: self.show_object_browser,
            brush_connected: self.brush_connected,
            brush_radius: self.brush_radius,
            open_sections: self.open_sections.clone(),
        }
    }

    pub(crate) fn is_section_open(&self, section: ToolSection) -> bool {
        self.open_sections.contains(&section)
    }

    /// Expands a section (keeping the others as they are).
    pub(crate) fn open_section(&mut self, section: ToolSection) {
        if !self.open_sections.contains(&section) {
            self.open_sections.push(section);
        }
        self.active_section = Some(section);
    }

    pub(crate) fn toggle_section(&mut self, section: ToolSection) {
        if self.is_section_open(section) {
            self.open_sections.retain(|s| *s != section);
            if self.active_section == Some(section) {
                self.active_section = self.open_sections.last().copied();
            }
        } else {
            self.open_section(section);
        }
    }

    /// The mesh shown in the viewport: the decimation preview when one is
    /// active, otherwise the current working mesh.
    pub(crate) fn display(&self) -> Option<&Arc<Mesh>> {
        self.preview.as_ref().or(self.current.as_ref())
    }

    pub(crate) fn orig_mesh(&self) -> Option<&Arc<Mesh>> {
        self.original.as_ref()
    }

    pub(crate) fn has_mesh(&self) -> bool {
        self.display().is_some()
    }

    /// Records that the displayed mesh object changed: drops every derived
    /// cache and schedules a full GPU re-upload.
    pub(crate) fn mark_mesh_changed(&mut self) {
        self.mesh_generation += 1;
        self.bvh = None;
        self.topology = None;
        self.bvh_job = None;
        self.bvh_job_mesh = None;
        self.topo_job = None;
        self.topo_job_mesh = None;
        self.bridge_cluster_cache = None;
        self.hover_hit = None;
        self.hover_tris.clear();
        self.mesh_dirty = true;
        self.aux_dirty = true;
        self.sel_dirty = true;
        self.wire_dirty = true;
    }

    /// Records that the selection content changed.
    pub(crate) fn mark_selection_changed(&mut self) {
        self.sel_generation += 1;
        self.sel_dirty = true;
    }

    /// Records that face visibility (hidden regions) changed.
    pub(crate) fn mark_visibility_changed(&mut self) {
        self.update_hidden_mask();
        self.hover_hit = None;
        self.hover_tris.clear();
        self.mesh_dirty = true;
        self.wire_dirty = true;
        self.sel_dirty = true;
    }

    pub(crate) fn sync_bbox(&mut self) {
        if let Some(m) = self.display() {
            self.bbox = m.bbox();
        }
    }

    /// Resets the selection to "nothing selected" for the displayed mesh.
    pub(crate) fn reset_selection(&mut self) {
        let tris = self.display().map(|m| m.triangle_count()).unwrap_or(0);
        self.sel = Arc::new(vec![0u8; tris]);
        self.sel_count = 0;
        self.bridge_preview_active = false;
        self.bridge_preview_patch = None;
        self.bridge_status = None;
        self.mark_selection_changed();
    }

    pub(crate) fn feature_name(&self, feat: FeatureRef) -> String {
        <Self as crate::geom::alignment::AlignmentGeometrySource>::get_feature_name(self, feat)
    }

    pub(crate) fn feature_assigned_axis(&self, feat: FeatureRef) -> Option<AxisChoice> {
        self.align_slots.axis_of(feat)
    }

    pub(crate) fn is_feature_origin(&self, feat: FeatureRef) -> bool {
        match (feat, self.align_slots.origin) {
            (FeatureRef::SymmetryPlane, OriginRef::SymmetryPlane) => true,
            (FeatureRef::Plane(id), OriginRef::Plane(oid)) => id == oid,
            (FeatureRef::Circle(id), OriginRef::CircleCenter(oid)) => id == oid,
            _ => false,
        }
    }

    /// Frames the whole model (animated).
    #[allow(dead_code)]
    pub(crate) fn fit_view(&mut self, now: f64) {
        if self.has_mesh() {
            let bb = self.bbox;
            self.camera.animate_fit(&bb, now);
        }
    }

    /// Frames the current selection, or the whole model when nothing is
    /// selected (animated).
    pub(crate) fn fit_view_to_selection(&mut self, now: f64) {
        let bb = self.selection_bbox().unwrap_or(self.bbox);
        if self.has_mesh() {
            self.camera.animate_fit(&bb, now);
        }
    }
}

impl Default for App {
    fn default() -> Self {
        Self::new()
    }
}

impl App {
    /// Runs one frame of the user interface: panels, tool sections and the
    /// viewport input. Called from the Bevy egui pass.
    pub(crate) fn frame(&mut self, ctx: &egui::Context) {
        crate::ui::theme::sync(ctx, self.theme_mode);
        self.worker.set_wake_context(ctx);
        self.handle_dropped_files(ctx);
        self.maintain_caches();
        self.handle_worker(ctx);
        crate::ui::shortcuts::handle_global(self, ctx);

        let mut root = egui::Ui::new(
            ctx.clone(),
            "scanimprover_root".into(),
            egui::UiBuilder::new()
                .layer_id(egui::LayerId::background())
                .max_rect(ctx.viewport_rect()),
        );
        let ui = &mut root;
        egui::Panel::top("topbar")
            .frame(crate::ui::theme::top_bar_frame())
            .show_separator_line(false)
            .show(ui, |ui| {
                crate::ui::toolbar::render_toolbar(self, ui);
            });
        egui::Panel::bottom("status")
            .frame(crate::ui::theme::status_bar_frame())
            .show_separator_line(false)
            .show(ui, |ui| {
                crate::ui::status_bar::render_status_bar(self, ui);
            });
        let left = egui::Panel::left("tools")
            .default_size(self.left_panel_width)
            .min_size(290.0)
            .max_size(460.0)
            .frame(crate::ui::theme::side_panel_frame())
            .show(ui, |ui| {
                crate::ui::render_left_panel(self, ui);
            });
        self.left_panel_width = left.response.rect.width();
        if self.show_object_browser {
            let right = egui::Panel::right("objects")
                .default_size(self.right_panel_width)
                .min_size(240.0)
                .max_size(460.0)
                .frame(crate::ui::theme::side_panel_frame())
                .show(ui, |ui| {
                    crate::ui::object_browser::render_object_browser(self, ui);
                });
            self.right_panel_width = right.response.rect.width();
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| {
                self.viewport(ui);
            });

        if self.camera.is_animating() {
            ctx.request_repaint();
        }
    }

    pub(crate) fn save_settings(&self) {
        if let Err(e) = self.settings().save() {
            eprintln!("could not save settings: {e}");
        }
    }
}

impl crate::geom::alignment::AlignmentGeometrySource for App {
    fn get_feature_direction_and_point(&self, feat: FeatureRef) -> Option<(Vec3, Vec3)> {
        match feat {
            FeatureRef::SymmetryPlane => self.sym.map(|s| (s.plane.normal, s.plane.point)),
            FeatureRef::Plane(id) => self
                .planes
                .iter()
                .find(|p| p.id == id)
                .map(|p| (p.fit.normal, p.fit.point)),
            FeatureRef::Circle(id) => self
                .circles
                .iter()
                .find(|c| c.id == id)
                .map(|c| (c.fit.normal, c.fit.center)),
        }
    }

    fn get_feature_name(&self, feat: FeatureRef) -> String {
        match feat {
            FeatureRef::SymmetryPlane => "Symmetry plane".to_string(),
            FeatureRef::Plane(id) => self
                .planes
                .iter()
                .find(|p| p.id == id)
                .map(|p| p.name.clone())
                .unwrap_or_else(|| format!("Plane {id}")),
            FeatureRef::Circle(id) => self
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

/// Shortest rotation taking `from` onto `to` (robust for anti-parallel input).
pub(crate) fn rotation_between(from: Vec3, to: Vec3) -> Quat {
    let f = from.normalize_or_zero();
    let t = to.normalize_or_zero();
    if f.length_squared() < 1e-12 || t.length_squared() < 1e-12 {
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
