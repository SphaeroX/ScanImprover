//! Document lifecycle: loading, rigid transforms, background jobs,
//! decimation preview and mesh replacement.

use super::{ASYNC_INDEX_MIN_VERTS, App, Mode};
use crate::geom::bvh::Bvh;
use crate::geom::hole_detect::HoleLoop;
use crate::geom::hole_fill::MeshPatch;
use crate::geom::topology::MeshTopology;
use crate::io;
use crate::mesh::{Aabb, Mesh};
use crate::worker::{JobResult, SymmetryJobKind};
use glam::{Quat, Vec3};
use std::path::PathBuf;
use std::sync::Arc;

impl App {
    // ----------------------------------------------------------------------
    // Loading
    // ----------------------------------------------------------------------

    pub(crate) fn load_file(&mut self, path: PathBuf) {
        let bytes = match std::fs::read(&path) {
            Ok(b) => b,
            Err(e) => {
                self.status = format!("Could not read file: {e}");
                return;
            }
        };
        match io::load_any(&path, &bytes) {
            Ok(mesh) => self.install_loaded_mesh(path, mesh),
            Err(e) => self.status = format!("Load failed: {e}"),
        }
    }

    /// Makes a freshly parsed mesh the document (centered at the origin when
    /// the auto-center setting is on).
    pub(crate) fn install_loaded_mesh(&mut self, path: PathBuf, mut mesh: Mesh) {
        if self.auto_center_on_load {
            let center = mesh.bbox().center();
            if center.length_squared() > 1e-10 {
                mesh.transform(Quat::IDENTITY, -center);
            }
        }
        let tris = mesh.triangle_count();
        let verts = mesh.vertex_count();
        let m = Arc::new(mesh);

        self.current = Some(m.clone());
        self.original = Some(m);
        self.preview = None;
        self.base_mesh = None;
        self.file_path = Some(path);
        self.clear_document_state();
        self.mark_mesh_changed();
        self.sync_bbox();
        self.reset_selection();
        self.update_hidden_mask();
        self.camera.fit(&self.bbox);
        let placement = if self.auto_center_on_load {
            "Centered at global origin."
        } else {
            "Original coordinates kept."
        };
        self.status = format!("Loaded: {tris} triangles, {verts} vertices (welded). {placement}");
    }

    /// Clears everything that belongs to the previously loaded document.
    fn clear_document_state(&mut self) {
        self.deviation = None;
        self.heat = None;
        self.heat_on = false;
        self.mode = Mode::Orbit;
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
        self.align_slots = Default::default();
        self.invalidate_face_groups();
        self.repair_holes.clear();
        self.repair_selected_hole = None;
        self.repair_preview_patch = None;
        self.repair_health = None;
        self.repair_solve_status = None;
        self.bridge_preview_active = false;
        self.bridge_preview_patch = None;
        self.bridge_status = None;
        self.hidden_regions.clear();
        self.undo.clear();
        self.redo.clear();
        self.dec_job = None;
        self.dev_job = None;
        self.sym_job = None;
        self.groups_job = None;
        self.groups_rerun_pending = false;
        self.analysis_job = None;
        self.edit_job = None;
        self.solve_job = None;
        self.wheel_accum = 0.0;
        self.suppress_sel_drag = false;
        self.stroke_snapshot_pending = false;
    }

    /// Opens a mesh file dropped onto the window.
    pub(crate) fn handle_dropped_files(&mut self, ctx: &egui::Context) {
        let dropped: Vec<PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|f| f.path().to_path_buf())
                .collect()
        });
        if let Some(path) = dropped.into_iter().find(|p| io::is_supported_mesh_path(p)) {
            self.open_file_async(path);
        }
    }

    // ----------------------------------------------------------------------
    // Rigid transforms
    // ----------------------------------------------------------------------

    /// Applies a rigid transform to every mesh and every feature / cache that
    /// stores world coordinates.
    pub(crate) fn apply_transform(&mut self, rot: Quat, trans: Vec3) {
        self.push_snapshot();
        let transformed = |m: &Option<Arc<Mesh>>| {
            m.as_ref().map(|m| {
                let mut m2 = (**m).clone();
                m2.transform(rot, trans);
                Arc::new(m2)
            })
        };
        self.current = transformed(&self.current);
        self.original = transformed(&self.original);
        self.preview = transformed(&self.preview);
        self.base_mesh = transformed(&self.base_mesh);

        let xf_point = |p: Vec3| rot * p + trans;
        let xf_dir = |d: Vec3| (rot * d).normalize_or_zero();

        if let Some(s) = &mut self.sym {
            s.plane.point = xf_point(s.plane.point);
            s.plane.normal = xf_dir(s.plane.normal);
        }
        for p in &mut self.planes {
            p.fit.point = xf_point(p.fit.point);
            p.fit.normal = xf_dir(p.fit.normal);
        }
        for c in &mut self.circles {
            c.fit.center = xf_point(c.fit.center);
            c.fit.normal = xf_dir(c.fit.normal);
        }
        for f in &mut self.freeforms {
            if let Some(surf) = &mut f.surface {
                surf.transform(rot, trans);
            }
            for p in Arc::make_mut(&mut f.source_points).iter_mut() {
                *p = xf_point(Vec3::from(*p)).to_array();
            }
            for (a, b) in &mut f.boundary {
                *a = xf_point(Vec3::from(*a)).to_array();
                *b = xf_point(Vec3::from(*b)).to_array();
            }
        }
        for g in &mut self.face_groups {
            g.center = xf_point(g.center);
            g.point = xf_point(g.point);
            g.normal = xf_dir(g.normal);
        }
        if let Some(p) = &mut self.plane {
            p.point = xf_point(p.point);
            p.normal = xf_dir(p.normal);
        }
        if let Some(c) = &mut self.circle {
            c.center = xf_point(c.center);
            c.normal = xf_dir(c.normal);
        }
        for p in &mut self.sym_pick {
            *p = xf_point(*p);
        }
        for hole in &mut self.repair_holes {
            transform_hole(hole, rot, trans);
        }
        if let Some(patch) = &mut self.repair_preview_patch {
            transform_patch(patch, rot, trans);
        }
        if let Some(patch) = &mut self.bridge_preview_patch {
            transform_patch(patch, rot, trans);
        }
        // The camera follows the model so the view does not jump.
        self.camera.target = xf_point(self.camera.target);
        self.mark_mesh_changed();
        self.sync_bbox();
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
            let axes: Vec<&str> = [(x, "X"), (y, "Y"), (z, "Z")]
                .into_iter()
                .filter(|(on, _)| *on)
                .map(|(_, n)| n)
                .collect();
            self.status = format!("Centered {} axis origin.", axes.join(", "));
        }
    }

    // ----------------------------------------------------------------------
    // Spatial caches
    // ----------------------------------------------------------------------

    /// Returns the picking BVH for the displayed mesh, building it on the
    /// spot for small meshes. Large meshes are indexed on the worker thread;
    /// `None` is returned until that finishes.
    pub(crate) fn ensure_bvh(&mut self) -> Option<Arc<Bvh>> {
        if let Some(b) = &self.bvh {
            return Some(b.clone());
        }
        let m = self.display()?.clone();
        if m.vertex_count() < ASYNC_INDEX_MIN_VERTS {
            let bvh = Arc::new(Bvh::new(&m.positions, &m.indices));
            self.bvh = Some(bvh.clone());
            Some(bvh)
        } else {
            self.submit_bvh_build(m);
            None
        }
    }

    /// Returns the edge-adjacency topology for the displayed mesh (same
    /// synchronous / asynchronous split as [`Self::ensure_bvh`]).
    pub(crate) fn ensure_topology(&mut self) -> Option<Arc<MeshTopology>> {
        if let Some(t) = &self.topology {
            return Some(t.clone());
        }
        let m = self.display()?.clone();
        if m.vertex_count() < ASYNC_INDEX_MIN_VERTS {
            let topo = Arc::new(MeshTopology::build(&m));
            self.topology = Some(topo.clone());
            Some(topo)
        } else {
            self.submit_topology_build(m);
            None
        }
    }

    fn submit_bvh_build(&mut self, m: Arc<Mesh>) {
        if self.bvh_job.is_none() {
            self.bvh_job_mesh = Some(m.clone());
            self.bvh_job = Some(self.worker.submit_bvh_build(m));
            self.status = "Preparing picking (building spatial index)…".to_string();
        }
    }

    fn submit_topology_build(&mut self, m: Arc<Mesh>) {
        if self.topo_job.is_none() {
            self.topo_job_mesh = Some(m.clone());
            self.topo_job = Some(self.worker.submit_topology_build(m));
            self.status = "Preparing mesh topology…".to_string();
        }
    }

    /// Kicks off background index builds for large meshes so they are ready
    /// by the time the user starts interacting.
    pub(crate) fn maintain_caches(&mut self) {
        let Some(m) = self.display().cloned() else {
            return;
        };
        if m.vertex_count() < ASYNC_INDEX_MIN_VERTS {
            return;
        }
        if self.bvh.is_none() && self.bvh_job.is_none() {
            self.submit_bvh_build(m.clone());
        }
        if self.topology.is_none() && self.topo_job.is_none() {
            self.submit_topology_build(m);
        }
    }

    // ----------------------------------------------------------------------
    // Decimation preview
    // ----------------------------------------------------------------------

    fn set_preview(&mut self, mesh: Arc<Mesh>, error: f32) {
        self.preview = Some(mesh);
        self.preview_error = error;
        self.invalidate_face_groups();
        self.mark_mesh_changed();
        // The preview is a different triangle set: selection and hidden faces
        // of the working mesh do not apply to it.
        self.sel = Arc::new(Vec::new());
        self.sel_count = 0;
        self.mark_selection_changed();
        self.update_hidden_mask();
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

    pub(crate) fn apply_preview(&mut self) {
        let Some(p) = self.preview.clone() else {
            return;
        };
        self.push_snapshot();
        self.current = Some(p.clone());
        self.original = Some(p);
        self.preview = None;
        self.deviation = None;
        self.heat = None;
        // Hidden regions index faces of the previous mesh and cannot be
        // carried over to the decimated triangle set.
        let dropped_hidden = self.hidden_regions.len();
        self.hidden_regions.clear();
        self.invalidate_face_groups();
        self.mark_mesh_changed();
        self.reset_selection();
        self.update_hidden_mask();
        self.sync_bbox();
        self.status = if dropped_hidden > 0 {
            format!(
                "Decimation applied. New mesh is the reference ({dropped_hidden} hidden region(s) restored)."
            )
        } else {
            "Decimation applied. New mesh is the reference.".to_string()
        };
    }

    pub(crate) fn discard_preview(&mut self) {
        self.preview = None;
        self.deviation = None;
        self.heat = None;
        self.invalidate_face_groups();
        self.mark_mesh_changed();
        self.reset_selection();
        self.update_hidden_mask();
        self.sync_bbox();
        self.status = "Preview discarded.".to_string();
    }

    pub(crate) fn reset_mesh(&mut self) {
        let Some(orig) = self.original.clone() else {
            return;
        };
        self.push_snapshot();
        self.current = Some(orig);
        self.preview = None;
        self.deviation = None;
        self.heat = None;
        self.hidden_regions.clear();
        self.invalidate_face_groups();
        self.mark_mesh_changed();
        self.reset_selection();
        self.update_hidden_mask();
        self.sync_bbox();
        self.status = "Reset to original mesh.".to_string();
    }

    /// Replaces the working mesh after a topology-changing edit (hole fill,
    /// bridge, repair). Hidden regions are dropped because their face
    /// indices no longer apply; callers that can remap them re-assign
    /// `hidden_regions` afterwards.
    pub(crate) fn set_mesh_modified(&mut self, next_mesh: Mesh, status_msg: String) {
        let m = Arc::new(next_mesh);
        self.current = Some(m);
        self.preview = None;
        self.deviation = None;
        self.heat = None;
        self.hidden_regions.clear();
        self.invalidate_face_groups();
        self.mark_mesh_changed();
        self.reset_selection();
        self.update_hidden_mask();
        self.sync_bbox();
        self.status = status_msg;
        self.request_repair_analysis();
    }

    // ----------------------------------------------------------------------
    // Background job results
    // ----------------------------------------------------------------------

    pub(crate) fn handle_worker(&mut self, _ctx: &egui::Context) {
        for res in self.worker.poll() {
            match res {
                JobResult::Decimated { id, mesh, error } => {
                    if self.dec_job == Some(id) {
                        self.dec_job = None;
                        self.set_preview(mesh, error);
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
                    self.clear_job(id);
                    self.status = message;
                }
                JobResult::Deviation { id, dev, heat } => {
                    if self.dev_job == Some(id) {
                        self.dev_job = None;
                        self.deviation = Some(dev);
                        self.heat_max = dev.max_dev.max(1e-6);
                        self.heat = Some(heat);
                        self.aux_dirty = true;
                    }
                }
                JobResult::Symmetry { id, plane } => {
                    if self.sym_job == Some(id) {
                        self.sym_job = None;
                        self.sym_pick.clear();
                        match plane {
                            Some((p, rms)) => {
                                self.sym = Some(super::SymState {
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
                JobResult::TopologyReady { id, topology } => {
                    if self.topo_job == Some(id) {
                        self.topo_job = None;
                        let matches = match (self.display(), &self.topo_job_mesh) {
                            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                            _ => false,
                        };
                        self.topo_job_mesh = None;
                        if matches {
                            self.topology = Some(topology);
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
                        self.finish_freeform_fit(freeform_id, data);
                    }
                }
                JobResult::Task { id, output } => self.handle_task_output(id, output),
            }
        }
    }

    /// Forgets a job that failed, whichever kind it was.
    fn clear_job(&mut self, id: u64) {
        self.clear_task_job(id);
        if self.dec_job == Some(id) {
            self.dec_job = None;
        }
        if self.dev_job == Some(id) {
            self.dev_job = None;
        }
        if self.sym_job == Some(id) {
            self.sym_job = None;
        }
        if self.bvh_job == Some(id) {
            self.bvh_job = None;
            self.bvh_job_mesh = None;
        }
        if self.topo_job == Some(id) {
            self.topo_job = None;
            self.topo_job_mesh = None;
        }
        if self.freeform_job.map(|(jid, _)| jid) == Some(id) {
            let (_, fid) = self.freeform_job.take().unwrap();
            if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == fid) {
                f.refit_pending = false;
            }
        }
    }

    // ----------------------------------------------------------------------
    // Symmetry jobs
    // ----------------------------------------------------------------------

    fn symmetry_mask(&self) -> Option<Arc<Vec<u8>>> {
        if self.sym_exclude_selection && self.sel_count > 0 {
            Some(self.sel.clone())
        } else {
            None
        }
    }

    fn submit_symmetry(&mut self, kind: SymmetryJobKind, status: &str) {
        if let Some(m) = self.display().cloned() {
            let mask = self.symmetry_mask();
            self.sym_job = Some(
                self.worker
                    .submit_symmetry(m, kind, mask, self.sym_exclude_holes),
            );
            self.status = status.to_string();
        }
    }

    pub(crate) fn schedule_sym_auto(&mut self) {
        self.submit_symmetry(SymmetryJobKind::Auto, "Detecting symmetry plane…");
    }

    pub(crate) fn schedule_sym_from_line(&mut self, a: Vec3, b: Vec3) {
        self.submit_symmetry(
            SymmetryJobKind::FromLine { a, b },
            "Calculating and optimizing symmetry plane from line…",
        );
    }

    pub(crate) fn schedule_sym_refine(&mut self, init: crate::geom::symmetry::SymPlane) {
        self.submit_symmetry(SymmetryJobKind::Refine(init), "Optimizing symmetry plane…");
    }
}

fn transform_hole(hole: &mut HoleLoop, rot: Quat, trans: Vec3) {
    hole.centroid = rot * hole.centroid + trans;
    hole.normal = (rot * hole.normal).normalize_or_zero();
    // Rotate the eight corners and re-derive an axis aligned box.
    let (min, max) = (hole.bbox.min, hole.bbox.max);
    let mut nmin = Vec3::splat(f32::MAX);
    let mut nmax = Vec3::splat(f32::MIN);
    for i in 0..8 {
        let c = Vec3::new(
            if i & 1 == 0 { min.x } else { max.x },
            if i & 2 == 0 { min.y } else { max.y },
            if i & 4 == 0 { min.z } else { max.z },
        );
        let p = rot * c + trans;
        nmin = nmin.min(p);
        nmax = nmax.max(p);
    }
    hole.bbox = Aabb {
        min: nmin,
        max: nmax,
    };
}

fn transform_patch(patch: &mut MeshPatch, rot: Quat, trans: Vec3) {
    for p in patch
        .new_positions
        .iter_mut()
        .chain(patch.preview_positions.iter_mut())
    {
        *p = (rot * Vec3::from(*p) + trans).to_array();
    }
}
