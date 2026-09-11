//! Worker-backed variants of the slow document operations.
//!
//! Every operation has a synchronous implementation (used directly for
//! small meshes, by tests and by the command line loader). The `request_*`
//! functions used by the UI dispatch the same work to the worker thread once
//! the mesh is large enough for it to take a noticeable amount of time, and
//! apply the result when it arrives via [`App::handle_task_output`].

use super::App;
use crate::geom::hole_fill::{apply_patch, generate_hole_patch};
use crate::geom::hole_solver::refine_patch_to_references;
use crate::geom::repair::{
    analyze_mesh, auto_repair_mesh, remove_degenerate_faces, remove_small_components, unify_normals,
};
use crate::geom::segment::segment_faces;
use crate::geom::topology::MeshTopology;
use crate::io;
use crate::mesh::Mesh;
use crate::worker::{Progress, TaskOutput};
use std::path::PathBuf;
use std::sync::Arc;

/// Mesh editing operations that can run on the worker.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MeshEditKind {
    AutoRepair,
    UnifyNormals,
    RemoveDebris,
    RemoveDegenerate,
    FillAllHoles,
}

impl App {
    fn display_tri_count(&self) -> usize {
        self.display().map(|m| m.triangle_count()).unwrap_or(0)
    }

    /// True when the displayed mesh is large enough to justify a worker job.
    pub(super) fn prefers_async(&self) -> bool {
        self.display_tri_count() >= self.async_min_tris
    }

    /// True while an operation that will replace the mesh is running.
    pub(crate) fn is_editing(&self) -> bool {
        self.edit_job.is_some() || self.load_job.is_some()
    }

    fn refuse_if_editing(&mut self) -> bool {
        if self.is_editing() {
            self.status = "Please wait for the running operation to finish.".to_string();
            return true;
        }
        false
    }

    // ----------------------------------------------------------------------
    // Loading
    // ----------------------------------------------------------------------

    /// Loads a mesh file on the worker thread; the current document stays
    /// usable until the new one is ready.
    pub(crate) fn open_file_async(&mut self, path: PathBuf) {
        if self.refuse_if_editing() {
            return;
        }
        let name = path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("mesh")
            .to_string();
        let task_path = path.clone();
        self.status = format!("Loading {name}…");
        self.load_job = Some(
            self.worker
                .submit_task(format!("Loading {name}"), move |p| {
                    p.set(None, "reading file");
                    let mesh = std::fs::read(&task_path)
                        .map_err(|e| format!("Could not read file: {e}"))
                        .and_then(|bytes| {
                            p.set(None, "parsing");
                            io::load_any(&task_path, &bytes)
                        });
                    TaskOutput::Loaded {
                        path: task_path,
                        mesh,
                    }
                }),
        );
    }

    // ----------------------------------------------------------------------
    // Face groups
    // ----------------------------------------------------------------------

    /// Detects face groups, on the worker for large meshes. A request made
    /// while a detection is running is coalesced into one re-run.
    pub(crate) fn request_face_groups(&mut self) {
        if self.groups_job.is_some() {
            self.groups_rerun_pending = true;
            return;
        }
        if !self.prefers_async() {
            self.detect_face_groups();
            return;
        }
        let Some(mesh) = self.display().cloned() else {
            return;
        };
        let topo = self.topology.clone();
        let angle = self.group_angle_deg;
        let min_tris = self.group_min_tris.round().max(1.0) as usize;
        let tol = self.group_fit_tol;
        let feature = self.group_feature_frac;
        self.status = "Detecting face groups…".to_string();
        self.groups_job = Some(self.worker.submit_task("Face groups", move |p| {
            let topo = match topo {
                Some(t) => t,
                None => {
                    p.set(None, "building topology");
                    Arc::new(MeshTopology::build(&mesh))
                }
            };
            p.set(None, "segmenting");
            let (groups, ids) = segment_faces(&mesh, &topo, angle, min_tris, tol, feature);
            TaskOutput::FaceGroups { groups, ids }
        }));
    }

    // ----------------------------------------------------------------------
    // Repair analysis
    // ----------------------------------------------------------------------

    /// Re-runs hole detection and the health report (worker for large meshes).
    pub(crate) fn request_repair_analysis(&mut self) {
        if !self.prefers_async() {
            self.refresh_repair();
            return;
        }
        let Some(mesh) = self.display().cloned() else {
            return;
        };
        let generation = self.mesh_generation;
        self.analysis_job = Some((
            self.worker.submit_task("Mesh analysis", move |p| {
                p.set(None, "detecting holes");
                let holes = crate::geom::hole_detect::detect_holes(&mesh);
                p.set(None, "checking manifoldness");
                let health = analyze_mesh(&mesh);
                TaskOutput::RepairAnalysis { holes, health }
            }),
            generation,
        ));
    }

    // ----------------------------------------------------------------------
    // Mesh edits
    // ----------------------------------------------------------------------

    /// Runs a mesh editing operation, on the worker for large meshes.
    pub(crate) fn request_mesh_edit(&mut self, kind: MeshEditKind) {
        if self.refuse_if_editing() {
            return;
        }
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        if !self.prefers_async() {
            match kind {
                MeshEditKind::AutoRepair => self.auto_repair(),
                MeshEditKind::UnifyNormals => self.unify_normals_action(),
                MeshEditKind::RemoveDebris => self.remove_small_components_action(),
                MeshEditKind::RemoveDegenerate => self.remove_degenerate_faces_action(),
                MeshEditKind::FillAllHoles => self.fill_all_holes(),
            }
            return;
        }
        let Some(curr) = self.current.clone() else {
            return;
        };
        if kind == MeshEditKind::FillAllHoles && self.repair_holes.is_empty() {
            self.status = "No holes to fill.".to_string();
            return;
        }
        let holes = self.repair_holes.clone();
        let config = self.repair_config;
        let refs = if self.repair_refine_to_references {
            self.active_reference_geometries()
        } else {
            Vec::new()
        };
        let label = match kind {
            MeshEditKind::AutoRepair => "Auto repair",
            MeshEditKind::UnifyNormals => "Unify normals",
            MeshEditKind::RemoveDebris => "Remove debris",
            MeshEditKind::RemoveDegenerate => "Remove degenerate faces",
            MeshEditKind::FillAllHoles => "Fill holes",
        };
        self.status = format!("{label}…");
        let generation = self.mesh_generation;
        let id = self.worker.submit_task(label, move |p| {
            run_mesh_edit(kind, &curr, &holes, config, &refs, p)
        });
        self.edit_job = Some((id, generation));
    }

    /// Runs the hole solver, on the worker for large meshes.
    pub(crate) fn request_hole_solve(&mut self) {
        if !self.prefers_async() {
            self.solve_best_hole_fill();
            return;
        }
        let refs = self.active_reference_geometries();
        if refs.is_empty() {
            self.status = "No fitted planes or circles available to guide hole fill.".to_string();
            self.repair_solve_status =
                Some("No planes/circles available to guide solver.".to_string());
            return;
        }
        let hole = self
            .repair_selected_hole
            .and_then(|idx| self.repair_holes.get(idx).cloned())
            .or_else(|| self.repair_holes.first().cloned());
        let (Some(m), Some(hole)) = (self.display().cloned(), hole) else {
            self.status = "No holes detected to solve.".to_string();
            self.repair_solve_status = Some("No holes detected".to_string());
            return;
        };
        self.repair_solve_status = Some("Solving…".to_string());
        self.solve_job = Some(self.worker.submit_task("Hole solver", move |p| {
            p.set(None, "testing configurations");
            TaskOutput::HoleSolve {
                result: crate::geom::hole_solver::solve_best_hole_config(&m, &hole, &refs),
            }
        }));
    }

    // ----------------------------------------------------------------------
    // Export
    // ----------------------------------------------------------------------

    /// Serialises and writes the displayed mesh on the worker thread.
    pub(crate) fn export_mesh_async(&mut self, path: PathBuf) {
        let Some(mesh) = self.display().cloned() else {
            self.status = "Nothing to export.".to_string();
            return;
        };
        let name = path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("mesh")
            .to_string();
        self.status = format!("Exporting {name}…");
        self.export_job = Some(
            self.worker
                .submit_task(format!("Exporting {name}"), move |p| {
                    p.set(None, "writing");
                    let result =
                        io::save_any(&path, &mesh).map(|_| format!("Exported {}.", path.display()));
                    TaskOutput::Exported { result }
                }),
        );
    }

    // ----------------------------------------------------------------------
    // Results
    // ----------------------------------------------------------------------

    pub(crate) fn handle_task_output(&mut self, id: u64, output: TaskOutput) {
        match output {
            TaskOutput::Loaded { path, mesh } => {
                if self.load_job == Some(id) {
                    self.load_job = None;
                    match mesh {
                        Ok(mesh) => self.install_loaded_mesh(path, mesh),
                        Err(e) => self.status = format!("Load failed: {e}"),
                    }
                }
            }
            TaskOutput::FaceGroups { groups, ids } => {
                if self.groups_job == Some(id) {
                    self.groups_job = None;
                    let valid = self
                        .display()
                        .is_some_and(|m| ids.len() == m.triangle_count());
                    if valid {
                        self.install_face_groups(groups, ids);
                    }
                    if self.groups_rerun_pending {
                        self.groups_rerun_pending = false;
                        self.request_face_groups();
                    }
                }
            }
            TaskOutput::RepairAnalysis { holes, health } => {
                if let Some((jid, generation)) = self.analysis_job
                    && jid == id
                {
                    self.analysis_job = None;
                    if generation == self.mesh_generation {
                        self.repair_holes = holes;
                        self.repair_health = Some(health);
                        if let Some(sel) = self.repair_selected_hole
                            && sel >= self.repair_holes.len()
                        {
                            self.repair_selected_hole = None;
                        }
                        self.update_hole_preview();
                    }
                }
            }
            TaskOutput::MeshEdit { mesh, summary } => {
                if let Some((jid, generation)) = self.edit_job
                    && jid == id
                {
                    self.edit_job = None;
                    match mesh {
                        Ok(mesh) if generation == self.mesh_generation => {
                            self.push_snapshot();
                            self.set_mesh_modified(mesh, summary);
                        }
                        Ok(_) => {
                            self.status =
                                "Operation discarded: the mesh changed while it was running."
                                    .to_string();
                        }
                        Err(e) => self.status = e,
                    }
                }
            }
            TaskOutput::HoleSolve { result } => {
                if self.solve_job == Some(id) {
                    self.solve_job = None;
                    self.apply_hole_solve_result(result);
                }
            }
            TaskOutput::Exported { result } => {
                if self.export_job == Some(id) {
                    self.export_job = None;
                }
                self.status = match result {
                    Ok(msg) => msg,
                    Err(e) => format!("Export failed: {e}"),
                };
            }
            TaskOutput::Solid { result } => self.finish_solid_job(id, result),
        }
    }

    /// Forgets a task job that failed.
    pub(crate) fn clear_task_job(&mut self, id: u64) {
        if self.load_job == Some(id) {
            self.load_job = None;
        }
        if self.groups_job == Some(id) {
            self.groups_job = None;
            self.groups_rerun_pending = false;
        }
        if self.analysis_job.map(|(j, _)| j) == Some(id) {
            self.analysis_job = None;
        }
        if self.edit_job.map(|(j, _)| j) == Some(id) {
            self.edit_job = None;
        }
        if self.solve_job == Some(id) {
            self.solve_job = None;
            self.repair_solve_status = Some("Solver failed.".to_string());
        }
        if self.export_job == Some(id) {
            self.export_job = None;
        }
        self.clear_solid_job(id);
    }
}

fn run_mesh_edit(
    kind: MeshEditKind,
    curr: &Mesh,
    holes: &[crate::geom::hole_detect::HoleLoop],
    config: crate::geom::hole_fill::HoleFillConfig,
    refs: &[crate::geom::hole_solver::ReferenceGeometry],
    progress: &Progress,
) -> TaskOutput {
    let (mesh, summary) = match kind {
        MeshEditKind::AutoRepair => {
            progress.set(None, "repairing");
            let (m, s) = auto_repair_mesh(curr);
            (Ok(m), s)
        }
        MeshEditKind::UnifyNormals => {
            progress.set(None, "unifying normals");
            (
                Ok(unify_normals(curr)),
                "Unified triangle normals across all shared edges.".to_string(),
            )
        }
        MeshEditKind::RemoveDebris => {
            progress.set(None, "finding shells");
            (
                Ok(remove_small_components(curr, false, 0.005)),
                "Removed small floating components (< 0.5% faces).".to_string(),
            )
        }
        MeshEditKind::RemoveDegenerate => {
            progress.set(None, "scanning faces");
            (
                Ok(remove_degenerate_faces(curr)),
                "Removed degenerate and zero-area faces.".to_string(),
            )
        }
        MeshEditKind::FillAllHoles => {
            let mut working = curr.clone();
            let mut filled = 0usize;
            for (i, hole) in holes.iter().enumerate() {
                progress.set(
                    Some(i as f32 / holes.len().max(1) as f32),
                    &format!("hole {} of {}", i + 1, holes.len()),
                );
                if let Ok(mut patch) = generate_hole_patch(&working, hole, config) {
                    if !refs.is_empty() {
                        refine_patch_to_references(&mut patch, &working, hole, refs);
                    }
                    apply_patch(&mut working, &patch);
                    filled += 1;
                }
            }
            if filled == 0 {
                (Err("Failed to fill holes.".to_string()), String::new())
            } else {
                (
                    Ok(working),
                    format!(
                        "Filled {filled}/{} holes using {}.",
                        holes.len(),
                        config.method.display_name()
                    ),
                )
            }
        }
    };
    TaskOutput::MeshEdit { mesh, summary }
}
