//! Face-group guided hole filling (Experimental section).
//!
//! Works on the hole list of the repair section and the face groups of the
//! Face groups section (no state of its own besides the density and the
//! preview). The preview is cached together with the inputs it was built
//! from and rebuilt lazily while the tool is visible.

use super::App;
use crate::geom::guided_fill::{
    Guide, GuidedBatch, GuidedFillConfig, GuidedFillResult, fill_holes_guided, guided_hole_patch,
    record_patch_groups,
};
use crate::geom::hole_fill::apply_patch;
use crate::geom::segment::FaceGroup;
use crate::mesh::Mesh;
use crate::worker::TaskOutput;

/// Guided preview of the selected hole and the inputs it was built from.
pub(crate) struct GuidedPreview {
    mesh_generation: u64,
    groups_generation: u64,
    hole: Vec<u32>,
    config: GuidedFillConfig,
    guides: Vec<Guide>,
    result: Result<GuidedFillResult, String>,
}

impl App {
    /// Fill parameters: the tool's density, the face group fit tolerance and
    /// the standard fill settings of the repair section as fallback.
    pub(crate) fn guided_fill_config(&self) -> GuidedFillConfig {
        GuidedFillConfig {
            density: self.guided_density,
            fit_tol: self.group_fit_tol,
            fallback: self.repair_config,
        }
    }

    /// Cylinders of the visible fitted circles, guiding the fill along round
    /// profiles the face groups miss.
    pub(crate) fn guided_fill_guides(&self) -> Vec<Guide> {
        self.circles
            .iter()
            .filter(|c| c.visible)
            .filter_map(Guide::from_circle)
            .collect()
    }

    /// True when face groups exist for the displayed mesh.
    pub(crate) fn face_groups_valid(&self) -> bool {
        !self.face_groups.is_empty()
            && self
                .display()
                .is_some_and(|m| self.group_ids.len() == m.triangle_count())
    }

    /// The preview, if it is enabled and belongs to the current mesh and
    /// face groups.
    pub(crate) fn guided_preview_result(&self) -> Option<&Result<GuidedFillResult, String>> {
        let p = self.guided_preview.as_ref()?;
        (self.guided_preview_active
            && p.mesh_generation == self.mesh_generation
            && p.groups_generation == self.groups_generation)
            .then_some(&p.result)
    }

    /// Rebuilds the preview when the mesh, the face groups, the selected
    /// hole or the settings changed.
    pub(crate) fn sync_guided_preview(&mut self) {
        let hole = self
            .repair_selected_hole
            .and_then(|i| self.repair_holes.get(i))
            .cloned();
        let (true, Some(hole), Some(mesh)) =
            (self.guided_preview_active, hole, self.display().cloned())
        else {
            self.guided_preview = None;
            return;
        };
        let config = self.guided_fill_config();
        let guides = self.guided_fill_guides();
        if let Some(p) = &self.guided_preview
            && p.mesh_generation == self.mesh_generation
            && p.groups_generation == self.groups_generation
            && p.hole == hole.vertices
            && p.config == config
            && p.guides == guides
        {
            return;
        }
        let result = guided_hole_patch(
            &mesh,
            &hole,
            &self.face_groups,
            &self.group_ids,
            &guides,
            config,
        );
        self.guided_preview = Some(GuidedPreview {
            mesh_generation: self.mesh_generation,
            groups_generation: self.groups_generation,
            hole: hole.vertices,
            config,
            guides,
            result,
        });
    }

    /// Fills the selected hole guided by the face groups (one undo step).
    pub(crate) fn fill_selected_hole_guided(&mut self) {
        if self.is_editing() {
            self.status = "Please wait for the running operation to finish.".to_string();
            return;
        }
        let Some(hole) = self
            .repair_selected_hole
            .and_then(|i| self.repair_holes.get(i))
            .cloned()
        else {
            return;
        };
        let Some(curr) = self.current.clone() else {
            return;
        };
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        let groups_valid = self.face_groups_valid();
        let config = self.guided_fill_config();
        let guides = self.guided_fill_guides();
        match guided_hole_patch(
            &curr,
            &hole,
            &self.face_groups,
            &self.group_ids,
            &guides,
            config,
        ) {
            Ok(result) => {
                let mut mesh = (*curr).clone();
                let first = mesh.triangle_count();
                apply_patch(&mut mesh, &result.patch);
                let groups = groups_valid.then(|| {
                    let mut groups = self.face_groups.clone();
                    let mut ids = self.group_ids.clone();
                    record_patch_groups(&mut groups, &mut ids, first, &result.tri_groups);
                    (groups, ids)
                });
                let summary = if result.report.guided {
                    format!("Hole #{} filled: {}.", hole.id, result.report.note)
                } else {
                    format!(
                        "Hole #{} filled with the standard fill ({}).",
                        hole.id, result.report.note
                    )
                };
                self.push_snapshot();
                self.install_guided_fill(mesh, groups, summary);
            }
            Err(e) => self.status = format!("Failed to fill hole: {e}"),
        }
    }

    /// Fills every detected hole guided by the face groups, on the worker for
    /// large meshes. The result is one undo step.
    pub(crate) fn request_guided_fill_all(&mut self) {
        if self.is_editing() {
            self.status = "Please wait for the running operation to finish.".to_string();
            return;
        }
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        if self.repair_holes.is_empty() {
            self.status = "No holes to fill.".to_string();
            return;
        }
        let Some(curr) = self.current.clone() else {
            return;
        };
        let holes = self.repair_holes.clone();
        let (groups, ids) = if self.face_groups_valid() {
            (self.face_groups.clone(), self.group_ids.clone())
        } else {
            (Vec::new(), Vec::new())
        };
        let config = self.guided_fill_config();
        let guides = self.guided_fill_guides();
        if !self.prefers_async() {
            let batch = fill_holes_guided(&curr, &holes, &groups, &ids, &guides, config, |_, _| {});
            self.finish_guided_batch(batch);
            return;
        }
        self.status = "Guided hole fill…".to_string();
        let generation = self.mesh_generation;
        let id = self.worker.submit_task("Guided hole fill", move |p| {
            let batch = fill_holes_guided(&curr, &holes, &groups, &ids, &guides, config, |i, n| {
                p.set(
                    Some(i as f32 / n.max(1) as f32),
                    &format!("hole {} of {}", i + 1, n),
                )
            });
            TaskOutput::GuidedFill {
                batch: Box::new(batch),
            }
        });
        self.edit_job = Some((id, generation));
    }

    /// Installs the result of filling all holes as one undo step.
    pub(crate) fn finish_guided_batch(&mut self, batch: GuidedBatch) {
        let filled = batch.guided + batch.fallback;
        if filled == 0 {
            self.status = "Failed to fill holes.".to_string();
            return;
        }
        let summary = format!(
            "Filled {filled}/{} holes: {} rebuilt from face groups, {} with the standard fill.",
            filled + batch.failed,
            batch.guided,
            batch.fallback
        );
        self.push_snapshot();
        self.install_guided_fill(batch.mesh, batch.groups, summary);
    }

    /// Replaces the mesh. Patches only append triangles, so the face groups
    /// stay valid: they are re-installed with the patch faces added.
    fn install_guided_fill(
        &mut self,
        mesh: Mesh,
        groups: Option<(Vec<FaceGroup>, Vec<i32>)>,
        summary: String,
    ) {
        self.set_mesh_modified(mesh, summary.clone());
        if let Some((groups, ids)) = groups
            && self
                .display()
                .is_some_and(|m| m.triangle_count() == ids.len())
        {
            self.install_face_groups(groups, ids);
            self.status = summary;
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::app::App;
    use crate::geom::guided_fill::fixtures::{cut_hole, lattice_box};
    use glam::Vec3;
    use std::path::PathBuf;

    /// Polls worker results until `done` holds (or fails after ~10 s).
    fn pump_until(app: &mut App, mut done: impl FnMut(&App) -> bool) {
        let ctx = egui::Context::default();
        for _ in 0..2000 {
            app.handle_worker(&ctx);
            if done(app) {
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        panic!("timed out waiting for the worker");
    }

    /// Box with a hole across one of its edges, face groups and holes
    /// detected.
    fn app_with_edge_hole(detect_groups: bool) -> App {
        let mesh = cut_hole(&lattice_box(10), Vec3::new(10.0, 5.0, 10.0), 2.6);
        let mut app = App::new();
        app.install_loaded_mesh(PathBuf::from("box.stl"), mesh);
        if detect_groups {
            app.request_face_groups();
        }
        app.refresh_repair();
        app.select_hole(Some(0));
        app
    }

    #[test]
    fn guided_preview_and_fill_are_undoable_and_keep_the_groups() {
        let mut app = app_with_edge_hole(true);
        assert_eq!(app.repair_holes.len(), 1);
        assert!(app.face_groups_valid());
        app.sync_guided_preview();
        let res = app.guided_preview_result().unwrap().as_ref().unwrap();
        assert!(res.report.guided, "{}", res.report.note);
        assert_eq!(res.report.creases, 1);
        let coarse = res.patch.new_positions.len();
        // Settings changes rebuild the cached preview.
        app.guided_density = 2.0;
        app.sync_guided_preview();
        let res = app.guided_preview_result().unwrap().as_ref().unwrap();
        assert!(res.patch.new_positions.len() > coarse);

        app.fill_selected_hole_guided();
        assert_eq!(app.undo.len(), 1);
        assert!(app.repair_holes.is_empty());
        assert!(app.repair_health.as_ref().unwrap().is_watertight);
        assert!(app.face_groups_valid(), "the groups follow the fill");
        assert!(app.status.contains("Rebuilt"), "{}", app.status);
        assert!(
            app.guided_preview_result().is_none(),
            "the old preview is stale"
        );

        app.undo();
        app.refresh_repair();
        assert_eq!(app.repair_holes.len(), 1);
    }

    #[test]
    fn guided_fill_all_runs_on_the_worker_for_large_meshes() {
        let mut app = app_with_edge_hole(true);
        app.async_min_tris = 0;
        app.request_guided_fill_all();
        assert!(app.edit_job.is_some());
        assert!(app.is_editing());
        pump_until(&mut app, |a| a.edit_job.is_none());
        assert_eq!(app.undo.len(), 1);
        assert!(app.status.contains("1 rebuilt"), "{}", app.status);
        assert!(app.face_groups_valid());
        pump_until(&mut app, |a| a.analysis_job.is_none());
        assert!(app.repair_holes.is_empty());
    }

    #[test]
    fn guided_fill_without_face_groups_uses_the_standard_fill() {
        let mut app = app_with_edge_hole(false);
        app.sync_guided_preview();
        let res = app.guided_preview_result().unwrap().as_ref().unwrap();
        assert!(!res.report.guided);
        assert!(
            res.report.note.contains("face groups"),
            "{}",
            res.report.note
        );
        app.fill_selected_hole_guided();
        assert!(app.status.contains("standard fill"), "{}", app.status);
        assert!(app.repair_holes.is_empty());
    }
}
