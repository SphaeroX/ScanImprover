//! Experimental solid reconstruction: document state, worker dispatch,
//! STEP export and the viewport preview of the reconstructed B-Rep.

use super::App;
use super::overlay::{Line, marker_lines, push_line};
use crate::geom::solid::{Solid, SolidParams, reconstruct_solid};
use crate::ui::ToolSection;
use crate::worker::TaskOutput;
use std::sync::Arc;

/// Preview colors of the edges by curve type (line, circle, ellipse,
/// B-spline); the panel legend uses the same colors.
pub(crate) const SOLID_EDGE_COLORS: [[f32; 4]; 4] = [
    [1.0, 0.82, 0.25, 1.0],
    [0.25, 0.85, 1.0, 1.0],
    [0.45, 1.0, 0.5, 1.0],
    [1.0, 0.45, 0.85, 1.0],
];

/// Solid reconstruction state of the document.
pub(crate) struct SolidState {
    pub(crate) params: SolidParams,
    /// Last result: the solid, or the reason it was refused.
    pub(crate) result: Option<Result<Arc<Solid>, String>>,
    /// (mesh generation, groups generation) the result was built from.
    pub(crate) built_for: (u64, u64),
    /// (job id, generations) of the reconstruction running on the worker.
    pub(crate) job: Option<(u64, (u64, u64))>,
    /// Draw the B-Rep edges and vertices in the viewport.
    pub(crate) show_edges: bool,
}

impl Default for SolidState {
    fn default() -> Self {
        SolidState {
            params: SolidParams::default(),
            result: None,
            built_for: (0, 0),
            job: None,
            show_edges: true,
        }
    }
}

impl App {
    fn solid_key(&self) -> (u64, u64) {
        (self.mesh_generation, self.groups_generation)
    }

    /// True when the stored result belongs to the current mesh and groups.
    pub(crate) fn solid_is_current(&self) -> bool {
        self.solid.result.is_some() && self.solid.built_for == self.solid_key()
    }

    /// The reconstructed solid, if it is up to date.
    pub(crate) fn current_solid(&self) -> Option<Arc<Solid>> {
        match &self.solid.result {
            Some(Ok(s)) if self.solid_is_current() => Some(s.clone()),
            _ => None,
        }
    }

    /// Reconstructs the solid from the face groups, on the worker for
    /// large meshes.
    pub(crate) fn request_solid_reconstruction(&mut self) {
        if self.solid.job.is_some() {
            self.status = "Solid reconstruction is already running.".to_string();
            return;
        }
        let Some(mesh) = self.display().cloned() else {
            self.status = "Nothing to reconstruct.".to_string();
            return;
        };
        if self.face_groups.is_empty() || self.group_ids.len() != mesh.triangle_count() {
            self.status = "Detect the face groups first.".to_string();
            return;
        }
        let key = self.solid_key();
        let params = self.solid.params;
        if !self.prefers_async() {
            let result = reconstruct_solid(&mesh, &self.face_groups, &self.group_ids, &params);
            self.install_solid(result, key);
            return;
        }
        let groups = self.face_groups.clone();
        let ids = self.group_ids.clone();
        self.status = "Reconstructing solid…".to_string();
        let id = self.worker.submit_task("Solid reconstruction", move |p| {
            p.set(None, "fitting surfaces, intersecting");
            TaskOutput::Solid {
                result: reconstruct_solid(&mesh, &groups, &ids, &params),
            }
        });
        self.solid.job = Some((id, key));
    }

    fn install_solid(&mut self, result: Result<Solid, String>, key: (u64, u64)) {
        self.status = match &result {
            Ok(s) => format!(
                "Solid: {} faces, {} edges, {} vertices.",
                s.faces.len(),
                s.edges.len(),
                s.vertices.len()
            ),
            Err(_) => "Solid reconstruction refused; see the Experimental section.".to_string(),
        };
        self.solid.result = Some(result.map(Arc::new));
        self.solid.built_for = key;
    }

    /// Applies a finished worker reconstruction (unless the document
    /// changed meanwhile).
    pub(crate) fn finish_solid_job(&mut self, id: u64, result: Result<Solid, String>) {
        let Some((jid, key)) = self.solid.job else {
            return;
        };
        if jid != id {
            return;
        }
        self.solid.job = None;
        if key == self.solid_key() {
            self.install_solid(result, key);
        } else {
            self.status = "Solid reconstruction discarded: the mesh or the face groups changed \
                           while it was running."
                .to_string();
        }
    }

    /// Forgets a reconstruction job that failed.
    pub(crate) fn clear_solid_job(&mut self, id: u64) {
        if self.solid.job.map(|j| j.0) == Some(id) {
            self.solid.job = None;
            self.solid.result = Some(Err("Solid reconstruction failed unexpectedly.".to_string()));
            self.solid.built_for = self.solid_key();
        }
    }

    /// Saves the current solid as STEP (file dialog).
    pub(crate) fn export_solid_step(&mut self) {
        let Some(solid) = self.current_solid() else {
            self.status = "Reconstruct the solid first.".to_string();
            return;
        };
        let name = self
            .file_path
            .as_ref()
            .and_then(|p| p.file_stem())
            .and_then(|s| s.to_str())
            .unwrap_or("solid")
            .to_string();
        self.status = match crate::export::export_solid_dialog(&solid, &name) {
            Ok(msg) => msg,
            Err(e) => format!("Export failed: {e}"),
        };
    }

    /// Viewport preview while the Experimental section is open: edges
    /// colored by curve type, vertices as markers.
    pub(crate) fn solid_overlay(&self, depth_lines: &mut Vec<Line>, overlay_lines: &mut Vec<Line>) {
        if !self.solid.show_edges || !self.is_section_open(ToolSection::Experimental) {
            return;
        }
        let Some(solid) = self.current_solid() else {
            return;
        };
        // Edges lie on the mesh surface: lift them slightly towards the eye
        // so the depth test does not hide them.
        let diag = self.bbox.diagonal().max(1e-3);
        let eye = self.camera.eye();
        let lift = diag * 0.002;
        let toward = |p: glam::Vec3| p + (eye - p).normalize_or_zero() * lift;
        for e in &solid.edges {
            let col = SOLID_EDGE_COLORS[e.curve.kind_index()];
            for w in e.preview.windows(2) {
                push_line(depth_lines, toward(w[0]), toward(w[1]), col);
            }
        }
        let size = diag * 0.006;
        for v in &solid.vertices {
            overlay_lines.extend(marker_lines(v.as_vec3(), size, [1.0, 1.0, 1.0, 0.9]));
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::app::App;
    use crate::mesh::Mesh;
    use std::path::PathBuf;

    fn box_mesh() -> Mesh {
        let p = |x: f32, y: f32, z: f32| [x * 10.0, y * 6.0, z * 4.0];
        let positions = vec![
            p(0.0, 0.0, 0.0),
            p(1.0, 0.0, 0.0),
            p(1.0, 1.0, 0.0),
            p(0.0, 1.0, 0.0),
            p(0.0, 0.0, 1.0),
            p(1.0, 0.0, 1.0),
            p(1.0, 1.0, 1.0),
            p(0.0, 1.0, 1.0),
        ];
        let indices = vec![
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6,
            5, 0, 4, 7, 0, 7, 3,
        ];
        Mesh::from_indexed(positions, indices)
    }

    fn app_with(mesh: Mesh) -> App {
        let mut app = App::new();
        app.install_loaded_mesh(PathBuf::from("part.stl"), mesh);
        app
    }

    #[test]
    fn reconstruction_runs_synchronously_and_goes_stale_with_the_groups() {
        let mut app = app_with(box_mesh());
        app.request_solid_reconstruction();
        assert!(app.solid.result.is_none(), "needs face groups first");
        assert!(app.status.contains("face groups"), "{}", app.status);

        app.detect_face_groups();
        app.request_solid_reconstruction();
        assert!(app.solid.job.is_none(), "small meshes run synchronously");
        let solid = app.current_solid().expect("solid");
        assert_eq!(
            (solid.faces.len(), solid.edges.len(), solid.vertices.len()),
            (6, 12, 8)
        );
        assert!(app.status.contains("6 faces"), "{}", app.status);

        app.detect_face_groups();
        assert!(!app.solid_is_current(), "new groups make the result stale");
        assert!(app.current_solid().is_none());
    }

    #[test]
    fn refusal_is_kept_as_the_result() {
        let mut mesh = box_mesh();
        mesh.indices.truncate(mesh.indices.len() - 3);
        let mut app = app_with(mesh);
        app.detect_face_groups();
        app.request_solid_reconstruction();
        match &app.solid.result {
            Some(Err(e)) => assert!(e.contains("not closed"), "{e}"),
            _ => panic!("open mesh must be refused"),
        }
        assert!(app.current_solid().is_none());
    }

    #[test]
    fn large_meshes_reconstruct_on_the_worker() {
        let mut app = app_with(box_mesh());
        app.detect_face_groups();
        app.async_min_tris = 0;
        app.request_solid_reconstruction();
        assert!(app.solid.job.is_some());
        let ctx = egui::Context::default();
        for _ in 0..2000 {
            app.handle_worker(&ctx);
            if app.solid.job.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(app.solid.job.is_none(), "worker finished");
        assert_eq!(app.current_solid().expect("solid").faces.len(), 6);

        // A result for groups that changed meanwhile is discarded.
        app.request_solid_reconstruction();
        app.detect_face_groups();
        for _ in 0..2000 {
            app.handle_worker(&ctx);
            if app.solid.job.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(app.status.contains("discarded"), "{}", app.status);
        assert!(!app.solid_is_current());
    }
}
