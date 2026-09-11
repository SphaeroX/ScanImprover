//! Experimental quad retopology of the working mesh (see
//! [`crate::geom::retopo`]): request, progress and panel readouts.

use super::App;
use crate::geom::retopo::{RetopoParams, quad_retopology};
use crate::mesh::Mesh;
use crate::worker::{Progress, TaskOutput};

/// Meshes below this many triangles are retopologized synchronously
/// (capped by `async_min_tris`, which tests lower to force the worker).
/// Lower than the general threshold: even small inputs take a moment.
const RETOPO_ASYNC_MIN_TRIS: usize = 20_000;

impl App {
    /// Rebuilds the working mesh with the quad retopology, on the worker for
    /// all but tiny meshes. The result replaces the mesh as one undoable
    /// step (it arrives as a regular mesh edit, see `handle_task_output`).
    pub(crate) fn request_quad_retopology(&mut self) {
        if self.is_editing() {
            self.status = "Please wait for the running operation to finish.".to_string();
            return;
        }
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        let Some(curr) = self.current.clone() else {
            return;
        };
        let params = self.retopo;
        if curr.triangle_count() < RETOPO_ASYNC_MIN_TRIS.min(self.async_min_tris) {
            if let TaskOutput::MeshEdit { mesh, summary } =
                run_quad_retopology(&curr, &params, &Progress::default())
            {
                match mesh {
                    Ok(mesh) => {
                        self.push_snapshot();
                        self.set_mesh_modified(mesh, summary);
                    }
                    Err(e) => self.status = e,
                }
            }
            return;
        }
        self.status = "Quad retopology…".to_string();
        let generation = self.mesh_generation;
        let id = self.worker.submit_task("Quad retopology", move |p| {
            run_quad_retopology(&curr, &params, p)
        });
        self.edit_job = Some((id, generation));
        self.retopo_job = Some(id);
    }

    /// (fraction, stage) of the running retopology, `None` when idle.
    pub(crate) fn retopo_progress(&self) -> Option<(Option<f32>, String)> {
        let id = self.retopo_job?;
        if self.edit_job.map(|(j, _)| j) != Some(id) {
            return None;
        }
        Some(
            self.worker
                .activities()
                .iter()
                .find(|a| a.id == id)
                .map(|a| (a.progress.fraction(), a.progress.stage()))
                .unwrap_or((None, String::new())),
        )
    }

    /// Surface area of the working mesh, cached per mesh generation.
    pub(crate) fn retopo_surface_area(&mut self) -> f32 {
        if let Some((generation, area)) = self.retopo_area
            && generation == self.mesh_generation
        {
            return area;
        }
        let area = self
            .current
            .as_ref()
            .map(|m| m.surface_area())
            .unwrap_or(0.0);
        self.retopo_area = Some((self.mesh_generation, area));
        area
    }
}

fn run_quad_retopology(mesh: &Mesh, params: &RetopoParams, progress: &Progress) -> TaskOutput {
    let started = std::time::Instant::now();
    match quad_retopology(mesh, params, &|f, stage| progress.set(Some(f), stage)) {
        Ok(out) => {
            let s = out.stats;
            TaskOutput::MeshEdit {
                mesh: Ok(out.mesh),
                summary: format!(
                    "Quad retopology: {} quads, {} triangles, {} vertices ({} irregular), \
                     edge ≈ {:.3} mm, {:.1} s.",
                    s.quads,
                    s.triangles,
                    s.vertices,
                    s.irregular_vertices,
                    s.edge_length,
                    started.elapsed().as_secs_f32()
                ),
            }
        }
        Err(e) => TaskOutput::MeshEdit {
            mesh: Err(e),
            summary: String::new(),
        },
    }
}

#[cfg(test)]
mod tests {
    use crate::app::App;
    use crate::mesh::Mesh;
    use std::path::PathBuf;

    /// Cube of edge length 20 with every face an m x m grid.
    fn grid_box(m: usize) -> Mesh {
        let coord = |i: usize| -10.0 + 20.0 * i as f32 / m as f32;
        let mut corners: Vec<[f32; 3]> = Vec::new();
        for axis in 0..3 {
            for sign in [-1.0f32, 1.0] {
                let point = |i: usize, j: usize| {
                    let mut p = [0.0f32; 3];
                    p[axis] = sign * 10.0;
                    p[(axis + 1) % 3] = coord(i);
                    p[(axis + 2) % 3] = coord(j);
                    p
                };
                for j in 0..m {
                    for i in 0..m {
                        let (a, b, c, d) = (
                            point(i, j),
                            point(i + 1, j),
                            point(i + 1, j + 1),
                            point(i, j + 1),
                        );
                        if sign > 0.0 {
                            corners.extend_from_slice(&[a, b, c, a, c, d]);
                        } else {
                            corners.extend_from_slice(&[a, c, b, a, d, c]);
                        }
                    }
                }
            }
        }
        Mesh::from_corners(&corners)
    }

    fn app_with(mesh: Mesh) -> App {
        let mut app = App::new();
        app.install_loaded_mesh(PathBuf::from("box.stl"), mesh);
        app.retopo.target_faces = 300;
        app
    }

    #[test]
    fn small_mesh_retopology_is_synchronous_and_undoable() {
        let mut app = app_with(grid_box(16));
        let before = app.current.as_ref().unwrap().triangle_count();
        app.request_quad_retopology();
        assert!(app.edit_job.is_none(), "small meshes run synchronously");
        assert_eq!(app.undo.len(), 1);
        let m = app.current.as_ref().unwrap();
        assert!(m.quad_count() > 200, "{}", app.status);
        assert_eq!(m.triangle_count(), 2 * m.quad_count(), "pure quads");
        assert!(app.status.starts_with("Quad retopology"), "{}", app.status);
        assert_eq!(app.sel.len(), m.triangle_count());
        app.undo();
        assert_eq!(app.current.as_ref().unwrap().triangle_count(), before);
        assert_eq!(app.current.as_ref().unwrap().quad_count(), 0);
    }

    #[test]
    fn large_mesh_retopology_runs_on_the_worker() {
        let mut app = app_with(grid_box(16));
        app.async_min_tris = 0;
        app.request_quad_retopology();
        assert!(app.edit_job.is_some());
        assert!(app.is_editing());
        assert!(app.retopo_progress().is_some());
        // A second request is refused while the first one runs.
        app.request_quad_retopology();
        assert!(app.status.contains("wait"), "{}", app.status);
        let ctx = egui::Context::default();
        for _ in 0..2000 {
            app.handle_worker(&ctx);
            if app.edit_job.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(app.edit_job.is_none(), "worker finished");
        assert!(app.retopo_progress().is_none());
        assert_eq!(app.undo.len(), 1);
        assert!(
            app.current.as_ref().unwrap().quad_count() > 200,
            "{}",
            app.status
        );
    }

    #[test]
    fn area_readout_follows_the_mesh() {
        let mut app = app_with(grid_box(4));
        let a = app.retopo_surface_area();
        assert!((a - 2400.0).abs() < 1.0, "{a}");
        app.apply_transform(glam::Quat::IDENTITY, glam::Vec3::X);
        assert!((app.retopo_surface_area() - 2400.0).abs() < 1.0);
    }
}
