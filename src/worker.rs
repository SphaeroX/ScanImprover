//! Background job execution.
//!
//! Long running work (decimation, symmetry search, spatial indices, file
//! loading, repair operations, …) runs on a small pool of worker threads so
//! the UI thread never blocks. Every submitted job is tracked as an
//! *activity* with a label, start time and optional progress so the UI can
//! show what is going on.

use crate::decimate;
use crate::geom::bvh::Bvh;
use crate::geom::distance::{self, Deviation};
use crate::geom::freeform::{FreeformFitData, FreeformParams, fit_freeform};
use crate::geom::hole_detect::HoleLoop;
use crate::geom::hole_solver::HoleSolveResult;
use crate::geom::repair::MeshHealthReport;
use crate::geom::segment::FaceGroup;
use crate::geom::symmetry::{self, SymPlane};
use crate::geom::topology::MeshTopology;
use crate::mesh::Mesh;
use glam::Vec3;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::Instant;

/// Number of worker threads. Heavy algorithms parallelise internally with
/// rayon; a second thread only keeps quick jobs from queueing behind a long one.
const WORKER_THREADS: usize = 2;

/// Shared progress state of a running job.
#[derive(Default)]
pub struct Progress {
    /// Fraction done in 1/1000 units; `u32::MAX` = unknown.
    fraction: AtomicU32,
    stage: Mutex<String>,
}

impl Progress {
    pub fn new() -> Arc<Progress> {
        Arc::new(Progress {
            fraction: AtomicU32::new(u32::MAX),
            stage: Mutex::new(String::new()),
        })
    }

    /// Reports progress. `fraction` in 0..=1, or `None` for indeterminate.
    pub fn set(&self, fraction: Option<f32>, stage: &str) {
        let v = match fraction {
            Some(f) => (f.clamp(0.0, 1.0) * 1000.0) as u32,
            None => u32::MAX,
        };
        self.fraction.store(v, Ordering::Relaxed);
        if let Ok(mut s) = self.stage.lock()
            && s.as_str() != stage
        {
            *s = stage.to_string();
        }
    }

    pub fn fraction(&self) -> Option<f32> {
        match self.fraction.load(Ordering::Relaxed) {
            u32::MAX => None,
            v => Some(v as f32 / 1000.0),
        }
    }

    pub fn stage(&self) -> String {
        self.stage.lock().map(|s| s.clone()).unwrap_or_default()
    }
}

/// A running job as shown in the UI activity feed.
#[derive(Clone)]
pub struct Activity {
    pub id: u64,
    pub label: String,
    pub started: Instant,
    pub progress: Arc<Progress>,
}

impl Activity {
    pub fn elapsed_secs(&self) -> f32 {
        self.started.elapsed().as_secs_f32()
    }
}

#[derive(Clone, Copy, Debug)]
pub enum SymmetryJobKind {
    Auto,
    FromLine { a: Vec3, b: Vec3 },
    Refine(SymPlane),
}

/// Result of a generic [`Job::Task`].
pub enum TaskOutput {
    Loaded {
        path: PathBuf,
        mesh: Result<Mesh, String>,
    },
    FaceGroups {
        groups: Vec<FaceGroup>,
        ids: Vec<i32>,
    },
    RepairAnalysis {
        holes: Vec<HoleLoop>,
        health: MeshHealthReport,
    },
    MeshEdit {
        mesh: Result<Mesh, String>,
        summary: String,
    },
    HoleSolve {
        result: Result<HoleSolveResult, String>,
    },
    Exported {
        result: Result<String, String>,
    },
    Solid {
        result: Result<crate::geom::solid::Solid, String>,
    },
}

pub type TaskFn = Box<dyn FnOnce(&Progress) -> TaskOutput + Send + 'static>;

pub enum Job {
    Decimate {
        id: u64,
        mesh: Arc<Mesh>,
        ratio: f32,
        error_abs: f32,
        lock_border: bool,
    },
    DecimateAuto {
        id: u64,
        mesh: Arc<Mesh>,
        target_dev: f32,
        lock_border: bool,
        progress: Arc<Progress>,
    },
    Symmetry {
        id: u64,
        mesh: Arc<Mesh>,
        kind: SymmetryJobKind,
        mask: Option<Arc<Vec<u8>>>,
        exclude_holes: bool,
    },
    Deviation {
        id: u64,
        source: Arc<Mesh>,
        target: Arc<Mesh>,
    },
    BvhBuild {
        id: u64,
        mesh: Arc<Mesh>,
    },
    TopologyBuild {
        id: u64,
        mesh: Arc<Mesh>,
    },
    FreeformFit {
        id: u64,
        freeform_id: u64,
        points: Arc<Vec<[f32; 3]>>,
        params: FreeformParams,
    },
    Task {
        id: u64,
        progress: Arc<Progress>,
        run: TaskFn,
    },
}

pub enum JobResult {
    Decimated {
        id: u64,
        mesh: Arc<Mesh>,
        error: f32,
    },
    DecimatedAuto {
        id: u64,
        mesh: Arc<Mesh>,
        error: f32,
        dev: Deviation,
        heat: Arc<Vec<f32>>,
        iterations: u32,
    },
    Failed {
        id: u64,
        message: String,
    },
    Symmetry {
        id: u64,
        plane: Option<(SymPlane, f64)>,
    },
    Deviation {
        id: u64,
        dev: Deviation,
        heat: Arc<Vec<f32>>,
    },
    BvhReady {
        id: u64,
        bvh: Arc<Bvh>,
    },
    TopologyReady {
        id: u64,
        topology: Arc<MeshTopology>,
    },
    FreeformFit {
        id: u64,
        freeform_id: u64,
        data: Result<FreeformFitData, String>,
    },
    Task {
        id: u64,
        output: TaskOutput,
    },
}

impl JobResult {
    fn id(&self) -> u64 {
        match self {
            JobResult::Decimated { id, .. }
            | JobResult::DecimatedAuto { id, .. }
            | JobResult::Failed { id, .. }
            | JobResult::Symmetry { id, .. }
            | JobResult::Deviation { id, .. }
            | JobResult::BvhReady { id, .. }
            | JobResult::TopologyReady { id, .. }
            | JobResult::FreeformFit { id, .. }
            | JobResult::Task { id, .. } => *id,
        }
    }
}

pub struct Worker {
    tx: mpsc::Sender<Job>,
    rx: Mutex<mpsc::Receiver<JobResult>>,
    next_id: u64,
    active: Vec<Activity>,
    /// UI context to wake up when a result is ready (set once the UI exists).
    wake: Arc<Mutex<Option<egui::Context>>>,
}

impl Default for Worker {
    fn default() -> Self {
        Self::new()
    }
}

impl Worker {
    pub fn new() -> Worker {
        let (tx, rx_job) = mpsc::channel::<Job>();
        let (tx_res, rx_res) = mpsc::channel::<JobResult>();
        let rx_job = Arc::new(Mutex::new(rx_job));
        let wake: Arc<Mutex<Option<egui::Context>>> = Arc::new(Mutex::new(None));
        for n in 0..WORKER_THREADS {
            let rx_job = rx_job.clone();
            let tx_res = tx_res.clone();
            let wake = wake.clone();
            thread::Builder::new()
                .name(format!("scanimprover-worker-{n}"))
                .spawn(move || {
                    loop {
                        let job = match rx_job.lock() {
                            Ok(rx) => rx.recv(),
                            Err(_) => break,
                        };
                        let Ok(job) = job else { break };
                        let id = job_id(&job);
                        let res =
                            match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                run(job)
                            })) {
                                Ok(res) => res,
                                Err(_) => JobResult::Failed {
                                    id,
                                    message: "Operation failed unexpectedly.".to_string(),
                                },
                            };
                        if tx_res.send(res).is_err() {
                            break;
                        }
                        // Repaint immediately so the result is picked up
                        // without waiting for the next input event.
                        if let Ok(guard) = wake.lock()
                            && let Some(ctx) = guard.as_ref()
                        {
                            ctx.request_repaint();
                        }
                    }
                })
                .expect("failed to spawn worker thread");
        }
        Worker {
            tx,
            rx: Mutex::new(rx_res),
            next_id: 1,
            active: Vec::new(),
            wake,
        }
    }

    /// Registers the UI context that finished jobs should wake up.
    pub fn set_wake_context(&self, ctx: &egui::Context) {
        if let Ok(mut guard) = self.wake.lock()
            && guard.is_none()
        {
            *guard = Some(ctx.clone());
        }
    }

    fn submit(
        &mut self,
        label: impl Into<String>,
        progress: Arc<Progress>,
        make: impl FnOnce(u64) -> Job,
    ) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        self.active.push(Activity {
            id,
            label: label.into(),
            started: Instant::now(),
            progress,
        });
        let _ = self.tx.send(make(id));
        id
    }

    /// Currently running / queued jobs, oldest first.
    pub fn activities(&self) -> &[Activity] {
        &self.active
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn is_idle(&self) -> bool {
        self.active.is_empty()
    }

    pub fn submit_decimate(
        &mut self,
        mesh: Arc<Mesh>,
        ratio: f32,
        error_abs: f32,
        lock_border: bool,
    ) -> u64 {
        self.submit("Simplifying preview", Progress::new(), |id| Job::Decimate {
            id,
            mesh,
            ratio,
            error_abs,
            lock_border,
        })
    }

    pub fn submit_decimate_auto(
        &mut self,
        mesh: Arc<Mesh>,
        target_dev: f32,
        lock_border: bool,
    ) -> u64 {
        let progress = Progress::new();
        self.submit("Auto decimation", progress.clone(), |id| {
            Job::DecimateAuto {
                id,
                mesh,
                target_dev,
                lock_border,
                progress,
            }
        })
    }

    pub fn submit_symmetry(
        &mut self,
        mesh: Arc<Mesh>,
        kind: SymmetryJobKind,
        mask: Option<Arc<Vec<u8>>>,
        exclude_holes: bool,
    ) -> u64 {
        self.submit("Symmetry plane", Progress::new(), |id| Job::Symmetry {
            id,
            mesh,
            kind,
            mask,
            exclude_holes,
        })
    }

    pub fn submit_deviation(&mut self, source: Arc<Mesh>, target: Arc<Mesh>) -> u64 {
        self.submit("Measuring deviation", Progress::new(), |id| {
            Job::Deviation { id, source, target }
        })
    }

    pub fn submit_bvh_build(&mut self, mesh: Arc<Mesh>) -> u64 {
        self.submit("Spatial index", Progress::new(), |id| Job::BvhBuild {
            id,
            mesh,
        })
    }

    pub fn submit_topology_build(&mut self, mesh: Arc<Mesh>) -> u64 {
        self.submit("Mesh topology", Progress::new(), |id| Job::TopologyBuild {
            id,
            mesh,
        })
    }

    pub fn submit_freeform_fit(
        &mut self,
        freeform_id: u64,
        points: Arc<Vec<[f32; 3]>>,
        params: FreeformParams,
    ) -> u64 {
        self.submit("Freeform fit", Progress::new(), |id| Job::FreeformFit {
            id,
            freeform_id,
            points,
            params,
        })
    }

    /// Submits an arbitrary task with a label for the activity feed.
    pub fn submit_task(
        &mut self,
        label: impl Into<String>,
        run: impl FnOnce(&Progress) -> TaskOutput + Send + 'static,
    ) -> u64 {
        let progress = Progress::new();
        self.submit(label, progress.clone(), |id| Job::Task {
            id,
            progress,
            run: Box::new(run),
        })
    }

    pub fn poll(&mut self) -> Vec<JobResult> {
        let mut out = Vec::new();
        let Ok(rx) = self.rx.lock() else {
            return out;
        };
        while let Ok(res) = rx.try_recv() {
            let id = res.id();
            self.active.retain(|a| a.id != id);
            out.push(res);
        }
        out
    }
}

fn job_id(job: &Job) -> u64 {
    match job {
        Job::Decimate { id, .. }
        | Job::DecimateAuto { id, .. }
        | Job::Symmetry { id, .. }
        | Job::Deviation { id, .. }
        | Job::BvhBuild { id, .. }
        | Job::TopologyBuild { id, .. }
        | Job::FreeformFit { id, .. }
        | Job::Task { id, .. } => *id,
    }
}

type AutoCandidate = (Mesh, f32, Deviation, Vec<f32>);

fn measure(mesh: &Mesh, simplified: &Mesh) -> (Deviation, Vec<f32>) {
    let bvh = Bvh::new(&simplified.positions, &simplified.indices);
    let dev = distance::deviation(&mesh.positions, &bvh);
    let per_tri = distance::per_triangle_max(&mesh.positions, &bvh, simplified.triangle_count());
    let heat = distance::per_vertex_max(&per_tri, &simplified.indices, simplified.positions.len());
    (dev, heat)
}

/// Iteratively searches a simplification error tolerance whose measured
/// max deviation stays below `target_dev` while removing as many triangles
/// as possible.
#[cfg_attr(not(test), allow(dead_code))]
pub fn auto_decimate(
    mesh: &Mesh,
    target_dev: f32,
    lock_border: bool,
) -> Result<(Mesh, f32, Deviation, Vec<f32>, u32), String> {
    auto_decimate_with_progress(mesh, target_dev, lock_border, &Progress::default())
}

const AUTO_DECIMATE_PASSES: u32 = 5;

pub fn auto_decimate_with_progress(
    mesh: &Mesh,
    target_dev: f32,
    lock_border: bool,
    progress: &Progress,
) -> Result<(Mesh, f32, Deviation, Vec<f32>, u32), String> {
    let mut t = (target_dev * 0.5).max(1e-6);
    let mut iterations = 0u32;
    let mut best_ok: Option<AutoCandidate> = None;
    let mut best_bad: Option<AutoCandidate> = None;
    for i in 0..AUTO_DECIMATE_PASSES {
        iterations = i + 1;
        progress.set(
            Some(i as f32 / AUTO_DECIMATE_PASSES as f32),
            &format!("pass {} of {}", i + 1, AUTO_DECIMATE_PASSES),
        );
        let (m, est) = decimate::decimate(mesh, 0.0001, t, lock_border)?;
        let (dev, heat) = measure(mesh, &m);
        if dev.max_dev <= target_dev {
            let keep = match &best_ok {
                Some((pm, _, _, _)) => m.triangle_count() < pm.triangle_count(),
                None => true,
            };
            if keep {
                best_ok = Some((m, est, dev, heat));
            }
            let tight = best_ok
                .as_ref()
                .map(|(_, _, d, _)| d.max_dev >= 0.7 * target_dev)
                .unwrap_or(false);
            if tight || i == AUTO_DECIMATE_PASSES - 1 {
                break;
            }
            t *= 1.35;
        } else {
            let keep = match &best_bad {
                Some((_, _, d, _)) => dev.max_dev < d.max_dev,
                None => true,
            };
            if keep {
                best_bad = Some((m, est, dev, heat));
            }
            t *= (target_dev / dev.max_dev).clamp(0.2, 0.9);
        }
    }
    let (m, est, dev, heat) = best_ok
        .or(best_bad)
        .ok_or_else(|| "Auto decimation failed".to_string())?;
    Ok((m, est, dev, heat, iterations))
}

fn run(job: Job) -> JobResult {
    match job {
        Job::Decimate {
            id,
            mesh,
            ratio,
            error_abs,
            lock_border,
        } => match decimate::decimate(&mesh, ratio, error_abs, lock_border) {
            Ok((m, e)) => JobResult::Decimated {
                id,
                mesh: Arc::new(m),
                error: e,
            },
            Err(message) => JobResult::Failed { id, message },
        },
        Job::DecimateAuto {
            id,
            mesh,
            target_dev,
            lock_border,
            progress,
        } => match auto_decimate_with_progress(&mesh, target_dev, lock_border, &progress) {
            Ok((m, est, dev, heat, iterations)) => JobResult::DecimatedAuto {
                id,
                mesh: Arc::new(m),
                error: est,
                dev,
                heat: Arc::new(heat),
                iterations,
            },
            Err(message) => JobResult::Failed { id, message },
        },
        Job::Symmetry {
            id,
            mesh,
            kind,
            mask,
            exclude_holes,
        } => {
            let bvh = Bvh::new(&mesh.positions, &mesh.indices);
            let effective_mask: Option<Vec<u8>> = match (mask, exclude_holes) {
                (Some(user), true) => {
                    let hole_mask = crate::geom::boundary::generate_hole_mask(&mesh, 2);
                    let mut combined = (*user).clone();
                    for (c, h) in combined.iter_mut().zip(hole_mask.iter()) {
                        if *h > 0 {
                            *c = 1;
                        }
                    }
                    Some(combined)
                }
                (Some(user), false) => Some((*user).clone()),
                (None, true) => Some(crate::geom::boundary::generate_hole_mask(&mesh, 2)),
                (None, false) => None,
            };
            let mask_slice = effective_mask.as_deref();
            let res = match kind {
                SymmetryJobKind::FromLine { a, b } => {
                    symmetry::detect_symmetry_from_line(&mesh, &bvh, a, b, mask_slice)
                }
                SymmetryJobKind::Refine(p) => Some(symmetry::refine_symmetry_masked(
                    &mesh, &bvh, &p, mask_slice,
                )),
                SymmetryJobKind::Auto => symmetry::detect_symmetry_masked(&mesh, &bvh, mask_slice),
            };
            JobResult::Symmetry { id, plane: res }
        }
        Job::Deviation { id, source, target } => {
            let (dev, heat) = measure(&source, &target);
            JobResult::Deviation {
                id,
                dev,
                heat: Arc::new(heat),
            }
        }
        Job::BvhBuild { id, mesh } => {
            let bvh = Arc::new(Bvh::new(&mesh.positions, &mesh.indices));
            JobResult::BvhReady { id, bvh }
        }
        Job::TopologyBuild { id, mesh } => {
            let topology = Arc::new(MeshTopology::build(&mesh));
            JobResult::TopologyReady { id, topology }
        }
        Job::FreeformFit {
            id,
            freeform_id,
            points,
            params,
        } => JobResult::FreeformFit {
            id,
            freeform_id,
            data: fit_freeform(&points, &params),
        },
        Job::Task { id, progress, run } => JobResult::Task {
            id,
            output: run(&progress),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tasks_report_activity_until_polled() {
        let mut w = Worker::new();
        assert!(w.is_idle());
        let id = w.submit_task("test", |p| {
            p.set(Some(0.5), "half");
            TaskOutput::Exported {
                result: Ok("done".to_string()),
            }
        });
        assert_eq!(w.activities().len(), 1);
        assert_eq!(w.activities()[0].label, "test");
        let mut got = None;
        for _ in 0..200 {
            for r in w.poll() {
                if let JobResult::Task { id: rid, output } = r {
                    assert_eq!(rid, id);
                    got = Some(output);
                }
            }
            if got.is_some() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(matches!(got, Some(TaskOutput::Exported { .. })));
        assert!(w.is_idle());
    }

    #[test]
    fn panicking_job_reports_failure() {
        let mut w = Worker::new();
        let id = w.submit_task("boom", |_| panic!("kaboom"));
        let mut failed = false;
        for _ in 0..200 {
            for r in w.poll() {
                if let JobResult::Failed { id: rid, .. } = r {
                    assert_eq!(rid, id);
                    failed = true;
                }
            }
            if failed {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(failed);
    }
}
