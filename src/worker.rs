use crate::decimate;
use crate::geom::bvh::Bvh;
use crate::geom::distance::{self, Deviation};
use crate::geom::symmetry::{self, SymPlane};
use crate::mesh::Mesh;
use glam::Vec3;
use std::sync::{Arc, mpsc};
use std::thread;

#[derive(Clone, Copy, Debug)]
pub enum SymmetryJobKind {
    Auto,
    FromLine { a: Vec3, b: Vec3 },
    Refine(SymPlane),
}

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
}

pub struct Worker {
    tx: mpsc::Sender<Job>,
    rx: mpsc::Receiver<JobResult>,
    next_id: u64,
}

impl Worker {
    pub fn new() -> Worker {
        let (tx, rx_job) = mpsc::channel::<Job>();
        let (tx_res, rx_res) = mpsc::channel::<JobResult>();
        thread::Builder::new()
            .name("scanimprover-worker".into())
            .spawn(move || {
                for job in rx_job {
                    let res = run(job);
                    if tx_res.send(res).is_err() {
                        break;
                    }
                }
            })
            .expect("failed to spawn worker thread");
        Worker {
            tx,
            rx: rx_res,
            next_id: 1,
        }
    }

    fn alloc_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    pub fn submit_decimate(
        &mut self,
        mesh: Arc<Mesh>,
        ratio: f32,
        error_abs: f32,
        lock_border: bool,
    ) -> u64 {
        let id = self.alloc_id();
        let _ = self.tx.send(Job::Decimate {
            id,
            mesh,
            ratio,
            error_abs,
            lock_border,
        });
        id
    }

    pub fn submit_decimate_auto(
        &mut self,
        mesh: Arc<Mesh>,
        target_dev: f32,
        lock_border: bool,
    ) -> u64 {
        let id = self.alloc_id();
        let _ = self.tx.send(Job::DecimateAuto {
            id,
            mesh,
            target_dev,
            lock_border,
        });
        id
    }

    pub fn submit_symmetry(
        &mut self,
        mesh: Arc<Mesh>,
        kind: SymmetryJobKind,
        mask: Option<Arc<Vec<u8>>>,
        exclude_holes: bool,
    ) -> u64 {
        let id = self.alloc_id();
        let _ = self.tx.send(Job::Symmetry {
            id,
            mesh,
            kind,
            mask,
            exclude_holes,
        });
        id
    }

    pub fn submit_deviation(&mut self, source: Arc<Mesh>, target: Arc<Mesh>) -> u64 {
        let id = self.alloc_id();
        let _ = self.tx.send(Job::Deviation { id, source, target });
        id
    }

    pub fn submit_bvh_build(&mut self, mesh: Arc<Mesh>) -> u64 {
        let id = self.alloc_id();
        let _ = self.tx.send(Job::BvhBuild { id, mesh });
        id
    }

    pub fn poll(&mut self) -> Vec<JobResult> {
        let mut out = Vec::new();
        while let Ok(res) = self.rx.try_recv() {
            out.push(res);
        }
        out
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

pub fn auto_decimate(
    mesh: &Mesh,
    target_dev: f32,
    lock_border: bool,
) -> Result<(Mesh, f32, Deviation, Vec<f32>, u32), String> {
    let mut t = (target_dev * 0.5).max(1e-6);
    let mut iterations = 0u32;
    let mut best_ok: Option<AutoCandidate> = None;
    let mut best_bad: Option<AutoCandidate> = None;
    for i in 0..5 {
        iterations = i + 1;
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
            if tight || i == 4 {
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
        } => match auto_decimate(&mesh, target_dev, lock_border) {
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
                (None, true) => {
                    let hole_mask = crate::geom::boundary::generate_hole_mask(&mesh, 2);
                    Some(hole_mask)
                }
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
    }
}
