//! Undo / redo history.
//!
//! A snapshot captures the document state that user actions mutate: the
//! meshes, the selection, fitted features and hidden regions. Derived caches
//! (spatial index, topology, face groups, hole list) are rebuilt lazily
//! after a restore.

use super::{App, HISTORY_LIMIT, HiddenRegion, SymState};
use crate::geom::alignment::AlignmentSlots;
use crate::geom::fitting::{CircleFit, FittedCircle, FittedPlane, PlaneFit};
use crate::geom::freeform::FittedFreeform;
use crate::mesh::Mesh;
use std::sync::Arc;

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
    pub(crate) align_slots: AlignmentSlots,
}

impl App {
    /// Captures the undoable document state, or `None` when no mesh is loaded.
    fn capture_snapshot(&self) -> Option<Snapshot> {
        let (current, original) = (self.current.as_ref()?, self.original.as_ref()?);
        Some(Snapshot {
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
            align_slots: self.align_slots.clone(),
        })
    }

    /// Restores a snapshot and invalidates every derived cache.
    fn restore_snapshot(&mut self, s: Snapshot) {
        self.current = Some(s.current);
        self.original = Some(s.original);
        self.preview = None;
        self.sel = s.sel;
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
        self.align_slots = s.align_slots;
        self.freeform_job = None;
        self.deviation = None;
        self.heat = None;
        self.invalidate_face_groups();
        self.repair_holes.clear();
        self.repair_selected_hole = None;
        self.repair_preview_patch = None;
        self.repair_health = None;
        self.bridge_preview_patch = None;
        self.bridge_status = None;
        self.mark_mesh_changed();
        self.update_hidden_mask();
        self.recount_sel();
        self.mark_selection_changed();
        self.sync_bbox();
    }

    /// Pushes the current state onto the undo stack (and clears redo).
    pub(crate) fn push_snapshot(&mut self) {
        if let Some(snapshot) = self.capture_snapshot() {
            self.undo.push(snapshot);
            if self.undo.len() > HISTORY_LIMIT {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
    }

    pub(crate) fn undo(&mut self) {
        let Some(s) = self.undo.pop() else {
            return;
        };
        if let Some(now) = self.capture_snapshot() {
            self.redo.push(now);
            if self.redo.len() > HISTORY_LIMIT {
                self.redo.remove(0);
            }
        }
        self.restore_snapshot(s);
        self.status = format!("Undone. ({} remaining)", self.undo.len());
    }

    pub(crate) fn redo(&mut self) {
        let Some(s) = self.redo.pop() else {
            return;
        };
        if let Some(now) = self.capture_snapshot() {
            self.undo.push(now);
            if self.undo.len() > HISTORY_LIMIT {
                self.undo.remove(0);
            }
        }
        self.restore_snapshot(s);
        self.status = format!("Redone. ({} remaining)", self.redo.len());
    }
}
