//! Mesh repair: hole detection / filling, contour bridges and clean-up tools.

use super::{App, BridgeClusterCache};
use crate::geom::bridge::{
    BridgeConfig, SelectionCluster, apply_bridge_patch, detect_selection_clusters,
    generate_bridge_patch,
};
use crate::geom::hole_detect::detect_holes;
use crate::geom::hole_fill::{HoleFillConfig, apply_patch, generate_hole_patch};
use crate::geom::hole_solver::{
    CircleGuideMode, refine_patch_to_references, solve_best_hole_config,
};
use crate::geom::repair::{
    analyze_mesh, auto_repair_mesh, remove_degenerate_faces, remove_small_components, unify_normals,
};

impl App {
    // ----------------------------------------------------------------------
    // Diagnostics and hole list
    // ----------------------------------------------------------------------

    pub(crate) fn refresh_repair(&mut self) {
        if let Some(m) = self.display().cloned() {
            self.repair_holes = detect_holes(&m);
            self.repair_health = Some(analyze_mesh(&m));
            if let Some(sel) = self.repair_selected_hole
                && sel >= self.repair_holes.len()
            {
                self.repair_selected_hole = None;
            }
            self.update_hole_preview();
        }
    }

    pub(crate) fn set_hole_fill_config(&mut self, config: HoleFillConfig) {
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

    pub(crate) fn focus_selected_hole(&mut self, now: f64) {
        if let Some(hole) = self
            .repair_selected_hole
            .and_then(|idx| self.repair_holes.get(idx))
        {
            let mut bb = hole.bbox;
            // Give small holes some context around them.
            let pad = (bb.diagonal() * 0.5).max(self.bbox.diagonal() * 0.02);
            bb.min -= glam::Vec3::splat(pad);
            bb.max += glam::Vec3::splat(pad);
            self.camera.animate_fit(&bb, now);
        }
    }

    pub(crate) fn set_hole_circle_mode(&mut self, mode: CircleGuideMode) {
        self.repair_circle_mode = mode;
        self.update_hole_preview();
    }

    pub(crate) fn set_hole_refine_to_references(&mut self, refine: bool) {
        self.repair_refine_to_references = refine;
        self.update_hole_preview();
    }

    pub(crate) fn solve_best_hole_fill(&mut self) {
        let refs = self.active_reference_geometries();
        if refs.is_empty() {
            self.status = "No fitted planes or circles available to guide hole fill.".to_string();
            self.repair_solve_status =
                Some("No planes/circles available to guide solver.".to_string());
            return;
        }
        let m_opt = self.display().cloned();
        let hole_opt = self
            .repair_selected_hole
            .and_then(|idx| self.repair_holes.get(idx).cloned())
            .or_else(|| self.repair_holes.first().cloned());
        let (Some(m), Some(hole)) = (m_opt, hole_opt) else {
            self.status = "No holes detected to solve.".to_string();
            self.repair_solve_status = Some("No holes detected".to_string());
            return;
        };
        let result = solve_best_hole_config(&m, &hole, &refs);
        self.apply_hole_solve_result(result);
    }

    /// Stores the outcome of the hole solver and refreshes the preview.
    pub(crate) fn apply_hole_solve_result(
        &mut self,
        result: Result<crate::geom::hole_solver::HoleSolveResult, String>,
    ) {
        match result {
            Ok(result) => {
                self.repair_config = result.best_config;
                self.repair_solve_status = Some(format!(
                    "Solved: {} (bulge: {:.2}, RMS: {:.3} mm)",
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
        let (Some(m), Some(hole)) = (m_opt, hole_opt) else {
            self.repair_preview_patch = None;
            return;
        };
        match generate_hole_patch(&m, &hole, self.repair_config) {
            Ok(mut patch) => {
                if self.repair_refine_to_references {
                    let refs = self.active_reference_geometries();
                    refine_patch_to_references(&mut patch, &m, &hole, &refs);
                }
                self.repair_preview_patch = Some(patch);
            }
            Err(e) => {
                self.status = format!("Preview error: {e}");
                self.repair_preview_patch = None;
            }
        }
    }

    pub(crate) fn fill_selected_hole(&mut self) {
        let Some(hole) = self
            .repair_selected_hole
            .and_then(|idx| self.repair_holes.get(idx).cloned())
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
        match generate_hole_patch(&curr, &hole, self.repair_config) {
            Ok(mut patch) => {
                if self.repair_refine_to_references {
                    let refs = self.active_reference_geometries();
                    refine_patch_to_references(&mut patch, &curr, &hole, &refs);
                }
                self.push_snapshot();
                let mut next_mesh = (*curr).clone();
                apply_patch(&mut next_mesh, &patch);
                let method_name = self.repair_config.method.display_name();
                let refined_suffix = if self.repair_refine_to_references {
                    " [CAD-refined]"
                } else {
                    ""
                };
                self.set_mesh_modified(
                    next_mesh,
                    format!(
                        "Hole #{} filled using {}{}.",
                        hole.id, method_name, refined_suffix
                    ),
                );
            }
            Err(e) => self.status = format!("Failed to fill hole: {e}"),
        }
    }

    pub(crate) fn fill_all_holes(&mut self) {
        if self.repair_holes.is_empty() {
            self.status = "No holes to fill.".to_string();
            return;
        }
        let Some(curr) = self.current.clone() else {
            return;
        };
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        let refs = if self.repair_refine_to_references {
            self.active_reference_geometries()
        } else {
            Vec::new()
        };
        let mut working = (*curr).clone();
        let mut filled_count = 0;
        for hole in &self.repair_holes {
            if let Ok(mut patch) = generate_hole_patch(&working, hole, self.repair_config) {
                if !refs.is_empty() {
                    refine_patch_to_references(&mut patch, &working, hole, &refs);
                }
                apply_patch(&mut working, &patch);
                filled_count += 1;
            }
        }
        if filled_count > 0 {
            self.push_snapshot();
            let method_name = self.repair_config.method.display_name();
            let total = self.repair_holes.len();
            self.set_mesh_modified(
                working,
                format!("Filled {filled_count}/{total} holes using {method_name}."),
            );
        } else {
            self.status = "Failed to fill holes.".to_string();
        }
    }

    // ----------------------------------------------------------------------
    // Contour bridge
    // ----------------------------------------------------------------------

    /// Cluster detection for the current selection, cached per selection /
    /// mesh generation so the UI can query it every frame.
    pub(crate) fn bridge_clusters(
        &mut self,
    ) -> Result<(SelectionCluster, SelectionCluster), String> {
        let (sg, mg) = (self.sel_generation, self.mesh_generation);
        if let Some(c) = &self.bridge_cluster_cache
            && c.sel_generation == sg
            && c.mesh_generation == mg
        {
            return c.result.clone();
        }
        let result = match self.current.clone() {
            Some(curr) if self.preview.is_none() && self.sel_count > 0 => {
                let topo = self.ensure_topology();
                detect_selection_clusters(&curr, &self.sel, topo.as_deref())
            }
            _ => Err("Bridge requires 2 separate selections across the gap (found 0).".to_string()),
        };
        self.bridge_cluster_cache = Some(BridgeClusterCache {
            sel_generation: sg,
            mesh_generation: mg,
            result: result.clone(),
        });
        result
    }

    /// True when the selection consists of exactly two regions that can be
    /// bridged.
    pub(crate) fn has_bridge_clusters(&mut self) -> bool {
        self.sel_count > 0 && self.bridge_clusters().is_ok()
    }

    pub(crate) fn set_bridge_config(&mut self, config: BridgeConfig) {
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
        let Some(curr) = self.current.clone() else {
            return;
        };
        match self.bridge_clusters() {
            Ok((cluster_a, cluster_b)) => {
                match generate_bridge_patch(&curr, &cluster_a, &cluster_b, self.bridge_config) {
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

    pub(crate) fn apply_bridge(&mut self) {
        let Some(curr) = self.current.clone() else {
            return;
        };
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        match self.bridge_clusters() {
            Ok((cluster_a, cluster_b)) => {
                match generate_bridge_patch(&curr, &cluster_a, &cluster_b, self.bridge_config) {
                    Ok(patch) => {
                        self.push_snapshot();
                        let mut next_mesh = (*curr).clone();
                        apply_bridge_patch(&mut next_mesh, &patch);
                        self.bridge_preview_patch = None;
                        self.bridge_status = None;
                        self.bridge_preview_active = false;
                        let segs = self.bridge_config.segments;
                        let added_tris = patch.new_indices.len() / 3;
                        self.set_mesh_modified(
                            next_mesh,
                            format!(
                                "Bridge applied ({segs} segments, {added_tris} triangles added)."
                            ),
                        );
                    }
                    Err(e) => self.status = format!("Bridge failed: {e}"),
                }
            }
            Err(e) => self.status = format!("Cannot bridge: {e}"),
        }
    }

    // ----------------------------------------------------------------------
    // Clean-up tools
    // ----------------------------------------------------------------------

    fn modify_current(
        &mut self,
        f: impl FnOnce(&crate::mesh::Mesh) -> (crate::mesh::Mesh, String),
    ) {
        let Some(curr) = self.current.clone() else {
            return;
        };
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        self.push_snapshot();
        let (next_mesh, summary) = f(&curr);
        self.set_mesh_modified(next_mesh, summary);
    }

    pub(crate) fn auto_repair(&mut self) {
        self.modify_current(auto_repair_mesh);
    }

    pub(crate) fn unify_normals_action(&mut self) {
        self.modify_current(|m| {
            (
                unify_normals(m),
                "Unified triangle normals across all shared edges.".to_string(),
            )
        });
    }

    pub(crate) fn remove_small_components_action(&mut self) {
        self.modify_current(|m| {
            (
                remove_small_components(m, false, 0.005),
                "Removed small floating components (< 0.5% faces).".to_string(),
            )
        });
    }

    pub(crate) fn remove_degenerate_faces_action(&mut self) {
        self.modify_current(|m| {
            (
                remove_degenerate_faces(m),
                "Removed degenerate and zero-area faces.".to_string(),
            )
        });
    }
}
