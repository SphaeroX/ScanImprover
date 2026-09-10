//! Fitted reference features (symmetry plane, planes, circles, freeform
//! surfaces) and the datum alignment slots built from them.

use super::{
    App, CIRCLE_COLORS, FREEFORM_COLORS, FREEFORM_MAX_POINTS, PLANE_COLORS, axis_name,
    rotation_between,
};
use crate::geom::alignment::{
    AlignmentSlots, AxisChoice, FeatureRef, OriginRef, compute_alignment,
};
use crate::geom::fitting::{FittedCircle, FittedPlane, fit_circle, fit_plane};
use crate::geom::freeform::{
    FittedFreeform, FreeformFitData, FreeformGradient, FreeformParams, boundary_segments,
    calculate_in_tolerance_pct, compute_freeform_deviation,
};
use crate::geom::hole_solver::ReferenceGeometry;
use glam::{Quat, Vec3};
use std::sync::Arc;

impl App {
    fn alloc_object_id(&mut self) -> u64 {
        let id = self.next_obj_id;
        self.next_obj_id += 1;
        id
    }

    // ----------------------------------------------------------------------
    // Symmetry plane / active plane / active circle shortcuts
    // ----------------------------------------------------------------------

    pub(crate) fn rotate_normal_to_axis(&mut self, axis: Vec3) {
        if let Some(n) = self
            .sym
            .map(|s| s.plane.normal)
            .or(self.plane.map(|p| p.normal))
        {
            let q = rotation_between(n, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned normal to {}.", axis_name(axis));
        }
    }

    pub(crate) fn circle_axis_to(&mut self, axis: Vec3) {
        if let Some(c) = self.circle {
            let q = rotation_between(c.normal, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned circle axis to {}.", axis_name(axis));
        }
    }

    pub(crate) fn origin_on_plane(&mut self) {
        if let Some(p) = self.plane {
            let d = p.point.dot(p.normal);
            self.apply_transform(Quat::IDENTITY, -p.normal * d);
            self.status = "Origin moved onto the fitted plane.".to_string();
        }
    }

    pub(crate) fn origin_at_symmetry(&mut self) {
        if let Some(s) = self.sym {
            let d = s.plane.point.dot(s.plane.normal);
            self.apply_transform(Quat::IDENTITY, -s.plane.normal * d);
            self.status =
                "Mesh translated so the symmetry plane passes through the origin.".to_string();
        }
    }

    pub(crate) fn origin_at_circle_center(&mut self) {
        if let Some(c) = self.circle {
            self.apply_transform(Quat::IDENTITY, -c.center);
            self.status = "Origin moved to the circle center.".to_string();
        }
    }

    /// Rotates so the symmetry plane normal points along `axis`, then moves
    /// the plane through the origin.
    pub(crate) fn align_sym_full(&mut self, axis: Vec3, q: Quat) {
        self.apply_transform(q, Vec3::ZERO);
        if let Some(s) = &self.sym {
            let d = s.plane.point.dot(s.plane.normal);
            self.apply_transform(Quat::IDENTITY, -s.plane.normal * d);
        }
        self.status = format!(
            "Symmetry plane aligned: mesh rotated so the plane is {} = 0 and passes through the origin.",
            axis_name(axis)
        );
    }

    // ----------------------------------------------------------------------
    // Planes
    // ----------------------------------------------------------------------

    pub(crate) fn fit_plane_from_selection(&mut self) {
        let points = self.selection_points();
        match points.map(|p| fit_plane(&p)) {
            Some(Some(f)) => {
                self.push_snapshot();
                let id = self.alloc_object_id();
                let color = PLANE_COLORS[self.planes.len() % PLANE_COLORS.len()];
                let name = format!("Plane {}", self.planes.len() + 1);
                self.planes.push(FittedPlane {
                    id,
                    name,
                    fit: f,
                    visible: true,
                    color,
                });
                self.selected_plane_id = Some(id);
                self.plane = Some(f);
                self.show_plane = true;
                self.status = format!(
                    "Plane fitted: RMS {:.4} mm, max {:.4} mm.",
                    f.rms.sqrt(),
                    f.max_dev
                );
            }
            Some(None) => self.status = "Not enough points to fit a plane.".to_string(),
            None => self.status = "Select faces first.".to_string(),
        }
    }

    pub(crate) fn select_plane(&mut self, id: u64) {
        self.selected_plane_id = Some(id);
        if let Some(p) = self.planes.iter().find(|p| p.id == id) {
            self.plane = Some(p.fit);
            self.show_plane = p.visible;
        }
    }

    pub(crate) fn delete_plane(&mut self, id: u64) {
        self.push_snapshot();
        self.planes.retain(|p| p.id != id);
        if self.selected_plane_id == Some(id) {
            self.selected_plane_id = self.planes.last().map(|p| p.id);
            self.plane = self.planes.last().map(|p| p.fit);
        }
        self.align_slots.remove_feature(FeatureRef::Plane(id));
        self.status = "Plane deleted.".to_string();
    }

    pub(crate) fn rotate_plane_normal_to_axis(&mut self, plane_id: u64, axis: Vec3) {
        if let Some(p) = self.planes.iter().find(|p| p.id == plane_id) {
            let q = rotation_between(p.fit.normal, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned normal to {}.", axis_name(axis));
        }
    }

    pub(crate) fn origin_on_plane_id(&mut self, plane_id: u64) {
        if let Some(p) = self.planes.iter().find(|p| p.id == plane_id) {
            let normal = p.fit.normal;
            let d = p.fit.point.dot(normal);
            self.apply_transform(Quat::IDENTITY, -normal * d);
            self.status = "Origin moved onto the fitted plane.".to_string();
        }
    }

    // ----------------------------------------------------------------------
    // Circles
    // ----------------------------------------------------------------------

    pub(crate) fn fit_circle_from_selection(&mut self) {
        let points = self.selection_points();
        match points.map(|p| fit_circle(&p)) {
            Some(Some(c)) => {
                self.push_snapshot();
                let id = self.alloc_object_id();
                let color = CIRCLE_COLORS[self.circles.len() % CIRCLE_COLORS.len()];
                let name = format!("Circle {}", self.circles.len() + 1);
                self.circles.push(FittedCircle {
                    id,
                    name,
                    fit: c,
                    visible: true,
                    color,
                });
                self.selected_circle_id = Some(id);
                self.circle = Some(c);
                self.show_circle = true;
                self.status = if c.cylinder {
                    format!(
                        "Circle fitted as cylinder cross-section: R = {:.4} mm (D = {:.4} mm), radial RMS {:.4} mm.",
                        c.radius,
                        c.radius * 2.0,
                        c.radial_rms.sqrt()
                    )
                } else {
                    format!(
                        "Circle fitted: R = {:.4} mm, radial RMS {:.4} mm.",
                        c.radius,
                        c.radial_rms.sqrt()
                    )
                };
            }
            Some(None) => self.status = "Circle fit failed on selection.".to_string(),
            None => self.status = "Select faces first.".to_string(),
        }
    }

    pub(crate) fn select_circle(&mut self, id: u64) {
        self.selected_circle_id = Some(id);
        if let Some(c) = self.circles.iter().find(|c| c.id == id) {
            self.circle = Some(c.fit);
            self.show_circle = c.visible;
        }
    }

    pub(crate) fn delete_circle(&mut self, id: u64) {
        self.push_snapshot();
        self.circles.retain(|c| c.id != id);
        if self.selected_circle_id == Some(id) {
            self.selected_circle_id = self.circles.last().map(|c| c.id);
            self.circle = self.circles.last().map(|c| c.fit);
        }
        self.align_slots.remove_feature(FeatureRef::Circle(id));
        self.status = "Circle deleted.".to_string();
    }

    pub(crate) fn rotate_circle_axis_to(&mut self, circle_id: u64, axis: Vec3) {
        if let Some(c) = self.circles.iter().find(|c| c.id == circle_id) {
            let q = rotation_between(c.fit.normal, axis);
            self.apply_transform(q, Vec3::ZERO);
            self.status = format!("Aligned circle axis to {}.", axis_name(axis));
        }
    }

    pub(crate) fn origin_at_circle_id(&mut self, circle_id: u64) {
        if let Some(c) = self.circles.iter().find(|c| c.id == circle_id) {
            let center = c.fit.center;
            self.apply_transform(Quat::IDENTITY, -center);
            self.status = "Origin moved to the circle center.".to_string();
        }
    }

    // ----------------------------------------------------------------------
    // Freeform surfaces
    // ----------------------------------------------------------------------

    /// Sub-sampled selection points for the freeform fitter.
    fn selection_points_capped(&self, max_points: usize) -> Option<Vec<[f32; 3]>> {
        let pts = self.selection_points()?;
        if pts.len() > max_points {
            let stride = pts.len().div_ceil(max_points);
            Some(pts.iter().step_by(stride).copied().collect())
        } else {
            Some(pts)
        }
    }

    /// Fits a freeform surface onto the current face selection.
    pub(crate) fn fit_freeform_from_selection(&mut self) {
        let Some(points) = self.selection_points_capped(FREEFORM_MAX_POINTS) else {
            self.status = "Select faces first.".to_string();
            return;
        };
        if points.len() < 12 {
            self.status = "Selection too small for a freeform surface.".to_string();
            return;
        }
        let mut min = [f32::MAX; 3];
        let mut max = [f32::MIN; 3];
        for p in &points {
            for k in 0..3 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        let extent = (max[0] - min[0])
            .max(max[1] - min[1])
            .max(max[2] - min[2])
            .max(1e-6);

        self.push_snapshot();
        let id = self.alloc_object_id();
        let color = FREEFORM_COLORS[self.freeforms.len() % FREEFORM_COLORS.len()];
        let name = format!("Freeform {}", self.freeforms.len() + 1);
        self.freeforms.push(FittedFreeform {
            id,
            name,
            visible: true,
            color,
            source_points: Arc::new(points),
            params: FreeformParams::default_for(extent),
            surface: None,
            boundary: Vec::new(),
            rms: 0.0,
            max_dev: 0.0,
            fold_ratio: 0.0,
            point_count: 0,
            refit_pending: false,
            heat_on: false,
            heat_max: 0.20,
            heat_gradient: FreeformGradient::TrafficLight,
            heat: None,
            in_tolerance_pct: 100.0,
        });
        self.selected_freeform_id = Some(id);
        self.status = "Fitting freeform surface…".to_string();
        self.submit_freeform_fit(id);
    }

    fn submit_freeform_fit(&mut self, freeform_id: u64) {
        if self.freeform_job.is_some() {
            // One fit at a time; queue this one until the running fit lands.
            if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id) {
                f.refit_pending = true;
            }
            return;
        }
        let Some(f) = self.freeforms.iter().find(|f| f.id == freeform_id) else {
            return;
        };
        let job = self
            .worker
            .submit_freeform_fit(freeform_id, f.source_points.clone(), f.params);
        self.freeform_job = Some((job, freeform_id));
    }

    /// Re-fits a freeform after a parameter change. If a fit is already
    /// running, the re-fit is queued and executed once it finishes.
    pub(crate) fn schedule_freeform_refit(&mut self, freeform_id: u64) {
        self.submit_freeform_fit(freeform_id);
    }

    pub(crate) fn set_freeform_params(&mut self, freeform_id: u64, params: FreeformParams) {
        if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id)
            && f.params != params
        {
            f.params = params;
            self.schedule_freeform_refit(freeform_id);
        }
    }

    /// Stores a finished freeform fit and starts the next queued re-fit.
    pub(crate) fn finish_freeform_fit(
        &mut self,
        freeform_id: u64,
        data: Result<FreeformFitData, String>,
    ) {
        match data {
            Ok(d) => {
                let boundary = boundary_segments(&d.surface);
                let tris = d.surface.triangle_count();
                let bvh = self.bvh.clone();
                if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id) {
                    let (heat, in_tol) = match &bvh {
                        Some(bvh) => {
                            let (h, _, _) = compute_freeform_deviation(&d.surface, bvh);
                            let in_tol = calculate_in_tolerance_pct(&h, f.heat_max);
                            (Some(h), in_tol)
                        }
                        None => (None, 100.0),
                    };
                    f.surface = Some(d.surface);
                    f.boundary = boundary;
                    f.rms = d.rms;
                    f.max_dev = d.max_dev;
                    f.fold_ratio = d.fold_ratio;
                    f.point_count = d.point_count;
                    f.heat = heat;
                    f.in_tolerance_pct = in_tol;
                    let ov = f.params.overshoot_mm;
                    self.status = format!(
                        "Freeform surface fitted: {tris} triangles, RMS {:.4} mm, overshoot {ov:.2} mm.",
                        d.rms
                    );
                    if d.fold_ratio > 0.15 {
                        self.status.push_str(
                            " Note: the selection appears to wrap around; \
                             the fit may be inaccurate there.",
                        );
                    }
                }
            }
            Err(message) => {
                self.status = format!("Freeform fit failed: {message}");
                if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id) {
                    f.refit_pending = false;
                }
            }
        }
        // Chained refits: the next freeform with pending parameter changes
        // gets the worker next.
        if let Some(next) = self.freeforms.iter_mut().find(|f| f.refit_pending) {
            next.refit_pending = false;
            let next_id = next.id;
            self.submit_freeform_fit(next_id);
        }
    }

    pub(crate) fn select_freeform(&mut self, id: u64) {
        self.selected_freeform_id = Some(id);
    }

    pub(crate) fn delete_freeform(&mut self, id: u64) {
        self.push_snapshot();
        self.freeforms.retain(|f| f.id != id);
        if self.selected_freeform_id == Some(id) {
            self.selected_freeform_id = self.freeforms.last().map(|f| f.id);
        }
        if self.freeform_job.map(|(_, fid)| fid) == Some(id) {
            self.freeform_job = None;
        }
        self.status = "Freeform surface deleted.".to_string();
    }

    pub(crate) fn ensure_freeform_heat(&mut self, freeform_id: u64) {
        let Some(bvh) = self.bvh.clone() else {
            return;
        };
        if let Some(f) = self.freeforms.iter_mut().find(|f| f.id == freeform_id)
            && f.heat.is_none()
            && let Some(surf) = &f.surface
        {
            let (heat, _, _) = compute_freeform_deviation(surf, &bvh);
            f.in_tolerance_pct = calculate_in_tolerance_pct(&heat, f.heat_max);
            f.heat = Some(heat);
        }
    }

    // ----------------------------------------------------------------------
    // Export
    // ----------------------------------------------------------------------

    /// Default half-size of exported plane patches, derived from the model.
    fn export_plane_size(&self) -> f32 {
        (self.bbox.diagonal() * 0.4).clamp(20.0, 500.0)
    }

    pub(crate) fn export_freeform_id(&mut self, freeform_id: u64) {
        if let Some(f) = self.freeforms.iter().find(|f| f.id == freeform_id) {
            self.status = match crate::export::export_freeform_dialog(f) {
                Ok(msg) => msg,
                Err(e) => format!("Export failed: {e}"),
            };
        }
    }

    pub(crate) fn export_plane_id(&mut self, plane_id: u64) {
        if let Some(p) = self.planes.iter().find(|p| p.id == plane_id) {
            let size = self.export_plane_size();
            self.status = match crate::export::export_plane_dialog(p, size) {
                Ok(msg) => msg,
                Err(e) => format!("Export failed: {e}"),
            };
        }
    }

    pub(crate) fn export_circle_id(&mut self, circle_id: u64) {
        if let Some(c) = self.circles.iter().find(|c| c.id == circle_id) {
            self.status = match crate::export::export_circle_dialog(c) {
                Ok(msg) => msg,
                Err(e) => format!("Export failed: {e}"),
            };
        }
    }

    pub(crate) fn export_all_references(&mut self) {
        let size = self.export_plane_size();
        self.status =
            match crate::export::export_all_references_dialog(&self.planes, &self.circles, size) {
                Ok(msg) => msg,
                Err(e) => format!("Export failed: {e}"),
            };
    }

    // ----------------------------------------------------------------------
    // Alignment slots (3-2-1 datum alignment)
    // ----------------------------------------------------------------------

    pub(crate) fn toggle_assign_feature(&mut self, feat: FeatureRef, axis: AxisChoice) {
        let is_current = self.align_slots.axis_of(feat) == Some(axis);
        self.align_slots.remove_feature_from_axes(feat);
        if !is_current {
            self.align_slots.set_axis(axis, Some(feat));
        }
    }

    pub(crate) fn toggle_origin_feature(&mut self, feat: FeatureRef) {
        if self.is_feature_origin(feat) {
            self.align_slots.origin = OriginRef::FromAssignedFeatures;
        } else {
            self.align_slots.origin = match feat {
                FeatureRef::SymmetryPlane => OriginRef::SymmetryPlane,
                FeatureRef::Plane(id) => OriginRef::Plane(id),
                FeatureRef::Circle(id) => OriginRef::CircleCenter(id),
            };
        }
    }

    pub(crate) fn clear_alignment_slots(&mut self) {
        self.align_slots = AlignmentSlots::default();
        self.status = "Cleared feature alignment slots.".to_string();
    }

    pub(crate) fn align_to_features(&mut self) -> bool {
        match compute_alignment(&self.align_slots, self) {
            Ok(t) => {
                self.apply_transform(t.rotation, t.translation);
                self.status = t.summary;
                true
            }
            Err(e) => {
                self.status = format!("Alignment failed: {e}");
                false
            }
        }
    }

    pub(crate) fn auto_assign_alignment_from_selection(&mut self) {
        let mut assigned_any = false;

        if self.sym.is_some() && self.align_slots.x.is_none() {
            self.align_slots.x = Some(FeatureRef::SymmetryPlane);
            assigned_any = true;
        }

        // Circle: prefer the selected circle, else the first one.
        let target_circle = self
            .selected_circle_id
            .or_else(|| self.circles.first().map(|c| c.id));
        if let Some(cid) = target_circle {
            if self.align_slots.z.is_none() {
                self.align_slots.z = Some(FeatureRef::Circle(cid));
                assigned_any = true;
            }
            if matches!(self.align_slots.origin, OriginRef::FromAssignedFeatures) {
                self.align_slots.origin = OriginRef::CircleCenter(cid);
            }
        }

        // Plane: prefer the selected plane, else the first one.
        let target_plane = self
            .selected_plane_id
            .or_else(|| self.planes.first().map(|p| p.id));
        if let Some(pid) = target_plane {
            if self.align_slots.y.is_none() {
                self.align_slots.y = Some(FeatureRef::Plane(pid));
                assigned_any = true;
            } else if self.align_slots.x.is_none() {
                self.align_slots.x = Some(FeatureRef::Plane(pid));
                assigned_any = true;
            }
        }

        self.status = if assigned_any {
            "Auto-assigned features to alignment slots.".to_string()
        } else {
            "No unassigned features available to auto-assign.".to_string()
        };
    }

    // ----------------------------------------------------------------------
    // Reference geometry for the hole solver
    // ----------------------------------------------------------------------

    pub(crate) fn active_reference_geometries(&self) -> Vec<ReferenceGeometry> {
        let mut refs = Vec::new();
        for p in self.planes.iter().filter(|p| p.visible) {
            refs.push(ReferenceGeometry::from_plane(p));
        }
        for c in self.circles.iter().filter(|c| c.visible) {
            refs.push(ReferenceGeometry::from_circle(c, self.repair_circle_mode));
        }
        if refs.is_empty() {
            if let Some(p) = self.plane {
                refs.push(ReferenceGeometry::Plane {
                    id: 0,
                    name: "Active Plane".to_string(),
                    point: p.point,
                    normal: p.normal,
                });
            }
            if let Some(c) = self.circle {
                refs.push(ReferenceGeometry::Circle {
                    id: 0,
                    name: "Active Circle".to_string(),
                    center: c.center,
                    normal: c.normal,
                    radius: c.radius,
                    mode: self.repair_circle_mode,
                });
            }
        }
        refs
    }
}
