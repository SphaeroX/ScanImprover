//! The 3D viewport: navigation, brush selection input, overlay generation
//! and synchronisation of the GPU buffers.

use super::overlay::{
    Line, bbox_lines, circle_lines, ground_grid_lines, marker_lines, nice_step, plane_fill,
    plane_grid_lines, push_line, push_tri, triad_lines,
};
use super::{App, Mode, ToolSection};
use crate::geom::freeform::freeform_vertex_color;
use crate::pick;
use crate::ui::theme;
use glam::Vec3;

/// Pixel-space input for one frame of the viewport.
struct Pointer {
    /// Cursor position relative to the viewport, if hovering it.
    pos: Option<(f32, f32)>,
    w: f32,
    h: f32,
}

impl App {
    /// Casts a ray through viewport pixel `(sx, sy)` and returns the closest
    /// visible mesh hit.
    fn pick_at(&mut self, sx: f32, sy: f32, w: f32, h: f32) -> Option<pick::Hit> {
        let bvh = self.ensure_bvh()?;
        let hidden = &self.hidden_mask;
        let is_hidden = |t: u32| (t as usize) < hidden.len() && hidden[t as usize];
        pick::ray_pick_filtered(&bvh, &self.camera, sx, sy, w, h, is_hidden)
    }

    /// World anchor for zoom / orbit: the surface point under the cursor, or
    /// the point on the target plane below the cursor.
    fn anchor_at(&mut self, sx: f32, sy: f32, w: f32, h: f32) -> Vec3 {
        match self.pick_at(sx, sy, w, h) {
            Some(hit) => hit.pos,
            None => self.camera.point_on_target_plane(sx, sy, w, h),
        }
    }

    pub(crate) fn viewport(&mut self, ui: &mut egui::Ui) {
        let rect = ui.available_rect_before_wrap();
        let (rect, response) = ui.allocate_exact_size(rect.size(), egui::Sense::click_and_drag());
        self.camera.aspect = rect.width() / rect.height().max(1.0);
        let now = ui.ctx().input(|i| i.time);
        self.camera.tick(now);

        let pointer = Pointer {
            pos: response
                .hover_pos()
                .or_else(|| response.interact_pointer_pos())
                .map(|p| (p.x - rect.min.x, p.y - rect.min.y)),
            w: rect.width(),
            h: rect.height(),
        };

        let is_navigating = self.handle_navigation(ui, &response, &pointer);
        self.handle_wheel(ui, &response, is_navigating);
        self.handle_selection_input(ui, &response, &pointer, is_navigating);
        let hover_pos_3d = self.handle_symmetry_pick(ui, &response, &pointer, is_navigating);
        self.update_hover(&response, &pointer, is_navigating);

        self.draw_brush_cursor(ui, &response, is_navigating);
        let (depth_lines, overlay_lines, fills) = self.build_overlays(hover_pos_3d, is_navigating);
        self.collect_frame(ui.ctx(), rect, depth_lines, overlay_lines, fills);

        self.draw_viewport_chrome(ui, rect);
        crate::ui::selection_hud::render_selection_hud(self, ui, rect);
        crate::ui::gizmo::render_nav_gizmo(self, ui, rect);
    }

    // ----------------------------------------------------------------------
    // Navigation
    // ----------------------------------------------------------------------

    /// Orbit (RMB), pan (MMB or Shift+RMB). Returns true while navigating.
    fn handle_navigation(
        &mut self,
        ui: &mut egui::Ui,
        response: &egui::Response,
        pointer: &Pointer,
    ) -> bool {
        let shift = ui.input(|i| i.modifiers.shift);
        let rmb_down = ui.input(|i| i.pointer.button_down(egui::PointerButton::Secondary));
        let mmb_down = ui.input(|i| i.pointer.button_down(egui::PointerButton::Middle));
        let orbit_started = response.drag_started_by(egui::PointerButton::Secondary) && !shift;
        let pan_started = response.drag_started_by(egui::PointerButton::Middle)
            || (response.drag_started_by(egui::PointerButton::Secondary) && shift);

        if orbit_started {
            self.orbit_pivot = pointer
                .pos
                .and_then(|(sx, sy)| self.pick_at(sx, sy, pointer.w, pointer.h))
                .map(|h| h.pos);
        }
        if pan_started {
            let eye = self.camera.eye();
            self.pan_depth = Some(
                pointer
                    .pos
                    .and_then(|(sx, sy)| self.pick_at(sx, sy, pointer.w, pointer.h))
                    .map(|h| (h.pos - eye).length())
                    .unwrap_or(self.camera.distance),
            );
        }

        let orbiting = response.dragged_by(egui::PointerButton::Secondary) && !shift;
        let panning = response.dragged_by(egui::PointerButton::Middle)
            || (response.dragged_by(egui::PointerButton::Secondary) && shift);
        if orbiting {
            let d = response.drag_delta();
            match self.orbit_pivot {
                Some(pivot) => self.camera.orbit_about(pivot, d.x, d.y),
                None => self.camera.rotate(d.x, d.y),
            }
        } else if panning {
            let d = response.drag_delta();
            let depth = self.pan_depth.unwrap_or(self.camera.distance);
            self.camera.pan_pixels(d.x, d.y, pointer.h, depth);
        }
        if !rmb_down {
            self.orbit_pivot = None;
        }
        if !mmb_down && !rmb_down {
            self.pan_depth = None;
        }
        rmb_down || mmb_down || orbiting || panning
    }

    /// Wheel: zoom towards the cursor; Alt = brush size; Ctrl = grow / shrink.
    fn handle_wheel(&mut self, ui: &mut egui::Ui, response: &egui::Response, is_navigating: bool) {
        if !response.hovered() || is_navigating {
            return;
        }
        let (ctrl, alt) = ui.input(|i| (i.modifiers.ctrl || i.modifiers.command, i.modifiers.alt));
        let (wheel_events, zoom_delta, scroll) = ui.input(|i| {
            let events: Vec<(egui::MouseWheelUnit, egui::Vec2)> = i
                .events
                .iter()
                .filter_map(|e| match e {
                    egui::Event::MouseWheel { unit, delta, .. } => Some((*unit, *delta)),
                    _ => None,
                })
                .collect();
            (events, i.zoom_delta(), i.smooth_scroll_delta.y)
        });

        if alt {
            let mut triggered = false;
            for (unit, delta) in wheel_events {
                let step = match unit {
                    egui::MouseWheelUnit::Line => delta.y * 3.0,
                    egui::MouseWheelUnit::Point | egui::MouseWheelUnit::Page => delta.y * 0.25,
                };
                if step != 0.0 {
                    self.brush_radius = (self.brush_radius + step).clamp(2.0, 150.0);
                    triggered = true;
                }
            }
            if !triggered {
                if zoom_delta > 1.01 {
                    self.brush_radius = (self.brush_radius + 3.0).clamp(2.0, 150.0);
                } else if zoom_delta < 0.99 {
                    self.brush_radius = (self.brush_radius - 3.0).clamp(2.0, 150.0);
                }
            }
        } else if ctrl {
            let mut triggered = false;
            for (unit, delta) in wheel_events {
                match unit {
                    egui::MouseWheelUnit::Line => {
                        let steps = delta.y.round() as i32;
                        for _ in 0..steps.max(0) {
                            self.grow_selection();
                        }
                        for _ in 0..(-steps).max(0) {
                            self.shrink_selection();
                        }
                        triggered |= steps != 0;
                    }
                    egui::MouseWheelUnit::Point | egui::MouseWheelUnit::Page => {
                        self.wheel_accum += delta.y;
                        while self.wheel_accum >= 10.0 {
                            self.grow_selection();
                            self.wheel_accum -= 10.0;
                            triggered = true;
                        }
                        while self.wheel_accum <= -10.0 {
                            self.shrink_selection();
                            self.wheel_accum += 10.0;
                            triggered = true;
                        }
                    }
                }
            }
            if !triggered {
                if zoom_delta > 1.01 {
                    self.grow_selection();
                } else if zoom_delta < 0.99 {
                    self.shrink_selection();
                }
            }
        } else {
            self.wheel_accum = 0.0;
            if scroll != 0.0 {
                let factor = 0.95f32.powf(scroll / 60.0);
                let anchor = match response.hover_pos() {
                    Some(p) => {
                        let r = response.rect;
                        self.anchor_at(p.x - r.min.x, p.y - r.min.y, r.width(), r.height())
                    }
                    None => self.camera.target,
                };
                self.camera.zoom_towards(factor, anchor);
            }
        }
    }

    // ----------------------------------------------------------------------
    // Selection input
    // ----------------------------------------------------------------------

    /// LMB paints the selection (Shift erases); double-click selects a region.
    fn handle_selection_input(
        &mut self,
        ui: &mut egui::Ui,
        response: &egui::Response,
        pointer: &Pointer,
        is_navigating: bool,
    ) {
        let shift = ui.input(|i| i.modifiers.shift);
        let lmb_down = ui.input(|i| i.pointer.button_down(egui::PointerButton::Primary));
        let lmb_pressed = ui.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary));
        if !lmb_down {
            self.suppress_sel_drag = false;
            self.stroke_snapshot_pending = false;
        }
        if self.mode == Mode::SymPickLine || is_navigating || self.suppress_sel_drag {
            return;
        }
        if response.drag_started_by(egui::PointerButton::Primary)
            || (response.hovered() && lmb_pressed)
        {
            self.stroke_snapshot_pending = true;
        }
        let painting =
            response.dragged_by(egui::PointerButton::Primary) || (response.hovered() && lmb_down);
        if painting
            && let Some((sx, sy)) = pointer.pos
            && let Some(hit) = self.pick_at(sx, sy, pointer.w, pointer.h)
        {
            let add = !shift;
            let changed = self.brush_apply(&hit, self.brush_radius, pointer.h, add);
            if changed && add && self.sel_count > 0 {
                self.open_section(ToolSection::Selection);
            }
        }
        if response.double_clicked()
            && let Some((sx, sy)) = pointer.pos
            && let Some(hit) = self.pick_at(sx, sy, pointer.w, pointer.h)
        {
            self.select_region_under(hit.tri);
        }
    }

    /// Hover preview of the brush footprint (not while navigating / painting).
    fn update_hover(&mut self, response: &egui::Response, pointer: &Pointer, is_navigating: bool) {
        let lmb_down = response
            .ctx
            .input(|i| i.pointer.button_down(egui::PointerButton::Primary));
        let shift = response.ctx.input(|i| i.modifiers.shift);
        let mut new_hit = None;
        let mut new_tris = Vec::new();
        let mut new_r = 0.0f32;
        if self.mode != Mode::SymPickLine
            && response.hovered()
            && !is_navigating
            && !lmb_down
            && let Some((sx, sy)) = pointer.pos
            && let (Some(mesh), Some(bvh)) = (self.display().cloned(), self.ensure_bvh())
            && let Some(hit) = self.pick_at(sx, sy, pointer.w, pointer.h)
        {
            let (r, tris) = self.brush_query(&mesh, &bvh, &hit, self.brush_radius, pointer.h);
            new_hit = Some(hit);
            new_tris = tris;
            new_r = r;
        }
        self.hover_hit = new_hit;
        self.hover_tris = new_tris;
        self.hover_radius_world = new_r;
        self.hover_is_erase = shift;
    }

    /// Symmetry line picking mode. Returns the surface point under the cursor.
    fn handle_symmetry_pick(
        &mut self,
        ui: &mut egui::Ui,
        response: &egui::Response,
        pointer: &Pointer,
        is_navigating: bool,
    ) -> Option<Vec3> {
        if self.mode != Mode::SymPickLine || !response.hovered() {
            return None;
        }
        let hover_pos_3d = pointer
            .pos
            .and_then(|(sx, sy)| self.pick_at(sx, sy, pointer.w, pointer.h))
            .map(|h| h.pos);
        let pressed = ui.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary));
        if pressed && !is_navigating {
            self.suppress_sel_drag = true;
            if let Some(pos) = hover_pos_3d {
                if self.sym_pick.len() >= 2 {
                    self.sym_pick.clear();
                }
                self.sym_pick.push(pos);
                if self.sym_pick.len() == 1 {
                    self.status = "Point 1 placed. Click point 2 on the mesh.".to_string();
                } else if self.sym_pick.len() == 2 {
                    let (a, b) = (self.sym_pick[0], self.sym_pick[1]);
                    self.schedule_sym_from_line(a, b);
                    self.mode = Mode::Orbit;
                }
            }
        }
        hover_pos_3d
    }

    fn draw_brush_cursor(&self, ui: &mut egui::Ui, response: &egui::Response, is_navigating: bool) {
        if self.mode == Mode::SymPickLine
            || !response.hovered()
            || is_navigating
            || !self.has_mesh()
        {
            return;
        }
        let Some(pos) = response.hover_pos() else {
            return;
        };
        let (stroke_col, fill_col) = if self.hover_is_erase {
            (
                theme::pal().erase_accent,
                theme::pal().erase_accent.gamma_multiply(0.10),
            )
        } else {
            (
                theme::pal().select_accent,
                theme::pal().select_accent.gamma_multiply(0.10),
            )
        };
        ui.painter().circle_filled(pos, self.brush_radius, fill_col);
        ui.painter()
            .circle_stroke(pos, self.brush_radius, egui::Stroke::new(1.5, stroke_col));
    }

    // ----------------------------------------------------------------------
    // Overlay generation
    // ----------------------------------------------------------------------

    fn build_overlays(
        &mut self,
        hover_pos_3d: Option<Vec3>,
        is_navigating: bool,
    ) -> (Vec<Line>, Vec<Line>, Vec<Line>) {
        let mut depth_lines: Vec<Line> = Vec::new();
        let mut overlay_lines: Vec<Line> = Vec::new();
        let mut fills: Vec<Line> = Vec::new();
        if !self.has_mesh() {
            return (depth_lines, overlay_lines, fills);
        }
        let diag = self.bbox.diagonal().max(1e-3);
        let sym_col = |a: f32| {
            let c = theme::pal().symmetry_plane;
            [c[0], c[1], c[2], a]
        };

        if self.show_grid {
            let half = (diag * 1.5).max(10.0);
            let step = nice_step(half * 2.0, 40.0);
            let p = theme::pal();
            depth_lines.extend(ground_grid_lines(
                self.camera.up_axis,
                half,
                step,
                p.grid_minor,
                p.grid_major,
            ));
        }
        if self.show_bbox {
            depth_lines.extend(bbox_lines(&self.bbox, theme::pal().bbox_line));
        }
        if self.show_triad {
            overlay_lines.extend(triad_lines(diag * 0.12));
        }
        if let Some(s) = &self.sym
            && s.show
        {
            let half = diag * 0.6;
            depth_lines.extend(plane_grid_lines(
                s.plane.point,
                s.plane.normal,
                half,
                10,
                sym_col(0.55),
            ));
            fills.extend(plane_fill(
                s.plane.point,
                s.plane.normal,
                half,
                sym_col(0.07),
            ));
        }
        for p in self.planes.iter().filter(|p| p.visible) {
            let half = diag * 0.35;
            let is_sel = self.selected_plane_id == Some(p.id);
            let mut col = p.color;
            let mut fill_col = [col[0], col[1], col[2], 0.12];
            if is_sel {
                col[3] = 1.0;
                fill_col[3] = 0.22;
                overlay_lines.extend(plane_grid_lines(
                    p.fit.point,
                    p.fit.normal,
                    half,
                    2,
                    [1.0, 1.0, 1.0, 0.7],
                ));
            }
            depth_lines.extend(plane_grid_lines(p.fit.point, p.fit.normal, half, 6, col));
            fills.extend(plane_fill(p.fit.point, p.fit.normal, half, fill_col));
        }
        if self.planes.is_empty()
            && self.show_plane
            && let Some(p) = &self.plane
        {
            let half = diag * 0.35;
            depth_lines.extend(plane_grid_lines(
                p.point,
                p.normal,
                half,
                6,
                [1.0, 0.6, 0.1, 0.9],
            ));
            fills.extend(plane_fill(p.point, p.normal, half, [1.0, 0.6, 0.1, 0.12]));
        }
        for c in self.circles.iter().filter(|c| c.visible) {
            let col = if self.selected_circle_id == Some(c.id) {
                [1.0, 1.0, 0.3, 1.0]
            } else {
                c.color
            };
            depth_lines.extend(circle_lines(c.fit.center, c.fit.normal, c.fit.radius, col));
        }
        if self.circles.is_empty()
            && self.show_circle
            && let Some(c) = &self.circle
        {
            depth_lines.extend(circle_lines(
                c.center,
                c.normal,
                c.radius,
                [1.0, 0.2, 0.8, 1.0],
            ));
        }
        for f in self.freeforms.iter().filter(|f| f.visible) {
            let Some(surf) = &f.surface else {
                continue;
            };
            let is_sel = self.selected_freeform_id == Some(f.id);
            let col = f.color;
            let fill_col = [col[0], col[1], col[2], if is_sel { 0.30 } else { 0.16 }];
            let edge_col = if is_sel {
                [1.0, 1.0, 0.3, 1.0]
            } else {
                [col[0], col[1], col[2], 0.9]
            };
            let heat = f
                .heat
                .as_ref()
                .filter(|h| f.heat_on && h.len() == surf.vertex_count());
            let heat_alpha = if is_sel { 0.85 } else { 0.65 };
            for chunk in surf.indices.chunks_exact(3) {
                for &idx in chunk {
                    let p = surf.positions[idx as usize];
                    let c = match heat {
                        Some(h) => freeform_vertex_color(
                            h[idx as usize],
                            f.heat_max,
                            f.heat_gradient,
                            heat_alpha,
                        ),
                        None => fill_col,
                    };
                    fills.push([p[0], p[1], p[2], c[0], c[1], c[2], c[3]]);
                }
            }
            for (a, b) in &f.boundary {
                push_line(&mut depth_lines, Vec3::from(*a), Vec3::from(*b), edge_col);
            }
        }

        // Symmetry line picking markers.
        if self.mode == Mode::SymPickLine
            && let Some(h) = hover_pos_3d
        {
            overlay_lines.extend(marker_lines(h, diag * 0.02, [0.2, 0.9, 1.0, 1.0]));
            overlay_lines.extend(circle_lines(
                h,
                self.camera.back(),
                diag * 0.008,
                [0.2, 0.9, 1.0, 1.0],
            ));
            if let Some(&a) = self.sym_pick.first() {
                push_line(&mut overlay_lines, a, h, [1.0, 1.0, 0.3, 1.0]);
            }
        }
        for &p in &self.sym_pick {
            overlay_lines.extend(marker_lines(p, diag * 0.02, [1.0, 1.0, 1.0, 1.0]));
            overlay_lines.extend(circle_lines(
                p,
                self.camera.back(),
                diag * 0.008,
                [1.0, 1.0, 1.0, 1.0],
            ));
        }
        if self.sym_pick.len() == 2 {
            push_line(
                &mut overlay_lines,
                self.sym_pick[0],
                self.sym_pick[1],
                [1.0, 1.0, 0.3, 1.0],
            );
        }

        // Holes and repair / bridge previews.
        if (self.is_section_open(ToolSection::Repair) || !self.repair_holes.is_empty())
            && let Some(m) = self.display()
        {
            let pos = &m.positions;
            for (h_idx, hole) in self.repair_holes.iter().enumerate() {
                let is_sel = self.repair_selected_hole == Some(h_idx);
                let edge_col = if is_sel {
                    [0.1, 0.95, 1.0, 1.0]
                } else {
                    [1.0, 0.65, 0.2, 0.75]
                };
                let n_verts = hole.vertices.len();
                for i in 0..n_verts {
                    let (Some(a), Some(b)) = (
                        pos.get(hole.vertices[i] as usize),
                        pos.get(hole.vertices[(i + 1) % n_verts] as usize),
                    ) else {
                        continue;
                    };
                    let (p0, p1) = (Vec3::from(*a), Vec3::from(*b));
                    if is_sel {
                        push_line(&mut overlay_lines, p0, p1, edge_col);
                    } else {
                        push_line(&mut depth_lines, p0, p1, edge_col);
                    }
                }
                if is_sel {
                    overlay_lines.extend(marker_lines(
                        hole.centroid,
                        diag * 0.015,
                        [0.1, 0.95, 1.0, 1.0],
                    ));
                }
            }
        }
        if self.repair_preview_active
            && let Some(patch) = &self.repair_preview_patch
        {
            push_patch(
                patch,
                &mut fills,
                &mut overlay_lines,
                [0.15, 0.85, 0.95, 0.35],
                [0.2, 0.95, 1.0, 0.9],
            );
        }
        if self.bridge_preview_active
            && let Some(patch) = &self.bridge_preview_patch
        {
            push_patch(
                patch,
                &mut fills,
                &mut overlay_lines,
                [0.15, 0.90, 0.65, 0.40],
                [0.20, 1.0, 0.75, 0.95],
            );
        }

        // Brush hover: footprint disc plus the faces the dab would change.
        if let (Some(hit), false) = (&self.hover_hit, is_navigating)
            && let Some(m) = self.display()
        {
            let r = self.hover_radius_world;
            if r > 0.0 && (hit.tri as usize) < m.triangle_count() {
                let [a, b, c] = m.triangle(hit.tri as usize);
                let mut n = (b - a).cross(c - a);
                n = if n.length_squared() > 1e-12 {
                    n.normalize()
                } else {
                    self.camera.back()
                };
                if n.dot(self.camera.eye() - hit.pos) < 0.0 {
                    n = -n;
                }
                let (col, fill_col) = if self.hover_is_erase {
                    ([1.0, 0.25, 0.25, 0.95], [1.0, 0.25, 0.25, 0.45])
                } else {
                    ([1.0, 0.65, 0.15, 0.95], [1.0, 0.70, 0.20, 0.42])
                };
                overlay_lines.extend(circle_lines(hit.pos + n * (r * 0.005), n, r, col));
                let sel_ok = self.sel.len() == m.triangle_count();
                let eye = self.camera.eye();
                let lift = r * 0.004;
                for &t in &self.hover_tris {
                    let tu = t as usize;
                    if tu >= m.triangle_count() {
                        continue;
                    }
                    let is_sel = sel_ok && self.sel[tu] > 0;
                    if self.hover_is_erase != is_sel {
                        continue;
                    }
                    let [a, b, c] = m.triangle(tu);
                    let toward = |p: Vec3| p + (eye - p).normalize_or_zero() * lift;
                    push_tri(&mut fills, toward(a), toward(b), toward(c), fill_col);
                }
            }
        }
        (depth_lines, overlay_lines, fills)
    }

    // ----------------------------------------------------------------------
    // Frame output for the Bevy scene
    // ----------------------------------------------------------------------

    fn collect_frame(
        &mut self,
        ctx: &egui::Context,
        rect: egui::Rect,
        depth_lines: Vec<Line>,
        overlay_lines: Vec<Line>,
        fills: Vec<Line>,
    ) {
        let diag = self.bbox.diagonal().max(1e-3);
        let extra = diag * 1.6 + (diag * 1.5).max(10.0);
        let scene = self.bbox;
        self.camera.update_clip_planes(&scene, extra);
        self.frame = super::FrameOutput {
            viewport_rect: rect,
            pixels_per_point: ctx.pixels_per_point(),
            depth_lines,
            overlay_lines,
            fills,
            show_mesh: self.show_mesh,
            show_wireframe: self.show_wireframe,
        };
    }

    // ----------------------------------------------------------------------
    // 2D chrome drawn over the viewport
    // ----------------------------------------------------------------------

    fn draw_viewport_chrome(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        let painter = ui.painter();
        if !self.has_mesh() && self.load_job.is_none() {
            let center = rect.center();
            painter.text(
                center - egui::vec2(0.0, 14.0),
                egui::Align2::CENTER_CENTER,
                "Drop a mesh file here or press Ctrl+O",
                egui::FontId::proportional(20.0),
                theme::pal().text_muted,
            );
            painter.text(
                center + egui::vec2(0.0, 14.0),
                egui::Align2::CENTER_CENTER,
                "STL · PLY · OBJ  —  units are assumed to be millimetres",
                egui::FontId::proportional(13.0),
                theme::pal().text_faint,
            );
        }
        let hovering_file = ui.ctx().input(|i| !i.raw.hovered_files.is_empty());
        if hovering_file {
            painter.rect_filled(rect, 0.0, theme::pal().accent.gamma_multiply(0.10));
            painter.rect_stroke(
                rect.shrink(6.0),
                8.0,
                egui::Stroke::new(2.0, theme::pal().accent),
                egui::StrokeKind::Inside,
            );
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Release to open",
                egui::FontId::proportional(22.0),
                theme::pal().text_strong,
            );
        }
        if self.is_editing() {
            let label = self
                .worker
                .activities()
                .iter()
                .find(|a| {
                    Some(a.id) == self.load_job || Some(a.id) == self.edit_job.map(|(j, _)| j)
                })
                .map(|a| {
                    let stage = a.progress.stage();
                    if stage.is_empty() {
                        a.label.clone()
                    } else {
                        format!("{} — {}", a.label, stage)
                    }
                })
                .unwrap_or_else(|| "Working…".to_string());
            let pos = egui::pos2(rect.center().x, rect.bottom() - 26.0);
            let galley = painter.layout_no_wrap(
                label,
                egui::FontId::proportional(13.0),
                theme::pal().text_strong,
            );
            let w = galley.size().x + 44.0;
            let pill = egui::Rect::from_center_size(pos, egui::vec2(w, 30.0));
            painter.rect(
                pill,
                15.0,
                theme::pal().panel_elevated.gamma_multiply(0.96),
                egui::Stroke::new(1.0, theme::pal().accent.gamma_multiply(0.6)),
                egui::StrokeKind::Inside,
            );
            let t = ui.ctx().input(|i| i.time) as f32;
            let c = egui::pos2(pill.min.x + 18.0, pill.center().y);
            for k in 0..8 {
                let a = t * 4.0 + k as f32 * std::f32::consts::TAU / 8.0;
                let alpha = (k as f32) / 8.0 * 0.85 + 0.15;
                painter.circle_filled(
                    c + egui::vec2(a.cos(), a.sin()) * 6.0,
                    1.6,
                    theme::pal().accent.gamma_multiply(alpha),
                );
            }
            painter.galley(
                egui::pos2(pill.min.x + 34.0, pill.center().y - galley.size().y * 0.5),
                galley,
                theme::pal().text_strong,
            );
            ui.ctx().request_repaint();
        }
    }
}

fn push_patch(
    patch: &crate::geom::hole_fill::MeshPatch,
    fills: &mut Vec<Line>,
    overlay_lines: &mut Vec<Line>,
    fill_col: [f32; 4],
    wire_col: [f32; 4],
) {
    for chunk in patch.preview_indices.chunks_exact(3) {
        let (Some(p0), Some(p1), Some(p2)) = (
            patch.preview_positions.get(chunk[0] as usize),
            patch.preview_positions.get(chunk[1] as usize),
            patch.preview_positions.get(chunk[2] as usize),
        ) else {
            continue;
        };
        let (p0, p1, p2) = (Vec3::from(*p0), Vec3::from(*p1), Vec3::from(*p2));
        push_tri(fills, p0, p1, p2, fill_col);
        push_line(overlay_lines, p0, p1, wire_col);
        push_line(overlay_lines, p1, p2, wire_col);
        push_line(overlay_lines, p2, p0, wire_col);
    }
}
