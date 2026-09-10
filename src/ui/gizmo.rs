//! Navigation gizmo: a small axis triad in the viewport corner that shows
//! the current orientation and snaps the view when an axis is clicked.

use crate::app::App;
use crate::ui::theme;
use egui;
use glam::Vec3;

const RADIUS: f32 = 34.0;
const MARGIN: f32 = 18.0;

pub fn render_nav_gizmo(app: &mut App, ui: &mut egui::Ui, viewport: egui::Rect) {
    if !app.has_mesh() {
        return;
    }
    let center = egui::pos2(
        viewport.right() - RADIUS - MARGIN,
        viewport.bottom() - RADIUS - MARGIN,
    );
    let area_rect = egui::Rect::from_center_size(center, egui::vec2(RADIUS * 2.4, RADIUS * 2.4));
    let response = ui.interact(area_rect, ui.id().with("nav_gizmo"), egui::Sense::click());
    let hovered = response.hovered();
    let painter = ui.painter();

    // Background disc.
    painter.circle_filled(
        center,
        RADIUS + 8.0,
        theme::pal()
            .panel_elevated
            .gamma_multiply(if hovered { 0.85 } else { 0.55 }),
    );

    // Project the world axes into screen space (camera frame).
    let inv = app.camera.orient.inverse();
    let axes = [
        (Vec3::X, theme::pal().axis_x, "X"),
        (Vec3::Y, theme::pal().axis_y, "Y"),
        (Vec3::Z, theme::pal().axis_z, "Z"),
    ];
    // (depth, end position, color, label, world dir, negative?)
    let mut ends: Vec<(f32, egui::Pos2, egui::Color32, &str, Vec3, bool)> = Vec::with_capacity(6);
    for (dir, col, label) in axes {
        for sign in [1.0f32, -1.0] {
            let v = inv * (dir * sign);
            let p = center + egui::vec2(v.x, -v.y) * RADIUS;
            ends.push((v.z, p, col, label, dir * sign, sign < 0.0));
        }
    }
    // Draw far ends first.
    ends.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let pointer = ui.ctx().pointer_hover_pos();
    let mut clicked_dir: Option<Vec3> = None;
    for (depth, p, col, label, world_dir, negative) in ends {
        let fade = 0.55 + 0.45 * ((depth + 1.0) * 0.5);
        let col = col.gamma_multiply(fade);
        let near_pointer = pointer.is_some_and(|m| m.distance(p) < 9.0);
        if !negative {
            painter.line_segment([center, p], egui::Stroke::new(2.0, col));
            painter.circle_filled(p, if near_pointer { 8.5 } else { 7.0 }, col);
            painter.text(
                p,
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::proportional(10.0),
                theme::pal().bg_app,
            );
        } else {
            painter.circle_stroke(
                p,
                if near_pointer { 6.5 } else { 5.0 },
                egui::Stroke::new(1.5, col),
            );
            painter.circle_filled(p, 3.0, theme::pal().panel_elevated.gamma_multiply(0.8));
        }
        if near_pointer && response.clicked() {
            clicked_dir = Some(world_dir);
        }
    }
    if let Some(dir) = clicked_dir {
        let now = ui.ctx().input(|i| i.time);
        // Clicking the axis we already look from flips to the opposite side.
        let current = app.camera.back();
        let target = if current.dot(dir) > 0.999 { -dir } else { dir };
        app.camera.animate_look_from(target, now);
    } else if response.clicked() {
        let now = ui.ctx().input(|i| i.time);
        app.camera.animate_view(crate::camera::ViewDir::Iso, now);
    }
    if hovered {
        response.on_hover_text("Click an axis to look along it · click the disc for the iso view");
    }
}
