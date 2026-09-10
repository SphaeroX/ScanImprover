//! Tool panel sections: brush, decimation, symmetry, fitting, face groups
//! and coordinate system.

use crate::app::{App, DecMode, GroupFilter, Mode, group_matches_filter, rotation_between};
use crate::geom::freeform::{
    FreeformExtend, FreeformGradient, calculate_in_tolerance_pct, freeform_vertex_color,
};
use crate::geom::segment::{GroupKind, KIND_COLORS, group_hue_color};
use crate::ui::accordion::group_box;
use crate::ui::theme;
use egui;
use glam::Vec3;

/// Maximum triangle count for live re-segmentation while dragging sliders.
const SEG_LIVE_MAX_TRIS: usize = 600_000;

fn busy_label(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        ui.add(egui::Spinner::new().size(13.0).color(theme::pal().accent));
        ui.label(egui::RichText::new(text).color(theme::pal().accent_text));
    });
}

// ---------------------------------------------------------------------------
// Brush selection (always visible)
// ---------------------------------------------------------------------------

pub fn render_brush_selection(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("Brush selection"), |ui| {
        ui.add(egui::Slider::new(&mut app.brush_radius, 2.0..=150.0).text("Brush radius (px)"));
        ui.add(
            egui::Slider::new(&mut app.expand_angle_deg, 1.0..=180.0)
                .text("Crease angle")
                .suffix("°"),
        );
        ui.checkbox(&mut app.brush_connected, "Connected faces only")
            .on_hover_text(
                "Only paint faces that are connected to the face under the cursor. \
                 Prevents the brush from bleeding through thin walls or onto nearby surfaces.",
            );
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            if ui
                .button("Grow")
                .on_hover_text("Expand by one ring (Ctrl+Wheel up, '.')")
                .clicked()
            {
                app.grow_selection();
            }
            if ui
                .button("Shrink")
                .on_hover_text("Shrink by one ring (Ctrl+Wheel down, ',')")
                .clicked()
            {
                app.shrink_selection();
            }
            if ui.button("Invert").clicked() {
                app.invert_selection();
            }
            if ui.button("Clear").clicked() {
                app.clear_selection();
            }
        });
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(format!(
                    "{} faces selected",
                    theme::format_count(app.sel_count)
                ))
                .size(11.5)
                .color(theme::pal().text_muted),
            );
        });
    });
}

// ---------------------------------------------------------------------------
// Decimation
// ---------------------------------------------------------------------------

pub fn render_decimation(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("Mode"), |ui| {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut app.dec_mode, DecMode::Fixed, "Fixed mm");
            ui.selectable_value(&mut app.dec_mode, DecMode::Accuracy, "Auto %");
            ui.selectable_value(&mut app.dec_mode, DecMode::Deviation, "Auto mm");
        });
        ui.checkbox(&mut app.dec_lock_border, "Lock open borders");
    });

    ui.add_space(4.0);
    let mut schedule_manual = false;
    group_box(ui, Some("Parameters"), |ui| match app.dec_mode {
        DecMode::Fixed => {
            let err_resp = ui.add(
                egui::Slider::new(&mut app.dec_error_mm, 0.0001..=1.0)
                    .logarithmic(true)
                    .text("Error tolerance (mm)"),
            );
            let ratio_resp = ui.add(
                egui::Slider::new(&mut app.dec_ratio, 0.002..=1.0).text("Target triangle ratio"),
            );
            ui.checkbox(&mut app.dec_auto_preview, "Auto preview on change");
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(app.dec_job.is_none(), egui::Button::new("Preview now"))
                    .clicked()
                {
                    schedule_manual = true;
                }
                if app.dec_job.is_some() {
                    busy_label(ui, "Simplifying…");
                }
            });
            if (err_resp.drag_stopped() || ratio_resp.drag_stopped())
                && app.dec_auto_preview
                && app.dec_job.is_none()
            {
                schedule_manual = true;
            }
        }
        DecMode::Accuracy | DecMode::Deviation => {
            let diag = app.bbox.diagonal().max(1e-9);
            if app.dec_mode == DecMode::Accuracy {
                let mut acc = app.dec_target_acc;
                let resp = ui.add(
                    egui::DragValue::new(&mut acc)
                        .range(90.0..=99.9999)
                        .speed(0.02)
                        .prefix("target ≥ ")
                        .suffix(" %")
                        .fixed_decimals(3),
                );
                if resp.changed() {
                    app.dec_target_acc = acc;
                    app.dec_target_mm = ((1.0 - acc / 100.0) * diag).max(1e-6);
                }
            } else {
                let mut mm = app.dec_target_mm;
                let resp = ui.add(
                    egui::DragValue::new(&mut mm)
                        .range(0.0001..=10.0)
                        .speed(0.01)
                        .prefix("target ≤ ")
                        .suffix(" mm")
                        .fixed_decimals(4),
                );
                if resp.changed() {
                    app.dec_target_mm = mm;
                    app.dec_target_acc = 100.0 * (1.0 - mm / diag);
                }
            }
            ui.label(
                egui::RichText::new(format!(
                    "Goal: max deviation {:.4} mm = {:.4} % accuracy",
                    app.dec_target_mm, app.dec_target_acc
                ))
                .size(11.5)
                .color(theme::pal().text_muted),
            );
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(app.dec_job.is_none(), egui::Button::new("Auto decimate"))
                    .clicked()
                {
                    app.run_auto_decimate();
                }
                if app.dec_job.is_some() {
                    busy_label(ui, "Searching tolerance…");
                }
            });
        }
    });

    if app.preview.is_some() {
        ui.add_space(4.0);
        egui::Frame::new()
            .fill(theme::pal().success.gamma_multiply(0.12))
            .stroke(egui::Stroke::new(
                1.0,
                theme::pal().success.gamma_multiply(0.5),
            ))
            .corner_radius(6.0)
            .inner_margin(egui::Margin::symmetric(8, 6))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if theme::primary_button(ui, "Apply decimation").clicked() {
                        app.apply_preview();
                    }
                    if ui.button("Discard").clicked() {
                        app.discard_preview();
                    }
                });
            });
    }

    let metrics_info = app
        .orig_mesh()
        .zip(app.preview.as_ref())
        .map(|(orig, prev)| {
            (
                orig.triangle_count(),
                prev.triangle_count(),
                100.0 * prev.triangle_count() as f32 / orig.triangle_count().max(1) as f32,
            )
        });
    if let Some((orig_tris, prev_tris, pct)) = metrics_info {
        ui.add_space(4.0);
        group_box(ui, Some("Metrics & deviation"), |ui| {
            ui.label(format!(
                "Triangles: {} to {} ({:.1}%)",
                theme::format_count(orig_tris),
                theme::format_count(prev_tris),
                pct
            ));
            ui.label(format!(
                "Simplifier error estimate: {:.4} mm",
                app.preview_error
            ));
            if app.dev_job.is_some() {
                busy_label(ui, "Measuring deviation…");
            } else if let Some(dev) = &app.deviation {
                let diag = app.bbox.diagonal();
                let acc = 100.0 * (1.0 - dev.max_dev / diag.max(1e-9));
                ui.label(format!("Max deviation: {:.4} mm", dev.max_dev));
                ui.label(format!("RMS deviation: {:.4} mm", dev.rms));
                ui.label(format!("Contour match: {:.5}%", acc));
                ui.label(format!(
                    "Measured on {} source points",
                    theme::format_count(dev.count)
                ));
            }
            if ui.checkbox(&mut app.heat_on, "Deviation heatmap").changed() {
                app.aux_dirty = true;
            }
        });
    }

    if schedule_manual && app.dec_job.is_none() {
        app.schedule_decimate();
    }
}

// ---------------------------------------------------------------------------
// Symmetry
// ---------------------------------------------------------------------------

pub fn render_symmetry(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("Detection"), |ui| {
        ui.horizontal(|ui| {
            if ui
                .add_enabled(app.sym_job.is_none(), egui::Button::new("Auto-detect"))
                .clicked()
            {
                app.schedule_sym_auto();
            }
            let picking = app.mode == Mode::SymPickLine;
            if ui
                .add(egui::Button::new("Pick line").selected(picking))
                .clicked()
            {
                app.mode = if picking {
                    Mode::Orbit
                } else {
                    app.sym_pick.clear();
                    Mode::SymPickLine
                };
            }
            if app.sym_job.is_some() {
                busy_label(ui, "Searching…");
            }
        });

        if app.sym_pick.len() >= 2 {
            ui.horizontal(|ui| {
                if ui.button("Calculate").clicked() {
                    let (a, b) = (app.sym_pick[0], app.sym_pick[1]);
                    app.schedule_sym_from_line(a, b);
                    app.mode = Mode::Orbit;
                }
                if ui.button("Clear line").clicked() {
                    app.sym_pick.clear();
                    app.mode = Mode::Orbit;
                }
            });
        } else if !app.sym_pick.is_empty() {
            ui.horizontal(|ui| {
                ui.label("1 point placed…");
                if ui.button("Clear line").clicked() {
                    app.sym_pick.clear();
                    app.mode = Mode::Orbit;
                }
            });
        }
        let sel_text = format!("Exclude selection ({} faces)", app.sel_count);
        ui.checkbox(&mut app.sym_exclude_selection, sel_text);
        ui.checkbox(&mut app.sym_exclude_holes, "Exclude mesh holes");
    });

    let Some(sym) = app.sym else {
        return;
    };
    ui.add_space(4.0);
    group_box(ui, Some("Symmetry plane"), |ui| {
        let mut show = sym.show;
        if ui.checkbox(&mut show, "Show plane").changed()
            && let Some(s) = &mut app.sym
        {
            s.show = show;
        }
        if sym.rms.is_finite() {
            ui.label(format!("RMS: {:.4} mm", sym.rms));
        } else {
            ui.label("Optimizing plane…");
        }
        let mut refine = false;
        let mut to_origin = false;
        let mut align_axis = None;
        ui.horizontal(|ui| {
            if ui
                .add_enabled(app.sym_job.is_none(), egui::Button::new("Optimize"))
                .clicked()
            {
                refine = true;
            }
            if ui.button("Origin on plane").clicked() {
                to_origin = true;
            }
        });
        ui.label(
            egui::RichText::new("Align symmetry to:")
                .size(11.5)
                .color(theme::pal().text_muted),
        );
        ui.horizontal(|ui| {
            for (label, axis) in [("X = 0", Vec3::X), ("Y = 0", Vec3::Y), ("Z = 0", Vec3::Z)] {
                if ui.button(label).clicked() {
                    align_axis = Some(axis);
                }
            }
        });
        if refine {
            app.schedule_sym_refine(sym.plane);
        }
        if to_origin {
            app.origin_at_symmetry();
        }
        if let Some(axis) = align_axis {
            let q = rotation_between(sym.plane.normal, axis);
            app.align_sym_full(axis, q);
        }
        ui.add_space(3.0);
        crate::ui::alignment::render_feature_assignment_buttons(
            app,
            ui,
            crate::geom::alignment::FeatureRef::SymmetryPlane,
        );
    });
}

// ---------------------------------------------------------------------------
// Fitting (face selection section)
// ---------------------------------------------------------------------------

pub fn render_selection(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("Fit geometry"), |ui| {
        ui.horizontal(|ui| {
            let enabled = app.sel_count > 0;
            if ui
                .add_enabled(enabled, egui::Button::new("Fit plane"))
                .clicked()
            {
                app.fit_plane_from_selection();
            }
            if ui
                .add_enabled(enabled, egui::Button::new("Fit circle"))
                .clicked()
            {
                app.fit_circle_from_selection();
            }
            if ui
                .add_enabled(enabled, egui::Button::new("Fit freeform"))
                .clicked()
            {
                app.fit_freeform_from_selection();
            }
        });
    });

    ui.add_space(4.0);
    crate::ui::bridge::render_bridge_section(app, ui);

    if let Some(p) = app.plane {
        ui.add_space(4.0);
        let plane_title = app
            .selected_plane_id
            .and_then(|id| app.planes.iter().find(|p| p.id == id))
            .map(|fp| format!("Fitted plane ({})", fp.name))
            .unwrap_or_else(|| "Fitted plane".to_string());
        group_box(ui, Some(&plane_title), |ui| {
            if ui.checkbox(&mut app.show_plane, "Show plane").changed()
                && let Some(fp) = app
                    .selected_plane_id
                    .and_then(|id| app.planes.iter_mut().find(|p| p.id == id))
            {
                fp.visible = app.show_plane;
            }
            ui.label(format!(
                "N ({:.3}, {:.3}, {:.3})  ·  offset {:.3} mm",
                p.normal.x,
                p.normal.y,
                p.normal.z,
                p.point.dot(p.normal)
            ));
            ui.label(format!(
                "RMS {:.4} mm · max {:.4} mm",
                p.rms.sqrt(),
                p.max_dev
            ));
            let mut axis = None;
            let mut origin = false;
            ui.horizontal(|ui| {
                for (label, a) in [
                    ("N to X", Vec3::X),
                    ("N to Y", Vec3::Y),
                    ("N to Z", Vec3::Z),
                ] {
                    if ui.button(label).clicked() {
                        axis = Some(a);
                    }
                }
                if ui.button("Origin on plane").clicked() {
                    origin = true;
                }
            });
            if let Some(a) = axis {
                app.rotate_normal_to_axis(a);
            }
            if origin {
                app.origin_on_plane();
            }
            if ui
                .button("Export plane…")
                .on_hover_text("Export this plane for Fusion 360 / CAD (STEP, Script, DXF)")
                .clicked()
                && let Some(id) = app.selected_plane_id
            {
                app.export_plane_id(id);
            }
            if let Some(id) = app.selected_plane_id {
                ui.add_space(3.0);
                crate::ui::alignment::render_feature_assignment_buttons(
                    app,
                    ui,
                    crate::geom::alignment::FeatureRef::Plane(id),
                );
            }
        });
    }

    if let Some(c) = app.circle {
        ui.add_space(4.0);
        let circle_title = app
            .selected_circle_id
            .and_then(|id| app.circles.iter().find(|c| c.id == id))
            .map(|fc| format!("Fitted circle ({})", fc.name))
            .unwrap_or_else(|| "Fitted circle".to_string());
        group_box(ui, Some(&circle_title), |ui| {
            if ui.checkbox(&mut app.show_circle, "Show circle").changed()
                && let Some(fc) = app
                    .selected_circle_id
                    .and_then(|id| app.circles.iter_mut().find(|c| c.id == id))
            {
                fc.visible = app.show_circle;
            }
            ui.label(format!(
                "R = {:.4} mm, center ({:.2}, {:.2}, {:.2})",
                c.radius, c.center.x, c.center.y, c.center.z
            ));
            if c.cylinder {
                ui.label(
                    egui::RichText::new(
                        "Fitted as cylinder cross-section (plane perpendicular to the axis).",
                    )
                    .size(11.0)
                    .color(theme::pal().text_muted),
                );
            }
            ui.label(format!(
                "Radial RMS {:.4} mm · {} {:.4} mm · max {:.4} mm",
                c.radial_rms.sqrt(),
                if c.cylinder {
                    "axial span RMS"
                } else {
                    "plane RMS"
                },
                c.plane_rms.sqrt(),
                c.radial_max
            ));
            let mut axis = None;
            let mut origin = false;
            ui.horizontal(|ui| {
                for (label, a) in [
                    ("Axis to X", Vec3::X),
                    ("Axis to Y", Vec3::Y),
                    ("Axis to Z", Vec3::Z),
                ] {
                    if ui.button(label).clicked() {
                        axis = Some(a);
                    }
                }
                if ui.button("Origin at center").clicked() {
                    origin = true;
                }
            });
            if let Some(a) = axis {
                app.circle_axis_to(a);
            }
            if origin {
                app.origin_at_circle_center();
            }
            if ui
                .button("Export circle…")
                .on_hover_text("Export this circle for Fusion 360 / CAD (STEP, Script, DXF)")
                .clicked()
                && let Some(id) = app.selected_circle_id
            {
                app.export_circle_id(id);
            }
            if let Some(id) = app.selected_circle_id {
                ui.add_space(3.0);
                crate::ui::alignment::render_feature_assignment_buttons(
                    app,
                    ui,
                    crate::geom::alignment::FeatureRef::Circle(id),
                );
            }
        });
    }

    render_selected_freeform(app, ui);
}

fn render_selected_freeform(app: &mut App, ui: &mut egui::Ui) {
    let Some(idx) = app
        .freeforms
        .iter()
        .position(|f| app.selected_freeform_id == Some(f.id))
    else {
        return;
    };
    let title = format!("Fitted freeform ({})", app.freeforms[idx].name);
    let id = app.freeforms[idx].id;
    let fitting =
        app.freeform_job.map(|(_, fid)| fid) == Some(id) || app.freeforms[idx].refit_pending;
    let diag = app.bbox.diagonal().max(1e-6);
    let mut new_params = None;
    let mut do_export = false;
    let mut need_heat = false;
    ui.add_space(4.0);
    group_box(ui, Some(&title), |ui| {
        let f = &mut app.freeforms[idx];
        ui.checkbox(&mut f.visible, "Show surface");
        ui.checkbox(&mut f.heat_on, "Deviation heatmap");
        if f.heat_on {
            if f.heat.is_none() {
                need_heat = true;
            }
            ui.horizontal(|ui| {
                ui.label("Gradient:");
                ui.selectable_value(
                    &mut f.heat_gradient,
                    FreeformGradient::TrafficLight,
                    "Traffic light",
                );
                ui.selectable_value(&mut f.heat_gradient, FreeformGradient::Spectrum, "Spectrum");
            });
            let old_max = f.heat_max;
            ui.add(
                egui::Slider::new(&mut f.heat_max, 0.005..=1.0)
                    .text("Max dev (mm)")
                    .logarithmic(true)
                    .max_decimals(4),
            );
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Presets:").small());
                for &val in &[0.05f32, 0.10, 0.20] {
                    if ui.small_button(format!("{val:.2}")).clicked() {
                        f.heat_max = val;
                    }
                }
                if f.max_dev > 0.0
                    && ui
                        .small_button("Auto")
                        .on_hover_text("Set max deviation to fitted peak")
                        .clicked()
                {
                    f.heat_max = f.max_dev.max(0.01);
                }
            });
            if f.heat_max != old_max
                && let Some(heat) = &f.heat
            {
                f.in_tolerance_pct = calculate_in_tolerance_pct(heat, f.heat_max);
            }

            // Gradient bar legend
            let (rect, _) = ui.allocate_exact_size(
                egui::vec2(ui.available_width().min(260.0), 12.0),
                egui::Sense::hover(),
            );
            if ui.is_rect_visible(rect) {
                let painter = ui.painter();
                let n_steps = 32;
                let step_w = rect.width() / n_steps as f32;
                for s in 0..n_steps {
                    let t = s as f32 / (n_steps - 1) as f32;
                    let col =
                        freeform_vertex_color(t * f.heat_max, f.heat_max, f.heat_gradient, 1.0);
                    let sub_rect = egui::Rect::from_min_size(
                        egui::pos2(rect.min.x + s as f32 * step_w, rect.min.y),
                        egui::vec2(step_w + 0.5, rect.height()),
                    );
                    painter.rect_filled(sub_rect, 0.0, theme::color32(col));
                }
                painter.rect_stroke(
                    rect,
                    2.0,
                    egui::Stroke::new(1.0, theme::pal().border_strong),
                    egui::StrokeKind::Inside,
                );
            }
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("0.0 mm").small().weak());
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        egui::RichText::new(format!("{:.3} mm", f.heat_max))
                            .small()
                            .weak(),
                    );
                });
            });
            if f.heat.is_some() {
                ui.label(
                    egui::RichText::new(format!(
                        "{:.1}% within tolerance (≤ {:.3} mm)",
                        f.in_tolerance_pct, f.heat_max
                    ))
                    .small()
                    .color(if f.in_tolerance_pct >= 90.0 {
                        theme::pal().success
                    } else {
                        theme::pal().warn
                    }),
                );
            }
        }
        if fitting {
            busy_label(ui, "Fitting surface…");
        }
        if let Some(surf) = &f.surface {
            ui.label(format!(
                "Fit RMS {:.4} mm · max {:.4} mm · {} tris",
                f.rms,
                f.max_dev,
                theme::format_count(surf.triangle_count())
            ));
            if f.fold_ratio > 0.15 {
                ui.label(
                    egui::RichText::new(
                        "Selection wraps around: the fit is inaccurate where it folds.",
                    )
                    .small()
                    .color(theme::pal().warn),
                );
            }
        } else if !fitting {
            ui.label(
                egui::RichText::new("No surface yet (fit failed).")
                    .weak()
                    .small(),
            );
        }
        ui.add_space(3.0);
        let mut p = f.params;
        let max_ov = (diag * 0.25).max(1.0);
        let ov_resp = ui
            .add(egui::Slider::new(&mut p.overshoot_mm, 0.0..=max_ov).text("Overshoot (mm)"))
            .on_hover_text("How far the freeform surface extends beyond the selection boundary.");
        let res_resp = ui
            .add(egui::Slider::new(&mut p.resolution, 16..=320).text("Resolution"))
            .on_hover_text("Grid density of the fitted surface (cells along the longer side).");
        let sm_resp = ui
            .add(egui::Slider::new(&mut p.smoothness, 0..=10).text("Smoothness"))
            .on_hover_text(
                "Smoothing passes over the fitted grid (QuickSurface-style). \
                 0 follows the scan closely; higher values give a cleaner, \
                 CAD-friendlier surface.",
            );
        if ov_resp.changed() || res_resp.changed() || sm_resp.changed() {
            new_params = Some(p);
        }
        ui.horizontal(|ui| {
            ui.label("Extend:");
            for ext in [FreeformExtend::Slope, FreeformExtend::Curvature] {
                if ui
                    .selectable_value(&mut p.extend, ext, ext.label())
                    .clicked()
                {
                    new_params = Some(p);
                }
            }
        })
        .response
        .on_hover_text(
            "How the surface continues into the overshoot region: tangent only, \
             or following the local curvature (better for organic shapes).",
        );
        ui.add_space(3.0);
        if ui
            .button("Export freeform…")
            .on_hover_text("Export as a CAD surface (STEP B-spline, trim it in Fusion) or as a mesh (STL, OBJ, PLY)")
            .clicked()
        {
            do_export = true;
        }
    });
    if need_heat {
        app.ensure_freeform_heat(id);
    }
    if let Some(p) = new_params {
        app.set_freeform_params(id, p);
    }
    if do_export {
        app.export_freeform_id(id);
    }
}

// ---------------------------------------------------------------------------
// Face groups
// ---------------------------------------------------------------------------

pub fn render_face_groups(app: &mut App, ui: &mut egui::Ui) {
    // Reset each frame; hovered rows re-set this to highlight in the viewport.
    app.hover_group = None;

    let tris = app.display().map(|m| m.triangle_count()).unwrap_or(0);
    let live = tris <= SEG_LIVE_MAX_TRIS;
    let has_groups = !app.face_groups.is_empty();
    let mut run_detect = false;

    group_box(ui, Some("Detection"), |ui| {
        let angle_resp = ui.add(
            egui::Slider::new(&mut app.group_angle_deg, 2.0..=90.0)
                .text("Crease angle")
                .suffix("°"),
        );
        let min_resp = ui.add(
            egui::Slider::new(&mut app.group_min_tris, 1.0..=5000.0)
                .text("Min faces")
                .logarithmic(true),
        );
        let mut tol_pct = app.group_fit_tol * 100.0;
        let tol_resp = ui.add(
            egui::Slider::new(&mut tol_pct, 0.02..=2.0)
                .text("Fit tolerance")
                .suffix("% of size"),
        );
        if tol_resp.changed() {
            app.group_fit_tol = tol_pct / 100.0;
        }
        // Re-run live while dragging (small meshes) or on slider release (big meshes).
        let rerun =
            |resp: &egui::Response| has_groups && resp.changed() && (live || resp.drag_stopped());
        if rerun(&angle_resp) || rerun(&min_resp) || rerun(&tol_resp) {
            run_detect = true;
        }
        ui.horizontal(|ui| {
            let label = if has_groups {
                "Re-detect"
            } else {
                "Detect groups"
            };
            if ui
                .add_enabled(app.has_mesh(), egui::Button::new(label))
                .clicked()
            {
                run_detect = true;
            }
            if has_groups && ui.button("Clear").clicked() {
                app.clear_face_groups();
            }
            if app.groups_job.is_some() {
                busy_label(ui, "Segmenting…");
            }
        });
        if has_groups && !live {
            ui.label(
                egui::RichText::new("Large mesh: colors update when the slider is released")
                    .weak()
                    .small(),
            );
        }
    });

    if run_detect {
        app.request_face_groups();
    }
    if app.face_groups.is_empty() {
        return;
    }

    let planes = app
        .face_groups
        .iter()
        .filter(|g| g.kind == GroupKind::Plane)
        .count();
    let cylinders = app
        .face_groups
        .iter()
        .filter(|g| g.kind == GroupKind::Cylinder)
        .count();
    let spheres = app
        .face_groups
        .iter()
        .filter(|g| g.kind == GroupKind::Sphere)
        .count();
    let other = app.face_groups.len() - planes - cylinders - spheres;

    ui.add_space(4.0);
    group_box(ui, Some("Coloring"), |ui| {
        if ui
            .checkbox(&mut app.groups_show, "Show group colors")
            .changed()
        {
            app.aux_dirty = true;
        }
        let before = app.groups_by_type;
        ui.horizontal(|ui| {
            ui.selectable_value(&mut app.groups_by_type, false, "Distinct");
            ui.selectable_value(&mut app.groups_by_type, true, "By type");
        });
        if app.groups_by_type != before {
            app.aux_dirty = true;
        }
        if app.groups_by_type {
            ui.horizontal(|ui| {
                for (kind, label) in [
                    (GroupKind::Plane, "Plane"),
                    (GroupKind::Cylinder, "Cylinder"),
                    (GroupKind::Sphere, "Sphere"),
                    (GroupKind::Freeform, "Freeform"),
                ] {
                    ui.label(
                        egui::RichText::new("■")
                            .color(kind_color32(kind))
                            .size(12.0),
                    );
                    ui.label(egui::RichText::new(label).small());
                    ui.add_space(4.0);
                }
            });
        }
        ui.label(
            egui::RichText::new(format!(
                "{planes} planes · {cylinders} cylinders · {spheres} spheres · {other} other"
            ))
            .size(11.5)
            .color(theme::pal().text_muted),
        );
    });

    ui.add_space(4.0);
    ui.checkbox(&mut app.group_sel_additive, "Keep existing selection")
        .on_hover_text(
            "When checked, double-clicking a group (or pressing Sel) adds its faces \
             to the current selection. When unchecked, the selection is replaced.",
        );

    ui.add_space(4.0);
    group_box(
        ui,
        Some(&format!("Groups ({})", app.face_groups.len())),
        |ui| {
            let before = app.groups_filter;
            ui.horizontal(|ui| {
                ui.selectable_value(&mut app.groups_filter, GroupFilter::All, "All");
                ui.selectable_value(&mut app.groups_filter, GroupFilter::Plane, "Planes");
                ui.selectable_value(&mut app.groups_filter, GroupFilter::Cylinder, "Cyls");
                ui.selectable_value(&mut app.groups_filter, GroupFilter::Sphere, "Spheres");
                ui.selectable_value(&mut app.groups_filter, GroupFilter::Freeform, "Freeform");
            });
            if app.groups_filter != before {
                app.aux_dirty = true;
            }
            ui.label(
            egui::RichText::new("Filter also grays out non-matching groups in the viewport · hover a row to highlight")
                .weak()
                .small(),
        );
            egui::ScrollArea::vertical()
                .max_height(220.0)
                .show(ui, |ui| {
                    for i in 0..app.face_groups.len() {
                        let (id, kind, n) = {
                            let g = &app.face_groups[i];
                            (g.id, g.kind, g.tris.len())
                        };
                        if !group_matches_filter(kind, app.groups_filter) {
                            continue;
                        }
                        let dot = if app.groups_by_type {
                            kind_color32(kind)
                        } else {
                            hue_color32(id)
                        };
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("■").color(dot).size(13.0));
                            let title = format!("{} · {:>6} faces", kind.label(), n);
                            let sel = app.selected_group == Some(id);
                            let text = egui::RichText::new(title);
                            let text = if sel { text.strong() } else { text };
                            let resp = ui.selectable_label(sel, text);
                            if resp.hovered() {
                                app.hover_group = Some(id);
                            }
                            if resp.clicked() {
                                app.selected_group = Some(id);
                            }
                            if ui.small_button("Sel").clicked() {
                                app.select_group_faces(id, app.group_sel_additive);
                            }
                        });
                    }
                });
        },
    );

    let Some(id) = app.selected_group else {
        return;
    };
    let Some((kind, n, area, rms, normal, point, radius)) =
        app.face_groups.iter().find(|g| g.id == id).map(|g| {
            (
                g.kind,
                g.tris.len(),
                g.area,
                g.rms,
                g.normal,
                g.point,
                g.radius,
            )
        })
    else {
        return;
    };
    ui.add_space(4.0);
    group_box(
        ui,
        Some(&format!("Group {} · {}", id + 1, kind.label())),
        |ui| {
            ui.label(format!(
                "{} faces · {:.1} mm² · fit RMS {:.4} mm",
                theme::format_count(n),
                area,
                rms
            ));
            match kind {
                GroupKind::Plane => {
                    ui.label(format!(
                        "Normal ({:.3}, {:.3}, {:.3}) · offset {:.3} mm",
                        normal.x,
                        normal.y,
                        normal.z,
                        point.dot(normal)
                    ));
                    ui.horizontal(|ui| {
                        if ui.button("Select faces").clicked() {
                            app.select_group_faces(id, app.group_sel_additive);
                        }
                        if ui.button("Fit plane to objects").clicked() {
                            app.fit_group_plane(id);
                        }
                    });
                    ui.horizontal(|ui| {
                        for (label, a) in [
                            ("N to X", Vec3::X),
                            ("N to Y", Vec3::Y),
                            ("N to Z", Vec3::Z),
                        ] {
                            if ui.button(label).clicked() {
                                app.align_group_axis(id, a);
                            }
                        }
                        if ui.button("Origin on plane").clicked() {
                            app.origin_on_group_plane(id);
                        }
                    });
                }
                GroupKind::Cylinder => {
                    ui.label(format!(
                        "Axis ({:.3}, {:.3}, {:.3}) · R = {:.4} mm",
                        normal.x, normal.y, normal.z, radius
                    ));
                    if ui.button("Select faces").clicked() {
                        app.select_group_faces(id, app.group_sel_additive);
                    }
                    ui.horizontal(|ui| {
                        for (label, a) in [
                            ("Axis to X", Vec3::X),
                            ("Axis to Y", Vec3::Y),
                            ("Axis to Z", Vec3::Z),
                        ] {
                            if ui.button(label).clicked() {
                                app.align_group_axis(id, a);
                            }
                        }
                        if ui.button("Origin on axis").clicked() {
                            app.origin_on_group_axis(id);
                        }
                    });
                }
                GroupKind::Sphere => {
                    ui.label(format!(
                        "Center ({:.3}, {:.3}, {:.3}) · R = {:.4} mm",
                        point.x, point.y, point.z, radius
                    ));
                    if ui.button("Select faces").clicked() {
                        app.select_group_faces(id, app.group_sel_additive);
                    }
                }
                GroupKind::Freeform => {
                    ui.horizontal(|ui| {
                        if ui.button("Select faces").clicked() {
                            app.select_group_faces(id, app.group_sel_additive);
                        }
                        if ui.button("Fit freeform to objects").clicked() {
                            app.fit_group_freeform(id);
                        }
                    });
                }
            }
        },
    );
}

fn kind_color32(kind: GroupKind) -> egui::Color32 {
    let c = KIND_COLORS[kind as usize];
    theme::color32([c[0], c[1], c[2], 1.0])
}

fn hue_color32(id: i32) -> egui::Color32 {
    let c = group_hue_color(id);
    theme::color32([c[0], c[1], c[2], 1.0])
}

// ---------------------------------------------------------------------------
// Coordinate system
// ---------------------------------------------------------------------------

pub fn render_coordinates(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("Feature alignment (quick surface)"), |ui| {
        crate::ui::alignment::render_feature_alignment_section(app, ui);
    });

    ui.add_space(4.0);
    group_box(ui, Some("Center on axes"), |ui| {
        ui.horizontal(|ui| {
            if ui.button("Center X").clicked() {
                app.center_axes(true, false, false);
            }
            if ui.button("Center Y").clicked() {
                app.center_axes(false, true, false);
            }
            if ui.button("Center Z").clicked() {
                app.center_axes(false, false, true);
            }
            if ui.button("Center all").clicked() {
                app.center_axes(true, true, true);
            }
        });
    });

    ui.add_space(4.0);
    group_box(ui, Some("History & reset"), |ui| {
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!app.undo.is_empty(), egui::Button::new("Undo"))
                .clicked()
            {
                app.undo();
            }
            if ui
                .add_enabled(!app.redo.is_empty(), egui::Button::new("Redo"))
                .clicked()
            {
                app.redo();
            }
            if ui
                .add_enabled(app.has_mesh(), egui::Button::new("Reset mesh"))
                .clicked()
            {
                app.reset_mesh();
            }
        });
    });
}
