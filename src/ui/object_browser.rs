//! Floating object browser listing the mesh, hidden regions and fitted
//! reference features with their per-object actions.

use crate::app::App;
use crate::geom::alignment::{AxisChoice, FeatureRef, OriginRef};
use crate::ui::alignment::{
    FeatureAssignmentAction, render_feature_assignment_buttons_ui, render_feature_badge_ui,
};
use crate::ui::theme;
use egui;
use glam::Vec3;

enum BrowserAction {
    ToggleMeshVisibility,
    HideSelection,
    ToggleHiddenRegionVisibility(u64),
    RestoreHiddenRegion(u64),
    RestoreAllHiddenRegions,
    DeleteHiddenRegion(u64),
    TogglePlaneVisibility(u64),
    SelectPlane(u64),
    DeletePlane(u64),
    AlignPlane(u64, Vec3),
    OriginOnPlane(u64),
    ToggleCircleVisibility(u64),
    SelectCircle(u64),
    DeleteCircle(u64),
    AlignCircle(u64, Vec3),
    OriginAtCircle(u64),
    ToggleFreeformVisibility(u64),
    SelectFreeform(u64),
    DeleteFreeform(u64),
    FitPlane,
    FitCircle,
    FitFreeform,
    ToggleSymmetryVisibility,
    ExportPlane(u64),
    ExportCircle(u64),
    ExportFreeform(u64),
    ExportAllReferences,
    AlignToFeatures,
    ToggleAssign(FeatureRef, AxisChoice),
    ToggleOrigin(FeatureRef),
}

/// Header line of one object row: visibility checkbox, color dot, name,
/// badges and a delete button. Returns (name clicked, visibility toggled,
/// delete clicked).
#[allow(clippy::too_many_arguments)]
fn object_row_header(
    ui: &mut egui::Ui,
    visible: bool,
    color: Option<egui::Color32>,
    name: &str,
    selected: bool,
    cur_axis: Option<AxisChoice>,
    is_orig: bool,
    delete_tip: &str,
    trailing: Option<String>,
) -> (bool, bool, bool) {
    let mut name_clicked = false;
    let mut vis_toggled = false;
    let mut delete_clicked = false;
    ui.horizontal(|ui| {
        let mut vis = visible;
        if ui.checkbox(&mut vis, "").changed() {
            vis_toggled = true;
        }
        if let Some(col) = color {
            let (dot_rect, _) =
                ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::empty());
            ui.painter().circle_filled(dot_rect.center(), 4.0, col);
        }
        let text = egui::RichText::new(name)
            .size(12.0)
            .strong()
            .color(if visible {
                theme::pal().text_strong
            } else {
                theme::pal().text_muted
            });
        if ui.selectable_label(selected, text).clicked() {
            name_clicked = true;
        }
        render_feature_badge_ui(ui, cur_axis, is_orig);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .small_button(
                    egui::RichText::new("×")
                        .size(11.0)
                        .color(theme::pal().danger),
                )
                .on_hover_text(delete_tip)
                .clicked()
            {
                delete_clicked = true;
            }
            if let Some(t) = trailing {
                ui.label(
                    egui::RichText::new(t)
                        .size(10.5)
                        .color(theme::pal().text_muted),
                );
            }
        });
    });
    (name_clicked, vis_toggled, delete_clicked)
}

fn row_frame(selected: bool) -> egui::Frame {
    egui::Frame::new()
        .fill(if selected {
            theme::pal().accent.gamma_multiply(0.16)
        } else {
            theme::pal().panel_hover.gamma_multiply(0.45)
        })
        .stroke(if selected {
            egui::Stroke::new(1.0, theme::pal().accent.gamma_multiply(0.55))
        } else {
            egui::Stroke::NONE
        })
        .corner_radius(5.0)
        .inner_margin(egui::Margin::symmetric(6, 5))
}

fn section_caption(ui: &mut egui::Ui, text: &str) {
    ui.add_space(6.0);
    theme::caption(ui, text);
}

fn detail_text(ui: &mut egui::Ui, text: String) {
    ui.label(
        egui::RichText::new(text)
            .size(10.5)
            .color(theme::pal().text_muted),
    );
}

fn axis_buttons(ui: &mut egui::Ui, prefix: &str) -> Option<Vec3> {
    let mut out = None;
    for (label, a) in [("X", Vec3::X), ("Y", Vec3::Y), ("Z", Vec3::Z)] {
        if ui.small_button(format!("{prefix} to {label}")).clicked() {
            out = Some(a);
        }
    }
    out
}

/// Renders the object browser content (hosted in the right side panel).
pub fn render_object_browser(app: &mut App, ui: &mut egui::Ui) {
    let align_slots = app.align_slots.clone();
    let mut actions: Vec<BrowserAction> = Vec::new();
    let total_count = usize::from(app.has_mesh())
        + app.hidden_regions.len()
        + app.planes.len()
        + app.circles.len()
        + app.freeforms.len()
        + usize::from(app.sym.is_some());

    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("Objects ({total_count})"))
                .strong()
                .size(13.5)
                .color(theme::pal().text_strong),
        );
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .small_button(
                    egui::RichText::new("×")
                        .size(11.0)
                        .color(theme::pal().text_muted),
                )
                .on_hover_text("Hide the object browser (toolbar: Objects)")
                .clicked()
            {
                app.show_object_browser = false;
            }
        });
    });
    ui.add_space(4.0);
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 3.0;
            {
                {
                    // Quick actions for the selection
                    if app.sel_count > 0 {
                        egui::Frame::new()
                            .fill(theme::pal().select_accent.gamma_multiply(0.10))
                            .stroke(egui::Stroke::new(1.0, theme::pal().select_accent.gamma_multiply(0.35)))
                            .corner_radius(5.0)
                            .inner_margin(egui::Margin::symmetric(6, 4))
                            .show(ui, |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    ui.label(egui::RichText::new("Selection:").size(11.0).color(theme::pal().select_accent));
                                    if ui.small_button("+ Fit plane").clicked() {
                                        actions.push(BrowserAction::FitPlane);
                                    }
                                    if ui.small_button("+ Fit circle").clicked() {
                                        actions.push(BrowserAction::FitCircle);
                                    }
                                    if ui.small_button("+ Fit freeform").clicked() {
                                        actions.push(BrowserAction::FitFreeform);
                                    }
                                    if ui.small_button("Hide").on_hover_text("Hide selection (H)").clicked() {
                                        actions.push(BrowserAction::HideSelection);
                                    }
                                });
                            });
                        ui.add_space(4.0);
                    }

                    // --- Mesh ---------------------------------------------
                    theme::caption(ui, "Mesh");
                    if let Some(m) = app.display() {
                        let name = app
                            .file_path
                            .as_ref()
                            .and_then(|p| p.file_name())
                            .and_then(|s| s.to_str())
                            .unwrap_or("Mesh")
                            .to_string();
                        let tris = m.triangle_count();
                        let verts = m.vertex_count();
                        row_frame(false).show(ui, |ui| {
                            ui.horizontal(|ui| {
                                let mut vis = app.show_mesh;
                                if ui.checkbox(&mut vis, "").changed() {
                                    actions.push(BrowserAction::ToggleMeshVisibility);
                                }
                                                                ui.label(egui::RichText::new(name).strong().size(12.0).color(theme::pal().text_strong));
                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                    ui.label(
                                        egui::RichText::new(format!("{} tris", theme::format_count(tris)))
                                            .size(10.5)
                                            .color(theme::pal().text_muted),
                                    );
                                });
                            });
                            ui.horizontal(|ui| {
                                ui.add_space(24.0);
                                detail_text(ui, format!("{} verts", theme::format_count(verts)));
                            });
                        });
                    } else {
                        ui.label(egui::RichText::new("No mesh loaded").italics().size(11.0).color(theme::pal().text_faint));
                    }

                    // --- Hidden regions -----------------------------------
                    if !app.hidden_regions.is_empty() {
                        ui.horizontal(|ui| {
                            section_caption(ui, &format!("Hidden regions ({})", app.hidden_regions.len()));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui
                                    .small_button(egui::RichText::new("Unhide all").size(10.0))
                                    .on_hover_text("Restore all hidden regions back into the mesh")
                                    .clicked()
                                {
                                    actions.push(BrowserAction::RestoreAllHiddenRegions);
                                }
                            });
                        });
                        for hr in &app.hidden_regions {
                            row_frame(false).show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    let mut vis = hr.visible;
                                    if ui.checkbox(&mut vis, "").on_hover_text("Toggle visibility").changed() {
                                        actions.push(BrowserAction::ToggleHiddenRegionVisibility(hr.id));
                                    }
                                    let text_color = if hr.visible { theme::pal().text_strong } else { theme::pal().text_muted };
                                    ui.label(egui::RichText::new(&hr.name).strong().size(11.5).color(text_color));
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        if ui
                                            .small_button(egui::RichText::new("×").size(11.0).color(theme::pal().danger))
                                            .on_hover_text("Delete these faces permanently")
                                            .clicked()
                                        {
                                            actions.push(BrowserAction::DeleteHiddenRegion(hr.id));
                                        }
                                        if ui
                                            .small_button(egui::RichText::new("Restore").size(10.0).color(theme::pal().accent_text))
                                            .on_hover_text("Restore back into the mesh permanently")
                                            .clicked()
                                        {
                                            actions.push(BrowserAction::RestoreHiddenRegion(hr.id));
                                        }
                                        ui.label(
                                            egui::RichText::new(format!("{} tris", theme::format_count(hr.triangle_count())))
                                                .size(10.5)
                                                .color(theme::pal().text_muted),
                                        );
                                    });
                                });
                            });
                        }
                    }

                    // --- Planes -------------------------------------------
                    section_caption(ui, &format!("Fitted planes ({})", app.planes.len()));
                    if app.planes.is_empty() {
                        ui.label(egui::RichText::new("No planes fitted yet").italics().size(11.0).color(theme::pal().text_faint));
                    }
                    for p in &app.planes {
                        let is_sel = app.selected_plane_id == Some(p.id);
                        let feat = FeatureRef::Plane(p.id);
                        let cur_axis = align_slots.axis_of(feat);
                        let is_orig = matches!(align_slots.origin, OriginRef::Plane(id) if id == p.id);
                        row_frame(is_sel).show(ui, |ui| {
                            let (clicked, toggled, deleted) = object_row_header(
                                ui,
                                p.visible,
                                Some(theme::color32(p.color)),
                                &p.name,
                                is_sel,
                                cur_axis,
                                is_orig,
                                "Delete plane",
                                None,
                            );
                            if clicked {
                                actions.push(BrowserAction::SelectPlane(p.id));
                            }
                            if toggled {
                                actions.push(BrowserAction::TogglePlaneVisibility(p.id));
                            }
                            if deleted {
                                actions.push(BrowserAction::DeletePlane(p.id));
                            }
                            if is_sel {
                                let n = p.fit.normal;
                                ui.horizontal(|ui| {
                                    ui.add_space(20.0);
                                    ui.vertical(|ui| {
                                        detail_text(
                                            ui,
                                            format!(
                                                "N: ({:.2}, {:.2}, {:.2})  off: {:.2} mm",
                                                n.x,
                                                n.y,
                                                n.z,
                                                p.fit.point.dot(n)
                                            ),
                                        );
                                        detail_text(ui, format!("RMS: {:.4} mm · max: {:.4} mm", p.fit.rms.sqrt(), p.fit.max_dev));
                                        ui.horizontal_wrapped(|ui| {
                                            if let Some(a) = axis_buttons(ui, "N") {
                                                actions.push(BrowserAction::AlignPlane(p.id, a));
                                            }
                                            if ui.small_button("Origin on plane").clicked() {
                                                actions.push(BrowserAction::OriginOnPlane(p.id));
                                            }
                                            if ui
                                                .small_button("Export…")
                                                .on_hover_text("Export this plane for Fusion 360 / CAD (STEP, Script, DXF)")
                                                .clicked()
                                            {
                                                actions.push(BrowserAction::ExportPlane(p.id));
                                            }
                                        });
                                        if let Some(action) = render_feature_assignment_buttons_ui(ui, feat, cur_axis, is_orig) {
                                            actions.push(match action {
                                                FeatureAssignmentAction::Assign(ax) => BrowserAction::ToggleAssign(feat, ax),
                                                FeatureAssignmentAction::ToggleOrigin => BrowserAction::ToggleOrigin(feat),
                                            });
                                        }
                                    });
                                });
                            }
                        });
                    }

                    // --- Circles ------------------------------------------
                    section_caption(ui, &format!("Fitted circles ({})", app.circles.len()));
                    if app.circles.is_empty() {
                        ui.label(egui::RichText::new("No circles fitted yet").italics().size(11.0).color(theme::pal().text_faint));
                    }
                    for c in &app.circles {
                        let is_sel = app.selected_circle_id == Some(c.id);
                        let feat = FeatureRef::Circle(c.id);
                        let cur_axis = align_slots.axis_of(feat);
                        let is_orig = matches!(align_slots.origin, OriginRef::CircleCenter(id) if id == c.id);
                        row_frame(is_sel).show(ui, |ui| {
                            let (clicked, toggled, deleted) = object_row_header(
                                ui,
                                c.visible,
                                Some(theme::color32(c.color)),
                                &c.name,
                                is_sel,
                                cur_axis,
                                is_orig,
                                "Delete circle",
                                None,
                            );
                            if clicked {
                                actions.push(BrowserAction::SelectCircle(c.id));
                            }
                            if toggled {
                                actions.push(BrowserAction::ToggleCircleVisibility(c.id));
                            }
                            if deleted {
                                actions.push(BrowserAction::DeleteCircle(c.id));
                            }
                            if is_sel {
                                let ce = c.fit.center;
                                ui.horizontal(|ui| {
                                    ui.add_space(20.0);
                                    ui.vertical(|ui| {
                                        detail_text(
                                            ui,
                                            format!("R: {:.3} mm  center: ({:.1}, {:.1}, {:.1})", c.fit.radius, ce.x, ce.y, ce.z),
                                        );
                                        detail_text(ui, format!("Radial RMS: {:.4} mm", c.fit.radial_rms.sqrt()));
                                        ui.horizontal_wrapped(|ui| {
                                            if let Some(a) = axis_buttons(ui, "Axis") {
                                                actions.push(BrowserAction::AlignCircle(c.id, a));
                                            }
                                            if ui.small_button("Center at origin").clicked() {
                                                actions.push(BrowserAction::OriginAtCircle(c.id));
                                            }
                                            if ui
                                                .small_button("Export…")
                                                .on_hover_text("Export this circle for Fusion 360 / CAD (STEP, Script, DXF)")
                                                .clicked()
                                            {
                                                actions.push(BrowserAction::ExportCircle(c.id));
                                            }
                                        });
                                        if let Some(action) = render_feature_assignment_buttons_ui(ui, feat, cur_axis, is_orig) {
                                            actions.push(match action {
                                                FeatureAssignmentAction::Assign(ax) => BrowserAction::ToggleAssign(feat, ax),
                                                FeatureAssignmentAction::ToggleOrigin => BrowserAction::ToggleOrigin(feat),
                                            });
                                        }
                                    });
                                });
                            }
                        });
                    }

                    // --- Freeforms ----------------------------------------
                    section_caption(ui, &format!("Fitted freeforms ({})", app.freeforms.len()));
                    if app.freeforms.is_empty() {
                        ui.label(
                            egui::RichText::new("No freeform surfaces fitted yet")
                                .italics()
                                .size(11.0)
                                .color(theme::pal().text_faint),
                        );
                    }
                    for f in &app.freeforms {
                        let is_sel = app.selected_freeform_id == Some(f.id);
                        let fitting = app.freeform_job.map(|(_, fid)| fid) == Some(f.id) || f.refit_pending;
                        row_frame(is_sel).show(ui, |ui| {
                            let (clicked, toggled, deleted) = object_row_header(
                                ui,
                                f.visible,
                                Some(theme::color32(f.color)),
                                &f.name,
                                is_sel,
                                None,
                                false,
                                "Delete freeform surface",
                                fitting.then(|| "fitting…".to_string()),
                            );
                            if clicked {
                                actions.push(BrowserAction::SelectFreeform(f.id));
                            }
                            if toggled {
                                actions.push(BrowserAction::ToggleFreeformVisibility(f.id));
                            }
                            if deleted {
                                actions.push(BrowserAction::DeleteFreeform(f.id));
                            }
                            if is_sel {
                                ui.horizontal(|ui| {
                                    ui.add_space(20.0);
                                    ui.vertical(|ui| {
                                        if fitting {
                                            ui.horizontal(|ui| {
                                                ui.add(egui::Spinner::new().size(11.0).color(theme::pal().accent));
                                                detail_text(ui, "Fitting surface…".to_string());
                                            });
                                        } else if let Some(surf) = &f.surface {
                                            detail_text(
                                                ui,
                                                format!(
                                                    "RMS: {:.4} mm · overshoot: {:.2} mm · {} tris",
                                                    f.rms,
                                                    f.params.overshoot_mm,
                                                    theme::format_count(surf.triangle_count())
                                                ),
                                            );
                                        }
                                        if ui
                                            .small_button("Export…")
                                            .on_hover_text("Export as STEP CAD surface (B-spline) or mesh (STL, OBJ, PLY)")
                                            .clicked()
                                        {
                                            actions.push(BrowserAction::ExportFreeform(f.id));
                                        }
                                    });
                                });
                            }
                        });
                    }

                    // --- Symmetry plane -----------------------------------
                    if let Some(sym) = app.sym {
                        let feat = FeatureRef::SymmetryPlane;
                        let cur_axis = align_slots.axis_of(feat);
                        let is_orig = matches!(align_slots.origin, OriginRef::SymmetryPlane);
                        section_caption(ui, "Symmetry plane");
                        row_frame(false).show(ui, |ui| {
                            ui.horizontal(|ui| {
                                let mut show = sym.show;
                                if ui.checkbox(&mut show, "").changed() {
                                    actions.push(BrowserAction::ToggleSymmetryVisibility);
                                }
                                ui.label(
                                    egui::RichText::new("Symmetry plane")
                                        .size(12.0)
                                        .strong()
                                        .color(egui::Color32::from_rgb(60, 220, 240)),
                                );
                                render_feature_badge_ui(ui, cur_axis, is_orig);
                            });
                            if let Some(action) = render_feature_assignment_buttons_ui(ui, feat, cur_axis, is_orig) {
                                actions.push(match action {
                                    FeatureAssignmentAction::Assign(ax) => BrowserAction::ToggleAssign(feat, ax),
                                    FeatureAssignmentAction::ToggleOrigin => BrowserAction::ToggleOrigin(feat),
                                });
                            }
                        });
                    }

                    if align_slots.any_axis_assigned() {
                        ui.add_space(8.0);
                        ui.vertical_centered(|ui| {
                            if theme::primary_button(ui, "Align to features")
                                .on_hover_text("Align scan coordinates based on assigned features")
                                .clicked()
                            {
                                actions.push(BrowserAction::AlignToFeatures);
                            }
                        });
                    }
                    if !app.planes.is_empty() || !app.circles.is_empty() {
                        ui.add_space(6.0);
                        ui.vertical_centered(|ui| {
                            if ui
                                .button(egui::RichText::new("Export all references…").size(11.5).color(theme::pal().accent_text))
                                .on_hover_text("Export all visible planes and circles into a single CAD file (STEP, Script, DXF)")
                                .clicked()
                            {
                                actions.push(BrowserAction::ExportAllReferences);
                            }
                        });
                    }
                }
            }
        });

    for action in actions {
        match action {
            BrowserAction::ToggleMeshVisibility => app.show_mesh = !app.show_mesh,
            BrowserAction::TogglePlaneVisibility(id) => {
                if let Some(p) = app.planes.iter_mut().find(|p| p.id == id) {
                    p.visible = !p.visible;
                    if app.selected_plane_id == Some(id) {
                        app.show_plane = p.visible;
                    }
                }
            }
            BrowserAction::SelectPlane(id) => app.select_plane(id),
            BrowserAction::DeletePlane(id) => app.delete_plane(id),
            BrowserAction::AlignPlane(id, axis) => app.rotate_plane_normal_to_axis(id, axis),
            BrowserAction::OriginOnPlane(id) => app.origin_on_plane_id(id),
            BrowserAction::ToggleCircleVisibility(id) => {
                if let Some(c) = app.circles.iter_mut().find(|c| c.id == id) {
                    c.visible = !c.visible;
                    if app.selected_circle_id == Some(id) {
                        app.show_circle = c.visible;
                    }
                }
            }
            BrowserAction::SelectCircle(id) => app.select_circle(id),
            BrowserAction::DeleteCircle(id) => app.delete_circle(id),
            BrowserAction::AlignCircle(id, axis) => app.rotate_circle_axis_to(id, axis),
            BrowserAction::OriginAtCircle(id) => app.origin_at_circle_id(id),
            BrowserAction::FitPlane => app.fit_plane_from_selection(),
            BrowserAction::FitCircle => app.fit_circle_from_selection(),
            BrowserAction::FitFreeform => app.fit_freeform_from_selection(),
            BrowserAction::ToggleFreeformVisibility(id) => {
                if let Some(f) = app.freeforms.iter_mut().find(|f| f.id == id) {
                    f.visible = !f.visible;
                }
            }
            BrowserAction::SelectFreeform(id) => app.select_freeform(id),
            BrowserAction::DeleteFreeform(id) => app.delete_freeform(id),
            BrowserAction::ExportFreeform(id) => app.export_freeform_id(id),
            BrowserAction::ToggleSymmetryVisibility => {
                if let Some(sym) = &mut app.sym {
                    sym.show = !sym.show;
                }
            }
            BrowserAction::ExportPlane(id) => app.export_plane_id(id),
            BrowserAction::ExportCircle(id) => app.export_circle_id(id),
            BrowserAction::ExportAllReferences => app.export_all_references(),
            BrowserAction::AlignToFeatures => {
                app.align_to_features();
            }
            BrowserAction::ToggleAssign(feat, axis) => app.toggle_assign_feature(feat, axis),
            BrowserAction::ToggleOrigin(feat) => app.toggle_origin_feature(feat),
            BrowserAction::HideSelection => app.hide_selection(),
            BrowserAction::ToggleHiddenRegionVisibility(id) => {
                app.toggle_hidden_region_visibility(id)
            }
            BrowserAction::RestoreHiddenRegion(id) => app.restore_hidden_region(id),
            BrowserAction::RestoreAllHiddenRegions => app.restore_all_hidden_regions(),
            BrowserAction::DeleteHiddenRegion(id) => app.delete_hidden_region(id),
        }
    }
}
