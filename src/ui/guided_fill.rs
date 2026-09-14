//! Experimental tool: hole filling guided by the face groups.
//!
//! The face group sliders edit the same settings as the Face groups section
//! and the hole list is the one of the Mesh repair section.

use crate::app::App;
use crate::geom::segment::GroupKind;
use crate::ui::theme;

/// True when a slider edit is finished (drag released or keyboard input):
/// the face groups are re-detected then instead of on every drag step.
fn edit_finished(resp: &egui::Response) -> bool {
    resp.drag_stopped() || (resp.changed() && !resp.dragged())
}

fn heading(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).strong().size(11.5));
}

pub fn render_guided_fill(app: &mut App, ui: &mut egui::Ui) {
    // The hole list is shared with the Mesh repair section; analyse lazily.
    if app.repair_health.is_none() && app.has_mesh() && app.analysis_job.is_none() {
        app.request_repair_analysis();
    }
    let editing = app.is_editing();
    ui.label(
        egui::RichText::new(
            "Extends the planes, cylinders and spheres of the face groups around a hole \
             into the gap and rebuilds the sharp edges where they meet. Visible fitted \
             circles act as cylinders along round profiles the groups miss. Freeform parts \
             use the standard fill of Mesh repair.",
        )
        .size(11.0)
        .color(theme::pal().text_muted),
    );
    ui.add_space(4.0);

    // 1. Face groups (same settings as the Face groups section).
    heading(ui, "Face groups");
    let has_groups = app.face_groups_valid();
    let angle = ui
        .add(
            egui::Slider::new(&mut app.group_angle_deg, 2.0..=90.0)
                .text("Crease angle")
                .suffix("°"),
        )
        .on_hover_text(
            "Edges bending by more than this separate the groups and are rebuilt sharp.",
        );
    let mut feature_pct = app.group_feature_frac * 100.0;
    let feature = ui
        .add(
            egui::Slider::new(&mut feature_pct, 0.0..=15.0)
                .text("Feature size")
                .suffix("% of size"),
        )
        .on_hover_text(
            "Surfaces that bend by more than the crease angle within this distance \
             (fillets, small features) separate the groups.",
        );
    if feature.changed() {
        app.group_feature_frac = feature_pct / 100.0;
    }
    let mut tol_pct = app.group_fit_tol * 100.0;
    let tol = ui
        .add(
            egui::Slider::new(&mut tol_pct, 0.02..=2.0)
                .text("Fit tolerance")
                .suffix("% of size"),
        )
        .on_hover_text(
            "How far a group may deviate from its plane / cylinder / sphere. A group \
             missing the hole rim by more is filled like freeform.",
        );
    if tol.changed() {
        app.group_fit_tol = tol_pct / 100.0;
    }
    let min = ui.add(
        egui::Slider::new(&mut app.group_min_tris, 1.0..=5000.0)
            .text("Min faces")
            .logarithmic(true),
    );
    let mut run_detect = has_groups
        && [&angle, &feature, &tol, &min]
            .into_iter()
            .any(edit_finished);
    ui.horizontal(|ui| {
        let label = if has_groups {
            "Re-detect"
        } else {
            "Detect face groups"
        };
        if ui
            .add_enabled(app.has_mesh() && !editing, egui::Button::new(label))
            .clicked()
        {
            run_detect = true;
        }
        if app.groups_job.is_some() {
            ui.add(egui::Spinner::new().size(12.0).color(theme::pal().accent));
        }
    });
    if has_groups {
        let count = |k: GroupKind| app.face_groups.iter().filter(|g| g.kind == k).count();
        let (planes, cyls, spheres) = (
            count(GroupKind::Plane),
            count(GroupKind::Cylinder),
            count(GroupKind::Sphere),
        );
        ui.label(
            egui::RichText::new(format!(
                "{} groups: {planes} planes · {cyls} cylinders · {spheres} spheres · {} other",
                app.face_groups.len(),
                app.face_groups.len() - planes - cyls - spheres
            ))
            .size(11.0)
            .color(theme::pal().text_muted),
        );
    } else {
        ui.label(
            egui::RichText::new(
                "No face groups yet: holes get the standard fill unless a fitted circle \
                 matches them.",
            )
            .size(11.0)
            .color(theme::pal().warn),
        );
    }
    let circles = app.circles.iter().filter(|c| c.visible).count();
    let circles_text = if circles == 0 {
        "Tip: fit a circle to a round profile (Face selection) to rebuild it as a cylinder."
            .to_string()
    } else {
        format!(
            "{circles} fitted circle{} guide the fill as cylinders.",
            if circles == 1 { "" } else { "s" }
        )
    };
    ui.label(
        egui::RichText::new(circles_text)
            .size(11.0)
            .color(theme::pal().text_muted),
    );
    if run_detect {
        app.request_face_groups();
    }

    ui.add_space(4.0);
    ui.separator();

    // 2. Hole (shared with the Mesh repair section).
    let holes = app.repair_holes.clone();
    let selected = app.repair_selected_hole.filter(|&i| i < holes.len());
    let mut new_selection: Option<Option<usize>> = None;
    let mut do_rescan = false;
    let mut do_focus = false;
    ui.horizontal(|ui| {
        heading(ui, &format!("Holes ({})", holes.len()));
        if app.analysis_job.is_none() && ui.small_button("Rescan").clicked() {
            do_rescan = true;
        }
        if selected.is_some() && ui.small_button("Focus").clicked() {
            do_focus = true;
        }
    });
    if holes.is_empty() {
        ui.label(
            egui::RichText::new("No open holes detected.")
                .size(11.0)
                .color(theme::pal().text_muted),
        );
    } else {
        egui::ScrollArea::vertical()
            .id_salt("guided_fill_holes")
            .max_height(110.0)
            .show(ui, |ui| {
                for (i, hole) in holes.iter().enumerate() {
                    let is_sel = selected == Some(i);
                    let label = format!(
                        "Hole #{}: {:.1} mm ({} edges)",
                        hole.id,
                        hole.perimeter,
                        hole.edge_count()
                    );
                    if ui.selectable_label(is_sel, label).clicked() {
                        new_selection = Some(if is_sel { None } else { Some(i) });
                    }
                }
            });
    }
    if do_rescan {
        app.request_repair_analysis();
    }
    if let Some(sel) = new_selection {
        app.select_hole(sel);
    }
    if do_focus {
        let now = ui.input(|i| i.time);
        app.focus_selected_hole(now);
    }

    ui.add_space(4.0);
    ui.separator();

    // 3. Fill.
    heading(ui, "Fill");
    ui.add(
        egui::Slider::new(&mut app.guided_density, 0.25..=3.0)
            .step_by(0.05)
            .suffix("×")
            .text("Density"),
    )
    .on_hover_text("Triangle size of the patch relative to the edges of the hole rim.");
    ui.checkbox(&mut app.guided_preview_active, "Show preview in viewport");
    app.sync_guided_preview();
    let pal = theme::pal();
    match app.guided_preview_result() {
        Some(Ok(res)) if res.report.guided => {
            ui.label(
                egui::RichText::new(format!(
                    "{} · max deviation {:.4} mm",
                    res.report.note, res.report.max_deviation
                ))
                .size(11.0)
                .color(pal.success),
            );
        }
        Some(Ok(res)) => {
            ui.label(
                egui::RichText::new(format!("Standard fill: {}", res.report.note))
                    .size(11.0)
                    .color(pal.warn),
            );
        }
        Some(Err(e)) => {
            ui.label(egui::RichText::new(e).size(11.0).color(pal.danger));
        }
        None if app.guided_preview_active && selected.is_none() && !holes.is_empty() => {
            ui.label(
                egui::RichText::new("Select a hole to preview the fill.")
                    .size(11.0)
                    .color(pal.text_muted),
            );
        }
        None => {}
    }
    let mut do_fill = false;
    let mut do_fill_all = false;
    ui.horizontal(|ui| {
        if ui
            .add_enabled(
                selected.is_some() && !editing,
                egui::Button::new("Fill selected hole"),
            )
            .clicked()
        {
            do_fill = true;
        }
        if ui
            .add_enabled(
                !holes.is_empty() && !editing,
                egui::Button::new("Fill all holes"),
            )
            .on_hover_text("One undo step; large meshes are processed in the background.")
            .clicked()
        {
            do_fill_all = true;
        }
        if editing {
            ui.add(egui::Spinner::new().size(12.0).color(pal.accent));
        }
    });
    if do_fill {
        app.fill_selected_hole_guided();
    }
    if do_fill_all {
        app.request_guided_fill_all();
    }
}
