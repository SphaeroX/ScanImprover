//! Mesh repair section: diagnostics, hole list, hole filling and clean-up.

use crate::app::{App, MeshEditKind};
use crate::geom::hole_fill::{FillDirectionMode, HoleFillMethod};
use crate::geom::hole_solver::CircleGuideMode;
use crate::ui::accordion::group_box;
use crate::ui::theme;
use egui;

pub fn render_repair(app: &mut App, ui: &mut egui::Ui) {
    // Trigger the initial analysis lazily when the section is opened.
    if app.repair_health.is_none() && app.has_mesh() && app.analysis_job.is_none() {
        app.request_repair_analysis();
    }
    let now = ui.input(|i| i.time);
    let analyzing = app.analysis_job.is_some();
    let editing = app.is_editing();
    let health_opt = app.repair_health.clone();
    let holes_clone = app.repair_holes.clone();
    let selected_hole = app.repair_selected_hole;
    let mut config = app.repair_config;
    let mut preview_active = app.repair_preview_active;

    // Deferred actions to avoid borrowing conflicts
    let mut do_refresh = false;
    let mut do_focus = false;
    let mut new_selection: Option<Option<usize>> = None;
    let mut config_changed = false;
    let mut new_preview_active: Option<bool> = None;
    let mut do_fill_selected = false;
    let mut do_solve_best = false;
    let mut new_refine: Option<bool> = None;
    let mut new_circle_mode: Option<CircleGuideMode> = None;
    let mut mesh_edit: Option<MeshEditKind> = None;

    // 1. Health
    group_box(ui, Some("Mesh health & diagnostics"), |ui| {
        if let Some(health) = health_opt {
            ui.horizontal(|ui| {
                if health.is_watertight {
                    ui.label(
                        egui::RichText::new("Watertight (closed 2-manifold)")
                            .color(theme::pal().success)
                            .strong(),
                    );
                } else {
                    ui.label(
                        egui::RichText::new("Open boundary / defective")
                            .color(theme::pal().warn)
                            .strong(),
                    );
                }
                if analyzing {
                    ui.add(egui::Spinner::new().size(12.0).color(theme::pal().accent));
                } else if ui.small_button("Refresh").clicked() {
                    do_refresh = true;
                }
            });
            egui::Grid::new("mesh_health_grid")
                .striped(true)
                .min_col_width(120.0)
                .spacing(egui::vec2(8.0, 3.0))
                .show(ui, |ui| {
                    let muted = |ui: &mut egui::Ui, s: &str| {
                        ui.label(egui::RichText::new(s).color(theme::pal().text_muted));
                    };
                    muted(ui, "Holes");
                    ui.label(format!("{}", health.hole_count));
                    ui.end_row();
                    muted(ui, "Open boundary edges");
                    ui.label(format!("{}", health.boundary_edges));
                    ui.end_row();
                    muted(ui, "Euler χ / genus");
                    ui.label(format!(
                        "χ = {}, g = {}",
                        health.euler_characteristic, health.genus
                    ));
                    ui.end_row();
                    muted(ui, "Non-manifold edges / verts");
                    let nm_col = if health.non_manifold_edges > 0 || health.non_manifold_verts > 0 {
                        theme::pal().danger
                    } else {
                        theme::pal().text
                    };
                    ui.colored_label(
                        nm_col,
                        format!(
                            "{} / {}",
                            health.non_manifold_edges, health.non_manifold_verts
                        ),
                    );
                    ui.end_row();
                    muted(ui, "Conflicting normals");
                    let norm_col = if health.inconsistent_normals > 0 {
                        theme::pal().warn
                    } else {
                        theme::pal().text
                    };
                    ui.colored_label(norm_col, format!("{}", health.inconsistent_normals));
                    ui.end_row();
                    muted(ui, "Degenerate / duplicate faces");
                    ui.label(format!(
                        "{} / {}",
                        health.degenerate_faces, health.duplicate_faces
                    ));
                    ui.end_row();
                    muted(ui, "Isolated vertices");
                    ui.label(format!("{}", health.isolated_verts));
                    ui.end_row();
                    muted(ui, "Shells (components)");
                    ui.label(format!("{}", health.component_count));
                    ui.end_row();
                });
            ui.add_space(2.0);
            if ui
                .add_enabled(!editing, egui::Button::new("1-click auto repair"))
                .on_hover_text(
                    "Removes degenerate faces, unifies normals, and eliminates floating debris",
                )
                .clicked()
            {
                mesh_edit = Some(MeshEditKind::AutoRepair);
            }
        } else if analyzing {
            ui.horizontal(|ui| {
                ui.add(egui::Spinner::new().size(13.0).color(theme::pal().accent));
                ui.label(egui::RichText::new("Analyzing mesh…").color(theme::pal().accent_text));
            });
        } else {
            ui.horizontal(|ui| {
                ui.label("No mesh loaded.");
                if ui
                    .add_enabled(app.has_mesh(), egui::Button::new("Analyze"))
                    .clicked()
                {
                    do_refresh = true;
                }
            });
        }
    });

    ui.add_space(4.0);

    // 2. Holes
    group_box(ui, Some("Detected holes"), |ui| {
        let n_holes = holes_clone.len();
        ui.horizontal(|ui| {
            ui.label(format!("{n_holes} hole(s) found"));
            if !analyzing && ui.small_button("Rescan").clicked() {
                do_refresh = true;
            }
        });
        if n_holes == 0 {
            ui.colored_label(
                theme::pal().success.gamma_multiply(0.85),
                "No open holes detected on the mesh.",
            );
        } else {
            ui.label(
                egui::RichText::new("Select a hole to view or fill:")
                    .size(11.0)
                    .color(theme::pal().text_muted),
            );
            egui::ScrollArea::vertical()
                .max_height(140.0)
                .show(ui, |ui| {
                    for (i, hole) in holes_clone.iter().enumerate() {
                        let is_selected = selected_hole == Some(i);
                        let label = format!(
                            "Hole #{}: {:.1} mm ({} edges, ~{:.1} mm²)",
                            hole.id,
                            hole.perimeter,
                            hole.edge_count(),
                            hole.approx_area
                        );
                        if ui.selectable_label(is_selected, label).clicked() {
                            new_selection = Some(if is_selected { None } else { Some(i) });
                        }
                    }
                });
            if selected_hole.is_some_and(|s| s < holes_clone.len()) {
                ui.horizontal(|ui| {
                    if ui.button("Focus camera").clicked() {
                        do_focus = true;
                    }
                    if ui.button("Deselect").clicked() {
                        new_selection = Some(None);
                    }
                });
            }
        }
    });

    ui.add_space(4.0);

    // 3. Hole filling
    group_box(ui, Some("Hole filling (contour & shape)"), |ui| {
        ui.label(egui::RichText::new("Filling algorithm").strong().size(11.5));
        ui.horizontal_wrapped(|ui| {
            for method in HoleFillMethod::all() {
                if ui
                    .selectable_value(&mut config.method, method, method.display_name())
                    .clicked()
                {
                    config_changed = true;
                }
            }
        });
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            if ui
                .add(
                    egui::Slider::new(&mut config.density, 0.2..=3.0)
                        .step_by(0.1)
                        .suffix("×")
                        .text("Density"),
                )
                .changed()
            {
                config_changed = true;
            }
            if ui
                .small_button("1.0×")
                .on_hover_text("Reset to standard density")
                .clicked()
            {
                config.density = 1.0;
                config_changed = true;
            }
        });
        ui.horizontal(|ui| {
            if ui
                .add(
                    egui::Slider::new(&mut config.bulge, -1.0..=1.0)
                        .step_by(0.05)
                        .text("Bulge"),
                )
                .changed()
            {
                config_changed = true;
            }
            if ui
                .small_button("Flat")
                .on_hover_text("Completely flat minimal surface")
                .clicked()
            {
                config.bulge = 0.0;
                config_changed = true;
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Direction:");
            for dir in FillDirectionMode::all() {
                if ui
                    .selectable_value(&mut config.direction_mode, dir, dir.display_name())
                    .clicked()
                {
                    config_changed = true;
                }
            }
        });
        if ui
            .add(
                egui::Slider::new(&mut config.smooth_iterations, 5..=50)
                    .suffix(" iters")
                    .text("Smoothing"),
            )
            .changed()
        {
            config_changed = true;
        }

        ui.add_space(4.0);
        ui.separator();

        // Reference guidance & solver
        let plane_count = app.planes.iter().filter(|p| p.visible).count()
            + usize::from(app.planes.is_empty() && app.plane.is_some());
        let circle_count = app.circles.iter().filter(|c| c.visible).count()
            + usize::from(app.circles.is_empty() && app.circle.is_some());
        let has_refs = plane_count + circle_count > 0;
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("Reference guidance")
                    .strong()
                    .size(11.5),
            );
            if has_refs {
                let mut ref_desc = Vec::new();
                if plane_count > 0 {
                    ref_desc.push(format!(
                        "{} plane{}",
                        plane_count,
                        if plane_count == 1 { "" } else { "s" }
                    ));
                }
                if circle_count > 0 {
                    ref_desc.push(format!(
                        "{} circle{}",
                        circle_count,
                        if circle_count == 1 { "" } else { "s" }
                    ));
                }
                ui.label(
                    egui::RichText::new(format!("({} active)", ref_desc.join(", ")))
                        .color(theme::pal().accent_text)
                        .size(11.0),
                );
            } else {
                ui.label(
                    egui::RichText::new("(no planes / circles fitted)")
                        .color(theme::pal().text_muted)
                        .size(11.0),
                );
            }
        });
        if circle_count > 0 {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Circle shape:").size(11.0));
                let mut cm = app.repair_circle_mode;
                if ui
                    .selectable_value(&mut cm, CircleGuideMode::DiskAndRim, "Disk / rim")
                    .on_hover_text("Treats the circle as a planar disk and circular rim (no infinite cylinder projection)")
                    .clicked()
                {
                    new_circle_mode = Some(cm);
                }
                if ui
                    .selectable_value(&mut cm, CircleGuideMode::CylinderWall, "Cylinder wall")
                    .on_hover_text("Treats the circle as a cylindrical bore / wall")
                    .clicked()
                {
                    new_circle_mode = Some(cm);
                }
            });
        }
        ui.horizontal(|ui| {
            let solving = app.solve_job.is_some();
            if ui
                .add_enabled(has_refs && !holes_clone.is_empty() && !solving, egui::Button::new("Auto-solve best fit"))
                .on_hover_text("Numerically solves for the hole filling method and parameters that best match the fitted planes and circles")
                .clicked()
            {
                do_solve_best = true;
            }
            if solving {
                ui.add(egui::Spinner::new().size(12.0).color(theme::pal().accent));
            }
            let mut refine = app.repair_refine_to_references;
            if ui
                .checkbox(&mut refine, "CAD refine")
                .on_hover_text("Projects interior patch vertices onto reference planes / cylinders with a smooth boundary blend")
                .changed()
            {
                new_refine = Some(refine);
            }
        });
        if let Some(status_str) = &app.repair_solve_status {
            ui.label(
                egui::RichText::new(status_str)
                    .size(11.0)
                    .color(theme::pal().success),
            );
        }

        ui.add_space(4.0);
        ui.separator();
        if ui
            .checkbox(&mut preview_active, "Show preview in viewport")
            .changed()
        {
            new_preview_active = Some(preview_active);
        }
        let has_sel = selected_hole.is_some_and(|s| s < holes_clone.len());
        ui.horizontal(|ui| {
            if ui
                .add_enabled(has_sel && !editing, egui::Button::new("Fill selected hole"))
                .clicked()
            {
                do_fill_selected = true;
            }
            if ui
                .add_enabled(
                    !holes_clone.is_empty() && !editing,
                    egui::Button::new("Fill all holes"),
                )
                .clicked()
            {
                mesh_edit = Some(MeshEditKind::FillAllHoles);
            }
        });
    });

    ui.add_space(4.0);

    // 4. Clean-up tools
    group_box(ui, Some("Clean-up tools"), |ui| {
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!editing, egui::Button::new("Unify normals"))
                .on_hover_text("Consistent orientation and outward facing normals across all faces")
                .clicked()
            {
                mesh_edit = Some(MeshEditKind::UnifyNormals);
            }
            if ui
                .add_enabled(!editing, egui::Button::new("Remove debris"))
                .on_hover_text("Removes small isolated shells / floating scan noise (< 0.5% faces)")
                .clicked()
            {
                mesh_edit = Some(MeshEditKind::RemoveDebris);
            }
            if ui
                .add_enabled(!editing, egui::Button::new("Remove degenerate faces"))
                .on_hover_text("Cleans zero-area and collapsed triangles")
                .clicked()
            {
                mesh_edit = Some(MeshEditKind::RemoveDegenerate);
            }
        });
        if editing {
            ui.horizontal(|ui| {
                ui.add(egui::Spinner::new().size(12.0).color(theme::pal().accent));
                ui.label(
                    egui::RichText::new("Working on the mesh…").color(theme::pal().accent_text),
                );
            });
        }
    });

    // Execute deferred mutations
    if do_refresh {
        app.request_repair_analysis();
    }
    if do_focus {
        app.focus_selected_hole(now);
    }
    if let Some(sel) = new_selection {
        app.select_hole(sel);
    }
    if config_changed {
        app.set_hole_fill_config(config);
    }
    if let Some(p) = new_preview_active {
        app.set_hole_preview_active(p);
    }
    if do_fill_selected {
        app.fill_selected_hole();
    }
    if do_solve_best {
        app.request_hole_solve();
    }
    if let Some(r) = new_refine {
        app.set_hole_refine_to_references(r);
    }
    if let Some(cm) = new_circle_mode {
        app.set_hole_circle_mode(cm);
    }
    if let Some(kind) = mesh_edit {
        app.request_mesh_edit(kind);
    }
}
