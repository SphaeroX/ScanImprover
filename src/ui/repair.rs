use crate::app::App;
use crate::geom::hole_fill::{FillDirectionMode, HoleFillMethod};
use crate::ui::accordion::group_box;
use eframe::egui;

/// Renders the complete Mesh Repair section inside the accordion body.
pub fn render_repair(app: &mut App, ui: &mut egui::Ui) {
    // If repair data is not yet computed, trigger initial refresh
    if app.repair_health.is_none() && app.display().is_some() {
        app.refresh_repair();
    }

    let health_opt = app.repair_health.clone();
    let holes_clone = app.repair_holes.clone();
    let selected_hole = app.repair_selected_hole;
    let mut config = app.repair_config;
    let mut preview_active = app.repair_preview_active;

    // Deferred actions to avoid borrowing conflicts
    let mut do_refresh = false;
    let mut do_auto_repair = false;
    let mut do_focus = false;
    let mut new_selection: Option<Option<usize>> = None;
    let mut config_changed = false;
    let mut new_preview_active: Option<bool> = None;
    let mut do_fill_selected = false;
    let mut do_fill_all = false;
    let mut do_solve_best = false;
    let mut new_refine: Option<bool> = None;
    let mut new_circle_mode: Option<crate::geom::hole_solver::CircleGuideMode> = None;
    let mut do_unify_normals = false;
    let mut do_remove_debris = false;
    let mut do_remove_degenerates = false;

    // 1. MESH HEALTH & DIAGNOSTICS
    group_box(ui, Some("MESH HEALTH & DIAGNOSTICS"), |ui| {
        if let Some(health) = health_opt {
            ui.horizontal(|ui| {
                if health.is_watertight {
                    ui.label(
                        egui::RichText::new("✔ Watertight (Closed 2-Manifold)")
                            .color(egui::Color32::from_rgb(80, 220, 120))
                            .strong(),
                    );
                } else {
                    ui.label(
                        egui::RichText::new("⚠ Open Boundary / Defective")
                            .color(egui::Color32::from_rgb(240, 160, 60))
                            .strong(),
                    );
                }
                if ui.small_button("↻ Refresh").clicked() {
                    do_refresh = true;
                }
            });

            ui.add_space(3.0);
            egui::Grid::new("mesh_health_grid")
                .striped(true)
                .min_col_width(110.0)
                .show(ui, |ui| {
                    ui.label("Holes detected:");
                    ui.label(format!("{}", health.hole_count));
                    ui.end_row();

                    ui.label("Open boundary edges:");
                    ui.label(format!("{}", health.boundary_edges));
                    ui.end_row();

                    ui.label("Euler char (χ) / Genus:");
                    ui.label(format!("χ = {}, g = {}", health.euler_characteristic, health.genus));
                    ui.end_row();

                    ui.label("Non-manifold e/v:");
                    let nm_col = if health.non_manifold_edges > 0 || health.non_manifold_verts > 0 {
                        egui::Color32::from_rgb(255, 100, 100)
                    } else {
                        egui::Color32::from_rgb(180, 180, 180)
                    };
                    ui.colored_label(nm_col, format!("{}/{}", health.non_manifold_edges, health.non_manifold_verts));
                    ui.end_row();

                    ui.label("Conflicting normals:");
                    let norm_col = if health.inconsistent_normals > 0 {
                        egui::Color32::from_rgb(255, 160, 60)
                    } else {
                        egui::Color32::from_rgb(180, 180, 180)
                    };
                    ui.colored_label(norm_col, format!("{}", health.inconsistent_normals));
                    ui.end_row();

                    ui.label("Degenerate / Dupl faces:");
                    ui.label(format!("{}/{}", health.degenerate_faces, health.duplicate_faces));
                    ui.end_row();

                    ui.label("Isolated vertices:");
                    ui.label(format!("{}", health.isolated_verts));
                    ui.end_row();

                    ui.label("Shells (components):");
                    ui.label(format!("{}", health.component_count));
                    ui.end_row();
                });

            ui.add_space(4.0);
            if ui
                .button("✨ 1-Click Auto Repair")
                .on_hover_text("Removes degenerate faces, unifies normals, and eliminates floating debris")
                .clicked()
            {
                do_auto_repair = true;
            }
        } else {
            ui.horizontal(|ui| {
                ui.label("No mesh loaded.");
                if ui.button("Analyze").clicked() {
                    do_refresh = true;
                }
            });
        }
    });

    ui.add_space(4.0);

    // 2. HOLE DETECTION & INDIVIDUAL SELECTION
    group_box(ui, Some("DETECTED HOLES"), |ui| {
        let n_holes = holes_clone.len();
        ui.horizontal(|ui| {
            ui.label(format!("{n_holes} hole(s) found"));
            if ui.small_button("↻ Scan").clicked() {
                do_refresh = true;
            }
        });

        if n_holes == 0 {
            ui.colored_label(
                egui::Color32::from_rgb(140, 190, 140),
                "No open holes detected on the mesh.",
            );
        } else {
            ui.add_space(2.0);
            ui.label(
                egui::RichText::new("Select a hole to view or fill:")
                    .size(11.0)
                    .color(egui::Color32::from_gray(160)),
            );

            // Scrollable list of holes
            egui::ScrollArea::vertical()
                .max_height(140.0)
                .show(ui, |ui| {
                    for i in 0..n_holes {
                        let is_selected = selected_hole == Some(i);
                        let hole = &holes_clone[i];
                        let label = format!(
                            "Hole #{}: {:.1} mm ({} edges, ~{:.1} mm²)",
                            hole.id,
                            hole.perimeter,
                            hole.edge_count(),
                            hole.approx_area
                        );

                        ui.horizontal(|ui| {
                            if ui.selectable_label(is_selected, label).clicked() {
                                if is_selected {
                                    new_selection = Some(None);
                                } else {
                                    new_selection = Some(Some(i));
                                }
                            }
                        });
                    }
                });

            if let Some(sel_idx) = selected_hole {
                if sel_idx < holes_clone.len() {
                    ui.add_space(2.0);
                    ui.horizontal(|ui| {
                        if ui.button("🔍 Focus Camera").clicked() {
                            do_focus = true;
                        }
                        if ui.button("Deselect").clicked() {
                            new_selection = Some(None);
                        }
                    });
                }
            }
        }
    });

    ui.add_space(4.0);

    // 3. HOLE FILLING & MESHMUXER-STYLE CONTOUR CONTROLS
    group_box(ui, Some("HOLE FILLING (CONTOUR & SHAPE)"), |ui| {
        ui.label(egui::RichText::new("Filling Algorithm:").strong());
        ui.horizontal(|ui| {
            for method in HoleFillMethod::all() {
                if ui.selectable_value(&mut config.method, method, method.display_name()).clicked() {
                    config_changed = true;
                }
            }
        });

        ui.add_space(4.0);
        ui.separator();
        ui.add_space(2.0);

        // Density / Resolution (Feinheit)
        ui.horizontal(|ui| {
            ui.label("Density / Feinheit:");
            let resp = ui.add(
                egui::Slider::new(&mut config.density, 0.2..=3.0)
                    .step_by(0.1)
                    .suffix("×")
            );
            if resp.changed() {
                config_changed = true;
            }
            if ui.small_button("1.0×").on_hover_text("Reset to standard density").clicked() {
                config.density = 1.0;
                config_changed = true;
            }
        });

        // Bulge / Roundness / Flatness (Rundung, Wölbung vs Flach)
        ui.horizontal(|ui| {
            ui.label("Bulge / Rundung:");
            let resp = ui.add(
                egui::Slider::new(&mut config.bulge, -1.0..=1.0)
                    .step_by(0.05)
            );
            if resp.changed() {
                config_changed = true;
            }
            if ui.small_button("Flat (0)").on_hover_text("Set completely flat minimal surface").clicked() {
                config.bulge = 0.0;
                config_changed = true;
            }
        });

        // Direction mode (Richtung)
        ui.horizontal(|ui| {
            ui.label("Direction:");
            for dir in FillDirectionMode::all() {
                if ui.selectable_value(&mut config.direction_mode, dir, dir.display_name()).clicked() {
                    config_changed = true;
                }
            }
        });

        // Fairing / Smoothing iterations (Glättung)
        ui.horizontal(|ui| {
            ui.label("Smoothing steps:");
            let resp = ui.add(
                egui::Slider::new(&mut config.smooth_iterations, 5..=50)
                    .suffix(" iters")
            );
            if resp.changed() {
                config_changed = true;
            }
        });

        ui.add_space(4.0);
        ui.separator();
        ui.add_space(2.0);

        // Reference Guidance & Numerical Solver
        let plane_count = app.planes.iter().filter(|p| p.visible).count()
            + (if app.planes.is_empty() && app.plane.is_some() { 1 } else { 0 });
        let circle_count = app.circles.iter().filter(|c| c.visible).count()
            + (if app.circles.is_empty() && app.circle.is_some() { 1 } else { 0 });
        let has_refs = plane_count + circle_count > 0;

        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Reference Guidance:").strong());
            if has_refs {
                let mut ref_desc = Vec::new();
                if plane_count > 0 {
                    ref_desc.push(format!("{} plane{}", plane_count, if plane_count == 1 { "" } else { "s" }));
                }
                if circle_count > 0 {
                    ref_desc.push(format!("{} circle{}", circle_count, if circle_count == 1 { "" } else { "s" }));
                }
                ui.label(
                    egui::RichText::new(format!("({} active)", ref_desc.join(", ")))
                        .color(egui::Color32::from_rgb(100, 200, 255))
                        .size(11.0),
                );
            } else {
                ui.label(
                    egui::RichText::new("(No planes/circles fitted)")
                        .color(egui::Color32::GRAY)
                        .size(11.0),
                );
            }
        });

        if circle_count > 0 {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Circle Shape:").size(11.0));
                let mut cm = app.repair_circle_mode;
                if ui
                    .selectable_value(
                        &mut cm,
                        crate::geom::hole_solver::CircleGuideMode::DiskAndRim,
                        "Disk / Rim",
                    )
                    .on_hover_text("Treats circle as a planar disk and circular perimeter contour (prevents infinite cylinder projection)")
                    .clicked()
                {
                    new_circle_mode = Some(cm);
                }
                if ui
                    .selectable_value(
                        &mut cm,
                        crate::geom::hole_solver::CircleGuideMode::CylinderWall,
                        "Cylinder Wall",
                    )
                    .on_hover_text("Treats circle as a cylindrical bore/wall")
                    .clicked()
                {
                    new_circle_mode = Some(cm);
                }
            });
        }

        ui.horizontal(|ui| {
            let solve_btn = ui.add_enabled(
                has_refs && !holes_clone.is_empty(),
                egui::Button::new("🎯 Auto-Solve Best Fit"),
            );
            if solve_btn
                .on_hover_text("Numerically solves for the optimal hole filling tool and parameters to best match existing fitted planes and circles")
                .clicked()
            {
                do_solve_best = true;
            }

            let mut refine = app.repair_refine_to_references;
            if ui.checkbox(&mut refine, "CAD Refine")
                .on_hover_text("Projects interior patch vertices onto reference planes/cylinders with smooth boundary blend")
                .changed()
            {
                new_refine = Some(refine);
            }
        });

        if let Some(status_str) = &app.repair_solve_status {
            ui.label(
                egui::RichText::new(status_str)
                    .size(11.0)
                    .color(egui::Color32::from_rgb(120, 220, 140)),
            );
        }

        ui.add_space(4.0);
        ui.separator();
        ui.add_space(2.0);

        if ui.checkbox(&mut preview_active, "Show preview in viewport").changed() {
            new_preview_active = Some(preview_active);
        }

        ui.add_space(4.0);
        let has_sel = selected_hole.is_some()
            && selected_hole.unwrap() < holes_clone.len();

        ui.horizontal(|ui| {
            let fill_btn = ui.add_enabled(has_sel, egui::Button::new("Fill Selected Hole"));
            if fill_btn.clicked() {
                do_fill_selected = true;
            }

            let fill_all_btn = ui.add_enabled(
                !holes_clone.is_empty(),
                egui::Button::new("Fill All Holes"),
            );
            if fill_all_btn.clicked() {
                do_fill_all = true;
            }
        });
    });

    ui.add_space(4.0);

    // 4. ADVANCED REPAIR TOOLS
    group_box(ui, Some("ADVANCED REPAIR TOOLS"), |ui| {
        ui.horizontal(|ui| {
            if ui
                .button("Unify normals")
                .on_hover_text("Ensures consistent orientation and outward facing normals across all faces")
                .clicked()
            {
                do_unify_normals = true;
            }

            if ui
                .button("Remove debris")
                .on_hover_text("Removes small isolated shells / floating scan noise (< 0.5% faces)")
                .clicked()
            {
                do_remove_debris = true;
            }
        });

        ui.add_space(2.0);
        if ui
            .button("Remove degenerate faces")
            .on_hover_text("Cleans zero-area and collapsed triangles")
            .clicked()
        {
            do_remove_degenerates = true;
        }
    });

    // Execute deferred mutations
    if do_refresh {
        app.refresh_repair();
    }
    if do_auto_repair {
        app.auto_repair();
    }
    if do_focus {
        app.focus_selected_hole();
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
    if do_fill_all {
        app.fill_all_holes();
    }
    if do_solve_best {
        app.solve_best_hole_fill();
    }
    if let Some(r) = new_refine {
        app.set_hole_refine_to_references(r);
    }
    if let Some(cm) = new_circle_mode {
        app.set_hole_circle_mode(cm);
    }
    if do_unify_normals {
        app.unify_normals_action();
    }
    if do_remove_debris {
        app.remove_small_components_action();
    }
    if do_remove_degenerates {
        app.remove_degenerate_faces_action();
    }
}
