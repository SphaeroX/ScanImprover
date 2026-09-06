use crate::app::{rotation_between, App, DecMode, Mode};
use crate::ui::accordion::group_box;
use eframe::egui;
use glam::Vec3;
use std::sync::Arc;

/// Renders the Decimation section inside the accordion body.
pub fn render_decimation(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("MODE"), |ui| {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut app.dec_mode, DecMode::Fixed, "Fixed mm");
            ui.selectable_value(&mut app.dec_mode, DecMode::Accuracy, "Auto %");
            ui.selectable_value(&mut app.dec_mode, DecMode::Deviation, "Auto mm");
        });
        ui.add_space(2.0);
        ui.checkbox(&mut app.dec_lock_border, "Lock open borders");
    });

    ui.add_space(4.0);

    let mut schedule_manual = false;
    group_box(ui, Some("PARAMETERS"), |ui| {
        match app.dec_mode {
            DecMode::Fixed => {
                let err_resp = ui.add(
                    egui::Slider::new(&mut app.dec_error_mm, 0.0001..=1.0)
                        .logarithmic(true)
                        .text("Error tolerance (mm)"),
                );
                let ratio_resp = ui.add(
                    egui::Slider::new(&mut app.dec_ratio, 0.002..=1.0)
                        .text("Target triangle ratio"),
                );
                ui.checkbox(&mut app.dec_auto_preview, "Auto preview on change");
                ui.add_space(2.0);
                ui.horizontal(|ui| {
                    if ui.button("Preview now").clicked() {
                        schedule_manual = true;
                    }
                    if app.dec_job.is_some() {
                        ui.add(egui::Spinner::new());
                        ui.label("working…");
                    }
                });
                if err_resp.drag_stopped() || ratio_resp.drag_stopped() {
                    if app.dec_auto_preview && app.dec_job.is_none() {
                        schedule_manual = true;
                    }
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
                ui.label(format!(
                    "Goal: max deviation {:.4} mm = {:.4} % accuracy",
                    app.dec_target_mm, app.dec_target_acc
                ));
                ui.add_space(2.0);
                ui.horizontal(|ui| {
                    if ui.button("Auto decimate").clicked() {
                        app.run_auto_decimate();
                    }
                    if app.dec_job.is_some() {
                        ui.add(egui::Spinner::new());
                        ui.label("working…");
                    }
                });
            }
        }
    });

    if app.preview.is_some() {
        ui.add_space(4.0);
        egui::Frame::new()
            .fill(egui::Color32::from_rgba_unmultiplied(40, 90, 60, 45))
            .stroke(egui::Stroke::new(
                1.0,
                egui::Color32::from_rgba_unmultiplied(60, 180, 100, 70),
            ))
            .corner_radius(4.0)
            .inner_margin(egui::Margin::symmetric(8, 6))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("✔ Apply decimation").clicked() {
                        app.apply_preview();
                    }
                    if ui.button("✖ Discard").clicked() {
                        app.discard_preview();
                    }
                });
            });
    }

    let metrics_info = app.orig_mesh().zip(app.preview.as_ref()).map(|(orig, prev)| {
        (
            orig.triangle_count(),
            prev.triangle_count(),
            100.0 * prev.triangle_count() as f32 / orig.triangle_count().max(1) as f32,
        )
    });

    if let Some((orig_tris, prev_tris, pct)) = metrics_info {
        ui.add_space(4.0);
        group_box(ui, Some("METRICS & DEVIATION"), |ui| {
            ui.label(format!(
                "Triangles: {} → {} ({:.1}%)",
                orig_tris, prev_tris, pct
            ));
            ui.label(format!(
                "Simplifier error estimate: {:.4} mm",
                app.preview_error
            ));
            if let Some(dev) = &app.deviation {
                if app.dev_job.is_some() && app.dec_mode == DecMode::Fixed {
                    ui.label("Measuring deviation…");
                } else {
                    let diag = app.bbox.diagonal();
                    let acc = 100.0 * (1.0 - dev.max_dev / diag.max(1e-9));
                    ui.label(format!("Max deviation: {:.4} mm", dev.max_dev));
                    ui.label(format!("RMS deviation: {:.4} mm", dev.rms));
                    ui.label(format!("Contour match: {:.5}%", acc));
                    ui.label(format!("Measured on {} source points", dev.count));
                }
            }
            ui.add_space(2.0);
            ui.checkbox(&mut app.heat_on, "Deviation heatmap");
        });
    }

    if schedule_manual && app.dec_job.is_none() {
        app.schedule_decimate();
    }
}

/// Renders the Symmetry section inside the accordion body.
pub fn render_symmetry(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("DETECTION"), |ui| {
        ui.horizontal(|ui| {
            if ui.button("Auto-detect").clicked() {
                app.schedule_sym_auto();
            }
            let pick_label = if app.mode == Mode::SymPickLine {
                "Pick line: ON"
            } else {
                "Pick line"
            };
            if ui.button(pick_label).clicked() {
                app.mode = if app.mode == Mode::SymPickLine {
                    Mode::Orbit
                } else {
                    app.sym_pick.clear();
                    Mode::SymPickLine
                };
            }
            if app.sym_job.is_some() {
                ui.add(egui::Spinner::new());
            }
        });

        if app.sym_pick.len() >= 2 {
            ui.add_space(3.0);
            ui.horizontal(|ui| {
                if ui.button("Calculate").clicked() {
                    let a = app.sym_pick[0];
                    let b = app.sym_pick[1];
                    app.schedule_sym_from_line(a, b);
                    app.mode = Mode::Orbit;
                }
                if ui.button("Clear line").clicked() {
                    app.sym_pick.clear();
                    app.mode = Mode::Orbit;
                }
            });
        } else if !app.sym_pick.is_empty() {
            ui.add_space(3.0);
            ui.horizontal(|ui| {
                ui.label("1 point placed…");
                if ui.button("Clear line").clicked() {
                    app.sym_pick.clear();
                    app.mode = Mode::Orbit;
                }
            });
        }

        ui.add_space(3.0);
        let sel_text = format!("Exclude selection ({} faces)", app.sel_count);
        ui.checkbox(&mut app.sym_exclude_selection, sel_text);
        ui.checkbox(&mut app.sym_exclude_holes, "Exclude mesh holes");
    });

    let sym_copy = app.sym;
    if sym_copy.is_some() {
        ui.add_space(4.0);
        group_box(ui, Some("SYMMETRY PLANE"), |ui| {
            let mut show = sym_copy.unwrap().show;
            if ui.checkbox(&mut show, "Show plane").changed() {
                if let Some(s) = &mut app.sym {
                    s.show = show;
                }
            }
            let rms = sym_copy.unwrap().rms;
            if rms.is_finite() {
                ui.label(format!("RMS: {:.4} mm", rms));
            } else {
                ui.label("Optimizing plane…");
            }
            ui.add_space(3.0);
            let mut refine = false;
            let mut to_origin = false;
            let mut align_axis = None;
            ui.horizontal(|ui| {
                if ui.button("Optimize").clicked() {
                    refine = true;
                }
                if ui.button("Origin → plane").clicked() {
                    to_origin = true;
                }
            });
            ui.add_space(4.0);
            ui.label("Align symmetry to:");
            ui.horizontal(|ui| {
                if ui.button("X = 0").clicked() {
                    align_axis = Some(Vec3::X);
                }
                if ui.button("Y = 0").clicked() {
                    align_axis = Some(Vec3::Y);
                }
                if ui.button("Z = 0").clicked() {
                    align_axis = Some(Vec3::Z);
                }
            });
            let sp = sym_copy.unwrap().plane;
            if refine {
                app.schedule_sym_refine(sp);
            }
            if to_origin {
                app.origin_at_symmetry();
            }
            if let Some(axis) = align_axis {
                let q = rotation_between(sp.normal, axis);
                app.align_sym_full(axis, q);
            }
        });
    }
}

/// Renders the Selection & Fitting section inside the accordion body.
pub fn render_selection(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("BRUSH SELECTION"), |ui| {
        ui.add(
            egui::Slider::new(&mut app.brush_radius, 2.0..=150.0).text("Brush radius (px)"),
        );
        ui.add_space(2.0);
        ui.add(
            egui::Slider::new(&mut app.expand_angle_deg, 1.0..=180.0)
                .text("Crease angle")
                .suffix("°"),
        );
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            if ui.button("Grow").clicked() {
                app.grow_selection();
            }
            if ui.button("Shrink").clicked() {
                app.shrink_selection();
            }
            ui.label(
                egui::RichText::new("(Ctrl + Wheel)")
                    .weak()
                    .small(),
            );
        });
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            if ui.button("Clear").clicked() {
                app.clear_selection();
            }
            if ui.button("Invert").clicked() {
                if let Some(m) = app.display() {
                    if app.sel.len() == m.triangle_count() {
                        let sel = Arc::make_mut(&mut app.sel);
                        for v in sel.iter_mut() {
                            *v = if *v > 0 { 0 } else { 1 };
                        }
                        app.recount_sel();
                        app.aux_dirty = true;
                    }
                }
            }
            ui.label(format!("{} faces", app.sel_count));
        });
    });

    ui.add_space(4.0);
    group_box(ui, Some("FIT GEOMETRY"), |ui| {
        ui.horizontal(|ui| {
            if ui.button("Fit plane").clicked() {
                app.fit_plane_from_selection();
            }
            if ui.button("Fit circle").clicked() {
                app.fit_circle_from_selection();
            }
        });
    });

    let plane_copy = app.plane;
    if plane_copy.is_some() {
        let p = plane_copy.unwrap();
        ui.add_space(4.0);
        let plane_title = if let Some(id) = app.selected_plane_id {
            if let Some(fp) = app.planes.iter().find(|p| p.id == id) {
                format!("FITTED PLANE ({})", fp.name)
            } else {
                "FITTED PLANE".to_string()
            }
        } else {
            "FITTED PLANE".to_string()
        };
        group_box(ui, Some(&plane_title), |ui| {
            if ui.checkbox(&mut app.show_plane, "Show plane").changed() {
                if let Some(id) = app.selected_plane_id {
                    if let Some(fp) = app.planes.iter_mut().find(|p| p.id == id) {
                        fp.visible = app.show_plane;
                    }
                }
            }
            ui.label(format!(
                "Plane  N ({:.3}, {:.3}, {:.3})  off {:.3} mm",
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
            ui.add_space(3.0);
            let mut axis = None;
            let mut origin = false;
            ui.horizontal(|ui| {
                if ui.button("N → X").clicked() {
                    axis = Some(Vec3::X);
                }
                if ui.button("N → Y").clicked() {
                    axis = Some(Vec3::Y);
                }
                if ui.button("N → Z").clicked() {
                    axis = Some(Vec3::Z);
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
        });
    }

    let circle_copy = app.circle;
    if circle_copy.is_some() {
        let c = circle_copy.unwrap();
        ui.add_space(4.0);
        let circle_title = if let Some(id) = app.selected_circle_id {
            if let Some(fc) = app.circles.iter().find(|c| c.id == id) {
                format!("FITTED CIRCLE ({})", fc.name)
            } else {
                "FITTED CIRCLE".to_string()
            }
        } else {
            "FITTED CIRCLE".to_string()
        };
        group_box(ui, Some(&circle_title), |ui| {
            if ui.checkbox(&mut app.show_circle, "Show circle").changed() {
                if let Some(id) = app.selected_circle_id {
                    if let Some(fc) = app.circles.iter_mut().find(|c| c.id == id) {
                        fc.visible = app.show_circle;
                    }
                }
            }
            ui.label(format!(
                "Circle R = {:.4} mm, center ({:.2}, {:.2}, {:.2})",
                c.radius, c.center.x, c.center.y, c.center.z
            ));
            ui.label(format!(
                "Radial RMS {:.4} mm · plane RMS {:.4} mm · max {:.4} mm",
                c.radial_rms.sqrt(),
                c.plane_rms.sqrt(),
                c.radial_max
            ));
            ui.add_space(3.0);
            let mut axis = None;
            let mut origin = false;
            ui.horizontal(|ui| {
                if ui.button("Axis → X").clicked() {
                    axis = Some(Vec3::X);
                }
                if ui.button("Axis → Y").clicked() {
                    axis = Some(Vec3::Y);
                }
                if ui.button("Axis → Z").clicked() {
                    axis = Some(Vec3::Z);
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
        });
    }
}

/// Renders the Coordinate System section inside the accordion body.
pub fn render_coordinates(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("CENTER ON AXES"), |ui| {
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
    group_box(ui, Some("HISTORY & RESET"), |ui| {
        ui.horizontal(|ui| {
            if ui.button("Undo").clicked() {
                app.undo();
            }
            if ui.button("Reset mesh").clicked() {
                app.reset_mesh();
            }
            ui.label(format!("{} undo steps", app.undo.len()));
        });
    });
}
