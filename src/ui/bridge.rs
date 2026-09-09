use crate::app::App;
use crate::geom::bridge::{detect_selection_clusters, BridgeMethod};
use crate::ui::accordion::group_box;
use eframe::egui;

/// Renders the Contour Bridge tool section in the sidebar.
pub fn render_bridge_section(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("CONTOUR BRIDGE"), |ui| {
        let sel_count = app.sel_count;
        let mut do_apply_bridge = false;
        let mut config_changed = false;
        let mut new_config = app.bridge_config;

        if sel_count == 0 {
            ui.label(
                egui::RichText::new("Paint 2 separate spots across a hole to bridge the contour.")
                    .size(11.0)
                    .color(egui::Color32::from_gray(160)),
            );
            return;
        }

        // Check clusters
        let mut cluster_info = None;
        if let Some(curr) = app.current.as_ref() {
            let topo = app.topology.clone();
            match detect_selection_clusters(curr, &app.sel, topo.as_deref()) {
                Ok((ca, cb)) => {
                    cluster_info = Some((ca, cb));
                }
                Err(err_msg) => {
                    ui.horizontal(|ui| {
                        ui.colored_label(
                            egui::Color32::from_rgb(240, 170, 70),
                            format!("ℹ {err_msg}"),
                        );
                    });
                }
            }
        }

        if let Some((ca, cb)) = cluster_info {
            ui.horizontal(|ui| {
                ui.colored_label(
                    egui::Color32::from_rgb(80, 220, 140),
                    format!(
                        "✔ 2 Bridgeheads: A ({} v) ↔ B ({} v)",
                        ca.boundary_chain.len(),
                        cb.boundary_chain.len()
                    ),
                );
            });

            ui.add_space(3.0);

            // 1. Algorithm Selection
            ui.label(egui::RichText::new("Curvature Algorithm:").strong().size(11.5));
            ui.horizontal(|ui| {
                for method in BridgeMethod::all() {
                    if ui
                        .selectable_value(&mut new_config.method, method, method.display_name())
                        .clicked()
                    {
                        config_changed = true;
                    }
                }
            });

            ui.add_space(2.0);

            // 2. Segments (Longitudinal resolution)
            ui.horizontal(|ui| {
                ui.label("Segments:");
                let resp = ui.add(
                    egui::Slider::new(&mut new_config.segments, 1..=30)
                        .step_by(1.0)
                        .suffix(" segs"),
                );
                if resp.changed() {
                    config_changed = true;
                }
            });

            // 3. Tension / Curvature (Cubic Hermite)
            if new_config.method == BridgeMethod::CubicHermite {
                ui.horizontal(|ui| {
                    ui.label("Tension / Curvature:");
                    let resp = ui.add(
                        egui::Slider::new(&mut new_config.tension, 0.0..=2.0)
                            .step_by(0.05),
                    );
                    if resp.changed() {
                        config_changed = true;
                    }
                    if ui.small_button("1.0").on_hover_text("Natural tangent continuation").clicked() {
                        new_config.tension = 1.0;
                        config_changed = true;
                    }
                });
            }

            // 4. Bulge / Arch Offset
            ui.horizontal(|ui| {
                ui.label("Bulge / Wölbung:");
                let resp = ui.add(
                    egui::Slider::new(&mut new_config.bulge, -1.0..=1.0)
                        .step_by(0.05),
                );
                if resp.changed() {
                    config_changed = true;
                }
                if ui.small_button("Flat (0)").clicked() {
                    new_config.bulge = 0.0;
                    config_changed = true;
                }
            });

            // 5. Toggles: Alignment / Twist, Flip Normals, Preview
            ui.horizontal(|ui| {
                if ui
                    .checkbox(&mut new_config.flip_twist, "Flip Twist")
                    .on_hover_text("Invert vertex pairing to eliminate cross-twisting")
                    .changed()
                {
                    config_changed = true;
                }

                if ui
                    .checkbox(&mut new_config.flip_normals, "Flip Normals")
                    .on_hover_text("Invert surface normals and triangle winding")
                    .changed()
                {
                    config_changed = true;
                }

                if ui.checkbox(&mut app.bridge_preview_active, "3D Preview").changed() {
                    app.update_bridge_preview();
                }
            });

            ui.add_space(4.0);

            // 6. Action Button
            ui.horizontal(|ui| {
                let btn = egui::Button::new(
                    egui::RichText::new("🌉 Apply Bridge")
                        .color(egui::Color32::WHITE)
                        .strong(),
                )
                .fill(egui::Color32::from_rgb(35, 130, 95));

                if ui
                    .add_sized([120.0, 26.0], btn)
                    .on_hover_text("Permanently stitch the bridge into the mesh geometry and divide the hole")
                    .clicked()
                {
                    do_apply_bridge = true;
                }

                if ui.button("Clear Selection").clicked() {
                    app.clear_selection();
                }
            });
        }

        if config_changed {
            app.set_bridge_config(new_config);
        }

        if do_apply_bridge {
            app.apply_bridge();
        }
    });
}
