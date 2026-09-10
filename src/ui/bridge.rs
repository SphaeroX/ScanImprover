//! Contour bridge tool section (stitches two selected regions across a gap).

use crate::app::App;
use crate::geom::bridge::BridgeMethod;
use crate::ui::accordion::group_box;
use crate::ui::theme;
use egui;

pub fn render_bridge_section(app: &mut App, ui: &mut egui::Ui) {
    group_box(ui, Some("Contour bridge"), |ui| {
        if app.sel_count == 0 {
            ui.label(
                egui::RichText::new("Paint 2 separate spots across a hole to bridge the contour.")
                    .size(11.0)
                    .color(theme::pal().text_muted),
            );
            return;
        }
        let clusters = match app.bridge_clusters() {
            Ok((ca, cb)) => Some((ca.boundary_chain.len(), cb.boundary_chain.len())),
            Err(err_msg) => {
                ui.colored_label(theme::pal().warn, err_msg.to_string());
                None
            }
        };
        let Some((na, nb)) = clusters else {
            return;
        };
        ui.colored_label(
            theme::pal().success,
            format!("2 bridgeheads: A ({na} v) <-> B ({nb} v)"),
        );
        ui.add_space(3.0);

        let mut new_config = app.bridge_config;
        let mut config_changed = false;
        let mut do_apply_bridge = false;

        ui.label(
            egui::RichText::new("Curvature algorithm")
                .strong()
                .size(11.5),
        );
        ui.horizontal_wrapped(|ui| {
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
        if ui
            .add(
                egui::Slider::new(&mut new_config.segments, 1..=30)
                    .step_by(1.0)
                    .text("Segments"),
            )
            .changed()
        {
            config_changed = true;
        }
        if new_config.method == BridgeMethod::CubicHermite {
            ui.horizontal(|ui| {
                if ui
                    .add(
                        egui::Slider::new(&mut new_config.tension, 0.0..=2.0)
                            .step_by(0.05)
                            .text("Tension"),
                    )
                    .changed()
                {
                    config_changed = true;
                }
                if ui
                    .small_button("1.0")
                    .on_hover_text("Natural tangent continuation")
                    .clicked()
                {
                    new_config.tension = 1.0;
                    config_changed = true;
                }
            });
        }
        ui.horizontal(|ui| {
            if ui
                .add(
                    egui::Slider::new(&mut new_config.bulge, -1.0..=1.0)
                        .step_by(0.05)
                        .text("Bulge"),
                )
                .changed()
            {
                config_changed = true;
            }
            if ui.small_button("Flat").clicked() {
                new_config.bulge = 0.0;
                config_changed = true;
            }
        });
        ui.horizontal(|ui| {
            if ui
                .checkbox(&mut new_config.flip_twist, "Flip twist")
                .on_hover_text("Invert vertex pairing to eliminate cross-twisting")
                .changed()
            {
                config_changed = true;
            }
            if ui
                .checkbox(&mut new_config.flip_normals, "Flip normals")
                .on_hover_text("Invert surface normals and triangle winding")
                .changed()
            {
                config_changed = true;
            }
            if ui
                .checkbox(&mut app.bridge_preview_active, "3D preview")
                .changed()
            {
                app.update_bridge_preview();
            }
        });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if theme::primary_button(ui, "Apply bridge")
                .on_hover_text(
                    "Permanently stitch the bridge into the mesh geometry and divide the hole",
                )
                .clicked()
            {
                do_apply_bridge = true;
            }
            if ui.button("Clear selection").clicked() {
                app.clear_selection();
            }
        });

        if config_changed {
            app.set_bridge_config(new_config);
        }
        if do_apply_bridge {
            app.apply_bridge();
        }
    });
}
