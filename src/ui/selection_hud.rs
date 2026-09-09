use crate::app::App;
use eframe::egui;

/// Renders a floating HUD pill at the top-center of the 3D viewport when faces are selected.
/// Displays the number of selected faces and provides an "Unselect" button to clear the selection.
pub fn render_selection_hud(app: &mut App, ui: &mut egui::Ui, viewport_rect: egui::Rect) {
    if app.sel_count == 0 {
        return;
    }

    let hud_pos = egui::pos2(viewport_rect.center().x, viewport_rect.top() + 12.0);

    egui::Area::new(egui::Id::new("viewport_selection_hud"))
        .fixed_pos(hud_pos)
        .pivot(egui::Align2::CENTER_TOP)
        .order(egui::Order::Foreground)
        .show(ui.ctx(), |ui| {
            egui::Frame::new()
                .fill(egui::Color32::from_rgba_unmultiplied(18, 22, 30, 235))
                .stroke(egui::Stroke::new(
                    1.0,
                    egui::Color32::from_rgba_unmultiplied(255, 150, 40, 110),
                ))
                .corner_radius(16.0)
                .inner_margin(egui::Margin::symmetric(14, 6))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        // Orange indicator dot
                        let (dot_rect, _) =
                            ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::empty());
                        ui.painter().circle_filled(
                            dot_rect.center(),
                            4.0,
                            egui::Color32::from_rgb(255, 155, 45),
                        );

                        ui.add_space(2.0);

                        // Selected faces counter
                        let count_str = format_number(app.sel_count);
                        ui.label(
                            egui::RichText::new(format!("{count_str} faces selected"))
                                .strong()
                                .size(12.5)
                                .color(egui::Color32::from_rgb(230, 235, 245)),
                        );

                        ui.add_space(6.0);
                        ui.separator();
                        ui.add_space(4.0);

                        // Bridge shortcut button when 2 separate clusters are selected
                        let mut has_two_clusters = false;
                        if let Some(curr) = app.current.as_ref() {
                            let topo = app.topology.clone();
                            if crate::geom::bridge::detect_selection_clusters(curr, &app.sel, topo.as_deref()).is_ok() {
                                has_two_clusters = true;
                            }
                        }

                        if has_two_clusters {
                            let is_active = app.bridge_preview_active;
                            let (fill_col, stroke_col, text_col) = if is_active {
                                (
                                    egui::Color32::from_rgba_unmultiplied(45, 175, 115, 100),
                                    egui::Color32::from_rgba_unmultiplied(80, 230, 160, 180),
                                    egui::Color32::from_rgb(220, 255, 235),
                                )
                            } else {
                                (
                                    egui::Color32::from_rgba_unmultiplied(35, 140, 95, 45),
                                    egui::Color32::from_rgba_unmultiplied(50, 180, 120, 90),
                                    egui::Color32::from_rgb(140, 240, 190),
                                )
                            };

                            let bridge_btn = egui::Button::new(
                                egui::RichText::new("🌉 Bridge")
                                    .size(11.5)
                                    .color(text_col),
                            )
                            .fill(fill_col)
                            .stroke(egui::Stroke::new(1.0, stroke_col))
                            .corner_radius(10.0);

                            let tooltip = if is_active {
                                "Hide bridge preview & deactivate bridge mode"
                            } else {
                                "Open Contour Bridge tool & view bridge preview"
                            };

                            if ui
                                .add(bridge_btn)
                                .on_hover_text(tooltip)
                                .clicked()
                            {
                                if is_active {
                                    app.set_bridge_preview_active(false);
                                } else {
                                    app.active_section = Some(crate::ui::ToolSection::Selection);
                                    app.set_bridge_preview_active(true);
                                }
                            }

                            ui.add_space(4.0);
                        }

                        // Hide button
                        let hide_btn = egui::Button::new(
                            egui::RichText::new("👁 Hide")
                                .size(11.5)
                                .color(egui::Color32::from_rgb(180, 210, 255)),
                        )
                        .fill(egui::Color32::from_rgba_unmultiplied(60, 110, 200, 45))
                        .stroke(egui::Stroke::new(
                            1.0,
                            egui::Color32::from_rgba_unmultiplied(100, 160, 255, 90),
                        ))
                        .corner_radius(10.0);

                        if ui
                            .add(hide_btn)
                            .on_hover_text("Hide selected faces (H) to view and select geometry behind")
                            .clicked()
                        {
                            app.hide_selection();
                        }

                        ui.add_space(4.0);

                        // Unselect button
                        let btn = egui::Button::new(
                            egui::RichText::new("Unselect")
                                .size(11.5)
                                .color(egui::Color32::from_rgb(255, 200, 150)),
                        )
                        .fill(egui::Color32::from_rgba_unmultiplied(255, 120, 30, 35))
                        .stroke(egui::Stroke::new(
                            1.0,
                            egui::Color32::from_rgba_unmultiplied(255, 140, 40, 80),
                        ))
                        .corner_radius(10.0);

                        if ui.add(btn).clicked() {
                            app.clear_selection();
                        }
                    });
                });
        });
}

fn format_number(n: usize) -> String {
    let s = n.to_string();
    let mut result = String::new();
    let chars: Vec<char> = s.chars().rev().collect();
    for (i, ch) in chars.iter().enumerate() {
        if i > 0 && i % 3 == 0 {
            result.push(',');
        }
        result.push(*ch);
    }
    result.chars().rev().collect()
}
