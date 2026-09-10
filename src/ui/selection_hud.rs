//! Floating pill at the top of the viewport summarising the face selection
//! with the most common follow-up actions.

use crate::app::App;
use crate::ui::theme;
use egui;

pub fn render_selection_hud(app: &mut App, ui: &mut egui::Ui, viewport_rect: egui::Rect) {
    if app.sel_count == 0 {
        return;
    }
    let hud_pos = egui::pos2(viewport_rect.center().x, viewport_rect.top() + 12.0);
    let has_two_clusters = app.has_bridge_clusters();

    egui::Area::new(egui::Id::new("viewport_selection_hud"))
        .fixed_pos(hud_pos)
        .pivot(egui::Align2::CENTER_TOP)
        .order(egui::Order::Foreground)
        .show(ui.ctx(), |ui| {
            theme::floating_frame()
                .corner_radius(16.0)
                .inner_margin(egui::Margin::symmetric(12, 5))
                .stroke(egui::Stroke::new(
                    1.0,
                    theme::pal().select_accent.gamma_multiply(0.5),
                ))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        let (dot_rect, _) =
                            ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::empty());
                        ui.painter().circle_filled(
                            dot_rect.center(),
                            4.0,
                            theme::pal().select_accent,
                        );
                        ui.add_space(2.0);
                        ui.label(
                            egui::RichText::new(format!(
                                "{} faces selected",
                                theme::format_count(app.sel_count)
                            ))
                            .strong()
                            .size(12.5)
                            .color(theme::pal().text_strong),
                        );
                        ui.add_space(6.0);
                        theme::vsep(ui);

                        if has_two_clusters {
                            let is_active = app.bridge_preview_active;
                            let btn = pill_button(
                                "Bridge",
                                theme::pal().success,
                                if is_active { 0.45 } else { 0.18 },
                            );
                            let tooltip = if is_active {
                                "Hide bridge preview & deactivate bridge mode"
                            } else {
                                "Open the Contour Bridge tool & preview the bridge"
                            };
                            if ui.add(btn).on_hover_text(tooltip).clicked() {
                                if is_active {
                                    app.set_bridge_preview_active(false);
                                } else {
                                    app.open_section(crate::ui::ToolSection::Selection);
                                    app.set_bridge_preview_active(true);
                                }
                            }
                        }
                        if ui
                            .add(pill_button("Hide", theme::pal().accent, 0.22))
                            .on_hover_text(
                                "Hide selected faces (H) to view and select geometry behind",
                            )
                            .clicked()
                        {
                            app.hide_selection();
                        }
                        if ui
                            .add(pill_button("Unselect", theme::pal().select_accent, 0.18))
                            .on_hover_text("Clear the selection")
                            .clicked()
                        {
                            app.clear_selection();
                        }
                    });
                });
        });
}

fn pill_button(label: &str, tint: egui::Color32, fill_alpha: f32) -> egui::Button<'_> {
    egui::Button::new(
        egui::RichText::new(label)
            .size(11.5)
            .color(theme::pal().text_strong),
    )
    .fill(tint.gamma_multiply(fill_alpha))
    .stroke(egui::Stroke::new(1.0, tint.gamma_multiply(0.6)))
    .corner_radius(10.0)
}
