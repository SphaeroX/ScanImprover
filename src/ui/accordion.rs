use eframe::egui;

/// Renders an accordion header bar with an expand/collapse arrow, title, and optional badge.
/// Features a modern CAD-style look with left accent indicator and hover effects.
/// Returns true if the user clicked the header.
pub fn accordion_header(
    ui: &mut egui::Ui,
    title: &str,
    is_open: bool,
    badge: Option<&str>,
) -> bool {
    let desired_size = egui::vec2(ui.available_width(), 32.0);
    let (rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::click());

    let is_hovered = response.hovered();
    if is_hovered {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }

    let bg_color = if is_open {
        egui::Color32::from_rgba_unmultiplied(55, 75, 115, 65)
    } else if is_hovered {
        egui::Color32::from_rgba_unmultiplied(55, 60, 72, 50)
    } else {
        egui::Color32::from_rgba_unmultiplied(35, 38, 46, 40)
    };

    let stroke = if is_open {
        egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(110, 155, 240, 95))
    } else if is_hovered {
        egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 28))
    } else {
        egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 12))
    };

    let text_color = if is_open {
        ui.visuals().strong_text_color()
    } else if is_hovered {
        ui.visuals().strong_text_color()
    } else {
        ui.visuals().text_color()
    };

    // Draw header background
    ui.painter().rect(
        rect,
        5.0,
        bg_color,
        stroke,
        egui::StrokeKind::Inside,
    );

    // Left accent indicator for open section
    if is_open {
        let accent_rect = egui::Rect::from_min_max(
            rect.min,
            egui::pos2(rect.min.x + 3.5, rect.max.y),
        );
        ui.painter().rect_filled(
            accent_rect,
            egui::CornerRadius { nw: 5, sw: 5, ne: 0, se: 0 },
            egui::Color32::from_rgb(120, 175, 255),
        );
    }

    // Render content in a child UI scoped to rect with padding
    let content_rect = rect.shrink2(egui::vec2(10.0, 0.0));
    let mut child_ui = ui.new_child(egui::UiBuilder::new().max_rect(content_rect));
    child_ui.horizontal_centered(|ui| {
        let arrow = if is_open { "▾" } else { "▸" };
        ui.label(
            egui::RichText::new(arrow)
                .strong()
                .size(14.0)
                .color(if is_open {
                    egui::Color32::from_rgb(120, 170, 255)
                } else if is_hovered {
                    egui::Color32::from_rgb(200, 210, 230)
                } else {
                    egui::Color32::from_rgb(150, 155, 165)
                }),
        );
        ui.add_space(2.0);
        ui.label(
            egui::RichText::new(title)
                .strong()
                .size(13.5)
                .color(text_color),
        );

        if let Some(b) = badge {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let badge_bg = if is_open {
                    egui::Color32::from_rgba_unmultiplied(100, 140, 220, 45)
                } else {
                    egui::Color32::from_rgba_unmultiplied(255, 255, 255, 18)
                };
                let badge_fg = if is_open {
                    egui::Color32::from_rgb(145, 195, 255)
                } else {
                    egui::Color32::from_rgb(175, 180, 190)
                };
                egui::Frame::new()
                    .fill(badge_bg)
                    .corner_radius(3.0)
                    .inner_margin(egui::Margin::symmetric(5, 2))
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(b)
                                .size(10.5)
                                .color(badge_fg),
                        );
                    });
            });
        }
    });

    response.clicked()
}

/// Helper container for styling the content body of an open accordion item.
pub fn accordion_body<R>(
    ui: &mut egui::Ui,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    egui::Frame::new()
        .fill(egui::Color32::from_rgba_unmultiplied(25, 28, 34, 40))
        .stroke(egui::Stroke::new(
            1.0,
            egui::Color32::from_rgba_unmultiplied(100, 140, 220, 35),
        ))
        .corner_radius(5.0)
        .inner_margin(egui::Margin::symmetric(10, 8))
        .show(ui, |ui| {
            ui.add_space(2.0);
            let ret = add_contents(ui);
            ui.add_space(2.0);
            ret
        })
}

/// Helper container to group related controls cleanly within a section.
pub fn group_box<R>(
    ui: &mut egui::Ui,
    title: Option<&str>,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    egui::Frame::new()
        .fill(egui::Color32::from_rgba_unmultiplied(35, 38, 46, 50))
        .stroke(egui::Stroke::new(
            1.0,
            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 10),
        ))
        .corner_radius(4.0)
        .inner_margin(egui::Margin::symmetric(8, 6))
        .show(ui, |ui| {
            if let Some(t) = title {
                ui.label(
                    egui::RichText::new(t)
                        .size(11.5)
                        .color(egui::Color32::from_rgb(160, 170, 190)),
                );
                ui.add_space(3.0);
            }
            add_contents(ui)
        })
}
