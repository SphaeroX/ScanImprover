//! Accordion section headers / bodies and the grouped control box used
//! inside the tool panel.

use crate::ui::theme;
use egui;

/// Renders an accordion header bar with an expand/collapse arrow, title,
/// optional badge and busy spinner. Returns true if the user clicked it.
pub fn accordion_header(
    ui: &mut egui::Ui,
    title: &str,
    is_open: bool,
    badge: Option<&str>,
    busy: bool,
) -> bool {
    let desired_size = egui::vec2(ui.available_width(), 30.0);
    let (rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::click());
    let is_hovered = response.hovered();
    if is_hovered {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }

    let bg_color = if is_open {
        theme::pal().accent.gamma_multiply(0.16)
    } else if is_hovered {
        theme::pal().panel_hover
    } else {
        theme::pal().panel_elevated.gamma_multiply(0.7)
    };
    ui.painter().rect_filled(rect, 6.0, bg_color);
    if is_open {
        let accent_rect =
            egui::Rect::from_min_max(rect.min, egui::pos2(rect.min.x + 3.0, rect.max.y));
        ui.painter().rect_filled(
            accent_rect,
            egui::CornerRadius {
                nw: 6,
                sw: 6,
                ne: 0,
                se: 0,
            },
            theme::pal().accent,
        );
    }

    let content_rect = rect.shrink2(egui::vec2(10.0, 0.0));
    let mut child_ui = ui.new_child(egui::UiBuilder::new().max_rect(content_rect));
    child_ui.style_mut().interaction.selectable_labels = false;
    child_ui.horizontal_centered(|ui| {
        let arrow_col = if is_open {
            theme::pal().accent_text
        } else if is_hovered {
            theme::pal().text_strong
        } else {
            theme::pal().text_muted
        };
        let (arrow_rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::empty());
        theme::paint_arrow(ui.painter(), arrow_rect.center(), is_open, arrow_col);
        ui.add_space(4.0);
        let text_color = if is_open || is_hovered {
            theme::pal().text_strong
        } else {
            theme::pal().text
        };
        ui.add(
            egui::Label::new(
                egui::RichText::new(title)
                    .family(theme::title_family(ui.ctx()))
                    .strong()
                    .size(13.0)
                    .color(text_color),
            )
            .selectable(false)
            .sense(egui::Sense::empty()),
        );
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if let Some(b) = badge {
                let (fg, bg) = if is_open {
                    (
                        theme::pal().accent_text,
                        theme::pal().accent.gamma_multiply(0.25),
                    )
                } else {
                    (theme::pal().text_muted, theme::pal().panel_hover)
                };
                theme::badge(ui, b, fg, bg);
            }
            if busy {
                ui.add(egui::Spinner::new().size(12.0).color(theme::pal().accent));
            }
        });
    });

    response.clicked()
}

/// Helper container for styling the content body of an open accordion item.
pub fn accordion_body<R>(
    ui: &mut egui::Ui,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    egui::Frame::new()
        .inner_margin(egui::Margin {
            left: 12,
            right: 4,
            top: 6,
            bottom: 6,
        })
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 5.0;
            add_contents(ui)
        })
}

/// Helper container to group related controls cleanly within a section.
pub fn group_box<R>(
    ui: &mut egui::Ui,
    title: Option<&str>,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    theme::card_frame().show(ui, |ui| {
        ui.set_min_width(ui.available_width());
        if let Some(t) = title {
            theme::caption(ui, t);
            ui.add_space(2.0);
        }
        add_contents(ui)
    })
}
