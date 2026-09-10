//! Bottom status bar: last message, background activity feed and the
//! navigation hint for the current mode.

use crate::app::{App, Mode};
use crate::ui::theme;
use egui;

pub fn render_status_bar(app: &mut App, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 8.0;
        let (status_color, status_text) = classify_status(&app.status);
        ui.label(
            egui::RichText::new(status_text)
                .size(12.0)
                .color(status_color),
        );

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let hint = match app.mode {
                Mode::Orbit => "RMB orbit · MMB / Shift+RMB pan · Wheel zoom · LMB paint · Shift+LMB erase · Dbl-click region · Ctrl+Wheel grow/shrink · Alt+Wheel brush",
                Mode::SymPickLine => {
                    if app.sym_pick.len() >= 2 {
                        "Symmetry line ready · Esc = reset"
                    } else if app.sym_pick.is_empty() {
                        "Symmetry line: click point 1 of 2 on the mesh · Esc = cancel"
                    } else {
                        "Symmetry line: click point 2 of 2 on the mesh · Esc = cancel"
                    }
                }
            };
            ui.label(egui::RichText::new(hint).size(11.0).color(theme::pal().text_faint));

            // Activity feed: one entry per running background job.
            let activities: Vec<_> = app.worker.activities().to_vec();
            if !activities.is_empty() {
                ui.add_space(6.0);
                theme::vsep(ui);
                for a in activities.iter().rev().take(3) {
                    let stage = a.progress.stage();
                    let mut text = a.label.clone();
                    if !stage.is_empty() {
                        text.push_str(" · ");
                        text.push_str(&stage);
                    }
                    let secs = a.elapsed_secs();
                    if secs >= 1.0 {
                        text.push_str(&format!(" ({secs:.0} s)"));
                    }
                    if let Some(f) = a.progress.fraction() {
                        ui.add(
                            egui::ProgressBar::new(f)
                                .desired_width(70.0)
                                .desired_height(8.0)
                                .fill(theme::pal().accent),
                        );
                    }
                    ui.label(egui::RichText::new(text).size(11.5).color(theme::pal().accent_text));
                    ui.add(egui::Spinner::new().size(12.0).color(theme::pal().accent));
                }
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(120));
            }
        });
    });
}

/// Picks a color for the status message based on its content.
fn classify_status(status: &str) -> (egui::Color32, &str) {
    let lower = status.to_ascii_lowercase();
    if lower.contains("failed") || lower.contains("error") || lower.contains("cannot") {
        (theme::pal().warn, status)
    } else if lower.contains("…") {
        (theme::pal().accent_text, status)
    } else {
        (theme::pal().text, status)
    }
}
