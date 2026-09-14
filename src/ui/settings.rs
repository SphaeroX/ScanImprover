//! Settings window: preferences that persist between sessions, grouped into
//! pages. Adding an option takes a field in `crate::settings::Settings` (with
//! its key in `serialize` / `parse`), the matching `App` field in
//! `App::apply_settings` / `App::settings`, and a row in one of the pages.

use crate::app::App;
use crate::settings::Settings;
use crate::ui::accordion::group_box;
use crate::ui::theme;

/// Renders its options and returns true when one of them changed.
type Page = fn(&mut App, &mut egui::Ui) -> bool;

const PAGES: &[(&str, Page)] = &[("Import", import_page)];

pub fn render_settings_window(app: &mut App, ctx: &egui::Context) {
    if !app.show_settings {
        return;
    }
    let mut open = true;
    let mut changed = false;
    egui::Window::new("Settings")
        .open(&mut open)
        .collapsible(false)
        .resizable(false)
        .default_width(340.0)
        .show(ctx, |ui| {
            for &(title, page) in PAGES {
                group_box(ui, Some(title), |ui| changed |= page(app, ui));
                ui.add_space(4.0);
            }
            if let Some(path) = Settings::path() {
                ui.label(
                    egui::RichText::new(format!("Saved to {}", path.display()))
                        .size(11.0)
                        .color(theme::pal().text_muted),
                );
            }
        });
    app.show_settings = open;
    // Save right away rather than only on exit, so a crash keeps the change.
    if changed {
        app.save_settings();
    }
}

fn import_page(app: &mut App, ui: &mut egui::Ui) -> bool {
    ui.checkbox(&mut app.auto_center_on_load, "Auto-center model on load")
        .on_hover_text(
            "Move a loaded mesh so its bounding box center sits at the origin. \
             Turn off to keep the coordinates stored in the file.",
        )
        .changed()
}
