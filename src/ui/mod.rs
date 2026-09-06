pub mod accordion;
pub mod object_browser;
pub mod sections;
pub mod selection_hud;

use crate::app::App;
use accordion::{accordion_body, accordion_header};
use eframe::egui;

/// Available sections in the left accordion menu.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ToolSection {
    Decimation,
    Symmetry,
    Selection,
    Coordinates,
}

/// Renders the complete left panel with the accordion sections.
pub fn render_left_panel(app: &mut App, ui: &mut egui::Ui) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        // Section 1: Decimation
        let dec_badge = if app.preview.is_some() {
            Some("Preview active")
        } else {
            None
        };
        let dec_open = app.active_section == Some(ToolSection::Decimation);
        if accordion_header(ui, "Decimation", dec_open, dec_badge) {
            app.active_section = if dec_open {
                None
            } else {
                Some(ToolSection::Decimation)
            };
        }
        if dec_open {
            accordion_body(ui, |ui| {
                sections::render_decimation(app, ui);
            });
        }
        ui.add_space(3.0);

        // Section 2: Symmetry plane
        let sym_badge = if app.sym.is_some() {
            Some("Plane active")
        } else {
            None
        };
        let sym_open = app.active_section == Some(ToolSection::Symmetry);
        if accordion_header(ui, "Symmetry plane", sym_open, sym_badge) {
            app.active_section = if sym_open {
                None
            } else {
                Some(ToolSection::Symmetry)
            };
        }
        if sym_open {
            accordion_body(ui, |ui| {
                sections::render_symmetry(app, ui);
            });
        }
        ui.add_space(3.0);

        // Section 3: Face selection
        let sel_badge_str = if app.sel_count > 0 {
            Some(format!("{} faces", app.sel_count))
        } else {
            None
        };
        let sel_open = app.active_section == Some(ToolSection::Selection);
        if accordion_header(ui, "Face selection", sel_open, sel_badge_str.as_deref()) {
            app.active_section = if sel_open {
                None
            } else {
                Some(ToolSection::Selection)
            };
        }
        if sel_open {
            accordion_body(ui, |ui| {
                sections::render_selection(app, ui);
            });
        }
        ui.add_space(3.0);

        // Section 4: Coordinate system
        let coord_badge_str = if !app.undo.is_empty() {
            Some(format!("{} undos", app.undo.len()))
        } else {
            None
        };
        let coord_open = app.active_section == Some(ToolSection::Coordinates);
        if accordion_header(ui, "Coordinate system", coord_open, coord_badge_str.as_deref()) {
            app.active_section = if coord_open {
                None
            } else {
                Some(ToolSection::Coordinates)
            };
        }
        if coord_open {
            accordion_body(ui, |ui| {
                sections::render_coordinates(app, ui);
            });
        }
        ui.add_space(6.0);
    });
}
