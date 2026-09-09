pub mod accordion;
pub mod alignment;
pub mod object_browser;
pub mod repair;
pub mod sections;
pub mod selection_hud;

use crate::app::App;
use accordion::{accordion_body, accordion_header};
use eframe::egui;

/// Available sections in the left accordion menu.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ToolSection {
    Symmetry,
    Selection,
    FaceGroups,
    Coordinates,
    Repair,
    Decimation,
}

/// Renders the complete left panel with the accordion sections.
pub fn render_left_panel(app: &mut App, ui: &mut egui::Ui) {
    // Fixed Brush Selection tool permanently visible above accordions
    sections::render_brush_selection(app, ui);
    ui.add_space(4.0);
    ui.separator();
    ui.add_space(4.0);

    egui::ScrollArea::vertical().show(ui, |ui| {
        // Section 1: Symmetry plane
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

        // Section 2: Face selection
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

        // Section 3: Face groups
        let fg_badge_str = if !app.face_groups.is_empty() {
            Some(format!("{} groups", app.face_groups.len()))
        } else {
            None
        };
        let fg_open = app.active_section == Some(ToolSection::FaceGroups);
        if !fg_open {
            app.hover_group = None;
        }
        if accordion_header(ui, "Face groups", fg_open, fg_badge_str.as_deref()) {
            app.active_section = if fg_open {
                None
            } else {
                Some(ToolSection::FaceGroups)
            };
        }
        if fg_open {
            accordion_body(ui, |ui| {
                sections::render_face_groups(app, ui);
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
        if accordion_header(
            ui,
            "Coordinate system",
            coord_open,
            coord_badge_str.as_deref(),
        ) {
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
        ui.add_space(3.0);

        // Section 5: Mesh repair
        let repair_badge_str = if !app.repair_holes.is_empty() {
            Some(format!("{} holes", app.repair_holes.len()))
        } else if let Some(h) = &app.repair_health {
            if h.is_watertight {
                Some("Watertight".to_string())
            } else {
                None
            }
        } else {
            None
        };
        let repair_open = app.active_section == Some(ToolSection::Repair);
        if accordion_header(
            ui,
            "Mesh repair",
            repair_open,
            repair_badge_str.as_deref(),
        ) {
            app.active_section = if repair_open {
                None
            } else {
                Some(ToolSection::Repair)
            };
        }
        if repair_open {
            accordion_body(ui, |ui| {
                repair::render_repair(app, ui);
            });
        }
        ui.add_space(3.0);

        // Section 6: Decimation
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
        ui.add_space(6.0);
    });
}
