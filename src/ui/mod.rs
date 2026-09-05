pub mod accordion;
pub mod sections;

use crate::app::{App, Mode};
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

/// Renders the top tool mode banner and mode selection buttons (Orbit, Select, Erase, Pick Line).
pub fn render_mode_header(app: &mut App, ui: &mut egui::Ui) {
    let (name, desc, accent) = match app.mode {
        Mode::Orbit => (
            "ORBIT",
            "LMB drag = orbit · MMB = pan · wheel = zoom".to_string(),
            (110, 140, 255),
        ),
        Mode::BrushAdd => (
            "SELECT",
            "LMB drag = paint-select faces under the brush".to_string(),
            (255, 150, 40),
        ),
        Mode::BrushErase => (
            "ERASE",
            "LMB drag = erase faces from the selection".to_string(),
            (255, 70, 70),
        ),
        Mode::SymPickLine => (
            "PICK LINE",
            if app.sym_pick.len() >= 2 {
                "Line ready · Click 'Calculate' in Symmetry panel".to_string()
            } else {
                format!(
                    "Click point {} of 2 on the mesh · Esc = cancel",
                    app.sym_pick.len() + 1
                )
            },
            (40, 220, 255),
        ),
    };

    let bg = egui::Color32::from_rgba_unmultiplied(accent.0, accent.1, accent.2, 30);
    let fg = egui::Color32::from_rgb(accent.0, accent.1, accent.2);
    egui::Frame::new()
        .fill(bg)
        .corner_radius(6.0)
        .inner_margin(egui::Margin::symmetric(10, 8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(egui::RichText::new(name).strong().size(19.0).color(fg));
            ui.label(desc);
        });

    ui.add_space(6.0);
    let mut new_mode = None;
    ui.horizontal(|ui| {
        if ui.button("Orbit").clicked() {
            new_mode = Some(Mode::Orbit);
        }
        if ui.button("Select").clicked() {
            new_mode = Some(Mode::BrushAdd);
        }
        if ui.button("Erase").clicked() {
            new_mode = Some(Mode::BrushErase);
        }
        if ui.button("Pick Line").clicked() {
            new_mode = Some(Mode::SymPickLine);
        }
    });
    if let Some(m) = new_mode {
        if m != app.mode {
            app.mode = m;
        }
    }
    ui.add_space(4.0);
    ui.separator();
    ui.add_space(4.0);
}

/// Renders the complete left panel with the top mode banner and the accordion sections.
pub fn render_left_panel(app: &mut App, ui: &mut egui::Ui) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        // Top tool mode section (Orbit, Select, Erase, Pick Line)
        render_mode_header(app, ui);

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
