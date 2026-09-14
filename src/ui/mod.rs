//! egui user interface: panels, toolbar, tool sections and overlays.

pub mod accordion;
pub mod alignment;
pub mod bridge;
pub mod experimental;
pub mod gizmo;
pub mod object_browser;
pub mod repair;
pub mod retopo;
pub mod sections;
pub mod selection_hud;
pub mod shortcuts;
pub mod status_bar;
pub mod theme;
pub mod toolbar;

use crate::app::App;
use accordion::{accordion_body, accordion_header};

/// Available sections in the left accordion menu.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ToolSection {
    Symmetry,
    Selection,
    FaceGroups,
    Coordinates,
    Repair,
    Decimation,
    Experimental,
}

impl ToolSection {
    const ALL: [ToolSection; 7] = [
        ToolSection::Symmetry,
        ToolSection::Selection,
        ToolSection::FaceGroups,
        ToolSection::Coordinates,
        ToolSection::Repair,
        ToolSection::Decimation,
        ToolSection::Experimental,
    ];

    pub fn key(self) -> &'static str {
        match self {
            ToolSection::Symmetry => "symmetry",
            ToolSection::Selection => "selection",
            ToolSection::FaceGroups => "face_groups",
            ToolSection::Coordinates => "coordinates",
            ToolSection::Repair => "repair",
            ToolSection::Decimation => "decimation",
            ToolSection::Experimental => "experimental",
        }
    }

    pub fn from_key(key: &str) -> Option<ToolSection> {
        Self::ALL.into_iter().find(|s| s.key() == key.trim())
    }

    fn title(self) -> &'static str {
        match self {
            ToolSection::Symmetry => "Symmetry plane",
            ToolSection::Selection => "Face selection",
            ToolSection::FaceGroups => "Face groups",
            ToolSection::Coordinates => "Coordinate system",
            ToolSection::Repair => "Mesh repair",
            ToolSection::Decimation => "Decimation",
            ToolSection::Experimental => "Experimental",
        }
    }
}

/// Badge text shown on a collapsed section header.
fn section_badge(app: &App, section: ToolSection) -> Option<String> {
    match section {
        ToolSection::Symmetry => app.sym.map(|_| "Plane active".to_string()),
        ToolSection::Selection => (app.sel_count > 0).then(|| format!("{} faces", app.sel_count)),
        ToolSection::FaceGroups => {
            (!app.face_groups.is_empty()).then(|| format!("{} groups", app.face_groups.len()))
        }
        ToolSection::Coordinates => {
            (!app.undo.is_empty()).then(|| format!("{} undos", app.undo.len()))
        }
        ToolSection::Repair => {
            if !app.repair_holes.is_empty() {
                Some(format!("{} holes", app.repair_holes.len()))
            } else if app.repair_health.as_ref().is_some_and(|h| h.is_watertight) {
                Some("Watertight".to_string())
            } else {
                None
            }
        }
        ToolSection::Decimation => app.preview.as_ref().map(|_| "Preview active".to_string()),
        ToolSection::Experimental => None,
    }
}

/// True when a background job related to the section is running.
fn section_busy(app: &App, section: ToolSection) -> bool {
    match section {
        ToolSection::Symmetry => app.sym_job.is_some(),
        ToolSection::Selection => app.freeform_job.is_some(),
        ToolSection::FaceGroups => app.groups_job.is_some(),
        ToolSection::Coordinates => false,
        ToolSection::Repair => {
            app.analysis_job.is_some() || app.edit_job.is_some() || app.solve_job.is_some()
        }
        ToolSection::Decimation => app.dec_job.is_some() || app.dev_job.is_some(),
        ToolSection::Experimental => false,
    }
}

/// Renders the complete left panel with the accordion sections.
pub fn render_left_panel(app: &mut App, ui: &mut egui::Ui) {
    // Fixed Brush Selection tool permanently visible above accordions
    sections::render_brush_selection(app, ui);
    ui.add_space(6.0);

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for section in ToolSection::ALL {
                let open = app.is_section_open(section);
                if section == ToolSection::FaceGroups && !open {
                    app.hover_group = None;
                }
                let badge = section_badge(app, section);
                let busy = section_busy(app, section);
                if accordion_header(ui, section.title(), open, badge.as_deref(), busy) {
                    app.toggle_section(section);
                }
                if open {
                    accordion_body(ui, |ui| match section {
                        ToolSection::Symmetry => sections::render_symmetry(app, ui),
                        ToolSection::Selection => sections::render_selection(app, ui),
                        ToolSection::FaceGroups => sections::render_face_groups(app, ui),
                        ToolSection::Coordinates => sections::render_coordinates(app, ui),
                        ToolSection::Repair => repair::render_repair(app, ui),
                        ToolSection::Decimation => sections::render_decimation(app, ui),
                        ToolSection::Experimental => experimental::render_experimental(app, ui),
                    });
                }
                ui.add_space(3.0);
            }
            ui.add_space(6.0);
        });
}
