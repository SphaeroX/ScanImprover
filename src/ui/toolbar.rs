//! Top toolbar: file actions, history, view toggles and view presets.

use crate::app::App;
use crate::camera::{UpAxis, ViewDir};
use crate::ui::theme::{self, ThemeMode, toggle_chip, tool_button, vsep};
use egui;

pub fn render_toolbar(app: &mut App, ui: &mut egui::Ui) {
    let now = ui.input(|i| i.time);
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;

        // --- File ---------------------------------------------------------
        if tool_button(ui, "Open…", "Open a mesh file (Ctrl+O)", !app.is_editing()).clicked() {
            open_dialog(app);
        }
        let can_export = app.has_mesh() && app.export_job.is_none();
        ui.add_enabled_ui(can_export, |ui| {
            ui.menu_button(egui::RichText::new("Export…").size(12.5), |ui| {
                ui.set_min_width(190.0);
                for (ext, label) in [
                    ("stl", "Binary STL (.stl)"),
                    ("ply", "Binary PLY (.ply)"),
                    ("obj", "Wavefront OBJ (.obj)"),
                ] {
                    if ui.button(label).clicked() {
                        export_mesh_dialog(app, ext);
                        ui.close();
                    }
                }
                if !app.planes.is_empty() || !app.circles.is_empty() {
                    ui.separator();
                    if ui
                        .button("Reference geometry (STEP / DXF / Fusion)…")
                        .clicked()
                    {
                        app.export_all_references();
                        ui.close();
                    }
                }
            });
        });

        vsep(ui);

        // --- History --------------------------------------------------------
        let undo_label = if app.undo.is_empty() {
            "Undo".to_string()
        } else {
            format!("Undo ({})", app.undo.len())
        };
        if tool_button(
            ui,
            &undo_label,
            "Undo last action (Ctrl+Z)",
            !app.undo.is_empty(),
        )
        .clicked()
        {
            app.undo();
        }
        let redo_label = if app.redo.is_empty() {
            "Redo".to_string()
        } else {
            format!("Redo ({})", app.redo.len())
        };
        if tool_button(
            ui,
            &redo_label,
            "Redo (Ctrl+Y / Ctrl+Shift+Z)",
            !app.redo.is_empty(),
        )
        .clicked()
        {
            app.redo();
        }

        vsep(ui);

        // --- Display toggles -----------------------------------------------
        if toggle_chip(ui, &mut app.show_wireframe, "Wireframe", "Show mesh edges").changed() {
            app.wire_dirty = true;
        }
        toggle_chip(ui, &mut app.show_bbox, "Bounds", "Show the bounding box");
        toggle_chip(ui, &mut app.show_triad, "Origin", "Show the origin axes");
        toggle_chip(ui, &mut app.show_grid, "Grid", "Show the ground grid (G)");
        toggle_chip(
            ui,
            &mut app.show_object_browser,
            "Objects",
            "Show the object browser",
        );

        vsep(ui);

        // --- Views ----------------------------------------------------------
        for (label, dir, tip) in [
            ("Top", ViewDir::Top, "Top view (2)"),
            ("Front", ViewDir::Front, "Front view"),
            ("Right", ViewDir::Right, "Right view"),
            ("Iso", ViewDir::Iso, "Isometric view (4)"),
        ] {
            if tool_button(ui, label, tip, app.has_mesh()).clicked() {
                app.camera.animate_view(dir, now);
            }
        }
        ui.menu_button(egui::RichText::new("More views…").size(12.5), |ui| {
            for (label, dir) in [
                ("Bottom", ViewDir::Bottom),
                ("Back", ViewDir::Back),
                ("Left", ViewDir::Left),
            ] {
                if ui.button(label).clicked() {
                    app.camera.animate_view(dir, now);
                    ui.close();
                }
            }
        })
        .response
        .on_hover_text("More view presets");
        if tool_button(
            ui,
            "Fit",
            "Fit the model, or the selection, into view (F)",
            app.has_mesh(),
        )
        .clicked()
        {
            app.fit_view_to_selection(now);
        }
        let up_label = app.camera.up_axis.label();
        ui.menu_button(
            egui::RichText::new(format!("Up: {up_label}")).size(12.5),
            |ui| {
                for axis in [UpAxis::Y, UpAxis::Z] {
                    if ui
                        .selectable_label(app.camera.up_axis == axis, axis.label())
                        .clicked()
                    {
                        app.camera.set_up_axis(axis);
                        ui.close();
                    }
                }
            },
        )
        .response
        .on_hover_text("World up axis used for orbiting and the ground grid");

        let theme_label = app.theme_mode.label();
        ui.menu_button(
            egui::RichText::new(format!("Theme: {theme_label}")).size(12.5),
            |ui| {
                for m in [ThemeMode::Dark, ThemeMode::Light] {
                    if ui
                        .selectable_label(app.theme_mode == m, m.label())
                        .clicked()
                    {
                        app.theme_mode = m;
                        ui.close();
                    }
                }
            },
        )
        .response
        .on_hover_text("Color theme");

        // --- Right aligned: model info ------------------------------------
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.available_width() < 340.0 {
                return;
            }
            if let Some(m) = app.display() {
                let name = app
                    .file_path
                    .as_ref()
                    .and_then(|p| p.file_name())
                    .and_then(|s| s.to_str())
                    .unwrap_or("Mesh");
                let info = format!(
                    "{}  ·  {} tris  ·  {} verts",
                    name,
                    theme::format_count(m.triangle_count()),
                    theme::format_count(m.vertex_count())
                );
                ui.label(
                    egui::RichText::new(info)
                        .size(12.0)
                        .color(theme::pal().text_muted),
                );
                if app.preview.is_some() {
                    theme::badge(
                        ui,
                        "PREVIEW",
                        theme::pal().success,
                        theme::pal().success.gamma_multiply(0.18),
                    );
                }
            }
        });
    });
}

fn open_dialog(app: &mut App) {
    if let Some(p) = rfd::FileDialog::new()
        .add_filter("Mesh files", &["stl", "ply", "obj"])
        .pick_file()
    {
        app.open_file_async(p);
    }
}

fn export_mesh_dialog(app: &mut App, ext: &str) {
    if !app.has_mesh() {
        app.status = "Nothing to export.".to_string();
        return;
    }
    let default_name = app
        .file_path
        .as_ref()
        .and_then(|p| p.file_stem())
        .and_then(|s| s.to_str())
        .map(|s| format!("{s}.{ext}"))
        .unwrap_or_else(|| format!("mesh.{ext}"));
    if let Some(p) = rfd::FileDialog::new()
        .add_filter(ext, &[ext])
        .set_file_name(default_name)
        .save_file()
    {
        app.export_mesh_async(p);
    }
}
