//! Global keyboard shortcuts (ignored while a text field has focus).

use crate::app::{App, Mode};
use crate::camera::ViewDir;
use egui;

pub fn handle_global(app: &mut App, ctx: &egui::Context) {
    let now = ctx.input(|i| i.time);
    let cmd_o = ctx.input_mut(|i| {
        i.consume_shortcut(&egui::KeyboardShortcut::new(
            egui::Modifiers::COMMAND,
            egui::Key::O,
        ))
    });
    if cmd_o
        && !app.is_editing()
        && let Some(p) = rfd::FileDialog::new()
            .add_filter("Mesh files", &["stl", "ply", "obj"])
            .pick_file()
    {
        app.open_file_async(p);
    }
    let undo = ctx.input_mut(|i| {
        i.consume_shortcut(&egui::KeyboardShortcut::new(
            egui::Modifiers::COMMAND,
            egui::Key::Z,
        ))
    });
    if undo {
        app.undo();
    }
    let redo = ctx.input_mut(|i| {
        i.consume_shortcut(&egui::KeyboardShortcut::new(
            egui::Modifiers::COMMAND,
            egui::Key::Y,
        )) || i.consume_shortcut(&egui::KeyboardShortcut::new(
            egui::Modifiers::COMMAND | egui::Modifiers::SHIFT,
            egui::Key::Z,
        ))
    });
    if redo {
        app.redo();
    }

    if ctx.egui_wants_keyboard_input() {
        return;
    }
    let (esc, h, f, g, k1, k2, k3, k4, grow, shrink, plain) = ctx.input(|i| {
        (
            i.key_pressed(egui::Key::Escape),
            i.key_pressed(egui::Key::H),
            i.key_pressed(egui::Key::F),
            i.key_pressed(egui::Key::G),
            i.key_pressed(egui::Key::Num1),
            i.key_pressed(egui::Key::Num2),
            i.key_pressed(egui::Key::Num3),
            i.key_pressed(egui::Key::Num4),
            i.key_pressed(egui::Key::Period) || i.key_pressed(egui::Key::Plus),
            i.key_pressed(egui::Key::Comma) || i.key_pressed(egui::Key::Minus),
            !i.modifiers.ctrl && !i.modifiers.command && !i.modifiers.alt,
        )
    });
    if esc {
        app.mode = Mode::Orbit;
        app.sym_pick.clear();
    }
    if !plain {
        return;
    }
    if h && app.sel_count > 0 {
        app.hide_selection();
    }
    if f {
        app.fit_view_to_selection(now);
    }
    if g {
        app.show_grid = !app.show_grid;
    }
    if k1 {
        app.camera.animate_view(ViewDir::X, now);
    }
    if k2 {
        app.camera.animate_view(ViewDir::Y, now);
    }
    if k3 {
        app.camera.animate_view(ViewDir::Z, now);
    }
    if k4 {
        app.camera.animate_view(ViewDir::Iso, now);
    }
    if grow {
        app.grow_selection();
    }
    if shrink {
        app.shrink_selection();
    }
}
