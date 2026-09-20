//! Quad retopology tool of the Experimental section.

use crate::app::App;
use crate::geom::retopo::{MIN_TARGET_FACES, default_target_faces, target_edge_length};
use crate::ui::theme;

pub fn render_quad_retopology(app: &mut App, ui: &mut egui::Ui) {
    let muted = |s: &str| {
        egui::RichText::new(s)
            .size(11.5)
            .color(theme::pal().text_muted)
    };
    let Some(tris) = app.current.as_ref().map(|m| m.triangle_count()) else {
        ui.label(muted("Open a mesh to use this tool."));
        return;
    };
    ui.label(muted(
        "Rebuilds the working mesh from quads whose edge loops follow the \
         curvature and sharp edges. Export as OBJ or PLY to keep the quads.",
    ));
    ui.add_space(2.0);

    let area = app.retopo_surface_area();
    let auto = app.retopo.target_faces == 0;
    let mut target = if auto {
        default_target_faces(tris)
    } else {
        app.retopo.target_faces
    };
    let editing = app.is_editing();
    egui::Grid::new("retopo_grid")
        .num_columns(2)
        .spacing(egui::vec2(8.0, 4.0))
        .show(ui, |ui| {
            ui.label("Target faces");
            ui.horizontal(|ui| {
                let speed = (target as f64 * 0.01).max(1.0);
                if ui
                    .add(
                        egui::DragValue::new(&mut target)
                            .range(MIN_TARGET_FACES..=1_000_000)
                            .speed(speed),
                    )
                    .on_hover_text("Number of quads the new mesh should have")
                    .changed()
                {
                    app.retopo.target_faces = target;
                }
                if auto {
                    ui.label(muted("auto"));
                } else if ui
                    .small_button("Auto")
                    .on_hover_text("Derive the face count from the mesh size")
                    .clicked()
                {
                    app.retopo.target_faces = 0;
                }
            });
            ui.end_row();

            ui.label("Edge length");
            ui.label(format!("≈ {:.3} mm", target_edge_length(area, target)));
            ui.end_row();

            ui.label("Sharp edges");
            ui.checkbox(&mut app.retopo.align_features, "Follow")
                .on_hover_text("Align edge loops with creases and open boundaries");
            ui.end_row();

            ui.label("Crease angle");
            ui.add_enabled(
                app.retopo.align_features,
                egui::DragValue::new(&mut app.retopo.crease_angle_deg)
                    .range(5.0..=120.0)
                    .speed(0.5)
                    .suffix("°"),
            )
            .on_hover_text("Faces meeting at a sharper angle form a crease");
            ui.end_row();

            ui.label("Pure quads");
            ui.checkbox(&mut app.retopo.pure_quads, "").on_hover_text(
                "Every face becomes a quad (one subdivision step after the \
                     extraction). Off: quad-dominant with a few triangles.",
            );
            ui.end_row();

            ui.label("Relax passes");
            ui.add(egui::DragValue::new(&mut app.retopo.relax_iterations).range(0..=10))
                .on_hover_text("Tangential smoothing of the new vertices on the surface");
            ui.end_row();
        });
    ui.add_space(4.0);

    if let Some((fraction, stage)) = app.retopo_progress() {
        ui.horizontal(|ui| {
            ui.add(egui::Spinner::new().size(13.0).color(theme::pal().accent));
            ui.label(egui::RichText::new("Retopologizing…").color(theme::pal().accent_text));
        });
        let bar = egui::ProgressBar::new(fraction.unwrap_or(0.0)).text(stage);
        ui.add(bar);
    } else {
        let blocked = editing || app.preview.is_some();
        if ui
            .add_enabled(!blocked, egui::Button::new("Retopologize"))
            .on_hover_text("Replaces the working mesh; undo restores the triangles")
            .clicked()
        {
            app.request_quad_retopology();
        }
        if app.preview.is_some() {
            ui.label(muted("Apply or discard the decimation preview first."));
        }
    }
}
