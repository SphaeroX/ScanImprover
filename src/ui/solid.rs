//! Experimental "Solid reconstruction" tool: builds a closed B-Rep solid
//! from the face groups, reports how well it fits the mesh and exports it
//! as STEP.

use crate::app::App;
use crate::ui::theme;

const SURFACE_NAMES: [(&str, &str); 5] = [
    ("plane", "planes"),
    ("cylinder", "cylinders"),
    ("cone", "cones"),
    ("sphere", "spheres"),
    ("B-spline", "B-splines"),
];
const CURVE_NAMES: [(&str, &str); 4] = [
    ("line", "lines"),
    ("circle", "circles"),
    ("ellipse", "ellipses"),
    ("B-spline", "B-splines"),
];

/// "6 planes · 1 cylinder" for the non-zero counts.
fn counts_text(counts: &[usize], names: &[(&str, &str)]) -> String {
    let parts: Vec<String> = counts
        .iter()
        .zip(names)
        .filter(|(n, _)| **n > 0)
        .map(|(n, (one, many))| format!("{n} {}", if *n == 1 { one } else { many }))
        .collect();
    if parts.is_empty() {
        "none".to_string()
    } else {
        parts.join(" · ")
    }
}

fn muted(ui: &mut egui::Ui, text: impl Into<String>) {
    ui.label(
        egui::RichText::new(text.into())
            .size(11.5)
            .color(theme::pal().text_muted),
    );
}

pub fn render_solid_reconstruction(app: &mut App, ui: &mut egui::Ui) {
    muted(
        ui,
        "Builds a closed B-Rep solid from the face groups: one face per group on its fitted \
         plane, cylinder, cone, sphere or B-spline surface, edges and corners from the surface \
         intersections. Needs a closed mesh (fill the holes first).",
    );
    ui.add(
        egui::Slider::new(&mut app.solid.params.snap_angle_deg, 0.0..=5.0)
            .text("Snap angle")
            .suffix("°"),
    )
    .on_hover_text(
        "Plane normals and axes within this angle become exactly parallel or \
         perpendicular; axes closer than 3× the fit tolerance become coaxial. \
         0 = keep the independent fits.",
    );
    ui.checkbox(
        &mut app.solid.params.allow_freeform,
        "B-spline faces for freeform groups",
    )
    .on_hover_text(
        "Freeform groups that are not cones become bicubic B-spline faces. When off, \
         they are refused so only analytic faces are written.",
    );

    let has_groups = !app.face_groups.is_empty();
    let running = app.solid.job.is_some();
    ui.horizontal(|ui| {
        let resp = ui
            .add_enabled(
                has_groups && !running && !app.is_editing(),
                egui::Button::new("Reconstruct solid"),
            )
            .on_disabled_hover_text("Detect the face groups first (Face groups section).");
        if resp.clicked() {
            // Same classification tolerance as the face group detection.
            app.solid.params.fit_tol = app.group_fit_tol as f64;
            app.request_solid_reconstruction();
        }
        if running {
            ui.add(egui::Spinner::new().size(13.0).color(theme::pal().accent));
            ui.label(egui::RichText::new("Reconstructing…").color(theme::pal().accent_text));
        }
    });

    let Some(result) = app.solid.result.clone() else {
        return;
    };
    let current = app.solid_is_current();
    if !current {
        ui.label(
            egui::RichText::new("The mesh or the face groups changed; reconstruct again.")
                .size(11.5)
                .color(theme::pal().warn),
        );
    }
    let solid = match result {
        Err(e) => {
            ui.label(
                egui::RichText::new(format!("Not possible: {e}"))
                    .size(11.5)
                    .color(theme::pal().danger),
            );
            return;
        }
        Ok(s) => s,
    };
    let r = &solid.report;
    ui.add_space(2.0);
    ui.label(
        egui::RichText::new(format!(
            "{} faces · {} edges · {} vertices · genus {}",
            solid.faces.len(),
            solid.edges.len(),
            solid.vertices.len(),
            r.genus
        ))
        .strong()
        .color(if current {
            theme::pal().success
        } else {
            theme::pal().text_muted
        }),
    );
    muted(
        ui,
        format!("Faces: {}", counts_text(&r.surface_counts, &SURFACE_NAMES)),
    );
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing.x = 3.0;
        muted(ui, "Edges:");
        let mut first = true;
        for (k, (n, (one, many))) in r.curve_counts.iter().zip(&CURVE_NAMES).enumerate() {
            if *n == 0 {
                continue;
            }
            if !first {
                muted(ui, "·");
            }
            first = false;
            let c = crate::app::SOLID_EDGE_COLORS[k];
            ui.label(egui::RichText::new("■").color(theme::color32(c)).size(12.0));
            muted(ui, format!("{n} {}", if *n == 1 { one } else { many }));
        }
    });
    muted(
        ui,
        format!(
            "Mesh deviation: max {:.3} mm (group {}) · RMS {:.3} mm",
            r.mesh_max_dev,
            r.worst_group + 1,
            r.mesh_rms_dev
        ),
    );
    muted(
        ui,
        format!(
            "Corner gap {:.1e} mm · edge gap {:.1e} mm · edges vs mesh {:.3} mm",
            r.vertex_gap, r.edge_gap, r.edge_mesh_dev
        ),
    );
    let rel = r.relations;
    if rel != Default::default() {
        muted(
            ui,
            format!(
                "Snapped: {} parallel sets ({} surfaces) · {} perpendicular · {} coaxial · {} \
                 centred spheres",
                rel.parallel_sets,
                rel.snapped,
                rel.perpendicular,
                rel.coaxial_sets,
                rel.centered_spheres
            ),
        );
    }
    for note in &r.notes {
        ui.label(
            egui::RichText::new(note)
                .size(11.5)
                .color(theme::pal().warn),
        );
    }
    egui::CollapsingHeader::new(format!("Faces ({})", solid.faces.len()))
        .id_salt("solid_faces")
        .show(ui, |ui| {
            egui::ScrollArea::vertical()
                .max_height(160.0)
                .show(ui, |ui| {
                    for f in &solid.faces {
                        ui.horizontal(|ui| {
                            ui.label(format!(
                                "Group {} · {} · max {:.3} mm",
                                f.group + 1,
                                f.surface.label(),
                                f.max_dev
                            ));
                            if current && ui.small_button("Sel").clicked() {
                                app.select_group_faces(f.group, app.group_sel_additive);
                            }
                        });
                    }
                });
        });
    ui.checkbox(
        &mut app.solid.show_edges,
        "Show edges and vertices in the viewport",
    );
    if ui
        .add_enabled(current, egui::Button::new("Export solid STEP…"))
        .clicked()
    {
        app.export_solid_step();
    }
}
