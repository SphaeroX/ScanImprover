//! Feature -> axis assignment widgets and the datum alignment section.

use crate::app::App;
use crate::geom::alignment::{AxisChoice, FeatureRef, OriginRef};
use crate::ui::theme;
use egui;

/// Action returned by feature assignment buttons.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FeatureAssignmentAction {
    Assign(AxisChoice),
    ToggleOrigin,
}

fn axis_color(axis: AxisChoice) -> egui::Color32 {
    match axis {
        AxisChoice::X => theme::pal().axis_x,
        AxisChoice::Y => theme::pal().axis_y,
        AxisChoice::Z => theme::pal().axis_z,
    }
}

const ORIGIN_COLOR: egui::Color32 = egui::Color32::from_rgb(190, 90, 200);

/// Renders quick assignment toggle buttons [X] [Y] [Z] [Origin] returning any clicked action.
pub fn render_feature_assignment_buttons_ui(
    ui: &mut egui::Ui,
    _feat: FeatureRef,
    cur_axis: Option<AxisChoice>,
    is_orig: bool,
) -> Option<FeatureAssignmentAction> {
    let mut clicked_action = None;
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new("Assign:")
                .size(11.0)
                .color(theme::pal().text_muted),
        );
        for axis in [AxisChoice::X, AxisChoice::Y, AxisChoice::Z] {
            let active = cur_axis == Some(axis);
            let text = egui::RichText::new(axis.name()).size(10.5).strong();
            let mut btn = egui::Button::new(text).corner_radius(4.0);
            if active {
                btn = btn
                    .fill(axis_color(axis).gamma_multiply(0.55))
                    .stroke(egui::Stroke::new(1.0, axis_color(axis)));
            }
            if ui
                .add(btn)
                .on_hover_text(format!(
                    "Assign to {} axis / plane ({} = 0)",
                    axis.name(),
                    axis.name()
                ))
                .clicked()
            {
                clicked_action = Some(FeatureAssignmentAction::Assign(axis));
            }
        }
        let text = egui::RichText::new("Origin").size(10.5);
        let mut btn = egui::Button::new(text).corner_radius(4.0);
        if is_orig {
            btn = btn
                .fill(ORIGIN_COLOR.gamma_multiply(0.55))
                .stroke(egui::Stroke::new(1.0, ORIGIN_COLOR));
        }
        if ui
            .add(btn)
            .on_hover_text("Set origin (0,0,0) at this feature's center / point")
            .clicked()
        {
            clicked_action = Some(FeatureAssignmentAction::ToggleOrigin);
        }
    });
    clicked_action
}

/// Renders quick assignment toggle buttons [X] [Y] [Z] [Origin] for a specific feature.
pub fn render_feature_assignment_buttons(app: &mut App, ui: &mut egui::Ui, feat: FeatureRef) {
    let cur_axis = app.feature_assigned_axis(feat);
    let is_orig = app.is_feature_origin(feat);
    if let Some(action) = render_feature_assignment_buttons_ui(ui, feat, cur_axis, is_orig) {
        match action {
            FeatureAssignmentAction::Assign(ax) => app.toggle_assign_feature(feat, ax),
            FeatureAssignmentAction::ToggleOrigin => app.toggle_origin_feature(feat),
        }
    }
}

/// Renders a small badge showing where this feature is currently assigned (e.g. [X], [Z], [Orig]).
pub fn render_feature_badge_ui(ui: &mut egui::Ui, cur_axis: Option<AxisChoice>, is_orig: bool) {
    if let Some(axis) = cur_axis {
        theme::badge(
            ui,
            axis.name(),
            egui::Color32::WHITE,
            axis_color(axis).gamma_multiply(0.7),
        );
    }
    if is_orig {
        theme::badge(
            ui,
            "Orig",
            egui::Color32::WHITE,
            ORIGIN_COLOR.gamma_multiply(0.7),
        );
    }
}

/// Renders the complete Quick Surface alignment section in the Coordinate System accordion.
pub fn render_feature_alignment_section(app: &mut App, ui: &mut egui::Ui) {
    let available_features = get_available_features(app);

    ui.scope(|ui| {
        for axis in [AxisChoice::X, AxisChoice::Y, AxisChoice::Z] {
            render_slot_row(app, ui, axis, &available_features);
        }
        ui.add_space(3.0);

        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("Origin:")
                    .size(11.0)
                    .color(theme::pal().text_muted),
            );
            let orig_name = match app.align_slots.origin {
                OriginRef::FromAssignedFeatures => "Intersection of features".to_string(),
                OriginRef::CircleCenter(id) => app
                    .circles
                    .iter()
                    .find(|c| c.id == id)
                    .map(|c| format!("Center of {}", c.name))
                    .unwrap_or_else(|| "Circle center".to_string()),
                OriginRef::Plane(id) => app
                    .planes
                    .iter()
                    .find(|p| p.id == id)
                    .map(|p| format!("On {}", p.name))
                    .unwrap_or_else(|| "Plane".to_string()),
                OriginRef::SymmetryPlane => "Symmetry plane".to_string(),
                OriginRef::BBoxCenter => "Mesh center (bbox)".to_string(),
            };
            egui::ComboBox::from_id_salt("origin_selector_combo")
                .selected_text(egui::RichText::new(orig_name).size(11.0))
                .width(180.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut app.align_slots.origin,
                        OriginRef::FromAssignedFeatures,
                        "Intersection of features",
                    );
                    if app.sym.is_some() {
                        ui.selectable_value(
                            &mut app.align_slots.origin,
                            OriginRef::SymmetryPlane,
                            "Symmetry plane",
                        );
                    }
                    for c in &app.circles {
                        ui.selectable_value(
                            &mut app.align_slots.origin,
                            OriginRef::CircleCenter(c.id),
                            format!("Center of {}", c.name),
                        );
                    }
                    for p in &app.planes {
                        ui.selectable_value(
                            &mut app.align_slots.origin,
                            OriginRef::Plane(p.id),
                            format!("On {}", p.name),
                        );
                    }
                    ui.selectable_value(
                        &mut app.align_slots.origin,
                        OriginRef::BBoxCenter,
                        "Mesh center (bbox)",
                    );
                });
        });

        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("Primary:")
                    .size(11.0)
                    .color(theme::pal().text_muted),
            );
            for axis in [AxisChoice::X, AxisChoice::Y, AxisChoice::Z] {
                ui.selectable_value(&mut app.align_slots.primary_axis, axis, axis.name());
            }
        })
        .response
        .on_hover_text(
            "The axis that is aligned exactly; the second axis is aligned as closely as possible.",
        );
    });

    ui.add_space(4.0);
    let has_any_slot = app.align_slots.any_axis_assigned();
    ui.horizontal(|ui| {
        if ui
            .add_enabled(
                has_any_slot,
                egui::Button::new(egui::RichText::new("Align to features").strong()),
            )
            .on_hover_text("Align scan coordinates and origin based on assigned features")
            .clicked()
        {
            app.align_to_features();
        }
        if ui
            .button("Auto-assign")
            .on_hover_text("Automatically assign available features to X, Y, Z and origin")
            .clicked()
        {
            app.auto_assign_alignment_from_selection();
        }
        if has_any_slot && ui.small_button("Clear").clicked() {
            app.clear_alignment_slots();
        }
    });
}

fn render_slot_row(
    app: &mut App,
    ui: &mut egui::Ui,
    axis: AxisChoice,
    features: &[(FeatureRef, String)],
) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(axis.name())
                .size(11.5)
                .strong()
                .color(axis_color(axis)),
        );
        ui.label(
            egui::RichText::new("axis / plane:")
                .size(11.0)
                .color(theme::pal().text_muted),
        );
        let current_slot = app.align_slots.get_axis(axis);
        let current_name = current_slot
            .map(|f| app.feature_name(f))
            .unwrap_or_else(|| "None".to_string());
        egui::ComboBox::from_id_salt(format!("slot_combo_{}", axis.name()))
            .selected_text(egui::RichText::new(current_name).size(11.0))
            .width(140.0)
            .show_ui(ui, |ui| {
                if ui
                    .selectable_label(current_slot.is_none(), "None")
                    .clicked()
                {
                    app.align_slots.set_axis(axis, None);
                }
                for (f, name) in features {
                    if ui
                        .selectable_label(current_slot == Some(*f), name)
                        .clicked()
                    {
                        app.align_slots.set_axis(axis, Some(*f));
                    }
                }
            });
        if current_slot.is_some()
            && ui
                .small_button(
                    egui::RichText::new("×")
                        .size(10.0)
                        .color(theme::pal().danger),
                )
                .on_hover_text("Clear this slot")
                .clicked()
        {
            app.align_slots.set_axis(axis, None);
        }
    });
}

fn get_available_features(app: &App) -> Vec<(FeatureRef, String)> {
    let mut list = Vec::new();
    if app.sym.is_some() {
        list.push((FeatureRef::SymmetryPlane, "Symmetry plane".to_string()));
    }
    for p in &app.planes {
        list.push((FeatureRef::Plane(p.id), p.name.clone()));
    }
    for c in &app.circles {
        list.push((FeatureRef::Circle(c.id), c.name.clone()));
    }
    list
}
