use crate::app::App;
use crate::geom::alignment::{AxisChoice, FeatureRef, OriginRef};
use eframe::egui;

/// Action returned by feature assignment buttons.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FeatureAssignmentAction {
    Assign(AxisChoice),
    ToggleOrigin,
}

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
                .color(egui::Color32::from_rgb(150, 165, 185)),
        );

        // X Button
        let x_active = cur_axis == Some(AxisChoice::X);
        let x_text = egui::RichText::new("X").size(10.5).strong();
        let x_btn = if x_active {
            egui::Button::new(x_text).fill(egui::Color32::from_rgb(45, 110, 190))
        } else {
            egui::Button::new(x_text)
        };
        if ui
            .add(x_btn)
            .on_hover_text("Assign to X axis / plane (X = 0)")
            .clicked()
        {
            clicked_action = Some(FeatureAssignmentAction::Assign(AxisChoice::X));
        }

        // Y Button
        let y_active = cur_axis == Some(AxisChoice::Y);
        let y_text = egui::RichText::new("Y").size(10.5).strong();
        let y_btn = if y_active {
            egui::Button::new(y_text).fill(egui::Color32::from_rgb(45, 150, 110))
        } else {
            egui::Button::new(y_text)
        };
        if ui
            .add(y_btn)
            .on_hover_text("Assign to Y axis / plane (Y = 0)")
            .clicked()
        {
            clicked_action = Some(FeatureAssignmentAction::Assign(AxisChoice::Y));
        }

        // Z Button
        let z_active = cur_axis == Some(AxisChoice::Z);
        let z_text = egui::RichText::new("Z").size(10.5).strong();
        let z_btn = if z_active {
            egui::Button::new(z_text).fill(egui::Color32::from_rgb(180, 110, 45))
        } else {
            egui::Button::new(z_text)
        };
        if ui
            .add(z_btn)
            .on_hover_text("Assign to Z axis / plane (Z = 0)")
            .clicked()
        {
            clicked_action = Some(FeatureAssignmentAction::Assign(AxisChoice::Z));
        }

        // Origin Button
        let orig_text = egui::RichText::new("Origin").size(10.5);
        let orig_btn = if is_orig {
            egui::Button::new(orig_text).fill(egui::Color32::from_rgb(160, 60, 140))
        } else {
            egui::Button::new(orig_text)
        };
        if ui
            .add(orig_btn)
            .on_hover_text("Set origin (0,0,0) at this feature's center/point")
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
        let (bg, txt) = match axis {
            AxisChoice::X => (egui::Color32::from_rgb(40, 95, 175), "X"),
            AxisChoice::Y => (egui::Color32::from_rgb(35, 130, 95), "Y"),
            AxisChoice::Z => (egui::Color32::from_rgb(160, 95, 35), "Z"),
        };
        let label = egui::RichText::new(txt)
            .size(9.5)
            .strong()
            .color(egui::Color32::WHITE);
        egui::Frame::new()
            .fill(bg)
            .corner_radius(3.0)
            .inner_margin(egui::Margin::symmetric(4, 1))
            .show(ui, |ui| {
                ui.label(label);
            });
    }

    if is_orig {
        let label = egui::RichText::new("Orig")
            .size(9.5)
            .strong()
            .color(egui::Color32::WHITE);
        egui::Frame::new()
            .fill(egui::Color32::from_rgb(150, 50, 130))
            .corner_radius(3.0)
            .inner_margin(egui::Margin::symmetric(4, 1))
            .show(ui, |ui| {
                ui.label(label);
            });
    }
}

/// Renders a small badge showing where this feature is currently assigned (e.g. [X], [Z], [Orig]).
#[allow(dead_code)]
pub fn render_feature_badge(app: &App, ui: &mut egui::Ui, feat: FeatureRef) {
    render_feature_badge_ui(
        ui,
        app.feature_assigned_axis(feat),
        app.is_feature_origin(feat),
    );
}

/// Renders the complete Quick Surface alignment section in the Coordinate System accordion.
pub fn render_feature_alignment_section(app: &mut App, ui: &mut egui::Ui) {
    let available_features = get_available_features(app);

    // Slots container
    egui::Frame::new()
        .fill(egui::Color32::from_rgba_unmultiplied(25, 30, 42, 60))
        .stroke(egui::Stroke::new(
            1.0,
            egui::Color32::from_rgba_unmultiplied(80, 110, 160, 40),
        ))
        .corner_radius(4.0)
        .inner_margin(egui::Margin::symmetric(8, 6))
        .show(ui, |ui| {
            render_slot_row(
                app,
                ui,
                AxisChoice::X,
                "X Axis / Plane:",
                &available_features,
            );
            ui.add_space(2.0);
            render_slot_row(
                app,
                ui,
                AxisChoice::Y,
                "Y Axis / Plane:",
                &available_features,
            );
            ui.add_space(2.0);
            render_slot_row(
                app,
                ui,
                AxisChoice::Z,
                "Z Axis / Plane:",
                &available_features,
            );
            ui.add_space(3.0);
            ui.separator();
            ui.add_space(3.0);

            // Origin selector
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new("Origin:")
                        .size(11.0)
                        .color(egui::Color32::from_rgb(180, 190, 205)),
                );

                let orig_name = match app.align_slots.origin {
                    OriginRef::FromAssignedFeatures => "Intersection of features".to_string(),
                    OriginRef::CircleCenter(id) => app
                        .circles
                        .iter()
                        .find(|c| c.id == id)
                        .map(|c| format!("Center of {}", c.name))
                        .unwrap_or_else(|| "Circle Center".to_string()),
                    OriginRef::Plane(id) => app
                        .planes
                        .iter()
                        .find(|p| p.id == id)
                        .map(|p| format!("Plane {}", p.name))
                        .unwrap_or_else(|| "Plane".to_string()),
                    OriginRef::SymmetryPlane => "Symmetry Plane".to_string(),
                    OriginRef::BBoxCenter => "Mesh Center (BBox)".to_string(),
                };

                egui::ComboBox::from_id_salt("origin_selector_combo")
                    .selected_text(egui::RichText::new(orig_name).size(11.0))
                    .width(170.0)
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
                                "Symmetry Plane",
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
                            "Mesh Center (BBox)",
                        );
                    });
            });

            // Primary Axis Selection
            ui.add_space(3.0);
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new("Primary:")
                        .size(11.0)
                        .color(egui::Color32::from_rgb(180, 190, 205)),
                );
                ui.selectable_value(&mut app.align_slots.primary_axis, AxisChoice::X, "X");
                ui.selectable_value(&mut app.align_slots.primary_axis, AxisChoice::Y, "Y");
                ui.selectable_value(&mut app.align_slots.primary_axis, AxisChoice::Z, "Z");
            });
        });

    ui.add_space(4.0);

    // Action buttons
    let has_any_slot =
        app.align_slots.x.is_some() || app.align_slots.y.is_some() || app.align_slots.z.is_some();

    ui.horizontal(|ui| {
        let align_btn = egui::Button::new(
            egui::RichText::new("➔ Align to features")
                .size(12.0)
                .strong()
                .color(if has_any_slot {
                    egui::Color32::WHITE
                } else {
                    egui::Color32::from_gray(140)
                }),
        )
        .fill(if has_any_slot {
            egui::Color32::from_rgb(45, 110, 190)
        } else {
            egui::Color32::from_gray(40)
        });

        if ui
            .add_enabled(has_any_slot, align_btn)
            .on_hover_text("Align scan coordinates and origin based on assigned features")
            .clicked()
        {
            app.align_to_features();
        }

        if ui
            .button("Auto-assign")
            .on_hover_text("Automatically assign available features to X, Y, Z and Origin")
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
    label: &str,
    features: &[(FeatureRef, String)],
) {
    ui.horizontal(|ui| {
        let (axis_color, axis_letter) = match axis {
            AxisChoice::X => (egui::Color32::from_rgb(80, 150, 240), "X"),
            AxisChoice::Y => (egui::Color32::from_rgb(80, 200, 140), "Y"),
            AxisChoice::Z => (egui::Color32::from_rgb(240, 160, 80), "Z"),
        };

        ui.label(
            egui::RichText::new(axis_letter)
                .size(11.5)
                .strong()
                .color(axis_color),
        );
        ui.label(
            egui::RichText::new(label)
                .size(11.0)
                .color(egui::Color32::from_rgb(180, 190, 205)),
        );

        let current_slot = match axis {
            AxisChoice::X => app.align_slots.x,
            AxisChoice::Y => app.align_slots.y,
            AxisChoice::Z => app.align_slots.z,
        };

        let current_name = current_slot
            .map(|f| app.feature_name(f))
            .unwrap_or_else(|| "None".to_string());

        let combo_id = format!("slot_combo_{axis_letter}");
        let _resp = egui::ComboBox::from_id_salt(combo_id)
            .selected_text(egui::RichText::new(current_name).size(11.0))
            .width(130.0)
            .show_ui(ui, |ui| {
                let is_none = current_slot.is_none();
                if ui.selectable_label(is_none, "None").clicked() {
                    match axis {
                        AxisChoice::X => app.align_slots.x = None,
                        AxisChoice::Y => app.align_slots.y = None,
                        AxisChoice::Z => app.align_slots.z = None,
                    }
                }
                for (f, name) in features {
                    let is_sel = current_slot == Some(*f);
                    if ui.selectable_label(is_sel, name).clicked() {
                        match axis {
                            AxisChoice::X => app.align_slots.x = Some(*f),
                            AxisChoice::Y => app.align_slots.y = Some(*f),
                            AxisChoice::Z => app.align_slots.z = Some(*f),
                        }
                    }
                }
            });

        if current_slot.is_some() {
            if ui
                .small_button(
                    egui::RichText::new("✕")
                        .size(10.0)
                        .color(egui::Color32::from_rgb(220, 100, 100)),
                )
                .on_hover_text("Clear this slot")
                .clicked()
            {
                match axis {
                    AxisChoice::X => app.align_slots.x = None,
                    AxisChoice::Y => app.align_slots.y = None,
                    AxisChoice::Z => app.align_slots.z = None,
                }
            }
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
