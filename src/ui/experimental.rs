use crate::app::App;
use crate::geom::fit_to_object::{EdgeProjectionDir, FitMethod, FitOutputType};
use crate::ui::accordion::group_box;
use eframe::egui;

/// Renders the "Fit to Object" tool inside the "Experimental" accordion body.
pub fn render_experimental(app: &mut App, ui: &mut egui::Ui) {
    let freeform_count = app
        .freeforms
        .iter()
        .filter(|f| f.surface.is_some() && f.visible)
        .count();

    group_box(ui, Some("FIT TO OBJECT"), |ui| {
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("Aktive Freiformflächen:")
                    .size(11.5)
                    .color(egui::Color32::from_rgb(170, 180, 195)),
            );
            ui.label(
                egui::RichText::new(format!("{freeform_count}"))
                    .size(12.0)
                    .strong()
                    .color(if freeform_count > 0 {
                        egui::Color32::from_rgb(120, 210, 130)
                    } else {
                        egui::Color32::from_rgb(220, 140, 110)
                    }),
            );
        });

        if freeform_count == 0 {
            ui.add_space(2.0);
            ui.label(
                egui::RichText::new(
                    "Bitte zuerst mindestens eine Freiformfläche im Modell einpassen (Fit freeform).",
                )
                .size(10.5)
                .italics()
                .color(egui::Color32::from_gray(140)),
            );
        } else if freeform_count > 1 {
            ui.add_space(2.0);
            ui.label(
                egui::RichText::new(
                    "Mehrere Flächen erkannt: Schnittkanten werden berechnet, Überstände getrimmt und Kanten vernäht.",
                )
                .size(10.5)
                .color(egui::Color32::from_rgb(140, 185, 240)),
            );
        }
    });

    ui.add_space(4.0);

    // 1. Anpassungsmethode (Dropdown)
    group_box(ui, Some("ANPASSUNGSMETHODE"), |ui| {
        egui::ComboBox::from_id_salt("fit_method_dropdown")
            .selected_text(app.fit_config.method.label())
            .width(ui.available_width() - 8.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut app.fit_config.method,
                    FitMethod::SoftMembrane,
                    FitMethod::SoftMembrane.label(),
                );
                ui.selectable_value(
                    &mut app.fit_config.method,
                    FitMethod::HardRaycast,
                    FitMethod::HardRaycast.label(),
                );
            });

        ui.add_space(3.0);
        match app.fit_config.method {
            FitMethod::SoftMembrane => {
                ui.label(
                    egui::RichText::new(
                        "Elastische Membran: Harmonischer Krümmungsverlauf, Rauschfilterung. Bohrungen und Aussparungen werden formstabil überspannt.",
                    )
                    .size(10.5)
                    .color(egui::Color32::from_rgb(160, 175, 190)),
                );
            }
            FitMethod::HardRaycast => {
                ui.label(
                    egui::RichText::new(
                        "Harter Raycast: Starre Projektion entlang Normalen. Maximale Maßhaltigkeit und exakte Wiedergabe scharfer Kanten.",
                    )
                    .size(10.5)
                    .color(egui::Color32::from_rgb(160, 175, 190)),
                );
            }
        }
    });

    ui.add_space(4.0);

    // 2. Regelparameter
    group_box(ui, Some("REGELPARAMETER"), |ui| match app.fit_config.method {
        FitMethod::SoftMembrane => {
            ui.add(
                egui::Slider::new(&mut app.fit_config.tension, 0.05..=0.95)
                    .text("Spannungsfaktor")
                    .clamping(egui::SliderClamping::Always),
            )
            .on_hover_text("Steuert Biegesteifigkeit und Oberflächenspannung der Membran.");

            ui.add_space(2.0);
            ui.add(
                egui::Slider::new(&mut app.fit_config.smoothness, 1..=15)
                    .text("Glättungsgrad")
                    .clamping(egui::SliderClamping::Always),
            )
            .on_hover_text("Bestimmt die Filterung hochfrequenter Unebenheiten der Zielgeometrie.");
        }
        FitMethod::HardRaycast => {
            ui.add(
                egui::Slider::new(&mut app.fit_config.max_ray_dist, 0.5..=50.0)
                    .text("Max. Suchabstand")
                    .suffix(" mm")
                    .clamping(egui::SliderClamping::Always),
            )
            .on_hover_text("Definiert die maximale Reichweite der Suchstrahlen.");

            ui.add_space(2.0);
            ui.add(
                egui::Slider::new(&mut app.fit_config.snap_tolerance, 0.01..=2.0)
                    .text("Fang-Toleranz")
                    .suffix(" mm")
                    .clamping(egui::SliderClamping::Always),
            )
            .on_hover_text("Schwellenwert zur Handhabung von Kantenbereichen und Merkmalen.");
        }
    });

    ui.add_space(4.0);

    // 3. Ausgabetyp (Geometry Output)
    group_box(ui, Some("GEOMETRIE-AUSGABETYP"), |ui| {
        ui.horizontal(|ui| {
            ui.selectable_value(
                &mut app.fit_config.output_type,
                FitOutputType::Faces,
                "Flächen (Faces)",
            );
            ui.selectable_value(
                &mut app.fit_config.output_type,
                FitOutputType::Solid,
                "Volumen (Solid)",
            );
        });

        ui.add_space(3.0);
        match app.fit_config.output_type {
            FitOutputType::Faces => {
                ui.label(
                    egui::RichText::new(
                        "Offene Hülle (Sheet Body): Erzeugt die getrimmten und angepassten Flächen exakt bis zu den äußeren Begrenzungskanten.",
                    )
                    .size(10.5)
                    .color(egui::Color32::from_rgb(160, 175, 190)),
                );
            }
            FitOutputType::Solid => {
                ui.label(
                    egui::RichText::new(
                        "Geschlossener Volumenkörper: Extrudiert die Außenkanten nach hinten durch das Referenzteil und verschließt die Rückseite für saubere Boolesche Operationen.",
                    )
                    .size(10.5)
                    .color(egui::Color32::from_rgb(160, 175, 190)),
                );

                ui.add_space(4.0);
                ui.add(
                    egui::Slider::new(&mut app.fit_config.penetration_depth, 0.5..=30.0)
                        .text("Durchdringung")
                        .suffix(" mm")
                        .clamping(egui::SliderClamping::Always),
                )
                .on_hover_text("Bestimmt das Maß, um wie viel der Körper das Zielobjekt nach hinten durchschneidet.");

                ui.add_space(2.0);
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("Extrusionsrichtung:").size(11.0));
                    egui::ComboBox::from_id_salt("solid_dir_dropdown")
                        .selected_text(app.fit_config.projection_dir.label())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut app.fit_config.projection_dir,
                                EdgeProjectionDir::SurfaceNormal,
                                EdgeProjectionDir::SurfaceNormal.label(),
                            );
                            ui.selectable_value(
                                &mut app.fit_config.projection_dir,
                                EdgeProjectionDir::NegZ,
                                EdgeProjectionDir::NegZ.label(),
                            );
                            ui.selectable_value(
                                &mut app.fit_config.projection_dir,
                                EdgeProjectionDir::NegY,
                                EdgeProjectionDir::NegY.label(),
                            );
                            ui.selectable_value(
                                &mut app.fit_config.projection_dir,
                                EdgeProjectionDir::NegX,
                                EdgeProjectionDir::NegX.label(),
                            );
                        });
                });
            }
        }
    });

    ui.add_space(6.0);

    // 4. Aktionsbereich
    let can_fit = freeform_count > 0 && app.display().is_some();

    ui.vertical_centered(|ui| {
        let btn = egui::Button::new(
            egui::RichText::new("⚡ Fit to Object ausführen")
                .strong()
                .size(13.0)
                .color(if can_fit {
                    egui::Color32::from_rgb(240, 245, 255)
                } else {
                    egui::Color32::from_gray(120)
                }),
        )
        .min_size(egui::vec2(ui.available_width() - 8.0, 28.0));

        if ui
            .add_enabled(can_fit, btn)
            .on_hover_text(
                "Führt Trimmen, Vernähen und die gewählte Anpassung an das Bauteil durch und fügt das Ergebnis in den Modellbaum ein.",
            )
            .clicked()
        {
            app.run_fit_to_object();
        }
    });
}
