use crate::app::App;
use eframe::egui;
use glam::Vec3;

enum BrowserAction {
    ToggleMeshVisibility,
    HideSelection,
    ToggleHiddenRegionVisibility(u64),
    RestoreHiddenRegion(u64),
    RestoreAllHiddenRegions,
    DeleteHiddenRegion(u64),
    TogglePlaneVisibility(u64),
    SelectPlane(u64),
    DeletePlane(u64),
    AlignPlane(u64, Vec3),
    OriginOnPlane(u64),
    ToggleCircleVisibility(u64),
    SelectCircle(u64),
    DeleteCircle(u64),
    AlignCircle(u64, Vec3),
    OriginAtCircle(u64),
    ToggleFreeformVisibility(u64),
    SelectFreeform(u64),
    DeleteFreeform(u64),
    FitPlane,
    FitCircle,
    FitFreeform,
    ToggleSymmetryVisibility,
    ExportPlane(u64),
    ExportCircle(u64),
    ExportFreeform(u64),
    ExportFitFeature(u64),
    ToggleFitFeatureVisibility(u64),
    SelectFitFeature(u64),
    DeleteFitFeature(u64),
    ExportAllReferences,
    AlignToFeatures,
    ToggleAssign(
        crate::geom::alignment::FeatureRef,
        crate::geom::alignment::AxisChoice,
    ),
    ToggleOrigin(crate::geom::alignment::FeatureRef),
}

/// Renders the Object Browser window on the right side of the 3D viewport.
pub fn render_object_browser(app: &mut App, ui: &mut egui::Ui, _viewport_rect: egui::Rect) {
    if !app.show_object_browser {
        return;
    }

    let align_slots = app.align_slots.clone();
    let mut actions: Vec<BrowserAction> = Vec::new();

    let total_count = (if app.display().is_some() { 1 } else { 0 })
        + app.hidden_regions.len()
        + app.planes.len()
        + app.circles.len()
        + app.freeforms.len()
        + app.fit_features.len()
        + (if app.sym.is_some() { 1 } else { 0 });

    let window_title = format!("Objects ({total_count})");

    egui::Window::new(window_title)
        .id(egui::Id::new("object_browser_window"))
        .default_open(true)
        .resizable(true)
        .collapsible(true)
        .default_size(egui::vec2(270.0, 340.0))
        .min_width(230.0)
        .max_width(380.0)
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-14.0, 12.0))
        .frame(
            egui::Frame::window(&ui.style())
                .fill(egui::Color32::from_rgba_unmultiplied(20, 24, 32, 235))
                .stroke(egui::Stroke::new(
                    1.0,
                    egui::Color32::from_rgba_unmultiplied(100, 140, 220, 50),
                ))
                .corner_radius(8.0)
                .inner_margin(egui::Margin::symmetric(10, 8)),
        )
        .show(ui.ctx(), |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .max_height(450.0)
                .show(ui, |ui| {
                    // Quick Action Bar if faces are selected
                    if app.sel_count > 0 {
                        egui::Frame::new()
                            .fill(egui::Color32::from_rgba_unmultiplied(255, 150, 40, 20))
                            .stroke(egui::Stroke::new(
                                1.0,
                                egui::Color32::from_rgba_unmultiplied(255, 150, 40, 60),
                            ))
                            .corner_radius(4.0)
                            .inner_margin(egui::Margin::symmetric(6, 4))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.label(
                                        egui::RichText::new("Selection:")
                                            .size(11.0)
                                            .color(egui::Color32::from_rgb(255, 180, 100)),
                                    );
                                    if ui.small_button("+ Fit Plane").clicked() {
                                        actions.push(BrowserAction::FitPlane);
                                    }
                                    if ui.small_button("+ Fit Circle").clicked() {
                                        actions.push(BrowserAction::FitCircle);
                                    }
                                    if ui.small_button("+ Fit Freeform").clicked() {
                                        actions.push(BrowserAction::FitFreeform);
                                    }
                                    if ui
                                        .small_button("👁 Hide")
                                        .on_hover_text("Hide selection (H)")
                                        .clicked()
                                    {
                                        actions.push(BrowserAction::HideSelection);
                                    }
                                });
                            });
                        ui.add_space(6.0);
                    }

                    // --- SECTION 1: MESH ---
                    ui.label(
                        egui::RichText::new("MESH")
                            .size(10.5)
                            .strong()
                            .color(egui::Color32::from_rgb(130, 145, 170)),
                    );

                    if let Some(m) = app.display() {
                        let name = app
                            .file_path
                            .as_ref()
                            .and_then(|p| p.file_name())
                            .and_then(|s| s.to_str())
                            .unwrap_or("Mesh");

                        let tris = m.triangle_count();
                        let verts = m.vertex_count();

                        egui::Frame::new()
                            .fill(egui::Color32::from_rgba_unmultiplied(35, 40, 52, 60))
                            .corner_radius(4.0)
                            .inner_margin(egui::Margin::symmetric(6, 4))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    let mut vis = app.show_mesh;
                                    if ui.checkbox(&mut vis, "").changed() {
                                        actions.push(BrowserAction::ToggleMeshVisibility);
                                    }

                                    // Mesh indicator icon
                                    ui.label(
                                        egui::RichText::new("◬")
                                            .size(13.0)
                                            .color(egui::Color32::from_rgb(120, 175, 255)),
                                    );

                                    ui.label(
                                        egui::RichText::new(name)
                                            .strong()
                                            .size(12.0)
                                            .color(egui::Color32::from_rgb(230, 235, 245)),
                                    );

                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            ui.label(
                                                egui::RichText::new(format!("{tris} △"))
                                                    .size(10.5)
                                                    .color(egui::Color32::from_rgb(140, 150, 165)),
                                            );
                                        },
                                    );
                                });

                                ui.horizontal(|ui| {
                                    ui.add_space(24.0);
                                    ui.label(
                                        egui::RichText::new(format!("{verts} verts"))
                                            .size(10.0)
                                            .color(egui::Color32::from_rgb(120, 130, 145)),
                                    );
                                });
                            });
                    } else {
                        ui.label(
                            egui::RichText::new("No mesh loaded")
                                .italics()
                                .size(11.0)
                                .color(egui::Color32::from_gray(120)),
                        );
                    }

                    // --- SECTION: HIDDEN REGIONS ---
                    let hidden_count = app.hidden_regions.len();
                    if hidden_count > 0 {
                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(4.0);

                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(format!("HIDDEN REGIONS ({hidden_count})"))
                                    .size(10.5)
                                    .strong()
                                    .color(egui::Color32::from_rgb(130, 145, 170)),
                            );
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui
                                    .small_button(egui::RichText::new("Unhide All").size(10.0))
                                    .on_hover_text("Restore all hidden regions back into mesh")
                                    .clicked()
                                {
                                    actions.push(BrowserAction::RestoreAllHiddenRegions);
                                }
                            });
                        });

                        for hr in &app.hidden_regions {
                            let tris = hr.triangle_count();
                            let bg_color = if hr.visible {
                                egui::Color32::from_rgba_unmultiplied(35, 45, 60, 60)
                            } else {
                                egui::Color32::from_rgba_unmultiplied(25, 28, 36, 40)
                            };

                            egui::Frame::new()
                                .fill(bg_color)
                                .corner_radius(4.0)
                                .inner_margin(egui::Margin::symmetric(6, 4))
                                .show(ui, |ui| {
                                    ui.horizontal(|ui| {
                                        let mut vis = hr.visible;
                                        if ui
                                            .checkbox(&mut vis, "")
                                            .on_hover_text("Toggle visibility (Ein-/Ausblenden)")
                                            .changed()
                                        {
                                            actions.push(BrowserAction::ToggleHiddenRegionVisibility(hr.id));
                                        }

                                        let icon_color = if hr.visible {
                                            egui::Color32::from_rgb(100, 180, 255)
                                        } else {
                                            egui::Color32::from_rgb(100, 110, 125)
                                        };
                                        ui.label(egui::RichText::new("👁").size(12.0).color(icon_color));

                                        let text_color = if hr.visible {
                                            egui::Color32::from_rgb(220, 225, 235)
                                        } else {
                                            egui::Color32::from_rgb(150, 155, 170)
                                        };
                                        ui.label(
                                            egui::RichText::new(&hr.name)
                                                .strong()
                                                .size(11.5)
                                                .color(text_color),
                                        );

                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .small_button(
                                                        egui::RichText::new("✕")
                                                            .size(11.0)
                                                            .color(egui::Color32::from_rgb(220, 100, 100)),
                                                    )
                                                    .on_hover_text("Delete this hidden region")
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::DeleteHiddenRegion(hr.id));
                                                }

                                                if ui
                                                    .small_button(
                                                        egui::RichText::new("Restore")
                                                            .size(10.0)
                                                            .color(egui::Color32::from_rgb(150, 200, 255)),
                                                    )
                                                    .on_hover_text("Restore back into the mesh permanently")
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::RestoreHiddenRegion(hr.id));
                                                }

                                                ui.label(
                                                    egui::RichText::new(format!("{tris} △"))
                                                        .size(10.5)
                                                        .color(egui::Color32::from_rgb(140, 150, 165)),
                                                );
                                            },
                                        );
                                    });
                                });
                            ui.add_space(2.0);
                        }
                    }

                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // --- SECTION 2: FITTED PLANES ---
                    let plane_count = app.planes.len();
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(format!("FITTED PLANES ({plane_count})"))
                                .size(10.5)
                                .strong()
                                .color(egui::Color32::from_rgb(130, 145, 170)),
                        );
                    });

                    if app.planes.is_empty() {
                        ui.label(
                            egui::RichText::new("No planes fitted yet")
                                .italics()
                                .size(11.0)
                                .color(egui::Color32::from_gray(110)),
                        );
                    } else {
                        for p in &app.planes {
                            let is_sel = app.selected_plane_id == Some(p.id);
                            let feat = crate::geom::alignment::FeatureRef::Plane(p.id);
                            let cur_axis = if align_slots.x == Some(feat) {
                                Some(crate::geom::alignment::AxisChoice::X)
                            } else if align_slots.y == Some(feat) {
                                Some(crate::geom::alignment::AxisChoice::Y)
                            } else if align_slots.z == Some(feat) {
                                Some(crate::geom::alignment::AxisChoice::Z)
                            } else {
                                None
                            };
                            let is_orig = matches!(
                                align_slots.origin,
                                crate::geom::alignment::OriginRef::Plane(id) if id == p.id
                            );

                            let col32 = egui::Color32::from_rgba_unmultiplied(
                                (p.color[0] * 255.0) as u8,
                                (p.color[1] * 255.0) as u8,
                                (p.color[2] * 255.0) as u8,
                                255,
                            );

                            let bg_color = if is_sel {
                                egui::Color32::from_rgba_unmultiplied(50, 70, 105, 80)
                            } else {
                                egui::Color32::from_rgba_unmultiplied(35, 40, 52, 40)
                            };

                            let border_stroke = if is_sel {
                                egui::Stroke::new(
                                    1.0,
                                    egui::Color32::from_rgba_unmultiplied(120, 170, 255, 120),
                                )
                            } else {
                                egui::Stroke::NONE
                            };

                            egui::Frame::new()
                                .fill(bg_color)
                                .stroke(border_stroke)
                                .corner_radius(4.0)
                                .inner_margin(egui::Margin::symmetric(6, 5))
                                .show(ui, |ui| {
                                    ui.horizontal(|ui| {
                                        // Visibility toggle
                                        let mut vis = p.visible;
                                        if ui.checkbox(&mut vis, "").changed() {
                                            actions.push(BrowserAction::TogglePlaneVisibility(p.id));
                                        }

                                        // Color indicator dot
                                        let (dot_rect, _) = ui.allocate_exact_size(
                                            egui::vec2(10.0, 10.0),
                                            egui::Sense::empty(),
                                        );
                                        ui.painter().circle_filled(dot_rect.center(), 4.0, col32);

                                        // Selectable name
                                        let name_resp = ui.selectable_label(
                                            is_sel,
                                            egui::RichText::new(&p.name).size(12.0).strong(),
                                        );
                                        if name_resp.clicked() {
                                            actions.push(BrowserAction::SelectPlane(p.id));
                                        }

                                        crate::ui::alignment::render_feature_badge_ui(
                                            ui,
                                            cur_axis,
                                            is_orig,
                                        );

                                        // Delete button on the right
                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .small_button(
                                                        egui::RichText::new("✕")
                                                            .size(11.0)
                                                            .color(egui::Color32::from_rgb(
                                                                220, 100, 100,
                                                            )),
                                                    )
                                                    .on_hover_text("Delete plane")
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::DeletePlane(p.id));
                                                }
                                            },
                                        );
                                    });

                                    // If this plane is selected, show details & alignment actions
                                    if is_sel {
                                        ui.add_space(3.0);
                                        let normal = p.fit.normal;
                                        let offset = p.fit.point.dot(normal);
                                        let rms = p.fit.rms.sqrt();

                                        ui.horizontal(|ui| {
                                            ui.add_space(20.0);
                                            ui.vertical(|ui| {
                                                ui.label(
                                                    egui::RichText::new(format!(
                                                        "N: ({:.2}, {:.2}, {:.2})  off: {:.2} mm",
                                                        normal.x, normal.y, normal.z, offset
                                                    ))
                                                    .size(10.5)
                                                    .color(egui::Color32::from_rgb(170, 180, 195)),
                                                );
                                                ui.label(
                                                    egui::RichText::new(format!(
                                                        "RMS: {:.4} mm · max: {:.4} mm",
                                                        rms, p.fit.max_dev
                                                    ))
                                                    .size(10.0)
                                                    .color(egui::Color32::from_rgb(140, 150, 165)),
                                                );

                                                ui.add_space(2.0);
                                                ui.horizontal(|ui| {
                                                    if ui.small_button("N → X").clicked() {
                                                        actions.push(BrowserAction::AlignPlane(
                                                            p.id,
                                                            Vec3::X,
                                                        ));
                                                    }
                                                    if ui.small_button("N → Y").clicked() {
                                                        actions.push(BrowserAction::AlignPlane(
                                                            p.id,
                                                            Vec3::Y,
                                                        ));
                                                    }
                                                    if ui.small_button("N → Z").clicked() {
                                                        actions.push(BrowserAction::AlignPlane(
                                                            p.id,
                                                            Vec3::Z,
                                                        ));
                                                    }
                                                    if ui
                                                        .small_button("Origin on plane")
                                                        .clicked()
                                                    {
                                                        actions.push(
                                                            BrowserAction::OriginOnPlane(p.id),
                                                        );
                                                    }
                                                });
                                                ui.add_space(2.0);
                                                ui.horizontal(|ui| {
                                                    if ui
                                                        .small_button("Export Plane…")
                                                        .on_hover_text("Export this plane for Fusion 360 / CAD (STEP, Script, DXF)")
                                                        .clicked()
                                                    {
                                                        actions.push(BrowserAction::ExportPlane(p.id));
                                                    }
                                                });
                                                ui.add_space(2.0);
                                                if let Some(action) = crate::ui::alignment::render_feature_assignment_buttons_ui(
                                                    ui,
                                                    feat,
                                                    cur_axis,
                                                    is_orig,
                                                ) {
                                                    match action {
                                                        crate::ui::alignment::FeatureAssignmentAction::Assign(ax) => {
                                                            actions.push(BrowserAction::ToggleAssign(feat, ax));
                                                        }
                                                        crate::ui::alignment::FeatureAssignmentAction::ToggleOrigin => {
                                                            actions.push(BrowserAction::ToggleOrigin(feat));
                                                        }
                                                    }
                                                }
                                            });
                                        });
                                    }
                                });
                            ui.add_space(2.0);
                        }
                    }

                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // --- SECTION 3: FITTED CIRCLES ---
                    let circle_count = app.circles.len();
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(format!("FITTED CIRCLES ({circle_count})"))
                                .size(10.5)
                                .strong()
                                .color(egui::Color32::from_rgb(130, 145, 170)),
                        );
                    });

                    if app.circles.is_empty() {
                        ui.label(
                            egui::RichText::new("No circles fitted yet")
                                .italics()
                                .size(11.0)
                                .color(egui::Color32::from_gray(110)),
                        );
                    } else {
                        for c in &app.circles {
                            let is_sel = app.selected_circle_id == Some(c.id);
                            let feat = crate::geom::alignment::FeatureRef::Circle(c.id);
                            let cur_axis = if align_slots.x == Some(feat) {
                                Some(crate::geom::alignment::AxisChoice::X)
                            } else if align_slots.y == Some(feat) {
                                Some(crate::geom::alignment::AxisChoice::Y)
                            } else if align_slots.z == Some(feat) {
                                Some(crate::geom::alignment::AxisChoice::Z)
                            } else {
                                None
                            };
                            let is_orig = matches!(
                                align_slots.origin,
                                crate::geom::alignment::OriginRef::CircleCenter(id) if id == c.id
                            );

                            let col32 = egui::Color32::from_rgba_unmultiplied(
                                (c.color[0] * 255.0) as u8,
                                (c.color[1] * 255.0) as u8,
                                (c.color[2] * 255.0) as u8,
                                255,
                            );

                            let bg_color = if is_sel {
                                egui::Color32::from_rgba_unmultiplied(50, 70, 105, 80)
                            } else {
                                egui::Color32::from_rgba_unmultiplied(35, 40, 52, 40)
                            };

                            let border_stroke = if is_sel {
                                egui::Stroke::new(
                                    1.0,
                                    egui::Color32::from_rgba_unmultiplied(120, 170, 255, 120),
                                )
                            } else {
                                egui::Stroke::NONE
                            };

                            egui::Frame::new()
                                .fill(bg_color)
                                .stroke(border_stroke)
                                .corner_radius(4.0)
                                .inner_margin(egui::Margin::symmetric(6, 5))
                                .show(ui, |ui| {
                                    ui.horizontal(|ui| {
                                        // Visibility toggle
                                        let mut vis = c.visible;
                                        if ui.checkbox(&mut vis, "").changed() {
                                            actions.push(BrowserAction::ToggleCircleVisibility(c.id));
                                        }

                                        // Color indicator dot
                                        let (dot_rect, _) = ui.allocate_exact_size(
                                            egui::vec2(10.0, 10.0),
                                            egui::Sense::empty(),
                                        );
                                        ui.painter().circle_filled(dot_rect.center(), 4.0, col32);

                                        // Selectable name
                                        let name_resp = ui.selectable_label(
                                            is_sel,
                                            egui::RichText::new(&c.name).size(12.0).strong(),
                                        );
                                        if name_resp.clicked() {
                                            actions.push(BrowserAction::SelectCircle(c.id));
                                        }

                                        crate::ui::alignment::render_feature_badge_ui(
                                            ui,
                                            cur_axis,
                                            is_orig,
                                        );

                                        // Delete button on the right
                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .small_button(
                                                        egui::RichText::new("✕")
                                                            .size(11.0)
                                                            .color(egui::Color32::from_rgb(
                                                                220, 100, 100,
                                                            )),
                                                    )
                                                    .on_hover_text("Delete circle")
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::DeleteCircle(c.id));
                                                }
                                            },
                                        );
                                    });

                                    // If selected, show details & alignment actions
                                    if is_sel {
                                        ui.add_space(3.0);
                                        let center = c.fit.center;
                                        let radius = c.fit.radius;
                                        let rad_rms = c.fit.radial_rms.sqrt();

                                        ui.horizontal(|ui| {
                                            ui.add_space(20.0);
                                            ui.vertical(|ui| {
                                                ui.label(
                                                    egui::RichText::new(format!(
                                                        "R: {:.3} mm  Center: ({:.1}, {:.1}, {:.1})",
                                                        radius, center.x, center.y, center.z
                                                    ))
                                                    .size(10.5)
                                                    .color(egui::Color32::from_rgb(170, 180, 195)),
                                                );
                                                ui.label(
                                                    egui::RichText::new(format!(
                                                        "Radial RMS: {:.4} mm",
                                                        rad_rms
                                                    ))
                                                    .size(10.0)
                                                    .color(egui::Color32::from_rgb(140, 150, 165)),
                                                );

                                                ui.add_space(2.0);
                                                ui.horizontal(|ui| {
                                                    if ui.small_button("Axis → X").clicked() {
                                                        actions.push(BrowserAction::AlignCircle(
                                                            c.id,
                                                            Vec3::X,
                                                        ));
                                                    }
                                                    if ui.small_button("Axis → Y").clicked() {
                                                        actions.push(BrowserAction::AlignCircle(
                                                            c.id,
                                                            Vec3::Y,
                                                        ));
                                                    }
                                                    if ui.small_button("Axis → Z").clicked() {
                                                        actions.push(BrowserAction::AlignCircle(
                                                            c.id,
                                                            Vec3::Z,
                                                        ));
                                                    }
                                                    if ui
                                                        .small_button("Center at origin")
                                                        .clicked()
                                                    {
                                                        actions.push(
                                                            BrowserAction::OriginAtCircle(c.id),
                                                        );
                                                    }
                                                });
                                                ui.add_space(2.0);
                                                ui.horizontal(|ui| {
                                                    if ui
                                                        .small_button("Export Circle…")
                                                        .on_hover_text("Export this circle for Fusion 360 / CAD (STEP, Script, DXF)")
                                                        .clicked()
                                                    {
                                                        actions.push(BrowserAction::ExportCircle(c.id));
                                                    }
                                                });
                                                ui.add_space(2.0);
                                                if let Some(action) = crate::ui::alignment::render_feature_assignment_buttons_ui(
                                                    ui,
                                                    feat,
                                                    cur_axis,
                                                    is_orig,
                                                ) {
                                                    match action {
                                                        crate::ui::alignment::FeatureAssignmentAction::Assign(ax) => {
                                                            actions.push(BrowserAction::ToggleAssign(feat, ax));
                                                        }
                                                        crate::ui::alignment::FeatureAssignmentAction::ToggleOrigin => {
                                                            actions.push(BrowserAction::ToggleOrigin(feat));
                                                        }
                                                    }
                                                }
                                            });
                                        });
                                    }
                                });
                            ui.add_space(2.0);
                        }
                    }

                    // --- SECTION 4: FITTED FREEFORMS ---
                    let freeform_count = app.freeforms.len();
                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(format!("FITTED FREEFORMS ({freeform_count})"))
                                .size(10.5)
                                .strong()
                                .color(egui::Color32::from_rgb(130, 145, 170)),
                        );
                    });

                    if app.freeforms.is_empty() {
                        ui.label(
                            egui::RichText::new("No freeform surfaces fitted yet")
                                .italics()
                                .size(11.0)
                                .color(egui::Color32::from_gray(110)),
                        );
                    } else {
                        for f in &app.freeforms {
                            let is_sel = app.selected_freeform_id == Some(f.id);
                            let fitting = app.freeform_job.map(|(_, fid)| fid) == Some(f.id)
                                || f.refit_pending;

                            let col32 = egui::Color32::from_rgba_unmultiplied(
                                (f.color[0] * 255.0) as u8,
                                (f.color[1] * 255.0) as u8,
                                (f.color[2] * 255.0) as u8,
                                255,
                            );

                            let bg_color = if is_sel {
                                egui::Color32::from_rgba_unmultiplied(50, 70, 105, 80)
                            } else {
                                egui::Color32::from_rgba_unmultiplied(35, 40, 52, 40)
                            };

                            let border_stroke = if is_sel {
                                egui::Stroke::new(
                                    1.0,
                                    egui::Color32::from_rgba_unmultiplied(120, 170, 255, 120),
                                )
                            } else {
                                egui::Stroke::NONE
                            };

                            egui::Frame::new()
                                .fill(bg_color)
                                .stroke(border_stroke)
                                .corner_radius(4.0)
                                .inner_margin(egui::Margin::symmetric(6, 5))
                                .show(ui, |ui| {
                                    ui.horizontal(|ui| {
                                        // Visibility toggle
                                        let mut vis = f.visible;
                                        if ui.checkbox(&mut vis, "").changed() {
                                            actions.push(BrowserAction::ToggleFreeformVisibility(f.id));
                                        }

                                        // Color indicator dot
                                        let (dot_rect, _) = ui.allocate_exact_size(
                                            egui::vec2(10.0, 10.0),
                                            egui::Sense::empty(),
                                        );
                                        ui.painter().circle_filled(dot_rect.center(), 4.0, col32);

                                        // Selectable name
                                        let name_resp = ui.selectable_label(
                                            is_sel,
                                            egui::RichText::new(&f.name).size(12.0).strong(),
                                        );
                                        if name_resp.clicked() {
                                            actions.push(BrowserAction::SelectFreeform(f.id));
                                        }

                                        // Delete button on the right
                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .small_button(
                                                        egui::RichText::new("✕")
                                                            .size(11.0)
                                                            .color(egui::Color32::from_rgb(
                                                                220, 100, 100,
                                                            )),
                                                    )
                                                    .on_hover_text("Delete freeform surface")
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::DeleteFreeform(f.id));
                                                }
                                            },
                                        );
                                    });

                                    // If selected, show details
                                    if is_sel {
                                        ui.add_space(3.0);
                                        ui.horizontal(|ui| {
                                            ui.add_space(20.0);
                                            ui.vertical(|ui| {
                                                if fitting {
                                                    ui.horizontal(|ui| {
                                                        ui.add(egui::Spinner::new());
                                                        ui.label(
                                                            egui::RichText::new("Fitting surface…")
                                                                .size(10.5)
                                                                .color(egui::Color32::from_rgb(
                                                                    170, 180, 195,
                                                                )),
                                                        );
                                                    });
                                                } else if f.surface.is_some() {
                                                    ui.label(
                                                        egui::RichText::new(format!(
                                                            "RMS: {:.4} mm · overshoot: {:.2} mm · {} tris",
                                                            f.rms,
                                                            f.params.overshoot_mm,
                                                            f.surface.as_ref().unwrap().triangle_count()
                                                        ))
                                                        .size(10.5)
                                                        .color(egui::Color32::from_rgb(170, 180, 195)),
                                                    );
                                                }
                                                ui.add_space(2.0);
                                                if ui
                                                    .small_button("Export Freeform…")
                                                    .on_hover_text(
                                                        "Export as STEP CAD surface (B-spline) or mesh (STL, OBJ, PLY)",
                                                    )
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::ExportFreeform(f.id));
                                                }
                                            });
                                        });
                                    }
                                });
                            ui.add_space(2.0);
                        }
                    }

                    // --- SECTION 5: FIT TO OBJECT FEATURES ---
                    let fit_feat_count = app.fit_features.len();
                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(format!("FIT TO OBJECT FEATURES ({fit_feat_count})"))
                                .size(10.5)
                                .strong()
                                .color(egui::Color32::from_rgb(130, 145, 170)),
                        );
                    });

                    if app.fit_features.is_empty() {
                        ui.label(
                            egui::RichText::new("No fit-to-object features created yet")
                                .italics()
                                .size(11.0)
                                .color(egui::Color32::from_gray(110)),
                        );
                    } else {
                        for feat in &app.fit_features {
                            let is_sel = app.selected_fit_feature_id == Some(feat.id);
                            let col32 = egui::Color32::from_rgba_unmultiplied(
                                (feat.color[0] * 255.0) as u8,
                                (feat.color[1] * 255.0) as u8,
                                (feat.color[2] * 255.0) as u8,
                                255,
                            );

                            let bg_color = if is_sel {
                                egui::Color32::from_rgba_unmultiplied(50, 70, 105, 80)
                            } else {
                                egui::Color32::from_rgba_unmultiplied(35, 40, 52, 40)
                            };

                            let border_stroke = if is_sel {
                                egui::Stroke::new(
                                    1.0,
                                    egui::Color32::from_rgba_unmultiplied(120, 170, 255, 120),
                                )
                            } else {
                                egui::Stroke::NONE
                            };

                            egui::Frame::new()
                                .fill(bg_color)
                                .stroke(border_stroke)
                                .corner_radius(4.0)
                                .inner_margin(egui::Margin::symmetric(6, 5))
                                .show(ui, |ui| {
                                    ui.horizontal(|ui| {
                                        // Visibility toggle
                                        let mut vis = feat.visible;
                                        if ui.checkbox(&mut vis, "").changed() {
                                            actions.push(BrowserAction::ToggleFitFeatureVisibility(feat.id));
                                        }

                                        // Color indicator dot
                                        let (dot_rect, _) = ui.allocate_exact_size(
                                            egui::vec2(10.0, 10.0),
                                            egui::Sense::empty(),
                                        );
                                        ui.painter().circle_filled(dot_rect.center(), 4.0, col32);

                                        // Selectable name
                                        let name_resp = ui.selectable_label(
                                            is_sel,
                                            egui::RichText::new(&feat.name).size(12.0).strong(),
                                        );
                                        if name_resp.clicked() {
                                            actions.push(BrowserAction::SelectFitFeature(feat.id));
                                        }

                                        // Type badge: Surface or Solid
                                        let (badge_text, badge_color) = match feat.output_type {
                                            crate::geom::fit_to_object::FitOutputType::Faces => {
                                                ("Surface", egui::Color32::from_rgb(140, 190, 255))
                                            }
                                            crate::geom::fit_to_object::FitOutputType::Solid => {
                                                if feat.is_watertight {
                                                    ("Solid · Watertight", egui::Color32::from_rgb(100, 225, 140))
                                                } else {
                                                    ("Solid", egui::Color32::from_rgb(235, 180, 80))
                                                }
                                            }
                                        };

                                        ui.label(
                                            egui::RichText::new(badge_text)
                                                .size(10.0)
                                                .color(badge_color),
                                        );

                                        // Delete button on the right
                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .small_button(
                                                        egui::RichText::new("✕")
                                                            .size(11.0)
                                                            .color(egui::Color32::from_rgb(
                                                                220, 100, 100,
                                                            )),
                                                    )
                                                    .on_hover_text("Delete fit feature")
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::DeleteFitFeature(feat.id));
                                                }
                                            },
                                        );
                                    });

                                    // If selected, show details & export
                                    if is_sel {
                                        ui.add_space(3.0);
                                        ui.horizontal(|ui| {
                                            ui.add_space(20.0);
                                            ui.vertical(|ui| {
                                                let info_str = if let Some(v) = feat.volume {
                                                    format!(
                                                        "{} tris · area: {:.1} mm² · vol: {:.1} mm³",
                                                        feat.triangle_count, feat.surface_area, v
                                                    )
                                                } else {
                                                    format!(
                                                        "{} tris · area: {:.1} mm²",
                                                        feat.triangle_count, feat.surface_area
                                                    )
                                                };
                                                ui.label(
                                                    egui::RichText::new(info_str)
                                                        .size(10.5)
                                                        .color(egui::Color32::from_rgb(170, 180, 195)),
                                                );
                                                ui.label(
                                                    egui::RichText::new(format!(
                                                        "RMS: {:.4} mm · max dev: {:.4} mm",
                                                        feat.rms_dev, feat.max_dev
                                                    ))
                                                    .size(10.0)
                                                    .color(egui::Color32::from_rgb(140, 150, 165)),
                                                );

                                                ui.add_space(2.0);
                                                if ui
                                                    .small_button("Export Feature…")
                                                    .on_hover_text("Export feature as 3D mesh (STL, OBJ, PLY)")
                                                    .clicked()
                                                {
                                                    actions.push(BrowserAction::ExportFitFeature(feat.id));
                                                }
                                            });
                                        });
                                    }
                                });
                            ui.add_space(2.0);
                        }
                    }

                    // --- SECTION 6: SYMMETRY PLANE (if active) ---
                    let sym_show = app.sym.map(|s| s.show);
                    if let Some(mut show) = sym_show {
                        let feat = crate::geom::alignment::FeatureRef::SymmetryPlane;
                        let cur_axis = if align_slots.x == Some(feat) {
                            Some(crate::geom::alignment::AxisChoice::X)
                        } else if align_slots.y == Some(feat) {
                            Some(crate::geom::alignment::AxisChoice::Y)
                        } else if align_slots.z == Some(feat) {
                            Some(crate::geom::alignment::AxisChoice::Z)
                        } else {
                            None
                        };
                        let is_orig = matches!(
                            align_slots.origin,
                            crate::geom::alignment::OriginRef::SymmetryPlane
                        );

                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(4.0);

                        ui.label(
                            egui::RichText::new("SYMMETRY PLANE")
                                .size(10.5)
                                .strong()
                                .color(egui::Color32::from_rgb(130, 145, 170)),
                        );

                        egui::Frame::new()
                            .fill(egui::Color32::from_rgba_unmultiplied(35, 40, 52, 40))
                            .corner_radius(4.0)
                            .inner_margin(egui::Margin::symmetric(6, 5))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    if ui.checkbox(&mut show, "").changed() {
                                        actions.push(BrowserAction::ToggleSymmetryVisibility);
                                    }
                                    ui.label(
                                        egui::RichText::new("Symmetry Plane")
                                            .size(12.0)
                                            .strong()
                                            .color(egui::Color32::from_rgb(40, 220, 255)),
                                    );
                                    crate::ui::alignment::render_feature_badge_ui(
                                        ui,
                                        cur_axis,
                                        is_orig,
                                    );
                                });
                                ui.add_space(2.0);
                                if let Some(action) = crate::ui::alignment::render_feature_assignment_buttons_ui(
                                    ui,
                                    feat,
                                    cur_axis,
                                    is_orig,
                                ) {
                                    match action {
                                        crate::ui::alignment::FeatureAssignmentAction::Assign(ax) => {
                                            actions.push(BrowserAction::ToggleAssign(feat, ax));
                                        }
                                        crate::ui::alignment::FeatureAssignmentAction::ToggleOrigin => {
                                            actions.push(BrowserAction::ToggleOrigin(feat));
                                        }
                                    }
                                }
                            });
                    }

                    let has_assigned_slots = app.align_slots.x.is_some()
                        || app.align_slots.y.is_some()
                        || app.align_slots.z.is_some();

                    if has_assigned_slots {
                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(4.0);
                        ui.vertical_centered(|ui| {
                            let btn = egui::Button::new(
                                egui::RichText::new("➔ Align to features")
                                    .size(11.5)
                                    .strong()
                                    .color(egui::Color32::WHITE),
                            )
                            .fill(egui::Color32::from_rgb(45, 110, 190));

                            if ui
                                .add(btn)
                                .on_hover_text("Align scan coordinates based on assigned features")
                                .clicked()
                            {
                                actions.push(BrowserAction::AlignToFeatures);
                            }
                        });
                    }

                    if !app.planes.is_empty() || !app.circles.is_empty() {
                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(4.0);
                        ui.vertical_centered(|ui| {
                            if ui
                                .button(
                                    egui::RichText::new("Export all references…")
                                        .size(11.5)
                                        .color(egui::Color32::from_rgb(140, 200, 255)),
                                )
                                .on_hover_text("Export all visible planes and circles into a single CAD file (STEP, Script, DXF)")
                                .clicked()
                            {
                                actions.push(BrowserAction::ExportAllReferences);
                            }
                        });
                    }
                });
        });

    // Process queued actions
    for action in actions {
        match action {
            BrowserAction::ToggleMeshVisibility => {
                app.show_mesh = !app.show_mesh;
            }
            BrowserAction::TogglePlaneVisibility(id) => {
                if let Some(p) = app.planes.iter_mut().find(|p| p.id == id) {
                    p.visible = !p.visible;
                    if app.selected_plane_id == Some(id) {
                        app.show_plane = p.visible;
                    }
                }
            }
            BrowserAction::SelectPlane(id) => {
                app.select_plane(id);
            }
            BrowserAction::DeletePlane(id) => {
                app.delete_plane(id);
            }
            BrowserAction::AlignPlane(id, axis) => {
                app.rotate_plane_normal_to_axis(id, axis);
            }
            BrowserAction::OriginOnPlane(id) => {
                app.origin_on_plane_id(id);
            }
            BrowserAction::ToggleCircleVisibility(id) => {
                if let Some(c) = app.circles.iter_mut().find(|c| c.id == id) {
                    c.visible = !c.visible;
                    if app.selected_circle_id == Some(id) {
                        app.show_circle = c.visible;
                    }
                }
            }
            BrowserAction::SelectCircle(id) => {
                app.select_circle(id);
            }
            BrowserAction::DeleteCircle(id) => {
                app.delete_circle(id);
            }
            BrowserAction::AlignCircle(id, axis) => {
                app.rotate_circle_axis_to(id, axis);
            }
            BrowserAction::OriginAtCircle(id) => {
                app.origin_at_circle_id(id);
            }
            BrowserAction::FitPlane => {
                app.fit_plane_from_selection();
            }
            BrowserAction::FitCircle => {
                app.fit_circle_from_selection();
            }
            BrowserAction::FitFreeform => {
                app.fit_freeform_from_selection();
            }
            BrowserAction::ToggleFreeformVisibility(id) => {
                if let Some(f) = app.freeforms.iter_mut().find(|f| f.id == id) {
                    f.visible = !f.visible;
                }
            }
            BrowserAction::SelectFreeform(id) => {
                app.select_freeform(id);
            }
            BrowserAction::DeleteFreeform(id) => {
                app.delete_freeform(id);
            }
            BrowserAction::ExportFreeform(id) => {
                app.export_freeform_id(id);
            }
            BrowserAction::ToggleFitFeatureVisibility(id) => {
                app.toggle_fit_feature_visibility(id);
            }
            BrowserAction::SelectFitFeature(id) => {
                app.select_fit_feature(id);
            }
            BrowserAction::DeleteFitFeature(id) => {
                app.delete_fit_feature(id);
            }
            BrowserAction::ExportFitFeature(id) => {
                app.export_fit_feature_id(id);
            }
            BrowserAction::ToggleSymmetryVisibility => {
                if let Some(sym) = &mut app.sym {
                    sym.show = !sym.show;
                }
            }
            BrowserAction::ExportPlane(id) => {
                app.export_plane_id(id);
            }
            BrowserAction::ExportCircle(id) => {
                app.export_circle_id(id);
            }
            BrowserAction::ExportAllReferences => {
                app.export_all_references();
            }
            BrowserAction::AlignToFeatures => {
                app.align_to_features();
            }
            BrowserAction::ToggleAssign(feat, axis) => {
                app.toggle_assign_feature(feat, axis);
            }
            BrowserAction::ToggleOrigin(feat) => {
                app.toggle_origin_feature(feat);
            }
            BrowserAction::HideSelection => {
                app.hide_selection();
            }
            BrowserAction::ToggleHiddenRegionVisibility(id) => {
                app.toggle_hidden_region_visibility(id);
            }
            BrowserAction::RestoreHiddenRegion(id) => {
                app.restore_hidden_region(id);
            }
            BrowserAction::RestoreAllHiddenRegions => {
                app.restore_all_hidden_regions();
            }
            BrowserAction::DeleteHiddenRegion(id) => {
                app.delete_hidden_region(id);
            }
        }
    }
}
