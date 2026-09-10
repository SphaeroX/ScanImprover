//! Visual theme: WCAG AA conformant dark and light palettes, applied to
//! egui's style, plus shared frame / widget helpers so every panel uses the
//! same tokens.
//!
//! Every text color / background pair in [`Palette`] keeps a contrast ratio
//! of at least 4.5:1 (WCAG 2.1 AA for normal text); a unit test enforces it.

use egui::{Color32, CornerRadius, Margin, Stroke};
use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThemeMode {
    Dark,
    Light,
}

impl ThemeMode {
    pub fn label(self) -> &'static str {
        match self {
            ThemeMode::Dark => "Dark",
            ThemeMode::Light => "Light",
        }
    }
}

/// All color tokens of one theme.
pub struct Palette {
    /// Window and outer background.
    pub bg_app: Color32,
    /// Side panel / toolbar background.
    pub panel: Color32,
    /// Cards, headers, rows.
    pub panel_elevated: Color32,
    /// Hovered rows and inputs.
    pub panel_hover: Color32,
    pub border: Color32,
    pub border_strong: Color32,

    pub text_strong: Color32,
    pub text: Color32,
    pub text_muted: Color32,
    pub text_faint: Color32,

    /// Accent used for fills behind white text (buttons, active chips).
    pub accent: Color32,
    /// Accent used as text / icon color on panel backgrounds.
    pub accent_text: Color32,
    pub select_accent: Color32,
    pub erase_accent: Color32,
    pub success: Color32,
    pub warn: Color32,
    pub danger: Color32,

    pub axis_x: Color32,
    pub axis_y: Color32,
    pub axis_z: Color32,

    /// Viewport background gradient (top, bottom), linear RGB.
    pub viewport_top: [f32; 3],
    pub viewport_bottom: [f32; 3],
    /// Ground grid lines (RGBA, linear).
    pub grid_minor: [f32; 4],
    pub grid_major: [f32; 4],
    pub bbox_line: [f32; 4],
    pub symmetry_plane: [f32; 4],
    pub is_dark: bool,
}

pub const DARK: Palette = Palette {
    bg_app: Color32::from_rgb(27, 29, 34),
    panel: Color32::from_rgb(34, 37, 43),
    panel_elevated: Color32::from_rgb(43, 47, 54),
    panel_hover: Color32::from_rgb(53, 58, 67),
    border: Color32::from_rgba_premultiplied(16, 16, 16, 16),
    border_strong: Color32::from_rgba_premultiplied(34, 34, 34, 34),
    text_strong: Color32::from_rgb(240, 242, 246),
    text: Color32::from_rgb(210, 214, 222),
    text_muted: Color32::from_rgb(158, 164, 176),
    text_faint: Color32::from_rgb(158, 164, 176),
    accent: Color32::from_rgb(45, 106, 210),
    accent_text: Color32::from_rgb(150, 190, 255),
    select_accent: Color32::from_rgb(255, 160, 60),
    erase_accent: Color32::from_rgb(255, 128, 128),
    success: Color32::from_rgb(92, 205, 140),
    warn: Color32::from_rgb(240, 180, 80),
    danger: Color32::from_rgb(250, 135, 135),
    axis_x: Color32::from_rgb(250, 130, 135),
    axis_y: Color32::from_rgb(120, 215, 110),
    axis_z: Color32::from_rgb(120, 165, 255),
    viewport_top: [0.165, 0.175, 0.205],
    viewport_bottom: [0.085, 0.090, 0.108],
    grid_minor: [0.55, 0.58, 0.64, 0.18],
    grid_major: [0.62, 0.65, 0.72, 0.34],
    bbox_line: [0.60, 0.63, 0.70, 0.55],
    symmetry_plane: [0.25, 0.75, 0.85, 1.0],
    is_dark: true,
};

pub const LIGHT: Palette = Palette {
    bg_app: Color32::from_rgb(236, 238, 242),
    panel: Color32::from_rgb(247, 248, 250),
    panel_elevated: Color32::from_rgb(232, 235, 240),
    panel_hover: Color32::from_rgb(220, 224, 231),
    border: Color32::from_rgba_premultiplied(0, 0, 0, 22),
    border_strong: Color32::from_rgba_premultiplied(0, 0, 0, 40),
    text_strong: Color32::from_rgb(20, 23, 28),
    text: Color32::from_rgb(40, 45, 54),
    text_muted: Color32::from_rgb(82, 89, 100),
    text_faint: Color32::from_rgb(88, 95, 106),
    accent: Color32::from_rgb(31, 92, 205),
    accent_text: Color32::from_rgb(28, 82, 185),
    select_accent: Color32::from_rgb(150, 74, 0),
    erase_accent: Color32::from_rgb(190, 40, 40),
    success: Color32::from_rgb(18, 108, 58),
    warn: Color32::from_rgb(140, 85, 0),
    danger: Color32::from_rgb(185, 40, 40),
    axis_x: Color32::from_rgb(180, 40, 50),
    axis_y: Color32::from_rgb(25, 110, 35),
    axis_z: Color32::from_rgb(30, 80, 200),
    viewport_top: [0.905, 0.915, 0.935],
    viewport_bottom: [0.760, 0.775, 0.805],
    grid_minor: [0.25, 0.28, 0.33, 0.22],
    grid_major: [0.20, 0.23, 0.28, 0.40],
    bbox_line: [0.20, 0.23, 0.30, 0.65],
    symmetry_plane: [0.05, 0.45, 0.60, 1.0],
    is_dark: false,
};

static MODE: AtomicU8 = AtomicU8::new(0);

/// The active palette.
pub fn pal() -> &'static Palette {
    if MODE.load(Ordering::Relaxed) == 1 {
        &LIGHT
    } else {
        &DARK
    }
}

/// Viewport background gradient (top, bottom) of the active palette.
pub fn viewport_gradient() -> ([f32; 3], [f32; 3]) {
    let p = pal();
    (p.viewport_top, p.viewport_bottom)
}

// --- Style ------------------------------------------------------------------

/// Applies `mode` to the context when it differs from the active theme (or
/// on the first call).
pub fn sync(ctx: &egui::Context, mode: ThemeMode) {
    let id = egui::Id::new("scanimprover-theme-applied");
    let applied = ctx.data(|d| d.get_temp::<ThemeMode>(id));
    if applied == Some(mode) {
        return;
    }
    MODE.store(
        if mode == ThemeMode::Light { 1 } else { 0 },
        Ordering::Relaxed,
    );
    ctx.data_mut(|d| d.insert_temp(id, mode));
    let p = pal();
    ctx.set_theme(match mode {
        ThemeMode::Dark => egui::Theme::Dark,
        ThemeMode::Light => egui::Theme::Light,
    });
    ctx.all_styles_mut(|style| {
        style.spacing.item_spacing = egui::vec2(6.0, 5.0);
        style.spacing.button_padding = egui::vec2(8.0, 3.0);
        style.spacing.interact_size = egui::vec2(28.0, 20.0);
        style.spacing.slider_width = 130.0;
        style.spacing.combo_width = 120.0;
        style.spacing.indent = 14.0;
        style.spacing.window_margin = Margin::same(10);
        style.spacing.menu_margin = Margin::same(6);
        style.interaction.selectable_labels = false;
        style.interaction.tooltip_delay = 0.35;
        style.animation_time = 0.12;
        if let Some(body) = style.text_styles.get_mut(&egui::TextStyle::Body) {
            body.size = 13.0;
        }
        if let Some(button) = style.text_styles.get_mut(&egui::TextStyle::Button) {
            button.size = 13.0;
        }
        if let Some(small) = style.text_styles.get_mut(&egui::TextStyle::Small) {
            small.size = 10.5;
        }

        let v = &mut style.visuals;
        *v = if p.is_dark {
            egui::Visuals::dark()
        } else {
            egui::Visuals::light()
        };
        v.override_text_color = Some(p.text);
        v.panel_fill = p.panel;
        v.window_fill = p.panel_elevated;
        v.window_stroke = Stroke::new(1.0, p.border_strong);
        v.window_corner_radius = CornerRadius::same(8);
        v.window_shadow = egui::Shadow {
            offset: [0, 6],
            blur: 18,
            spread: 0,
            color: Color32::from_black_alpha(if p.is_dark { 120 } else { 60 }),
        };
        v.popup_shadow = egui::Shadow {
            offset: [0, 4],
            blur: 12,
            spread: 0,
            color: Color32::from_black_alpha(if p.is_dark { 110 } else { 50 }),
        };
        v.menu_corner_radius = CornerRadius::same(6);
        v.faint_bg_color = p.panel_elevated;
        v.extreme_bg_color = p.bg_app;
        v.code_bg_color = p.bg_app;
        v.text_edit_bg_color = Some(p.bg_app);
        v.hyperlink_color = p.accent_text;
        v.warn_fg_color = p.warn;
        v.error_fg_color = p.danger;
        v.striped = true;
        v.slider_trailing_fill = true;
        v.handle_shape = egui::style::HandleShape::Rect { aspect_ratio: 0.5 };
        v.selection.bg_fill = p.accent.gamma_multiply(0.55);
        v.selection.stroke = Stroke::new(1.0, p.accent_text);
        let widget = |bg: Color32, stroke: Color32, fg: Color32| egui::style::WidgetVisuals {
            bg_fill: bg,
            weak_bg_fill: bg,
            bg_stroke: Stroke::new(1.0, stroke),
            fg_stroke: Stroke::new(1.0, fg),
            corner_radius: CornerRadius::same(4),
            expansion: 0.0,
        };
        v.widgets.noninteractive = widget(p.panel, p.border, p.text);
        v.widgets.inactive = widget(p.panel_elevated, p.border, p.text);
        v.widgets.hovered = widget(p.panel_hover, p.border_strong, p.text_strong);
        v.widgets.active = widget(p.accent, p.accent, Color32::WHITE);
        v.widgets.open = widget(p.panel_hover, p.accent.gamma_multiply(0.6), p.text_strong);
    });
}

// --- Frames -----------------------------------------------------------------

pub fn top_bar_frame() -> egui::Frame {
    egui::Frame::new()
        .fill(pal().panel)
        .inner_margin(Margin::symmetric(10, 6))
        .stroke(Stroke::NONE)
}

pub fn status_bar_frame() -> egui::Frame {
    egui::Frame::new()
        .fill(pal().panel)
        .inner_margin(Margin::symmetric(10, 4))
}

pub fn side_panel_frame() -> egui::Frame {
    egui::Frame::new().fill(pal().panel).inner_margin(Margin {
        left: 10,
        right: 8,
        top: 8,
        bottom: 8,
    })
}

/// Frame for grouped controls: no border, just a little breathing room.
pub fn card_frame() -> egui::Frame {
    egui::Frame::new().inner_margin(Margin {
        left: 2,
        right: 2,
        top: 2,
        bottom: 4,
    })
}

/// Floating overlay frame (selection HUD).
pub fn floating_frame() -> egui::Frame {
    let p = pal();
    egui::Frame::new()
        .fill(p.panel_elevated.gamma_multiply(0.97))
        .stroke(Stroke::new(1.0, p.border_strong))
        .corner_radius(8.0)
        .inner_margin(Margin::symmetric(10, 8))
        .shadow(egui::Shadow {
            offset: [0, 6],
            blur: 16,
            spread: 0,
            color: Color32::from_black_alpha(if p.is_dark { 100 } else { 50 }),
        })
}

// --- Widgets ----------------------------------------------------------------

/// Small uppercase section caption.
pub fn caption(ui: &mut egui::Ui, text: &str) {
    ui.label(
        egui::RichText::new(text.to_uppercase())
            .size(10.5)
            .strong()
            .color(pal().text_muted),
    );
}

/// A toolbar toggle that reads as a "checked" chip when active.
pub fn toggle_chip(ui: &mut egui::Ui, on: &mut bool, label: &str, tooltip: &str) -> egui::Response {
    let p = pal();
    let text =
        egui::RichText::new(label)
            .size(12.5)
            .color(if *on { Color32::WHITE } else { p.text });
    let mut btn = egui::Button::new(text)
        .corner_radius(5.0)
        .min_size(egui::vec2(0.0, 24.0));
    if *on {
        btn = btn.fill(p.accent).stroke(Stroke::new(1.0, p.accent));
    } else {
        btn = btn
            .fill(Color32::TRANSPARENT)
            .stroke(Stroke::new(1.0, p.border_strong));
    }
    let resp = ui.add(btn).on_hover_text(tooltip);
    if resp.clicked() {
        *on = !*on;
    }
    resp
}

/// A flat toolbar button.
pub fn tool_button(ui: &mut egui::Ui, label: &str, tooltip: &str, enabled: bool) -> egui::Response {
    let text = egui::RichText::new(label).size(12.5);
    let btn = egui::Button::new(text)
        .corner_radius(5.0)
        .min_size(egui::vec2(0.0, 24.0))
        .fill(Color32::TRANSPARENT)
        .stroke(Stroke::new(1.0, pal().border_strong));
    ui.add_enabled(enabled, btn).on_hover_text(tooltip)
}

/// A primary (accent filled) action button.
pub fn primary_button(ui: &mut egui::Ui, label: &str) -> egui::Response {
    let p = pal();
    ui.add(
        egui::Button::new(egui::RichText::new(label).strong().color(Color32::WHITE))
            .fill(p.accent)
            .stroke(Stroke::new(1.0, p.accent))
            .corner_radius(5.0),
    )
}

/// Small pill badge.
pub fn badge(ui: &mut egui::Ui, text: &str, fg: Color32, bg: Color32) {
    egui::Frame::new()
        .fill(bg)
        .corner_radius(4.0)
        .inner_margin(Margin::symmetric(5, 1))
        .show(ui, |ui| {
            ui.label(egui::RichText::new(text).size(10.5).strong().color(fg));
        });
}

/// Small disclosure triangle (the UI font has no arrow glyphs).
pub fn paint_arrow(painter: &egui::Painter, center: egui::Pos2, open: bool, color: Color32) {
    let s = 4.0;
    let pts = if open {
        vec![
            center + egui::vec2(-s, -s * 0.6),
            center + egui::vec2(s, -s * 0.6),
            center + egui::vec2(0.0, s * 0.7),
        ]
    } else {
        vec![
            center + egui::vec2(-s * 0.6, -s),
            center + egui::vec2(s * 0.7, 0.0),
            center + egui::vec2(-s * 0.6, s),
        ]
    };
    painter.add(egui::Shape::convex_polygon(pts, color, Stroke::NONE));
}

/// Thin vertical separator for toolbars.
pub fn vsep(ui: &mut egui::Ui) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(1.0, 20.0), egui::Sense::hover());
    ui.painter().rect_filled(rect, 0.0, pal().border_strong);
    ui.add_space(2.0);
}

/// Converts a linear RGB(A) color into an egui color.
pub fn color32(c: [f32; 4]) -> Color32 {
    Color32::from_rgba_unmultiplied(
        (c[0].clamp(0.0, 1.0) * 255.0) as u8,
        (c[1].clamp(0.0, 1.0) * 255.0) as u8,
        (c[2].clamp(0.0, 1.0) * 255.0) as u8,
        (c[3].clamp(0.0, 1.0) * 255.0) as u8,
    )
}

/// Thousands separated integer.
pub fn format_count(n: usize) -> String {
    let s = n.to_string();
    let mut out = String::with_capacity(s.len() + s.len() / 3);
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(ch);
    }
    out
}

/// WCAG 2.1 relative luminance of an opaque sRGB color.
#[cfg_attr(not(test), allow(dead_code))]
pub fn relative_luminance(c: Color32) -> f64 {
    let lin = |v: u8| {
        let s = v as f64 / 255.0;
        if s <= 0.04045 {
            s / 12.92
        } else {
            ((s + 0.055) / 1.055).powf(2.4)
        }
    };
    0.2126 * lin(c.r()) + 0.7152 * lin(c.g()) + 0.0722 * lin(c.b())
}

/// WCAG 2.1 contrast ratio between two opaque colors (1..=21).
#[cfg_attr(not(test), allow(dead_code))]
pub fn contrast_ratio(a: Color32, b: Color32) -> f64 {
    let (la, lb) = (relative_luminance(a), relative_luminance(b));
    let (hi, lo) = if la > lb { (la, lb) } else { (lb, la) };
    (hi + 0.05) / (lo + 0.05)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_count_groups_thousands() {
        assert_eq!(format_count(0), "0");
        assert_eq!(format_count(999), "999");
        assert_eq!(format_count(1000), "1,000");
        assert_eq!(format_count(1234567), "1,234,567");
    }

    #[test]
    fn contrast_ratio_matches_reference_values() {
        assert!((contrast_ratio(Color32::WHITE, Color32::BLACK) - 21.0).abs() < 1e-9);
        assert!((contrast_ratio(Color32::WHITE, Color32::WHITE) - 1.0).abs() < 1e-9);
    }

    /// Every text token must reach WCAG AA (4.5:1) on every surface it is
    /// drawn on, in both themes.
    #[test]
    fn palettes_are_wcag_aa_conformant() {
        for (name, p) in [("dark", &DARK), ("light", &LIGHT)] {
            let surfaces = [
                ("bg_app", p.bg_app),
                ("panel", p.panel),
                ("panel_elevated", p.panel_elevated),
                ("panel_hover", p.panel_hover),
            ];
            let texts = [
                ("text_strong", p.text_strong),
                ("text", p.text),
                ("text_muted", p.text_muted),
                ("text_faint", p.text_faint),
                ("accent_text", p.accent_text),
                ("select_accent", p.select_accent),
                ("erase_accent", p.erase_accent),
                ("success", p.success),
                ("warn", p.warn),
                ("danger", p.danger),
                ("axis_x", p.axis_x),
                ("axis_y", p.axis_y),
                ("axis_z", p.axis_z),
            ];
            for (sn, s) in surfaces {
                for (tn, t) in texts {
                    let r = contrast_ratio(t, s);
                    assert!(r >= 4.5, "{name}: {tn} on {sn} = {r:.2}:1");
                }
            }
            let r = contrast_ratio(Color32::WHITE, p.accent);
            assert!(r >= 4.5, "{name}: white on accent = {r:.2}:1");
            let r = contrast_ratio(p.accent, p.panel);
            assert!(r >= 3.0, "{name}: accent on panel = {r:.2}:1");
        }
    }
}
