//! Scripted demo recording (`--demo <file.mp4> <part> <scan> <large>`) and
//! README screenshots (`--screenshots <dir> <part> <scan> <large> [extra]`).
//!
//! Drives the application with synthetic pointer and keyboard input on a
//! fixed 30 fps clock, draws a cursor and captions over the interface and
//! streams every rendered frame to `ffmpeg`, which encodes the video; the
//! screenshot mode runs a shorter script and saves single frames as PNG. The
//! real mouse and keyboard are ignored while a script runs.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, ScreenshotCaptured};
use bevy::time::TimeUpdateStrategy;
use bevy_egui::{EguiInput, EguiInputSet, EguiPreUpdateSet, PrimaryEguiContext};
use glam::Vec3;

use crate::app::App as ScanApp;
use crate::camera::UpAxis;
use crate::ui::ToolSection;
use crate::ui::theme::{self, ThemeMode};

pub const FPS: u32 = 30;
/// Longest video side after scaling (the window is captured at its physical
/// resolution, which is larger on high-DPI displays).
const OUTPUT_WIDTH: u32 = 1920;

/// Mesh files used by the script.
pub struct DemoAssets {
    /// A CAD-like part with flat faces and cylinders (face groups, fitting,
    /// alignment, symmetry).
    pub part: PathBuf,
    /// A scan with holes (repair).
    pub scan: PathBuf,
    /// A dense mesh (decimation).
    pub large: PathBuf,
    /// Optional second part for the overview screenshots.
    pub extra: Option<PathBuf>,
}

/// What a scripted run produces.
pub enum DemoMode {
    /// A video file encoded by ffmpeg.
    Video(PathBuf),
    /// Single PNG frames in a directory.
    Stills(PathBuf),
}

// ----------------------------------------------------------------------
// Overlay state owned by the app (drawn by the egui pass)
// ----------------------------------------------------------------------

/// Cursor and caption drawn over the interface while a demo runs.
#[derive(Default)]
pub struct DemoOverlay {
    pub cursor: Option<egui::Pos2>,
    pub pressed: bool,
    pub caption: Option<Caption>,
    pub frame: u32,
}

pub struct Caption {
    pub title: String,
    pub text: String,
    /// Frame at which the caption appeared (for the fade-in).
    pub since: u32,
    /// Large centred title card instead of the corner card.
    pub big: bool,
}

/// Paints the demo cursor and caption on a foreground layer.
pub fn draw_overlay(app: &ScanApp, ctx: &egui::Context) {
    let Some(demo) = &app.demo else {
        return;
    };
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Tooltip,
        egui::Id::new("demo_overlay"),
    ));
    let pal = theme::pal();
    let viewport = app.frame.viewport_rect;

    if let Some(c) = &demo.caption {
        let alpha = ((demo.frame.saturating_sub(c.since)) as f32 / 12.0).clamp(0.0, 1.0);
        let card = pal.panel.gamma_multiply(0.92 * alpha);
        let title_col = pal.text_strong.gamma_multiply(alpha);
        let text_col = pal.text.gamma_multiply(alpha);
        if c.big {
            let title_font = theme::semibold(ctx, 72.0);
            let text_font = egui::FontId::proportional(26.0);
            let title = painter.layout_no_wrap(c.title.clone(), title_font, title_col);
            let text = painter.layout_no_wrap(c.text.clone(), text_font, text_col);
            let w = title.size().x.max(text.size().x) + 110.0;
            let h = title.size().y + text.size().y + 90.0;
            let rect = egui::Rect::from_center_size(viewport.center(), egui::vec2(w, h));
            painter.rect_filled(rect, 12.0, card);
            painter.rect_stroke(
                rect,
                12.0,
                egui::Stroke::new(1.0, pal.accent.gamma_multiply(0.6 * alpha)),
                egui::StrokeKind::Inside,
            );
            let mut y = rect.min.y + 40.0;
            painter.galley(
                egui::pos2(rect.center().x - title.size().x * 0.5, y),
                title,
                title_col,
            );
            y += 72.0 + 18.0;
            painter.galley(
                egui::pos2(rect.center().x - text.size().x * 0.5, y),
                text,
                text_col,
            );
        } else {
            let title_font = theme::semibold(ctx, 26.0);
            let text_font = egui::FontId::proportional(17.0);
            let max_w = (viewport.width() * 0.55).max(320.0);
            let title = painter.layout(c.title.clone(), title_font, title_col, max_w);
            let text = painter.layout(c.text.clone(), text_font, text_col, max_w);
            let w = title.size().x.max(text.size().x) + 48.0;
            let h = title.size().y + text.size().y + 40.0;
            let min = egui::pos2(viewport.min.x + 28.0, viewport.max.y - 28.0 - h);
            let rect = egui::Rect::from_min_size(min, egui::vec2(w, h));
            painter.rect_filled(rect, 10.0, card);
            painter.rect_filled(
                egui::Rect::from_min_size(rect.min, egui::vec2(4.0, h)),
                egui::CornerRadius {
                    nw: 10,
                    sw: 10,
                    ne: 0,
                    se: 0,
                },
                pal.accent.gamma_multiply(alpha),
            );
            let x = rect.min.x + 24.0;
            painter.galley(egui::pos2(x, rect.min.y + 16.0), title.clone(), title_col);
            painter.galley(
                egui::pos2(x, rect.min.y + 16.0 + title.size().y + 8.0),
                text,
                text_col,
            );
        }
    }

    if let Some(p) = demo.cursor {
        if demo.pressed {
            painter.circle_filled(p, 14.0, pal.accent.gamma_multiply(0.35));
        }
        // Classic arrow cursor.
        let pts = [
            (0.0, 0.0),
            (0.0, 17.0),
            (4.5, 13.0),
            (7.5, 20.0),
            (10.0, 19.0),
            (7.0, 12.0),
            (12.5, 12.0),
        ];
        let poly: Vec<egui::Pos2> = pts
            .iter()
            .map(|&(x, y)| egui::pos2(p.x + x * 1.15, p.y + y * 1.15))
            .collect();
        painter.add(egui::Shape::convex_polygon(
            poly,
            egui::Color32::WHITE,
            egui::Stroke::new(1.5, egui::Color32::BLACK),
        ));
    }
}

// ----------------------------------------------------------------------
// Script
// ----------------------------------------------------------------------

type AppFn = Box<dyn Fn(&mut ScanApp) + Send + Sync>;
type AppPred = Box<dyn Fn(&ScanApp) -> bool + Send + Sync>;

/// Positions are offsets from the projected model centre, as fractions of
/// the viewport size (`(0.0, 0.0)` is the model centre, `x` right, `y`
/// down); without a model they are offsets from the viewport centre.
enum Step {
    Do(AppFn),
    Caption {
        title: &'static str,
        text: &'static str,
        big: bool,
    },
    ClearCaption,
    Wait(u32),
    WaitUntil {
        done: AppPred,
        timeout: u32,
    },
    MoveTo {
        to: (f32, f32),
        frames: u32,
    },
    Drag {
        button: egui::PointerButton,
        to: (f32, f32),
        frames: u32,
        modifiers: egui::Modifiers,
    },
    Wheel {
        delta: f32,
        frames: u32,
    },
    Key(egui::Key, egui::Modifiers),
    /// Save the current frame as `<name>.png` (screenshot mode only).
    Shot(&'static str),
    /// Stop capturing, let pending frames drain, close the encoder and exit.
    Finish,
}

fn secs(s: f32) -> u32 {
    (s * FPS as f32).round() as u32
}

fn act(f: impl Fn(&mut ScanApp) + Send + Sync + 'static) -> Step {
    Step::Do(Box::new(f))
}

fn until(f: impl Fn(&ScanApp) -> bool + Send + Sync + 'static, timeout_s: f32) -> Step {
    Step::WaitUntil {
        done: Box::new(f),
        timeout: secs(timeout_s),
    }
}

fn caption(title: &'static str, text: &'static str) -> Step {
    Step::Caption {
        title,
        text,
        big: false,
    }
}

/// Cursor position in empty space (below-right of the model) from which a
/// right-drag orbits around the model centre.
const FREE: (f32, f32) = (0.30, 0.30);

fn orbit(to: (f32, f32), frames: u32) -> Step {
    Step::Drag {
        button: egui::PointerButton::Secondary,
        to,
        frames,
        modifiers: egui::Modifiers::NONE,
    }
}

fn paint(to: (f32, f32), frames: u32) -> Step {
    Step::Drag {
        button: egui::PointerButton::Primary,
        to,
        frames,
        modifiers: egui::Modifiers::NONE,
    }
}

fn key(k: egui::Key) -> Step {
    Step::Key(k, egui::Modifiers::NONE)
}

fn idle(app: &ScanApp) -> bool {
    app.load_job.is_none()
        && app.groups_job.is_none()
        && app.analysis_job.is_none()
        && app.edit_job.is_none()
        && app.dec_job.is_none()
        && app.dev_job.is_none()
        && app.sym_job.is_none()
        && app.export_job.is_none()
        && app.bvh_job.is_none()
        && app.topo_job.is_none()
}

fn script(assets: &DemoAssets, export_path: PathBuf) -> Vec<Step> {
    let part = assets.part.clone();
    let scan = assets.scan.clone();
    let large = assets.large.clone();
    vec![
        // --- Title -------------------------------------------------------
        act(|a| {
            a.theme_mode = ThemeMode::Dark;
            a.camera.set_up_axis(UpAxis::Z);
        }),
        Step::Caption {
            title: "ScanImprover",
            text: "Prepare 3D scans for CAD: align, clean, simplify, repair",
            big: true,
        },
        Step::Wait(secs(2.6)),
        Step::ClearCaption,
        // --- Loading -----------------------------------------------------
        caption(
            "Fast parallel loading",
            "STL, PLY and OBJ load on worker threads with a live activity feed. The interface never blocks.",
        ),
        act(move |a| a.open_file_async(part.clone())),
        until(|a| a.load_job.is_none() && a.has_mesh(), 30.0),
        Step::MoveTo {
            to: (0.0, 0.0),
            frames: secs(0.8),
        },
        Step::Wait(secs(1.2)),
        // --- Navigation --------------------------------------------------
        caption(
            "Turntable orbit camera",
            "Right-drag orbits around the point under the cursor. The wheel zooms towards the cursor.",
        ),
        orbit((0.06, -0.03), secs(2.0)),
        Step::Wait(secs(0.3)),
        Step::MoveTo {
            to: (0.0, 0.0),
            frames: secs(0.5),
        },
        Step::Wheel {
            delta: 150.0,
            frames: secs(1.2),
        },
        Step::Wait(secs(0.8)),
        key(egui::Key::F),
        Step::Wait(secs(1.0)),
        caption(
            "Animated view presets",
            "1 / 2 / 3 / 4 for the X, Y, Z and isometric views, F fits the model, G toggles the grid.",
        ),
        key(egui::Key::Num1),
        Step::Wait(secs(1.3)),
        key(egui::Key::Num3),
        Step::Wait(secs(1.3)),
        key(egui::Key::Num4),
        Step::Wait(secs(1.4)),
        caption(
            "Wireframe, grid, bounding box, navigation gizmo",
            "4x anti-aliased viewport rendered with the Bevy engine; back faces are tinted so open shells are obvious.",
        ),
        act(|a| a.show_wireframe = true),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.4),
        },
        orbit((0.42, 0.25), secs(2.0)),
        Step::Wait(secs(0.5)),
        act(|a| a.show_wireframe = false),
        key(egui::Key::F),
        Step::Wait(secs(0.4)),
        // --- Face groups -------------------------------------------------
        caption(
            "Automatic face groups",
            "Region growing by crease angle, every region classified as plane, cylinder or sphere with fitted parameters.",
        ),
        act(|a| {
            a.open_section(ToolSection::FaceGroups);
            a.request_face_groups();
        }),
        until(
            |a| a.groups_job.is_none() && !a.face_groups.is_empty(),
            30.0,
        ),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.6),
        },
        orbit((0.40, 0.26), secs(2.0)),
        Step::Wait(secs(0.6)),
        act(|a| a.groups_by_type = true),
        caption(
            "Colour by primitive type",
            "Planes, cylinders and spheres get their own colour to find functional surfaces at a glance.",
        ),
        Step::Wait(secs(2.2)),
        act(|a| {
            a.groups_by_type = false;
            a.groups_show = false;
        }),
        key(egui::Key::F),
        Step::Wait(secs(0.8)),
        // --- Brush selection ---------------------------------------------
        caption(
            "Connected brush selection",
            "The brush paints only across connected faces, so it never bleeds through thin walls. Shift erases, Ctrl + wheel grows.",
        ),
        act(|a| a.brush_radius = 22.0),
        Step::MoveTo {
            to: (-0.08, -0.04),
            frames: secs(0.7),
        },
        Step::Wait(secs(0.4)),
        paint((0.07, -0.08), secs(1.2)),
        paint((0.0, 0.06), secs(1.0)),
        Step::Wait(secs(0.8)),
        Step::Wheel {
            delta: 0.0,
            frames: 1,
        },
        // --- Plane fit and alignment -------------------------------------
        caption(
            "Fit a plane, align the part",
            "A least-squares plane is fitted to the selection; its normal becomes the Z axis and the origin snaps onto the plane.",
        ),
        act(|a| {
            a.open_section(ToolSection::Coordinates);
            a.fit_plane_from_selection();
        }),
        Step::Wait(secs(1.6)),
        act(|a| {
            a.rotate_normal_to_axis(Vec3::Z);
            a.origin_on_plane();
        }),
        Step::Wait(secs(1.6)),
        // --- Hide ----------------------------------------------------------
        caption(
            "H hides the selected faces",
            "Hidden regions are non-destructive and listed in the object browser; bring them back any time. Double-click selects a whole region.",
        ),
        key(egui::Key::H),
        Step::Wait(secs(0.3)),
        key(egui::Key::F),
        Step::Wait(secs(1.0)),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.4),
        },
        orbit((0.38, 0.32), secs(1.8)),
        Step::Wait(secs(0.8)),
        act(|a| a.restore_all_hidden_regions()),
        key(egui::Key::F),
        Step::Wait(secs(0.6)),
        // --- Symmetry ----------------------------------------------------
        caption(
            "Symmetry plane detection",
            "Finds the mirror plane of a scan automatically and can align the coordinate system to it.",
        ),
        act(|a| {
            a.open_section(ToolSection::Symmetry);
            a.schedule_sym_auto();
        }),
        until(|a| a.sym_job.is_none() && a.sym.is_some(), 30.0),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.4),
        },
        orbit((0.20, 0.30), secs(2.0)),
        Step::Wait(secs(1.0)),
        // --- Repair ------------------------------------------------------
        caption(
            "Mesh repair",
            "Health report with holes, non-manifold edges and vertices, shells and degenerate faces.",
        ),
        act(move |a| {
            a.camera.set_up_axis(UpAxis::Y);
            a.open_file_async(scan.clone());
        }),
        until(|a| a.load_job.is_none() && a.has_mesh(), 30.0),
        act(|a| {
            a.open_section(ToolSection::Repair);
            a.request_repair_analysis();
        }),
        until(|a| a.analysis_job.is_none(), 30.0),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.5),
        },
        orbit((0.40, 0.34), secs(2.0)),
        Step::Wait(secs(0.6)),
        caption(
            "Fill all holes",
            "Refined, edge-flipped and faired patches; bridges close gaps between contours.",
        ),
        act(|a| a.fill_all_holes()),
        until(|a| a.edit_job.is_none() && a.analysis_job.is_none(), 30.0),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.4),
        },
        orbit((0.19, 0.31), secs(2.2)),
        Step::Wait(secs(0.8)),
        // --- Decimation --------------------------------------------------
        caption(
            "Decimation with deviation heatmap",
            "meshoptimizer based simplification with measured surface deviation, so you know what you lose.",
        ),
        act(move |a| {
            a.camera.set_up_axis(UpAxis::Z);
            a.open_file_async(large.clone());
        }),
        until(|a| a.load_job.is_none() && a.has_mesh() && idle(a), 60.0),
        Step::MoveTo {
            to: (0.0, 0.0),
            frames: secs(0.5),
        },
        act(|a| {
            a.open_section(ToolSection::Decimation);
            a.dec_ratio = 0.12;
            a.heat_on = true;
            a.schedule_decimate();
        }),
        until(|a| a.dec_job.is_none() && a.preview.is_some(), 60.0),
        until(|a| a.dev_job.is_none() && a.heat.is_some(), 60.0),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.4),
        },
        orbit((0.40, 0.27), secs(2.4)),
        Step::MoveTo {
            to: (0.0, 0.0),
            frames: secs(0.4),
        },
        Step::Wheel {
            delta: 120.0,
            frames: secs(1.0),
        },
        Step::Wait(secs(1.2)),
        act(|a| a.apply_preview()),
        Step::Wait(secs(0.4)),
        key(egui::Key::F),
        Step::Wait(secs(1.4)),
        // --- Themes ------------------------------------------------------
        caption(
            "Light and dark themes",
            "Both palettes meet WCAG AA contrast. Panel sizes and settings are remembered.",
        ),
        act(|a| a.theme_mode = ThemeMode::Light),
        Step::MoveTo {
            to: FREE,
            frames: secs(0.4),
        },
        orbit((0.22, 0.30), secs(2.2)),
        Step::Wait(secs(0.6)),
        act(|a| a.theme_mode = ThemeMode::Dark),
        key(egui::Key::F),
        Step::Wait(secs(0.6)),
        // --- Export ------------------------------------------------------
        caption(
            "Export",
            "STL, PLY and OBJ meshes; planes, cylinders and freeform patches as STEP, DXF or a Fusion 360 script.",
        ),
        act(move |a| a.export_mesh_async(export_path.clone())),
        until(|a| a.export_job.is_none(), 60.0),
        Step::Wait(secs(1.6)),
        Step::ClearCaption,
        Step::Caption {
            title: "ScanImprover",
            text: "Open source · Rust · github.com/SphaeroX/ScanImprover",
            big: true,
        },
        Step::Wait(secs(3.0)),
        Step::Finish,
    ]
}

/// Screenshot script for the README: one still per feature.
fn stills_script(assets: &DemoAssets) -> Vec<Step> {
    let part = assets.part.clone();
    let scan = assets.scan.clone();
    let large = assets.large.clone();
    let hero = assets.extra.clone().unwrap_or_else(|| assets.part.clone());
    let loaded = |a: &ScanApp| a.load_job.is_none() && a.has_mesh() && idle(a);
    vec![
        act(|a| {
            a.theme_mode = ThemeMode::Dark;
            a.camera.set_up_axis(UpAxis::Z);
        }),
        // Overview: a mechanical part, nothing else open.
        act(move |a| a.open_file_async(hero.clone())),
        until(loaded, 60.0),
        Step::MoveTo {
            to: (0.0, 0.0),
            frames: 2,
        },
        Step::Wheel {
            delta: 240.0,
            frames: secs(0.5),
        },
        Step::MoveTo {
            to: FREE,
            frames: 2,
        },
        orbit((0.34, 0.27), secs(0.6)),
        Step::Wait(secs(0.5)),
        Step::Shot("overview"),
        // Face groups on a CAD-like part.
        act(move |a| a.open_file_async(part.clone())),
        until(loaded, 60.0),
        act(|a| {
            a.open_section(ToolSection::FaceGroups);
            a.request_face_groups();
        }),
        until(
            |a| a.groups_job.is_none() && !a.face_groups.is_empty(),
            60.0,
        ),
        Step::Wait(secs(0.5)),
        Step::Shot("face_groups"),
        // Plane fit from a painted selection, aligned to Z.
        act(|a| {
            a.groups_show = false;
            a.brush_radius = 26.0;
        }),
        Step::MoveTo {
            to: (-0.06, -0.05),
            frames: 2,
        },
        paint((0.06, -0.09), secs(0.8)),
        paint((0.02, 0.02), secs(0.6)),
        act(|a| {
            a.open_section(ToolSection::Coordinates);
            a.fit_plane_from_selection();
            a.rotate_normal_to_axis(Vec3::Z);
            a.origin_on_plane();
            a.clear_selection();
        }),
        key(egui::Key::F),
        Step::Wait(secs(1.0)),
        Step::Shot("plane_alignment"),
        // Symmetry on a scan.
        act(move |a| {
            a.camera.set_up_axis(UpAxis::Y);
            a.open_file_async(large.clone());
        }),
        until(loaded, 90.0),
        act(|a| {
            a.open_section(ToolSection::Symmetry);
            a.schedule_sym_auto();
        }),
        until(|a| a.sym_job.is_none() && a.sym.is_some(), 90.0),
        Step::MoveTo {
            to: FREE,
            frames: 2,
        },
        orbit((0.36, 0.30), secs(0.8)),
        Step::Wait(secs(0.5)),
        Step::Shot("symmetry"),
        // Decimation heatmap on the same scan.
        act(|a| {
            a.open_section(ToolSection::Decimation);
            a.dec_ratio = 0.1;
            a.heat_on = true;
            a.schedule_decimate();
        }),
        until(|a| a.dec_job.is_none() && a.preview.is_some(), 90.0),
        until(|a| a.dev_job.is_none() && a.heat.is_some(), 90.0),
        Step::Wait(secs(0.5)),
        Step::Shot("decimation"),
        act(|a| a.discard_preview()),
        // Repair: holes of a scan.
        act(move |a| a.open_file_async(scan.clone())),
        until(loaded, 90.0),
        act(|a| {
            a.open_section(ToolSection::Repair);
            a.request_repair_analysis();
        }),
        until(|a| a.analysis_job.is_none(), 90.0),
        Step::MoveTo {
            to: FREE,
            frames: 2,
        },
        orbit((0.34, 0.16), secs(0.8)),
        Step::Wait(secs(0.5)),
        Step::Shot("repair"),
        // Light theme.
        act(|a| {
            a.theme_mode = ThemeMode::Light;
            a.show_wireframe = true;
        }),
        Step::Wait(secs(0.6)),
        Step::Shot("light_theme"),
        Step::Finish,
    ]
}

// ----------------------------------------------------------------------
// Frame encoder
// ----------------------------------------------------------------------

/// Reorders captured frames (readback is asynchronous) and pipes them to
/// ffmpeg as raw RGB.
struct FrameSink {
    output: PathBuf,
    encoder: Option<(Child, ChildStdin)>,
    next: u32,
    pending: BTreeMap<u32, Vec<u8>>,
    failed: bool,
}

impl FrameSink {
    fn new(output: PathBuf) -> Self {
        FrameSink {
            output,
            encoder: None,
            next: 0,
            pending: BTreeMap::new(),
            failed: false,
        }
    }

    fn start(&mut self, w: u32, h: u32) {
        let scale = format!("scale={OUTPUT_WIDTH}:-2:flags=lanczos");
        let spawned = Command::new("ffmpeg")
            .args([
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                &format!("{w}x{h}"),
                "-r",
                &FPS.to_string(),
                "-i",
                "-",
                "-vf",
                &scale,
                "-c:v",
                "libx264",
                "-preset",
                "slow",
                "-crf",
                "18",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
            ])
            .arg(&self.output)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .spawn();
        match spawned {
            Ok(mut child) => {
                let stdin = child.stdin.take().expect("piped stdin");
                self.encoder = Some((child, stdin));
            }
            Err(e) => {
                error!("cannot start ffmpeg: {e}");
                self.failed = true;
            }
        }
    }

    fn push(&mut self, index: u32, w: u32, h: u32, rgb: Vec<u8>) {
        if self.failed {
            return;
        }
        if self.encoder.is_none() {
            self.start(w, h);
        }
        self.pending.insert(index, rgb);
        while let Some(frame) = self.pending.remove(&self.next) {
            self.write(&frame);
            self.next += 1;
        }
    }

    fn write(&mut self, frame: &[u8]) {
        if let Some((_, stdin)) = &mut self.encoder
            && let Err(e) = stdin.write_all(frame)
        {
            error!("ffmpeg pipe closed: {e}");
            self.failed = true;
        }
    }

    fn finish(&mut self) -> bool {
        let rest: Vec<Vec<u8>> = std::mem::take(&mut self.pending).into_values().collect();
        for frame in rest {
            self.write(&frame);
        }
        let Some((mut child, stdin)) = self.encoder.take() else {
            return false;
        };
        drop(stdin);
        match child.wait() {
            Ok(status) if status.success() && !self.failed => true,
            Ok(status) => {
                error!("ffmpeg exited with {status}");
                false
            }
            Err(e) => {
                error!("ffmpeg wait failed: {e}");
                false
            }
        }
    }
}

// ----------------------------------------------------------------------
// Plugin
// ----------------------------------------------------------------------

pub struct DemoPlugin {
    pub mode: DemoMode,
    pub assets: DemoAssets,
}

#[derive(Resource)]
struct DemoState {
    steps: Vec<Step>,
    index: usize,
    /// Frames spent in the current step.
    step_frame: u32,
    frame: u32,
    cursor: egui::Pos2,
    drag_start: egui::Pos2,
    /// Screen target of the current move / drag, fixed when it starts (the
    /// projected model centre moves while the camera orbits).
    drag_target: egui::Pos2,
    modifiers: egui::Modifiers,
    pressed: Option<egui::PointerButton>,
    recording: bool,
    captured: u32,
    /// Frames to wait after the last capture before closing the encoder.
    drain: Option<u32>,
    sink: Arc<Mutex<FrameSink>>,
    /// Video output, or `None` in screenshot mode.
    video: Option<PathBuf>,
    /// Screenshot directory and the pending screenshot name.
    stills_dir: PathBuf,
    shot: Option<&'static str>,
}

impl Plugin for DemoPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        let (video, stills_dir, steps) = match &self.mode {
            DemoMode::Video(out) => {
                let export_path = out.with_file_name("scanimprover_demo_export.stl");
                (
                    Some(out.clone()),
                    out.parent().map(PathBuf::from).unwrap_or_default(),
                    script(&self.assets, export_path),
                )
            }
            DemoMode::Stills(dir) => (None, dir.clone(), stills_script(&self.assets)),
        };
        let sink_path = video.clone().unwrap_or_default();
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / FPS as f64,
        )))
        .insert_resource(DemoState {
            steps,
            index: 0,
            step_frame: 0,
            frame: 0,
            cursor: egui::pos2(-100.0, -100.0),
            drag_start: egui::pos2(0.0, 0.0),
            drag_target: egui::pos2(0.0, 0.0),
            modifiers: egui::Modifiers::NONE,
            pressed: None,
            recording: false,
            captured: 0,
            drain: None,
            sink: Arc::new(Mutex::new(FrameSink::new(sink_path))),
            video,
            stills_dir,
            shot: None,
        })
        .add_systems(
            PreUpdate,
            drive_demo
                .after(EguiInputSet::WriteEguiEvents)
                .before(EguiPreUpdateSet::BeginPass),
        )
        .add_systems(Update, capture_frame);
    }
}

fn ease(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn viewport_pos(scan: &ScanApp, offset: (f32, f32)) -> egui::Pos2 {
    let r = scan.frame.viewport_rect;
    let center = if scan.has_mesh() {
        scan.camera
            .project(scan.bbox.center(), r.width(), r.height())
            .map(|(x, y)| egui::pos2(r.min.x + x, r.min.y + y))
            .filter(|p| r.contains(*p))
            .unwrap_or(r.center())
    } else {
        r.center()
    };
    egui::pos2(
        center.x + r.width() * offset.0,
        center.y + r.height() * offset.1,
    )
}

/// Replaces the real input with the scripted events for this frame and
/// advances the script.
fn drive_demo(
    mut state: ResMut<DemoState>,
    mut scan: ResMut<ScanApp>,
    mut input: Single<&mut EguiInput, With<PrimaryEguiContext>>,
    mut exit: MessageWriter<AppExit>,
) {
    let input: &mut egui::RawInput = &mut input.0;
    input.events.clear();
    input.hovered_files.clear();
    input.dropped_files.clear();

    let state = &mut *state;
    let scan = &mut *scan;
    let mut events: Vec<egui::Event> = Vec::new();
    // The viewport rectangle is known after the first egui pass.
    let have_layout = scan.frame.viewport_rect.width() > 0.0;

    if state.drain.is_none() && have_layout {
        state.recording = state.video.is_some();
        // Instant steps chain within one frame; a timed step consumes it.
        while let Some(step) = state.steps.get(state.index) {
            let f = state.step_frame;
            let mut done = false;
            match step {
                Step::Do(func) => {
                    func(scan);
                    done = true;
                }
                Step::Caption { title, text, big } => {
                    if let Some(d) = scan.demo.as_mut() {
                        d.caption = Some(Caption {
                            title: (*title).to_string(),
                            text: (*text).to_string(),
                            since: state.frame,
                            big: *big,
                        });
                    }
                    done = true;
                }
                Step::ClearCaption => {
                    if let Some(d) = scan.demo.as_mut() {
                        d.caption = None;
                    }
                    done = true;
                }
                Step::Wait(n) => {
                    if f + 1 >= *n {
                        done = true;
                    }
                }
                Step::WaitUntil {
                    done: pred,
                    timeout,
                } => {
                    if pred(scan) || f >= *timeout {
                        done = true;
                    }
                }
                Step::MoveTo { to, frames } => {
                    if f == 0 {
                        state.drag_start = state.cursor;
                        state.drag_target = viewport_pos(scan, *to);
                    }
                    let t = ease((f + 1) as f32 / (*frames).max(1) as f32);
                    state.cursor = state.drag_start.lerp(state.drag_target, t);
                    if f + 1 >= *frames {
                        done = true;
                    }
                }
                Step::Drag {
                    button,
                    to,
                    frames,
                    modifiers,
                } => {
                    if f == 0 {
                        state.drag_start = state.cursor;
                        state.drag_target = viewport_pos(scan, *to);
                        state.modifiers = *modifiers;
                        state.pressed = Some(*button);
                        events.push(egui::Event::PointerButton {
                            pos: state.cursor,
                            button: *button,
                            pressed: true,
                            modifiers: *modifiers,
                        });
                    } else {
                        let t = ease(f as f32 / (*frames).max(1) as f32);
                        state.cursor = state.drag_start.lerp(state.drag_target, t);
                        if f >= *frames {
                            events.push(egui::Event::PointerMoved(state.cursor));
                            events.push(egui::Event::PointerButton {
                                pos: state.cursor,
                                button: *button,
                                pressed: false,
                                modifiers: *modifiers,
                            });
                            state.pressed = None;
                            state.modifiers = egui::Modifiers::NONE;
                            done = true;
                        }
                    }
                }
                Step::Wheel { delta, frames } => {
                    let per_frame = *delta / (*frames).max(1) as f32;
                    if per_frame != 0.0 {
                        events.push(egui::Event::MouseWheel {
                            unit: egui::MouseWheelUnit::Point,
                            delta: egui::vec2(0.0, per_frame),
                            phase: egui::TouchPhase::Move,
                            modifiers: state.modifiers,
                        });
                    }
                    if f + 1 >= *frames {
                        done = true;
                    }
                }
                Step::Key(k, modifiers) => {
                    events.push(egui::Event::Key {
                        key: *k,
                        physical_key: None,
                        pressed: f == 0,
                        repeat: false,
                        modifiers: *modifiers,
                    });
                    if f == 1 {
                        done = true;
                    }
                }
                Step::Shot(name) => {
                    if f == 0 {
                        state.shot = Some(name);
                    }
                    // Give the readback a few frames before the next change.
                    if f >= 3 {
                        done = true;
                    }
                }
                Step::Finish => {
                    state.recording = false;
                    state.drain = Some(secs(0.5));
                    done = true;
                }
            }
            let instant = matches!(
                step,
                Step::Do(_) | Step::Caption { .. } | Step::ClearCaption | Step::Finish
            );
            if done {
                state.index += 1;
                state.step_frame = 0;
            } else {
                state.step_frame += 1;
            }
            if !instant {
                break;
            }
        }
    }

    if std::env::var_os("DEMO_TRACE").is_some() && state.frame < 400 {
        eprintln!(
            "TRACE f={} step={} dist={:.3} target={:?} orient={:?} cursor={:?} ev={}",
            state.frame,
            state.index,
            scan.camera.distance,
            scan.camera.target,
            scan.camera.orient,
            state.cursor,
            events.len()
        );
    }
    // Pointer position first so button events see the right hover state.
    if have_layout {
        events.insert(0, egui::Event::PointerMoved(state.cursor));
    }
    input
        .events
        .push(egui::Event::ModifiersChanged(state.modifiers));
    input.events.extend(events);

    if let Some(d) = scan.demo.as_mut() {
        d.cursor = (have_layout && state.video.is_some()).then_some(state.cursor);
        d.pressed = state.pressed.is_some();
        d.frame = state.frame;
    }
    state.frame += 1;

    if let Some(left) = state.drain {
        if left == 0 {
            let ok = match &state.video {
                Some(path) => {
                    let ok = state.sink.lock().unwrap().finish();
                    if ok {
                        info!("demo video written to {}", path.display());
                    }
                    ok
                }
                None => {
                    info!("screenshots written to {}", state.stills_dir.display());
                    true
                }
            };
            exit.write(if ok {
                AppExit::Success
            } else {
                AppExit::error()
            });
            state.drain = None;
            state.index = state.steps.len();
        } else {
            state.drain = Some(left - 1);
        }
    }
}

/// Requests a screenshot of the frame being rendered and forwards it to the
/// encoder when the readback completes.
fn capture_frame(mut commands: Commands, mut state: ResMut<DemoState>) {
    if let Some(name) = state.shot.take() {
        let path = state.stills_dir.join(format!("{name}.png"));
        commands
            .spawn(Screenshot::primary_window())
            .observe(bevy::render::view::screenshot::save_to_disk(path));
    }
    if !state.recording {
        return;
    }
    let index = state.captured;
    state.captured += 1;
    let sink = state.sink.clone();
    commands.spawn(Screenshot::primary_window()).observe(
        move |captured: On<ScreenshotCaptured>| {
            let image = captured.image.clone();
            match image.try_into_dynamic() {
                Ok(img) => {
                    let rgb = img.to_rgb8();
                    let (w, h) = rgb.dimensions();
                    sink.lock().unwrap().push(index, w, h, rgb.into_raw());
                }
                Err(e) => error!("cannot convert captured frame: {e}"),
            }
        },
    );
}
