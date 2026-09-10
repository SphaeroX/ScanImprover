// Index loops over small fixed-size matrices read clearer than iterator
// chains in the numeric code.
#![allow(clippy::needless_range_loop)]

mod app;
mod camera;
mod decimate;
mod demo;
mod export;
mod geom;
mod io;
mod mesh;
mod pick;
mod render;
mod rng;
mod settings;
mod tests;
mod ui;
mod worker;

use bevy::ecs::system::NonSendMarker;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use crate::app::App as ScanApp;

/// Optional `--screenshot <file.png>` mode: renders a few frames, saves the
/// window to disk and exits. Used to verify the viewport without a display
/// capture.
#[derive(Resource)]
struct ScreenshotRequest {
    path: std::path::PathBuf,
    frames: u32,
}

fn main() -> bevy::app::AppExit {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut screenshot = None;
    if let Some(i) = args.iter().position(|a| a == "--screenshot") {
        if i + 1 < args.len() {
            screenshot = Some(std::path::PathBuf::from(args.remove(i + 1)));
        }
        args.remove(i);
    }
    // `--demo <out.mp4> <part> <scan> <large> [extra]` records the scripted
    // demo; `--screenshots <dir> ...` saves README stills instead.
    let mut demo = None;
    if let Some(i) = args
        .iter()
        .position(|a| a == "--demo" || a == "--screenshots")
    {
        let stills = args[i] == "--screenshots";
        let rest: Vec<std::path::PathBuf> = args
            .drain(i..)
            .skip(1)
            .map(std::path::PathBuf::from)
            .collect();
        match rest.as_slice() {
            [out, part, scan, large, extra @ ..] => {
                demo = Some(demo::DemoPlugin {
                    mode: if stills {
                        demo::DemoMode::Stills(out.clone())
                    } else {
                        demo::DemoMode::Video(out.clone())
                    },
                    assets: demo::DemoAssets {
                        part: part.clone(),
                        scan: scan.clone(),
                        large: large.clone(),
                        extra: extra.first().cloned(),
                    },
                });
            }
            _ => {
                eprintln!(
                    "usage: ScanImprover --demo <out.mp4> <part> <scan> <large> [extra]\n       ScanImprover --screenshots <dir> <part> <scan> <large> [extra]"
                );
                return bevy::app::AppExit::error();
            }
        }
    }
    let fullscreen = demo.is_some();
    let mut scan_app = ScanApp::new();
    if demo.is_some() {
        // Deterministic look regardless of the user's saved settings.
        scan_app.apply_settings(&settings::Settings::default());
        scan_app.demo = Some(demo::DemoOverlay::default());
    }
    if let Some(arg) = args.first() {
        let path = std::path::PathBuf::from(arg);
        if path.exists() {
            scan_app.load_file(path);
        }
    }
    // The demo records the whole screen; the normal window starts maximized.
    let window_mode = if fullscreen {
        bevy::window::WindowMode::BorderlessFullscreen(bevy::window::MonitorSelection::Primary)
    } else {
        bevy::window::WindowMode::Windowed
    };

    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "ScanImprover".to_string(),
                    resolution: bevy::window::WindowResolution::new(1500, 950),
                    mode: window_mode,
                    ..default()
                }),
                ..default()
            })
            .set(ImagePlugin::default_nearest()),
    )
    .add_plugins(EguiPlugin::default())
    .add_plugins(render::ScenePlugin)
    .insert_resource(scan_app)
    .add_systems(EguiPrimaryContextPass, ui_system)
    .add_systems(Last, save_settings_on_exit);
    if demo.is_none() {
        app.add_systems(Startup, maximize_window);
        // Render only on input, window events and egui repaint requests
        // (animations, worker results) instead of continuously.
        app.insert_resource(bevy::winit::WinitSettings::desktop_app());
    }
    if let Some(path) = screenshot {
        app.insert_resource(ScreenshotRequest { path, frames: 0 })
            .add_systems(Update, screenshot_system);
    }
    if let Some(plugin) = demo {
        app.add_plugins(plugin);
    }
    app.run()
}

/// Runs the whole egui interface (panels, tool sections, viewport input).
/// Pinned to the main thread because native file dialogs require it.
fn ui_system(
    mut contexts: EguiContexts,
    mut scan: ResMut<ScanApp>,
    _main_thread: NonSendMarker,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    scan.frame(ctx);
    Ok(())
}

/// Starts with the window filling the screen.
fn maximize_window(mut window: Single<&mut Window, With<PrimaryWindow>>) {
    window.set_maximized(true);
}

fn save_settings_on_exit(mut exit: MessageReader<AppExit>, scan: Res<ScanApp>) {
    if exit.read().next().is_some() && scan.demo.is_none() {
        scan.save_settings();
    }
}

fn screenshot_system(
    mut commands: Commands,
    mut request: ResMut<ScreenshotRequest>,
    mut exit: MessageWriter<AppExit>,
    windows: Query<Entity, With<PrimaryWindow>>,
) {
    request.frames += 1;
    if request.frames == 40 {
        if windows.single().is_ok() {
            commands
                .spawn(bevy::render::view::screenshot::Screenshot::primary_window())
                .observe(bevy::render::view::screenshot::save_to_disk(
                    request.path.clone(),
                ));
        }
    } else if request.frames == 70 {
        exit.write(AppExit::Success);
    }
}
