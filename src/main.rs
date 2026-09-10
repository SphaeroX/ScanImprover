// Index loops over small fixed-size matrices read clearer than iterator
// chains in the numeric code.
#![allow(clippy::needless_range_loop)]

mod app;
mod camera;
mod decimate;
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
    let mut scan_app = ScanApp::new();
    if let Some(arg) = args.first() {
        let path = std::path::PathBuf::from(arg);
        if path.exists() {
            scan_app.load_file(path);
        }
    }

    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "ScanImprover".to_string(),
                    resolution: bevy::window::WindowResolution::new(1500, 950),
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
    if let Some(path) = screenshot {
        app.insert_resource(ScreenshotRequest { path, frames: 0 })
            .add_systems(Update, screenshot_system);
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

fn save_settings_on_exit(mut exit: MessageReader<AppExit>, scan: Res<ScanApp>) {
    if exit.read().next().is_some() {
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
