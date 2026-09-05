mod app;
mod camera;
mod decimate;
mod geom;
mod io;
mod mesh;
mod pick;
mod render;
mod rng;
mod tests;
mod ui;
mod worker;

fn main() -> Result<(), eframe::Error> {
    use eframe::egui;
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1500.0, 950.0])
            .with_min_inner_size([1000.0, 700.0])
            .with_title("ScanImprover"),
        ..Default::default()
    };
    eframe::run_native(
        "ScanImprover",
        options,
        Box::new(|_cc| Ok(Box::new(app::App::new()))),
    )
}
