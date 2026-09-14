pub mod dxf;
pub mod fusion_script;
pub mod step;
pub mod step_solid;

use crate::geom::fitting::{FittedCircle, FittedPlane};
use crate::geom::freeform::FittedFreeform;
use std::path::Path;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExportFormat {
    Step,
    FusionScript,
    Dxf,
}

impl ExportFormat {
    pub fn from_path(path: &Path) -> Self {
        match path
            .extension()
            .and_then(|s| s.to_str())
            .map(|s| s.to_ascii_lowercase())
            .as_deref()
        {
            Some("py") => ExportFormat::FusionScript,
            Some("dxf") => ExportFormat::Dxf,
            _ => ExportFormat::Step,
        }
    }
}

/// Prompts the user to save a single fitted plane in STEP, Python (Fusion 360), or DXF format.
pub fn export_plane_dialog(plane: &FittedPlane, default_plane_size: f32) -> Result<String, String> {
    let sanitized_name = sanitize_filename(&plane.name);
    let default_filename = format!("{sanitized_name}.step");

    let file_path = rfd::FileDialog::new()
        .add_filter("STEP CAD Surface (*.step, *.stp)", &["step", "stp"])
        .add_filter("Fusion 360 Python Script (*.py)", &["py"])
        .add_filter("AutoCAD DXF (*.dxf)", &["dxf"])
        .set_file_name(&default_filename)
        .save_file();

    let Some(path) = file_path else {
        return Ok("Export cancelled.".to_string());
    };

    let format = ExportFormat::from_path(&path);
    let content = match format {
        ExportFormat::Step => step::generate_step(&[plane], &[], default_plane_size),
        ExportFormat::FusionScript => fusion_script::generate_fusion_script(&[plane], &[]),
        ExportFormat::Dxf => dxf::generate_dxf(&[plane], &[], default_plane_size),
    };

    std::fs::write(&path, content)
        .map_err(|e| format!("Failed to write {}: {e}", path.display()))?;

    Ok(format!("Exported {} to {}", plane.name, path.display()))
}

/// Prompts the user to save a single fitted circle in STEP, Python (Fusion 360), or DXF format.
pub fn export_circle_dialog(circle: &FittedCircle) -> Result<String, String> {
    let sanitized_name = sanitize_filename(&circle.name);
    let default_filename = format!("{sanitized_name}.step");

    let file_path = rfd::FileDialog::new()
        .add_filter("STEP CAD Surface (*.step, *.stp)", &["step", "stp"])
        .add_filter("Fusion 360 Python Script (*.py)", &["py"])
        .add_filter("AutoCAD DXF (*.dxf)", &["dxf"])
        .set_file_name(&default_filename)
        .save_file();

    let Some(path) = file_path else {
        return Ok("Export cancelled.".to_string());
    };

    let format = ExportFormat::from_path(&path);
    let content = match format {
        ExportFormat::Step => step::generate_step(&[], &[circle], circle.fit.radius * 2.0),
        ExportFormat::FusionScript => fusion_script::generate_fusion_script(&[], &[circle]),
        ExportFormat::Dxf => dxf::generate_dxf(&[], &[circle], circle.fit.radius * 2.0),
    };

    std::fs::write(&path, content)
        .map_err(|e| format!("Failed to write {}: {e}", path.display()))?;

    Ok(format!("Exported {} to {}", circle.name, path.display()))
}

/// Prompts the user to save a fitted freeform surface. STEP (.step/.stp)
/// exports the surface as a real CAD B-spline face (4-sided patch over the
/// overshoot rectangle; trim it in CAD). STL/OBJ/PLY export the fitted mesh.
pub fn export_freeform_dialog(freeform: &FittedFreeform) -> Result<String, String> {
    let Some(surface) = &freeform.surface else {
        return Err("Freeform surface is still being fitted.".to_string());
    };
    let sanitized_name = sanitize_filename(&freeform.name);
    let default_filename = format!("{sanitized_name}.step");

    let file_path = rfd::FileDialog::new()
        .add_filter("STEP CAD Surface (*.step, *.stp)", &["step", "stp"])
        .add_filter("Mesh (*.stl)", &["stl"])
        .add_filter("Mesh (*.obj)", &["obj"])
        .add_filter("Mesh (*.ply)", &["ply"])
        .set_file_name(&default_filename)
        .save_file();

    let Some(path) = file_path else {
        return Ok("Export cancelled.".to_string());
    };

    let is_step = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| matches!(e.to_ascii_lowercase().as_str(), "step" | "stp"))
        .unwrap_or(false);

    if is_step {
        // Rebuild the fit over the full rectangle and convert it to a
        // bicubic B-spline patch. Same source points and parameters as the
        // displayed surface, so the shapes match up to the trimmed corners.
        let grid =
            crate::geom::freeform::fit_freeform_grid(&freeform.source_points, &freeform.params)
                .map_err(|e| format!("Could not rebuild the surface for STEP export: {e}"))?;
        let net = crate::geom::freeform::bicubic_control_net(&grid);
        let content = step::generate_freeform_step(&freeform.name, &net);
        std::fs::write(&path, content)
            .map_err(|e| format!("Failed to write {}: {e}", path.display()))?;
    } else {
        crate::io::save_any(&path, surface)
            .map_err(|e| format!("Failed to write {}: {e}", path.display()))?;
    }

    Ok(format!("Exported {} to {}", freeform.name, path.display()))
}

/// Prompts the user to save all visible planes and circles into a single STEP, Python, or DXF file.
pub fn export_all_references_dialog(
    planes: &[FittedPlane],
    circles: &[FittedCircle],
    default_plane_size: f32,
) -> Result<String, String> {
    let visible_planes: Vec<&FittedPlane> = planes.iter().filter(|p| p.visible).collect();
    let visible_circles: Vec<&FittedCircle> = circles.iter().filter(|c| c.visible).collect();

    if visible_planes.is_empty() && visible_circles.is_empty() {
        return Err("No visible planes or circles to export.".to_string());
    }

    let default_filename = "references.step";

    let file_path = rfd::FileDialog::new()
        .add_filter("STEP CAD Surface (*.step, *.stp)", &["step", "stp"])
        .add_filter("Fusion 360 Python Script (*.py)", &["py"])
        .add_filter("AutoCAD DXF (*.dxf)", &["dxf"])
        .set_file_name(default_filename)
        .save_file();

    let Some(path) = file_path else {
        return Ok("Export cancelled.".to_string());
    };

    let format = ExportFormat::from_path(&path);
    let content = match format {
        ExportFormat::Step => {
            step::generate_step(&visible_planes, &visible_circles, default_plane_size)
        }
        ExportFormat::FusionScript => {
            fusion_script::generate_fusion_script(&visible_planes, &visible_circles)
        }
        ExportFormat::Dxf => {
            dxf::generate_dxf(&visible_planes, &visible_circles, default_plane_size)
        }
    };

    std::fs::write(&path, content)
        .map_err(|e| format!("Failed to write {}: {e}", path.display()))?;

    Ok(format!(
        "Exported {} plane(s) and {} circle(s) to {}",
        visible_planes.len(),
        visible_circles.len(),
        path.display()
    ))
}

/// Prompts the user to save a reconstructed solid as a STEP B-Rep
/// (MANIFOLD_SOLID_BREP).
pub fn export_solid_dialog(
    solid: &crate::geom::solid::Solid,
    name: &str,
) -> Result<String, String> {
    let default_filename = format!("{}_solid.step", sanitize_filename(name));
    let file_path = rfd::FileDialog::new()
        .add_filter("STEP CAD Solid (*.step, *.stp)", &["step", "stp"])
        .set_file_name(&default_filename)
        .save_file();
    let Some(path) = file_path else {
        return Ok("Export cancelled.".to_string());
    };
    let content = step_solid::generate_solid_step(name, solid);
    std::fs::write(&path, content)
        .map_err(|e| format!("Failed to write {}: {e}", path.display()))?;
    Ok(format!(
        "Exported solid ({} faces, {} edges) to {}",
        solid.faces.len(),
        solid.edges.len(),
        path.display()
    ))
}

fn sanitize_filename(name: &str) -> String {
    name.chars()
        .map(|c| {
            if c.is_alphanumeric() || c == '_' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect()
}
