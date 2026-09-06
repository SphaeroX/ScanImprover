use crate::geom::fitting::{FittedCircle, FittedPlane};
use std::fmt::Write;

/// Generates a Python script (.py) for Autodesk Fusion 360 that creates native
/// Construction Planes and Sketch Circles directly in the active design.
pub fn generate_fusion_script(planes: &[&FittedPlane], circles: &[&FittedCircle]) -> String {
    let mut out = String::with_capacity(4096);

    out.push_str(
        r#"# Author: ScanImprover
# Description: Automatically creates Construction Planes and Sketch Circles matching your 3D scan.
# Usage in Fusion 360:
# 1. Open or insert your scan in Fusion 360.
# 2. Press Shift + S (or go to Utilities > Scripts and Add-Ins).
# 3. Choose this script and click 'Run'.

import adsk.core
import adsk.fusion
import traceback

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            if ui:
                ui.messageBox('No active Fusion 360 design found.\nPlease open a document first.')
            return

        root_comp = design.rootComponent
        ctor_planes = root_comp.constructionPlanes
        sketches = root_comp.sketches

        created_planes = 0
        created_circles = 0

"#,
    );

    // Planes definition
    for p in planes {
        let n = p.fit.normal.normalize();
        let c = p.fit.point;
        // Fusion 360 internal length unit is CENTIMETERS (cm), so divide mm by 10.0!
        let ox = c.x / 10.0;
        let oy = c.y / 10.0;
        let oz = c.z / 10.0;

        writeln!(
            out,
            r#"        # --- Plane: {name} ---
        origin_pt = adsk.core.Point3D.create({ox:.6}, {oy:.6}, {oz:.6})
        normal_vec = adsk.core.Vector3D.create({nx:.6}, {ny:.6}, {nz:.6})
        normal_vec.normalize()
        geom_plane = adsk.core.Plane.create(origin_pt, normal_vec)
        plane_input = ctor_planes.createInput()
        plane_input.setByPlane(geom_plane)
        plane_feat = ctor_planes.add(plane_input)
        plane_feat.name = "{name}"
        created_planes += 1
"#,
            name = p.name,
            ox = ox,
            oy = oy,
            oz = oz,
            nx = n.x,
            ny = n.y,
            nz = n.z
        )
        .unwrap();
    }

    // Circles definition
    for c in circles {
        let n = c.fit.normal.normalize();
        let center = c.fit.center;
        let r = c.fit.radius.max(0.01);
        let ox = center.x / 10.0;
        let oy = center.y / 10.0;
        let oz = center.z / 10.0;
        let radius_cm = r / 10.0;

        writeln!(
            out,
            r#"        # --- Circle: {name} ---
        circle_origin = adsk.core.Point3D.create({ox:.6}, {oy:.6}, {oz:.6})
        circle_normal = adsk.core.Vector3D.create({nx:.6}, {ny:.6}, {nz:.6})
        circle_normal.normalize()
        circle_plane = adsk.core.Plane.create(circle_origin, circle_normal)
        circle_plane_input = ctor_planes.createInput()
        circle_plane_input.setByPlane(circle_plane)
        circle_plane_feat = ctor_planes.add(circle_plane_input)
        circle_plane_feat.name = "{name} Plane"

        sketch = sketches.add(circle_plane_feat)
        sketch.name = "{name}"
        center_model = adsk.core.Point3D.create({ox:.6}, {oy:.6}, {oz:.6})
        center_sketch = sketch.modelToSketchSpace(center_model)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(center_sketch, {radius_cm:.6})
        sketch.sketchPoints.add(center_sketch)
        created_circles += 1
"#,
            name = c.name,
            ox = ox,
            oy = oy,
            oz = oz,
            nx = n.x,
            ny = n.y,
            nz = n.z,
            radius_cm = radius_cm
        )
        .unwrap();
    }

    out.push_str(
r#"        if ui:
            ui.messageBox(f'ScanImprover reference geometry imported successfully:\n- {created_planes} Construction Plane(s)\n- {created_circles} Sketch Circle(s)')

    except Exception:
        if ui:
            ui.messageBox(f'Error executing ScanImprover script:\n{traceback.format_exc()}')
        else:
            print(f'Error: {traceback.format_exc()}')
"#);

    out
}
