//! STEP (ISO 10303-21, AP214) export of a reconstructed solid: one
//! MANIFOLD_SOLID_BREP whose CLOSED_SHELL holds an ADVANCED_FACE per face
//! group, on PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE /
//! SPHERICAL_SURFACE / B_SPLINE_SURFACE_WITH_KNOTS geometry, bounded by
//! EDGE_CURVEs on LINE / CIRCLE / ELLIPSE / B_SPLINE_CURVE_WITH_KNOTS.

use super::step::{knot_summary, write_int_list, write_real_list};
use crate::geom::solid::{Curve, Solid, Surface};
use glam::DVec3;
use std::fmt::Write;

/// Entity writer with running instance ids.
struct StepWriter {
    out: String,
    id: usize,
}

/// STEP real literal (always with a decimal point).
fn real(v: f64) -> String {
    let v = if v == 0.0 || !v.is_finite() { 0.0 } else { v };
    format!("{v:.10}")
}

/// A unit vector perpendicular to `z`.
fn perp(z: DVec3) -> DVec3 {
    let up = if z.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
    z.cross(up).normalize()
}

impl StepWriter {
    fn add(&mut self, body: &str) -> usize {
        let id = self.id;
        writeln!(self.out, "#{id} = {body};").unwrap();
        self.id += 1;
        id
    }

    fn point(&mut self, p: DVec3) -> usize {
        self.add(&format!(
            "CARTESIAN_POINT('',({},{},{}))",
            real(p.x),
            real(p.y),
            real(p.z)
        ))
    }

    fn direction(&mut self, d: DVec3) -> usize {
        let d = d.normalize();
        self.add(&format!(
            "DIRECTION('',({},{},{}))",
            real(d.x),
            real(d.y),
            real(d.z)
        ))
    }

    fn placement(&mut self, origin: DVec3, z: DVec3, x: DVec3) -> usize {
        let o = self.point(origin);
        let dz = self.direction(z);
        let dx = self.direction(x);
        self.add(&format!("AXIS2_PLACEMENT_3D('',#{o},#{dz},#{dx})"))
    }

    fn id_list(ids: &[usize]) -> String {
        let s: Vec<String> = ids.iter().map(|i| format!("#{i}")).collect();
        format!("({})", s.join(","))
    }

    fn curve(&mut self, c: &Curve, closed: bool) -> usize {
        match c {
            Curve::Line { origin, dir } => {
                let p = self.point(*origin);
                let d = self.direction(*dir);
                let v = self.add(&format!("VECTOR('',#{d},1.)"));
                self.add(&format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle {
                center,
                axis,
                xdir,
                radius,
            } => {
                let pl = self.placement(*center, *axis, *xdir);
                self.add(&format!("CIRCLE('',#{pl},{})", real(*radius)))
            }
            Curve::Ellipse {
                center,
                axis,
                xdir,
                major,
                minor,
            } => {
                let pl = self.placement(*center, *axis, *xdir);
                self.add(&format!(
                    "ELLIPSE('',#{pl},{},{})",
                    real(*major),
                    real(*minor)
                ))
            }
            Curve::Spline { degree, cps, knots } => {
                let ids: Vec<usize> = cps.iter().map(|p| self.point(*p)).collect();
                let (k, m) = knot_summary(knots);
                let mut body = format!(
                    "B_SPLINE_CURVE_WITH_KNOTS('',{degree},{},.UNSPECIFIED.,{},.F.,",
                    Self::id_list(&ids),
                    if closed { ".T." } else { ".F." }
                );
                write_int_list(&mut body, &m);
                body.push(',');
                write_real_list(&mut body, &k);
                body.push_str(",.UNSPECIFIED.)");
                self.add(&body)
            }
        }
    }

    fn surface(&mut self, s: &Surface) -> usize {
        match s {
            Surface::Plane { origin, normal } => {
                let pl = self.placement(*origin, *normal, perp(*normal));
                self.add(&format!("PLANE('',#{pl})"))
            }
            Surface::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let pl = self.placement(*origin, *axis, perp(*axis));
                self.add(&format!("CYLINDRICAL_SURFACE('',#{pl},{})", real(*radius)))
            }
            Surface::Cone {
                origin,
                axis,
                radius,
                half_angle,
            } => {
                let pl = self.placement(*origin, *axis, perp(*axis));
                self.add(&format!(
                    "CONICAL_SURFACE('',#{pl},{},{})",
                    real(*radius),
                    real(*half_angle)
                ))
            }
            Surface::Sphere { center, radius } => {
                let pl = self.placement(*center, DVec3::Z, DVec3::X);
                self.add(&format!("SPHERICAL_SURFACE('',#{pl},{})", real(*radius)))
            }
            Surface::Spline(sp) => {
                let net = &sp.net;
                // Control points: outer index = u, inner = v.
                let mut rows = Vec::with_capacity(net.nx + 1);
                for i in 0..=net.nx {
                    let ids: Vec<usize> = (0..=net.ny)
                        .map(|j| self.point(net.cps[j][i].as_dvec3()))
                        .collect();
                    rows.push(Self::id_list(&ids));
                }
                let (uk, um) = knot_summary(&net.u_knots_full);
                let (vk, vm) = knot_summary(&net.v_knots_full);
                let mut body = format!(
                    "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({}),.UNSPECIFIED.,.F.,.F.,.F.,",
                    net.deg_u,
                    net.deg_v,
                    rows.join(",")
                );
                write_int_list(&mut body, &um);
                body.push(',');
                write_int_list(&mut body, &vm);
                body.push(',');
                write_real_list(&mut body, &uk);
                body.push(',');
                write_real_list(&mut body, &vk);
                body.push_str(",.UNSPECIFIED.)");
                self.add(&body)
            }
        }
    }
}

/// Writes the solid as a STEP file (millimetres).
pub fn generate_solid_step(name: &str, solid: &Solid) -> String {
    let name = name.replace('\'', "''");
    let mut w = StepWriter {
        out: String::with_capacity(64 * 1024),
        id: 1,
    };
    w.out.push_str("ISO-10303-21;\nHEADER;\n");
    w.out
        .push_str("FILE_DESCRIPTION(('ScanImprover Reconstructed Solid'),'2;1');\n");
    writeln!(
        w.out,
        "FILE_NAME('{name}.step','2026-09-11T12:00:00',('ScanImprover'),('ScanImprover'),'ScanImprover','ScanImprover','');"
    )
    .unwrap();
    w.out.push_str(
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n",
    );

    // Product structure and representation context (same layout as the
    // reference geometry export).
    let ctx = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.add(&format!(
        "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ctx})"
    ));
    let pctx = w.add(&format!("PRODUCT_CONTEXT('',#{ctx},'mechanical')"));
    let prod = w.add(&format!("PRODUCT('{name}','{name}','',(#{pctx}))"));
    let pdf = w.add(&format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"));
    let dctx = w.add(&format!(
        "PRODUCT_DEFINITION_CONTEXT('part definition',#{ctx},'design')"
    ));
    let pd = w.add(&format!("PRODUCT_DEFINITION('design','',#{pdf},#{dctx})"));
    let pds = w.add(&format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));
    let len_unit = w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
    let ang_unit = w.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
    let sol_unit = w.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
    // Model uncertainty: the reconstruction's own geometric gaps.
    let gap = solid.report.vertex_gap.max(solid.report.edge_gap);
    let confusion = (2.0 * gap).clamp(1e-5, 0.1);
    let unc = w.id + 1;
    let geo_ctx = w.add(&format!(
        "( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) \
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#{len_unit},#{ang_unit},#{sol_unit})) \
         REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"
    ));
    w.add(&format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{len_unit},'distance_accuracy_value','confusion accuracy')",
        real(confusion)
    ));
    let world = w.placement(DVec3::ZERO, DVec3::Z, DVec3::X);

    // Topology: vertices, edges, faces.
    let vids: Vec<usize> = solid
        .vertices
        .iter()
        .map(|p| {
            let cp = w.point(*p);
            w.add(&format!("VERTEX_POINT('',#{cp})"))
        })
        .collect();
    let eids: Vec<usize> = solid
        .edges
        .iter()
        .map(|e| {
            let c = w.curve(&e.curve, e.closed);
            w.add(&format!(
                "EDGE_CURVE('',#{},#{},#{c},.T.)",
                vids[e.start], vids[e.end]
            ))
        })
        .collect();
    let mut face_ids = Vec::with_capacity(solid.faces.len());
    for face in &solid.faces {
        let surf = w.surface(&face.surface);
        let mut bounds = Vec::with_capacity(face.loops.len());
        for lp in &face.loops {
            let oes: Vec<usize> = lp
                .edges
                .iter()
                .map(|&(e, fwd)| {
                    w.add(&format!(
                        "ORIENTED_EDGE('',*,*,#{},{})",
                        eids[e],
                        if fwd { ".T." } else { ".F." }
                    ))
                })
                .collect();
            let l = w.add(&format!("EDGE_LOOP('',{})", StepWriter::id_list(&oes)));
            let kind = if lp.outer {
                "FACE_OUTER_BOUND"
            } else {
                "FACE_BOUND"
            };
            bounds.push(w.add(&format!("{kind}('',#{l},.T.)")));
        }
        face_ids.push(w.add(&format!(
            "ADVANCED_FACE('Group {} {}',{},#{surf},{})",
            face.group + 1,
            face.surface.label(),
            StepWriter::id_list(&bounds),
            if face.same_sense { ".T." } else { ".F." }
        )));
    }
    let shell = w.add(&format!(
        "CLOSED_SHELL('',{})",
        StepWriter::id_list(&face_ids)
    ));
    let brep = w.add(&format!("MANIFOLD_SOLID_BREP('{name}',#{shell})"));
    let rep = w.add(&format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('{name}',(#{world},#{brep}),#{geo_ctx})"
    ));
    w.add(&format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));
    w.out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    w.out
}
