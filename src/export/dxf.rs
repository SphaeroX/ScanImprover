use crate::geom::fitting::{FittedCircle, FittedPlane, plane_basis};
use std::fmt::Write;

/// Generates an AutoCAD 3D-DXF file (R12/2000 compatible) containing
/// planar boundaries (3DFACE and LINEs) and 3D circles.
pub fn generate_dxf(
    planes: &[&FittedPlane],
    circles: &[&FittedCircle],
    default_plane_size: f32,
) -> String {
    let mut out = String::with_capacity(8192);
    let plane_half = (default_plane_size * 0.5).max(10.0);

    // DXF Header
    out.push_str("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n");

    // DXF Entities Section
    out.push_str("0\nSECTION\n2\nENTITIES\n");

    // Planes
    for p in planes {
        let n = p.fit.normal.normalize();
        let (u, v) = plane_basis(n);
        let c = p.fit.point;

        let p1 = c - u * plane_half - v * plane_half;
        let p2 = c + u * plane_half - v * plane_half;
        let p3 = c + u * plane_half + v * plane_half;
        let p4 = c - u * plane_half + v * plane_half;

        // 3DFACE for surface snapping
        write!(
            out,
            "0\n3DFACE\n8\nPLANES\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}\n11\n{:.6}\n21\n{:.6}\n31\n{:.6}\n12\n{:.6}\n22\n{:.6}\n32\n{:.6}\n13\n{:.6}\n23\n{:.6}\n33\n{:.6}\n",
            p1.x, p1.y, p1.z,
            p2.x, p2.y, p2.z,
            p3.x, p3.y, p3.z,
            p4.x, p4.y, p4.z,
        ).unwrap();

        // 4 boundary lines
        let edges = [(p1, p2), (p2, p3), (p3, p4), (p4, p1)];
        for (a, b) in edges {
            write!(
                out,
                "0\nLINE\n8\nPLANES\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}\n11\n{:.6}\n21\n{:.6}\n31\n{:.6}\n",
                a.x, a.y, a.z, b.x, b.y, b.z
            ).unwrap();
        }
    }

    // Circles
    for c in circles {
        let n = c.fit.normal.normalize();
        let (u, v) = plane_basis(n);
        let center = c.fit.center;
        let r = c.fit.radius.max(0.01);

        // Center point entity
        write!(
            out,
            "0\nPOINT\n8\nCIRCLES\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}\n",
            center.x, center.y, center.z
        )
        .unwrap();

        // Standard DXF CIRCLE entity with extrusion direction
        write!(
            out,
            "0\nCIRCLE\n8\nCIRCLES\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}\n40\n{:.6}\n210\n{:.6}\n220\n{:.6}\n230\n{:.6}\n",
            center.x, center.y, center.z, r, n.x, n.y, n.z
        ).unwrap();

        // Also output segmented 3D lines (64 segments) for CAD viewers that don't support 3D CIRCLE extrusions
        let segs = 64;
        for i in 0..segs {
            let a0 = 2.0 * std::f32::consts::PI * (i as f32 / segs as f32);
            let a1 = 2.0 * std::f32::consts::PI * ((i + 1) as f32 / segs as f32);
            let pt0 = center + u * (r * a0.cos()) + v * (r * a0.sin());
            let pt1 = center + u * (r * a1.cos()) + v * (r * a1.sin());
            write!(
                out,
                "0\nLINE\n8\nCIRCLES\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}\n11\n{:.6}\n21\n{:.6}\n31\n{:.6}\n",
                pt0.x, pt0.y, pt0.z, pt1.x, pt1.y, pt1.z
            ).unwrap();
        }
    }

    out.push_str("0\nENDSEC\n0\nEOF\n");
    out
}
