use crate::geom::fitting::{FittedCircle, FittedPlane, plane_basis};
use std::fmt::Write;

/// Generates a valid ISO 10303-21 (STEP AP214) file containing planar B-Rep surface
/// bodies for the given planes and circular planar faces for the given circles.
pub fn generate_step(
    planes: &[&FittedPlane],
    circles: &[&FittedCircle],
    default_plane_size: f32,
) -> String {
    let mut out = String::with_capacity(8192);
    let plane_half = (default_plane_size * 0.5).max(10.0);

    // Header
    out.push_str("ISO-10303-21;\n");
    out.push_str("HEADER;\n");
    out.push_str("FILE_DESCRIPTION(('ScanImprover Reference Geometry'),'2;1');\n");
    out.push_str("FILE_NAME('reference_geometry.step','2026-09-06T20:00:00',('ScanImprover'),('ScanImprover'),'ScanImprover','ScanImprover','');\n");
    out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n");
    out.push_str("ENDSEC;\n");
    out.push_str("DATA;\n");

    let mut id: usize = 1;

    // Standard structural context entities
    let ctx_id = id;
    writeln!(
        out,
        "#{id} = APPLICATION_CONTEXT('core data for automotive mechanical design processes');"
    )
    .unwrap();
    id += 1;

    let _apd_id = id;
    writeln!(
        out,
        "#{id} = APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ctx_id});"
    )
    .unwrap();
    id += 1;

    let pctx_id = id;
    writeln!(out, "#{id} = PRODUCT_CONTEXT('',#{ctx_id},'mechanical');").unwrap();
    id += 1;

    let prod_id = id;
    writeln!(
        out,
        "#{id} = PRODUCT('Reference Geometry','Reference Geometry','',((#{pctx_id})));"
    )
    .unwrap();
    id += 1;

    let pdf_id = id;
    writeln!(
        out,
        "#{id} = PRODUCT_DEFINITION_FORMATION('','',#{prod_id});"
    )
    .unwrap();
    id += 1;

    let pd_id = id;
    writeln!(
        out,
        "#{id} = PRODUCT_DEFINITION('design','',#{pdf_id},#{pctx_id});"
    )
    .unwrap();
    id += 1;

    let pds_id = id;
    writeln!(out, "#{id} = PRODUCT_DEFINITION_SHAPE('','',#{pd_id});").unwrap();
    id += 1;

    // Unit definition: Millimeter & Degree
    let len_unit_id = id;
    writeln!(
        out,
        "#{id} = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );"
    )
    .unwrap();
    id += 1;

    let ang_unit_id = id;
    writeln!(
        out,
        "#{id} = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );"
    )
    .unwrap();
    id += 1;

    let solid_ang_id = id;
    writeln!(
        out,
        "#{id} = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );"
    )
    .unwrap();
    id += 1;

    let unctx_id = id;
    writeln!(
        out,
        "#{id} = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{id_unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{len_unit_id},#{ang_unit_id},#{solid_ang_id})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') );",
        id_unc = id + 1
    )
    .unwrap();
    id += 1;

    let _unc_id = id;
    writeln!(
        out,
        "#{id} = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#{len_unit_id},'distance_accuracy_value','confusion accuracy');"
    )
    .unwrap();
    id += 1;

    // Origin point and axes
    let origin_id = id;
    writeln!(out, "#{id} = CARTESIAN_POINT('',(0.,0.,0.));").unwrap();
    id += 1;

    let dir_z_id = id;
    writeln!(out, "#{id} = DIRECTION('',(0.,0.,1.));").unwrap();
    id += 1;

    let dir_x_id = id;
    writeln!(out, "#{id} = DIRECTION('',(1.,0.,0.));").unwrap();
    id += 1;

    let world_axis_id = id;
    writeln!(
        out,
        "#{id} = AXIS2_PLACEMENT_3D('',#{origin_id},#{dir_z_id},#{dir_x_id});"
    )
    .unwrap();
    id += 1;

    let mut face_ids = Vec::new();

    // Export each Plane as an ADVANCED_FACE on a PLANE surface
    for p in planes {
        let n = p.fit.normal.normalize();
        let (u, v) = plane_basis(n);
        let c = p.fit.point;

        // 4 rectangle corner points
        let p1 = c - u * plane_half - v * plane_half;
        let p2 = c + u * plane_half - v * plane_half;
        let p3 = c + u * plane_half + v * plane_half;
        let p4 = c - u * plane_half + v * plane_half;

        let pt_c_id = id;
        writeln!(
            out,
            "#{id} = CARTESIAN_POINT('',({:.6},{:.6},{:.6}));",
            c.x, c.y, c.z
        )
        .unwrap();
        id += 1;

        let dir_n_id = id;
        writeln!(
            out,
            "#{id} = DIRECTION('',({:.6},{:.6},{:.6}));",
            n.x, n.y, n.z
        )
        .unwrap();
        id += 1;

        let dir_u_id = id;
        writeln!(
            out,
            "#{id} = DIRECTION('',({:.6},{:.6},{:.6}));",
            u.x, u.y, u.z
        )
        .unwrap();
        id += 1;

        let plane_axis_id = id;
        writeln!(
            out,
            "#{id} = AXIS2_PLACEMENT_3D('',#{pt_c_id},#{dir_n_id},#{dir_u_id});"
        )
        .unwrap();
        id += 1;

        let plane_surf_id = id;
        writeln!(out, "#{id} = PLANE('',#{plane_axis_id});").unwrap();
        id += 1;

        // Vertices
        let pts = [p1, p2, p3, p4];
        let mut vert_ids = [0; 4];
        for (i, pt) in pts.iter().enumerate() {
            let cp_id = id;
            writeln!(
                out,
                "#{id} = CARTESIAN_POINT('',({:.6},{:.6},{:.6}));",
                pt.x, pt.y, pt.z
            )
            .unwrap();
            id += 1;

            vert_ids[i] = id;
            writeln!(out, "#{id} = VERTEX_POINT('',#{cp_id});").unwrap();
            id += 1;
        }

        // Edges connecting: 0->1, 1->2, 2->3, 3->0
        let mut oriented_edge_ids = Vec::new();
        let edge_defs = [(0, 1, u), (1, 2, v), (2, 3, -u), (3, 0, -v)];

        for (v_start, v_end, edge_dir) in edge_defs {
            let edir_id = id;
            writeln!(
                out,
                "#{id} = DIRECTION('',({:.6},{:.6},{:.6}));",
                edge_dir.x, edge_dir.y, edge_dir.z
            )
            .unwrap();
            id += 1;

            let vec_id = id;
            writeln!(
                out,
                "#{id} = VECTOR('',#{edir_id},{:.6});",
                plane_half * 2.0
            )
            .unwrap();
            id += 1;

            let line_id = id;
            let start_cp_id = vert_ids[v_start] - 1;
            writeln!(out, "#{id} = LINE('',#{start_cp_id},#{vec_id});").unwrap();
            id += 1;

            let ec_id = id;
            writeln!(
                out,
                "#{id} = EDGE_CURVE('',#{v_s},#{v_e},#{line_id},.T.);",
                v_s = vert_ids[v_start],
                v_e = vert_ids[v_end]
            )
            .unwrap();
            id += 1;

            let oe_id = id;
            writeln!(out, "#{id} = ORIENTED_EDGE('',*,*,#{ec_id},.T.);").unwrap();
            id += 1;

            oriented_edge_ids.push(oe_id);
        }

        let loop_id = id;
        write!(out, "#{id} = EDGE_LOOP('',(").unwrap();
        for (i, oe) in oriented_edge_ids.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            write!(out, "#{oe}").unwrap();
        }
        out.push_str("));\n");
        id += 1;

        let bound_id = id;
        writeln!(out, "#{id} = FACE_OUTER_BOUND('',#{loop_id},.T.);").unwrap();
        id += 1;

        let face_id = id;
        writeln!(
            out,
            "#{id} = ADVANCED_FACE('{}',((#{bound_id})),#{plane_surf_id},.T.);",
            p.name
        )
        .unwrap();
        id += 1;

        face_ids.push(face_id);
    }

    // Export each Circle as an ADVANCED_FACE (circular planar disk)
    for c in circles {
        let n = c.fit.normal.normalize();
        let (u, _v) = plane_basis(n);
        let center = c.fit.center;
        let r = c.fit.radius.max(0.1);

        let pt_c_id = id;
        writeln!(
            out,
            "#{id} = CARTESIAN_POINT('',({:.6},{:.6},{:.6}));",
            center.x, center.y, center.z
        )
        .unwrap();
        id += 1;

        let dir_n_id = id;
        writeln!(
            out,
            "#{id} = DIRECTION('',({:.6},{:.6},{:.6}));",
            n.x, n.y, n.z
        )
        .unwrap();
        id += 1;

        let dir_u_id = id;
        writeln!(
            out,
            "#{id} = DIRECTION('',({:.6},{:.6},{:.6}));",
            u.x, u.y, u.z
        )
        .unwrap();
        id += 1;

        let circle_axis_id = id;
        writeln!(
            out,
            "#{id} = AXIS2_PLACEMENT_3D('',#{pt_c_id},#{dir_n_id},#{dir_u_id});"
        )
        .unwrap();
        id += 1;

        let plane_surf_id = id;
        writeln!(out, "#{id} = PLANE('',#{circle_axis_id});").unwrap();
        id += 1;

        let geom_circle_id = id;
        writeln!(out, "#{id} = CIRCLE('',#{circle_axis_id},{:.6});", r).unwrap();
        id += 1;

        // Two semicircular vertices: +u*r and -u*r
        let p_pos = center + u * r;
        let p_neg = center - u * r;

        let cp_pos_id = id;
        writeln!(
            out,
            "#{id} = CARTESIAN_POINT('',({:.6},{:.6},{:.6}));",
            p_pos.x, p_pos.y, p_pos.z
        )
        .unwrap();
        id += 1;

        let v_pos_id = id;
        writeln!(out, "#{id} = VERTEX_POINT('',#{cp_pos_id});").unwrap();
        id += 1;

        let cp_neg_id = id;
        writeln!(
            out,
            "#{id} = CARTESIAN_POINT('',({:.6},{:.6},{:.6}));",
            p_neg.x, p_neg.y, p_neg.z
        )
        .unwrap();
        id += 1;

        let v_neg_id = id;
        writeln!(out, "#{id} = VERTEX_POINT('',#{cp_neg_id});").unwrap();
        id += 1;

        // Semi-circle 1: pos -> neg
        let ec1_id = id;
        writeln!(
            out,
            "#{id} = EDGE_CURVE('',#{v_pos_id},#{v_neg_id},#{geom_circle_id},.T.);"
        )
        .unwrap();
        id += 1;

        let oe1_id = id;
        writeln!(out, "#{id} = ORIENTED_EDGE('',*,*,#{ec1_id},.T.);").unwrap();
        id += 1;

        // Semi-circle 2: neg -> pos
        let ec2_id = id;
        writeln!(
            out,
            "#{id} = EDGE_CURVE('',#{v_neg_id},#{v_pos_id},#{geom_circle_id},.T.);"
        )
        .unwrap();
        id += 1;

        let oe2_id = id;
        writeln!(out, "#{id} = ORIENTED_EDGE('',*,*,#{ec2_id},.T.);").unwrap();
        id += 1;

        let loop_id = id;
        writeln!(out, "#{id} = EDGE_LOOP('',(#{oe1_id},#{oe2_id}));").unwrap();
        id += 1;

        let bound_id = id;
        writeln!(out, "#{id} = FACE_OUTER_BOUND('',#{loop_id},.T.);").unwrap();
        id += 1;

        let face_id = id;
        writeln!(
            out,
            "#{id} = ADVANCED_FACE('{}',((#{bound_id})),#{plane_surf_id},.T.);",
            c.name
        )
        .unwrap();
        id += 1;

        face_ids.push(face_id);
    }

    if face_ids.is_empty() {
        return out;
    }

    // Wrap faces in an OPEN_SHELL
    let shell_id = id;
    write!(out, "#{id} = OPEN_SHELL('',(").unwrap();
    for (i, f) in face_ids.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        write!(out, "#{f}").unwrap();
    }
    out.push_str("));\n");
    id += 1;

    let sbsm_id = id;
    writeln!(
        out,
        "#{id} = SHELL_BASED_SURFACE_MODEL('Reference Surfaces',((#{shell_id})));"
    )
    .unwrap();
    id += 1;

    let rep_id = id;
    writeln!(
        out,
        "#{id} = MANIFOLD_SURFACE_SHAPE_REPRESENTATION('Reference Geometry',((#{sbsm_id},#{world_axis_id})),#{unctx_id});"
    )
    .unwrap();
    id += 1;

    writeln!(
        out,
        "#{id} = SHAPE_DEFINITION_REPRESENTATION(#{pds_id},#{rep_id});"
    )
    .unwrap();

    out.push_str("ENDSEC;\n");
    out.push_str("END-ISO-10303-21;\n");

    out
}
