use crate::geom::fitting::{FittedCircle, FittedPlane, plane_basis};
use crate::geom::freeform::BicubicNet;
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
        "#{id} = PRODUCT('Reference Geometry','Reference Geometry','',(#{pctx_id}));"
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
            "#{id} = ADVANCED_FACE('{}',(#{bound_id}),#{plane_surf_id},.T.);",
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
            "#{id} = ADVANCED_FACE('{}',(#{bound_id}),#{plane_surf_id},.T.);",
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
        "#{id} = SHELL_BASED_SURFACE_MODEL('Reference Surfaces',(#{shell_id}));"
    )
    .unwrap();
    id += 1;

    let rep_id = id;
    writeln!(
        out,
        "#{id} = MANIFOLD_SURFACE_SHAPE_REPRESENTATION('Reference Geometry',(#{sbsm_id},#{world_axis_id}),#{unctx_id});"
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

/// Distinct knots + multiplicities of a full (repeating) knot vector.
fn knot_summary(full: &[f64]) -> (Vec<f64>, Vec<u32>) {
    let mut knots: Vec<f64> = Vec::new();
    let mut mult: Vec<u32> = Vec::new();
    for &k in full {
        if knots.last().map(|&l| (k - l).abs() < 1e-12).unwrap_or(false) {
            *mult.last_mut().unwrap() += 1;
        } else {
            knots.push(k);
            mult.push(1);
        }
    }
    (knots, mult)
}

fn write_real_list(out: &mut String, values: &[f64]) {
    out.push('(');
    for (i, v) in values.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        write!(out, "{v:.9}").unwrap();
    }
    out.push(')');
}

fn write_int_list(out: &mut String, values: &[u32]) {
    out.push('(');
    for (i, v) in values.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        write!(out, "{v}").unwrap();
    }
    out.push(')');
}

/// Generates a valid ISO 10303-21 (STEP AP214) file containing the freeform
/// surface as a single clamped bicubic B-spline face (ADVANCED_FACE on a
/// B_SPLINE_SURFACE_WITH_KNOTS). The four boundary edges are exact surface
/// curves (B-splines over the boundary control points), so the patch imports
/// as a real CAD surface in Fusion 360 / any STEP-capable CAD — trim it
/// there against the neighboring geometry.
pub fn generate_freeform_step(name: &str, net: &BicubicNet) -> String {
    let mut out = String::with_capacity(8192);
    let (nx, ny) = (net.nx, net.ny);

    out.push_str("ISO-10303-21;\n");
    out.push_str("HEADER;\n");
    out.push_str("FILE_DESCRIPTION(('ScanImprover Freeform Surface'),'2;1');\n");
    out.push_str("FILE_NAME('freeform_surface.step','2026-09-09T12:00:00',('ScanImprover'),('ScanImprover'),'ScanImprover','ScanImprover','');\n");
    out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n");
    out.push_str("ENDSEC;\n");
    out.push_str("DATA;\n");

    let mut id: usize = 1;

    // Structural context entities (same layout as the plane/circle export).
    let ctx_id = id;
    writeln!(out, "#{id} = APPLICATION_CONTEXT('core data for automotive mechanical design processes');").unwrap();
    id += 1;
    let _apd_id = id;
    writeln!(out, "#{id} = APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ctx_id});").unwrap();
    id += 1;
    let pctx_id = id;
    writeln!(out, "#{id} = PRODUCT_CONTEXT('',#{ctx_id},'mechanical');").unwrap();
    id += 1;
    let prod_id = id;
    writeln!(out, "#{id} = PRODUCT('Freeform Surface','Freeform Surface','',(#{pctx_id}));").unwrap();
    id += 1;
    let pdf_id = id;
    writeln!(out, "#{id} = PRODUCT_DEFINITION_FORMATION('','',#{prod_id});").unwrap();
    id += 1;
    let pd_id = id;
    writeln!(out, "#{id} = PRODUCT_DEFINITION('design','',#{pdf_id},#{pctx_id});").unwrap();
    id += 1;
    let pds_id = id;
    writeln!(out, "#{id} = PRODUCT_DEFINITION_SHAPE('','',#{pd_id});").unwrap();
    id += 1;
    let len_unit_id = id;
    writeln!(out, "#{id} = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );").unwrap();
    id += 1;
    let ang_unit_id = id;
    writeln!(out, "#{id} = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );").unwrap();
    id += 1;
    let solid_ang_id = id;
    writeln!(out, "#{id} = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );").unwrap();
    id += 1;
    let unctx_id = id;
    writeln!(
        out,
        "#{id} = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{id_unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{len_unit_id},#{ang_unit_id},#{solid_ang_id})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') );",
        id_unc = id + 1
    )
    .unwrap();
    id += 1;
    writeln!(out, "#{id} = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#{len_unit_id},'distance_accuracy_value','confusion accuracy');").unwrap();
    id += 1;

    // Origin point and axes for global placement context
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

    // CARTESIAN_POINTs for the control net: cp_id[j][i].
    let mut cp_id = vec![vec![0usize; nx + 1]; ny + 1];
    for j in 0..=ny {
        for i in 0..=nx {
            let p = net.cps[j][i];
            writeln!(
                out,
                "#{id} = CARTESIAN_POINT('',({:.6},{:.6},{:.6}));",
                p.x, p.y, p.z
            )
            .unwrap();
            cp_id[j][i] = id;
            id += 1;
        }
    }

    // B_SPLINE_SURFACE_WITH_KNOTS: control_points_list outer index = u.
    let (u_knots, u_mult) = knot_summary(&net.u_knots_full);
    let (v_knots, v_mult) = knot_summary(&net.v_knots_full);
    let surf_id = id;
    {
        // B_SPLINE_SURFACE_WITH_KNOTS(name, u_deg, v_deg, control_points,
        //   surface_form, u_closed, v_closed, self_intersect,
        //   u_multiplicities, v_multiplicities, u_knots, v_knots, knot_spec).
        // Control points: outer index = u, inner = v.
        out.push_str(&format!(
            "#{id} = B_SPLINE_SURFACE_WITH_KNOTS('',{deg_u},{deg_v},(",
            deg_u = net.deg_u,
            deg_v = net.deg_v
        ));
        for i in 0..=nx {
            if i > 0 {
                out.push(',');
            }
            out.push('(');
            for j in 0..=ny {
                if j > 0 {
                    out.push(',');
                }
                write!(out, "#{}", cp_id[j][i]).unwrap();
            }
            out.push(')');
        }
        out.push_str("),.UNSPECIFIED.,.F.,.F.,.F.,");
        write_int_list(&mut out, &u_mult);
        out.push(',');
        write_int_list(&mut out, &v_mult);
        out.push(',');
        write_real_list(&mut out, &u_knots);
        out.push(',');
        write_real_list(&mut out, &v_knots);
        out.push_str(",.UNSPECIFIED.);\n");
    }
    id += 1;

    // Corner vertices (clamped surface: control net corners = surface corners).
    let mut vert_id = [[0usize; 2]; 2]; // [v-index (0|ny)][u-index (0|nx)]
    for (jj, j) in [0usize, ny].iter().enumerate() {
        for (ii, i) in [0usize, nx].iter().enumerate() {
            let cp = cp_id[*j][*i];
            writeln!(out, "#{id} = VERTEX_POINT('',#{cp});").unwrap();
            vert_id[jj][ii] = id;
            id += 1;
        }
    }
    let (v00, v0n, vn0, vnn) = (
        vert_id[0][0],
        vert_id[0][1],
        vert_id[1][0],
        vert_id[1][1],
    );

    // Four boundary B-spline curves (exact surface curves; reversed control
    // point order flips the traversal direction of a symmetric knot vector).
    let mut edge_ids = [0usize; 4];
    // Edge 1: v = 0, u: 0 -> nx.
    edge_ids[0] = {
        let ec = id;
        write_b_spline_curve(
            &mut out,
            &mut id,
            (0..=nx).map(|i| cp_id[0][i]).collect::<Vec<_>>(),
            net.deg_u,
            &u_knots,
            &u_mult,
        );
        edge_curve(&mut out, &mut id, v00, v0n, ec)
    };
    // Edge 2: u = nx, v: 0 -> ny.
    edge_ids[1] = {
        let ec = id;
        write_b_spline_curve(
            &mut out,
            &mut id,
            (0..=ny).map(|j| cp_id[j][nx]).collect::<Vec<_>>(),
            net.deg_v,
            &v_knots,
            &v_mult,
        );
        edge_curve(&mut out, &mut id, v0n, vnn, ec)
    };
    // Edge 3: v = ny, u: nx -> 0 (reversed).
    edge_ids[2] = {
        let ec = id;
        write_b_spline_curve(
            &mut out,
            &mut id,
            (0..=nx).rev().map(|i| cp_id[ny][i]).collect::<Vec<_>>(),
            net.deg_u,
            &u_knots,
            &u_mult,
        );
        edge_curve(&mut out, &mut id, vnn, vn0, ec)
    };
    // Edge 4: u = 0, v: ny -> 0 (reversed).
    edge_ids[3] = {
        let ec = id;
        write_b_spline_curve(
            &mut out,
            &mut id,
            (0..=ny).rev().map(|j| cp_id[j][0]).collect::<Vec<_>>(),
            net.deg_v,
            &v_knots,
            &v_mult,
        );
        edge_curve(&mut out, &mut id, vn0, v00, ec)
    };

    // Loop, bound, face.
    let loop_id = id;
    write!(out, "#{id} = EDGE_LOOP('',(#{e0},#{e1},#{e2},#{e3}));", e0 = edge_ids[0], e1 = edge_ids[1], e2 = edge_ids[2], e3 = edge_ids[3]).unwrap();
    out.push('\n');
    id += 1;
    let bound_id = id;
    writeln!(out, "#{id} = FACE_OUTER_BOUND('',#{loop_id},.T.);").unwrap();
    id += 1;
    let face_id = id;
    writeln!(
        out,
        "#{id} = ADVANCED_FACE('{}',(#{bound_id}),#{surf_id},.T.);",
        name.replace('\'', "''")
    )
    .unwrap();
    id += 1;

    // Wrap in a shell / surface model.
    let shell_id = id;
    writeln!(out, "#{id} = OPEN_SHELL('',(#{face_id}));").unwrap();
    id += 1;
    let sbsm_id = id;
    writeln!(out, "#{id} = SHELL_BASED_SURFACE_MODEL('Freeform Surface',(#{shell_id}));").unwrap();
    id += 1;
    let rep_id = id;
    writeln!(
        out,
        "#{id} = MANIFOLD_SURFACE_SHAPE_REPRESENTATION('Freeform Surface',(#{sbsm_id},#{world_axis_id}),#{unctx_id});"
    )
    .unwrap();
    id += 1;
    writeln!(out, "#{id} = SHAPE_DEFINITION_REPRESENTATION(#{pds_id},#{rep_id});").unwrap();

    out.push_str("ENDSEC;\n");
    out.push_str("END-ISO-10303-21;\n");

    out
}

/// Writes a B_SPLINE_CURVE_WITH_KNOTS entity; returns its id.
fn write_b_spline_curve(
    out: &mut String,
    id: &mut usize,
    control_points: Vec<usize>,
    deg: usize,
    knots: &[f64],
    mult: &[u32],
) -> usize {
    let curve_id = *id;
    out.push_str(&format!(
        "#{curve_id} = B_SPLINE_CURVE_WITH_KNOTS('',{deg},("
    ));
    for (i, cp) in control_points.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        write!(out, "#{cp}").unwrap();
    }
    out.push_str("),.UNSPECIFIED.,.F.,.F.,");
    write_int_list(out, mult);
    out.push(',');
    write_real_list(out, knots);
    out.push_str(",.UNSPECIFIED.);\n");
    *id += 1;
    curve_id
}

/// Writes an EDGE_CURVE + ORIENTED_EDGE pair; returns the oriented edge id.
fn edge_curve(out: &mut String, id: &mut usize, v1: usize, v2: usize, curve: usize) -> usize {
    let ec_id = *id;
    writeln!(out, "#{ec_id} = EDGE_CURVE('',#{v1},#{v2},#{curve},.T.);").unwrap();
    *id += 1;
    let oe_id = *id;
    writeln!(out, "#{oe_id} = ORIENTED_EDGE('',*,*,#{ec_id},.T.);").unwrap();
    *id += 1;
    oe_id
}
