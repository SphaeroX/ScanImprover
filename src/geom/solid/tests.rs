//! End-to-end tests on synthetic closed meshes: segmentation with the
//! application's default settings, reconstruction, validation and STEP text.

use super::*;
use crate::export::step_solid::generate_solid_step;
use crate::geom::segment::segment_faces;
use crate::geom::topology::MeshTopology;
use crate::rng::Rng;
use glam::{Quat, Vec3};
use std::collections::HashMap;
use std::f32::consts::TAU;

/// Face groups with the application's default detection settings.
fn groups_of(mesh: &Mesh) -> (Vec<FaceGroup>, Vec<i32>) {
    segment_faces(mesh, &MeshTopology::build(mesh), 45.0, 2, 0.0015, 0.04)
}

fn reconstruct(mesh: &Mesh) -> Result<Solid, String> {
    let (groups, ids) = groups_of(mesh);
    reconstruct_solid(mesh, &groups, &ids, &SolidParams::default())
}

fn count(step: &str, entity: &str) -> usize {
    step.matches(&format!("= {entity}(")).count()
}

/// Axis-aligned box from `min` with `size`, each side split into an
/// `n`×`n` grid; `transform` is applied to every welded vertex.
fn grid_box(min: Vec3, size: Vec3, n: u32, mut transform: impl FnMut(Vec3) -> Vec3) -> Mesh {
    let mut index: HashMap<[u32; 3], u32> = HashMap::new();
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut indices = Vec::new();
    let mut vid = |l: [u32; 3]| -> u32 {
        *index.entry(l).or_insert_with(|| {
            let t = Vec3::new(l[0] as f32, l[1] as f32, l[2] as f32) / n as f32;
            positions.push(transform(min + size * t).to_array());
            (positions.len() - 1) as u32
        })
    };
    for a in 0..3 {
        let (b, c) = ((a + 1) % 3, (a + 2) % 3);
        for side in [0, n] {
            for i in 0..n {
                for j in 0..n {
                    let lat = |u: u32, v: u32| {
                        let mut l = [0u32; 3];
                        l[a] = side;
                        l[b] = u;
                        l[c] = v;
                        l
                    };
                    let q = [
                        vid(lat(i, j)),
                        vid(lat(i + 1, j)),
                        vid(lat(i + 1, j + 1)),
                        vid(lat(i, j + 1)),
                    ];
                    // CCW in (b, c) points along +a.
                    let (t1, t2) = if side == n {
                        ([q[0], q[1], q[2]], [q[0], q[2], q[3]])
                    } else {
                        ([q[0], q[2], q[1]], [q[0], q[3], q[2]])
                    };
                    indices.extend(t1);
                    indices.extend(t2);
                }
            }
        }
    }
    Mesh::from_indexed(positions, indices)
}

/// Solid of revolution around +Z: side radius `r(z)` from z = 0 to `h`,
/// flat caps with one inner ring.
fn revolved(r: impl Fn(f32) -> f32, h: f32, segs: u32, rings: u32) -> Mesh {
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let ring = |positions: &mut Vec<[f32; 3]>, rad: f32, z: f32| -> u32 {
        let start = positions.len() as u32;
        for i in 0..segs {
            let t = i as f32 / segs as f32 * TAU;
            positions.push([rad * t.cos(), rad * t.sin(), z]);
        }
        start
    };
    let side: Vec<u32> = (0..=rings)
        .map(|k| {
            let z = h * k as f32 / rings as f32;
            ring(&mut positions, r(z), z)
        })
        .collect();
    let quad = |indices: &mut Vec<u32>, a: u32, b: u32, c: u32, d: u32| {
        indices.extend([a, b, c, a, c, d]);
    };
    for k in 0..rings as usize {
        for i in 0..segs {
            let j = (i + 1) % segs;
            quad(
                &mut indices,
                side[k] + i,
                side[k] + j,
                side[k + 1] + j,
                side[k + 1] + i,
            );
        }
    }
    for (z, outer, up) in [(0.0, side[0], false), (h, side[rings as usize], true)] {
        let inner = ring(&mut positions, r(z) * 0.5, z);
        let c = positions.len() as u32;
        positions.push([0.0, 0.0, z]);
        for i in 0..segs {
            let j = (i + 1) % segs;
            if up {
                indices.extend([c, inner + i, inner + j]);
                quad(&mut indices, inner + i, outer + i, outer + j, inner + j);
            } else {
                indices.extend([c, inner + j, inner + i]);
                quad(&mut indices, inner + j, outer + j, outer + i, inner + i);
            }
        }
    }
    Mesh::from_indexed(positions, indices)
}

/// Square plate (side 2 `half`, height `h`) with a centred round hole of
/// radius `r`; `4 m` samples around.
fn plate_with_hole(half: f32, r: f32, h: f32, m: u32) -> Mesh {
    let n = 4 * m;
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    // Rings per level: inner (hole), middle, outer (square).
    let mut ring_at = |z: f32| -> [u32; 3] {
        let starts = [0, 1, 2].map(|k| positions.len() as u32 + k * n);
        for k in 0..3 {
            for i in 0..n {
                let t = std::f32::consts::FRAC_PI_4 + i as f32 / n as f32 * TAU;
                let (c, s) = (t.cos(), t.sin());
                let sq = half / c.abs().max(s.abs());
                let rad = [r, 0.5 * (r + sq), sq][k];
                positions.push([rad * c, rad * s, z]);
            }
        }
        starts
    };
    let bottom = ring_at(0.0);
    let top = ring_at(h);
    let quad = |indices: &mut Vec<u32>, a: u32, b: u32, c: u32, d: u32| {
        indices.extend([a, b, c, a, c, d]);
    };
    for i in 0..n {
        let j = (i + 1) % n;
        for k in 0..2 {
            // Top: CCW around +Z; bottom reversed.
            quad(
                &mut indices,
                top[k] + i,
                top[k + 1] + i,
                top[k + 1] + j,
                top[k] + j,
            );
            quad(
                &mut indices,
                bottom[k] + j,
                bottom[k + 1] + j,
                bottom[k + 1] + i,
                bottom[k] + i,
            );
        }
        // Outer walls face out, the hole wall faces the axis.
        quad(
            &mut indices,
            bottom[2] + i,
            bottom[2] + j,
            top[2] + j,
            top[2] + i,
        );
        quad(
            &mut indices,
            bottom[0] + j,
            bottom[0] + i,
            top[0] + i,
            top[0] + j,
        );
    }
    Mesh::from_indexed(positions, indices)
}

fn corners(min: Vec3, size: Vec3) -> Vec<DVec3> {
    let mut out = Vec::new();
    for i in 0..8 {
        let t = Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32);
        out.push((min + size * t).as_dvec3());
    }
    out
}

fn nearest(p: DVec3, candidates: &[DVec3]) -> f64 {
    candidates
        .iter()
        .map(|c| (*c - p).length())
        .fold(f64::MAX, f64::min)
}

#[test]
fn box_becomes_six_planes_twelve_lines_eight_vertices() {
    let (min, size) = (Vec3::new(-10.0, -5.0, 0.0), Vec3::new(20.0, 10.0, 6.0));
    let mesh = grid_box(min, size, 1, |p| p);
    let solid = reconstruct(&mesh).expect("box");
    assert_eq!(solid.faces.len(), 6);
    assert_eq!(solid.edges.len(), 12);
    assert_eq!(solid.vertices.len(), 8);
    assert_eq!(solid.report.genus, 0);
    let (v, e, f) = (8i64, 12i64, 6i64);
    assert_eq!(v - e + f, 2);
    assert!(
        solid
            .faces
            .iter()
            .all(|f| matches!(f.surface, Surface::Plane { .. }))
    );
    assert!(
        solid
            .faces
            .iter()
            .all(|f| f.same_sense && f.loops.len() == 1)
    );
    assert!(
        solid
            .edges
            .iter()
            .all(|e| matches!(e.curve, Curve::Line { .. }))
    );
    let exact = corners(min, size);
    for p in &solid.vertices {
        assert!(
            nearest(*p, &exact) < 1e-9,
            "corner {p} off by {}",
            nearest(*p, &exact)
        );
    }
    validate(&solid).unwrap();

    let step = generate_solid_step("box", &solid);
    assert_eq!(count(&step, "MANIFOLD_SOLID_BREP"), 1);
    assert_eq!(count(&step, "CLOSED_SHELL"), 1);
    assert_eq!(count(&step, "ADVANCED_FACE"), 6);
    assert_eq!(count(&step, "PLANE"), 6);
    assert_eq!(count(&step, "EDGE_CURVE"), 12);
    assert_eq!(count(&step, "LINE"), 12);
    assert_eq!(count(&step, "VERTEX_POINT"), 8);
    assert_eq!(count(&step, "ORIENTED_EDGE"), 24);
    assert!(step.starts_with("ISO-10303-21;") && step.ends_with("END-ISO-10303-21;\n"));
}

#[test]
fn capped_cylinder_becomes_two_planes_and_a_cylinder_with_circle_edges() {
    let mesh = revolved(|_| 8.0, 20.0, 48, 4);
    let solid = reconstruct(&mesh).expect("cylinder");
    assert_eq!(solid.faces.len(), 3);
    assert_eq!(solid.report.surface_counts, [2, 1, 0, 0, 0]);
    assert_eq!(solid.edges.len(), 2);
    assert!(solid.edges.iter().all(|e| e.closed));
    // One seam vertex per closed edge.
    assert_eq!(solid.vertices.len(), 2);
    for e in &solid.edges {
        let Curve::Circle {
            center,
            radius,
            axis,
            ..
        } = e.curve
        else {
            panic!("expected a circle, got {}", e.curve.label());
        };
        assert!(axis.cross(DVec3::Z).length() < 1e-9);
        assert!(center.truncate().length() < 1e-3, "center {center}");
        // The polygonal mesh lies inside the true circle; the fit is close.
        assert!((radius - 8.0).abs() < 0.05, "radius {radius}");
    }
    let cyl = solid
        .faces
        .iter()
        .find(|f| matches!(f.surface, Surface::Cylinder { .. }))
        .unwrap();
    assert_eq!(cyl.loops.len(), 2);
    assert!(cyl.same_sense);
    let step = generate_solid_step("cylinder", &solid);
    assert_eq!(count(&step, "CYLINDRICAL_SURFACE"), 1);
    assert_eq!(count(&step, "CIRCLE"), 2);
    assert_eq!(count(&step, "ADVANCED_FACE"), 3);
}

#[test]
fn noisy_rotated_box_snaps_to_exact_corners() {
    let (min, size) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 14.0, 9.0));
    let rot = Quat::from_axis_angle(Vec3::new(0.3, -0.5, 0.8).normalize(), 0.7);
    let mut rng = Rng::new(7);
    let mesh = grid_box(min, size, 8, |p| {
        let j = Vec3::new(rng.f32() - 0.5, rng.f32() - 0.5, rng.f32() - 0.5) * 0.04;
        rot * (p + j)
    });
    let solid = reconstruct(&mesh).expect("noisy box");
    assert_eq!(solid.faces.len(), 6);
    assert_eq!(solid.edges.len(), 12);
    assert_eq!(solid.vertices.len(), 8);
    let exact: Vec<DVec3> = corners(min, size)
        .into_iter()
        .map(|c| (rot * c.as_vec3()).as_dvec3())
        .collect();
    for p in &solid.vertices {
        assert!(
            nearest(*p, &exact) < 0.03,
            "corner off by {}",
            nearest(*p, &exact)
        );
    }
    // Relationship inference: normals exactly parallel or perpendicular.
    let normals: Vec<DVec3> = solid
        .faces
        .iter()
        .map(|f| match f.surface {
            Surface::Plane { normal, .. } => normal,
            _ => panic!("not a plane"),
        })
        .collect();
    for a in &normals {
        for b in &normals {
            let d = a.dot(*b).abs();
            assert!(d < 1e-12 || (d - 1.0).abs() < 1e-12, "dot {d}");
        }
    }
    assert_eq!(solid.report.relations.parallel_sets, 3);
    assert!(solid.report.mesh_max_dev < 0.05);
}

#[test]
fn open_mesh_is_refused() {
    let mut mesh = grid_box(Vec3::ZERO, Vec3::splat(10.0), 2, |p| p);
    mesh.indices.truncate(mesh.indices.len() - 3);
    let err = reconstruct(&mesh).err().expect("must refuse");
    assert!(err.contains("not closed"), "{err}");
}

#[test]
fn plate_with_hole_has_genus_one_and_inner_loops() {
    let mesh = plate_with_hole(15.0, 5.0, 6.0, 12);
    let solid = reconstruct(&mesh).expect("plate");
    assert_eq!(solid.report.genus, 1);
    assert_eq!(solid.report.surface_counts, [6, 1, 0, 0, 0]);
    // 12 lines of the square block, 2 circles of the hole.
    assert_eq!(solid.report.curve_counts, [12, 2, 0, 0]);
    for f in &solid.faces {
        if let Surface::Plane { normal, .. } = f.surface
            && normal.z.abs() > 0.9
        {
            assert_eq!(f.loops.len(), 2);
            assert_eq!(f.loops.iter().filter(|l| l.outer).count(), 1);
            // The outer bound is the square (four lines).
            assert_eq!(f.loops.iter().find(|l| l.outer).unwrap().edges.len(), 4);
        }
    }
    let hole = solid
        .faces
        .iter()
        .find(|f| matches!(f.surface, Surface::Cylinder { .. }))
        .unwrap();
    // The hole wall faces the axis: opposite to the cylinder's normal.
    assert!(!hole.same_sense);
}

#[test]
fn cone_frustum_gets_a_conical_face() {
    let mesh = revolved(|z| 10.0 - 0.3 * z, 12.0, 64, 6);
    let solid = reconstruct(&mesh).expect("frustum");
    assert_eq!(solid.report.surface_counts, [2, 0, 1, 0, 0]);
    assert_eq!(solid.report.curve_counts, [0, 2, 0, 0]);
    let step = generate_solid_step("frustum", &solid);
    assert_eq!(count(&step, "CONICAL_SURFACE"), 1);
}

#[test]
fn bumpy_top_becomes_a_bspline_face() {
    let (min, size) = (Vec3::new(-10.0, -10.0, 0.0), Vec3::new(20.0, 20.0, 8.0));
    let mesh = grid_box(min, size, 12, |p| {
        let bump = 0.6 * (p.x / 5.0).sin() * (p.y / 6.0).cos();
        Vec3::new(p.x, p.y, p.z * (1.0 + bump / 8.0))
    });
    let solid = reconstruct(&mesh).expect("bumpy box");
    assert_eq!(solid.faces.len(), 6);
    assert_eq!(
        solid.report.surface_counts[4], 1,
        "{:?}",
        solid.report.surface_counts
    );
    // The four edges around the top follow the B-spline face.
    assert_eq!(
        solid.report.curve_counts[3], 4,
        "{:?}",
        solid.report.curve_counts
    );
    assert!(
        solid.report.vertex_gap < 1e-3,
        "{}",
        solid.report.vertex_gap
    );
    let step = generate_solid_step("bumpy", &solid);
    assert_eq!(count(&step, "B_SPLINE_SURFACE_WITH_KNOTS"), 1);
    assert_eq!(count(&step, "B_SPLINE_CURVE_WITH_KNOTS"), 4);

    // Without B-spline faces the freeform group is refused by name.
    let (groups, ids) = groups_of(&mesh);
    let params = SolidParams {
        allow_freeform: false,
        ..SolidParams::default()
    };
    let err = reconstruct_solid(&mesh, &groups, &ids, &params)
        .err()
        .expect("must refuse");
    assert!(err.contains("Group") && err.contains("disabled"), "{err}");
}

/// Writes the STEP files of the synthetic parts to `$SCANIMPROVER_STEP_DIR`
/// for validation with an external kernel (OpenCASCADE):
/// `SCANIMPROVER_STEP_DIR=/tmp/x cargo test --release dump_step -- --ignored`
#[test]
#[ignore]
fn dump_step_files() {
    let Ok(dir) = std::env::var("SCANIMPROVER_STEP_DIR") else {
        return;
    };
    let mut rng = Rng::new(3);
    let rot = Quat::from_axis_angle(Vec3::new(0.3, -0.5, 0.8).normalize(), 0.7);
    let parts: Vec<(&str, Mesh)> = vec![
        (
            "box",
            grid_box(Vec3::ZERO, Vec3::new(20.0, 10.0, 6.0), 1, |p| p),
        ),
        ("cylinder", revolved(|_| 8.0, 20.0, 48, 4)),
        (
            "noisy_box",
            grid_box(Vec3::ZERO, Vec3::new(20.0, 14.0, 9.0), 8, |p| {
                rot * (p + Vec3::new(rng.f32() - 0.5, rng.f32() - 0.5, rng.f32() - 0.5) * 0.04)
            }),
        ),
        ("plate_with_hole", plate_with_hole(15.0, 5.0, 6.0, 12)),
        ("frustum", revolved(|z| 10.0 - 0.3 * z, 12.0, 64, 6)),
        (
            "bumpy",
            grid_box(
                Vec3::new(-10.0, -10.0, 0.0),
                Vec3::new(20.0, 20.0, 8.0),
                12,
                |p| {
                    let bump = 0.6 * (p.x / 5.0).sin() * (p.y / 6.0).cos();
                    Vec3::new(p.x, p.y, p.z * (1.0 + bump / 8.0))
                },
            ),
        ),
    ];
    for (name, mesh) in parts {
        let solid = reconstruct(&mesh).unwrap_or_else(|e| panic!("{name}: {e}"));
        let path = std::path::Path::new(&dir).join(format!("{name}.step"));
        std::fs::write(&path, generate_solid_step(name, &solid)).unwrap();
        crate::io::save_any(
            &std::path::Path::new(&dir).join(format!("{name}.stl")),
            &mesh,
        )
        .unwrap();
    }
}

/// Orients every triangle consistently with its neighbours by walking shared
/// edges. Test helper for filled scans: `unify_normals` keys triangles by
/// directed edge, so it loses one of two triangles that run an edge the same
/// way and leaves that seam flipped.
fn orient_consistently(mesh: &Mesh) -> Mesh {
    let nt = mesh.triangle_count();
    let tri = |idx: &[u32], t: usize| [idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]];
    let mut by_edge: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for t in 0..nt {
        let v = tri(&mesh.indices, t);
        for k in 0..3 {
            let (a, b) = (v[k], v[(k + 1) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut idx = mesh.indices.clone();
    let mut seen = vec![false; nt];
    for s in 0..nt {
        if seen[s] {
            continue;
        }
        seen[s] = true;
        let mut stack = vec![s];
        while let Some(t) = stack.pop() {
            let v = tri(&idx, t);
            for k in 0..3 {
                let (a, b) = (v[k], v[(k + 1) % 3]);
                for &n in &by_edge[&(a.min(b), a.max(b))] {
                    if seen[n] {
                        continue;
                    }
                    seen[n] = true;
                    // A consistent neighbour runs the shared edge as b -> a.
                    let w = tri(&idx, n);
                    if (0..3).any(|j| w[j] == a && w[(j + 1) % 3] == b) {
                        idx.swap(3 * n + 1, 3 * n + 2);
                    }
                    stack.push(n);
                }
            }
        }
    }
    let mut out = Mesh::from_indexed(mesh.positions.clone(), idx);
    out.recompute_normals();
    out
}

/// Reconstructs the mesh file `$SCANIMPROVER_SOLID_MESH` with the default
/// settings and prints the report or the refusal; writes the STEP file into
/// `$SCANIMPROVER_STEP_DIR` when set:
/// `SCANIMPROVER_SOLID_MESH=part.stl cargo test --release reconstruct_mesh_file -- --ignored --nocapture`
#[test]
#[ignore]
fn reconstruct_mesh_file() {
    let Ok(path) = std::env::var("SCANIMPROVER_SOLID_MESH") else {
        return;
    };
    let path = std::path::PathBuf::from(path);
    let bytes = std::fs::read(&path).unwrap();
    let mut mesh = crate::io::load_any(&path, &bytes).unwrap();
    // `SCANIMPROVER_SOLID_REPAIR=1`: auto repair and fill all holes first, as
    // a user would in Mesh repair before reconstructing a raw scan.
    if std::env::var("SCANIMPROVER_SOLID_REPAIR").is_ok() {
        mesh = crate::geom::repair::auto_repair_mesh(&mesh).0;
        use crate::geom::hole_fill::{HoleFillConfig, HoleFillMethod};
        let config = match std::env::var("SCANIMPROVER_FILL").as_deref() {
            Ok("minimal") => HoleFillConfig {
                method: HoleFillMethod::MinimalArea,
                ..Default::default()
            },
            _ => HoleFillConfig::default(),
        };
        for pass in 0..5 {
            let holes = crate::geom::hole_detect::detect_holes(&mesh);
            if holes.is_empty() {
                break;
            }
            println!("repair pass {pass}: {} holes", holes.len());
            mesh = crate::geom::hole_fill::fill_holes(&mesh, &holes, config).unwrap();
            mesh = crate::geom::repair::auto_repair_mesh(&mesh).0;
        }
        mesh = orient_consistently(&mesh);
        let report = crate::geom::repair::analyze_mesh(&mesh);
        println!(
            "after repair: watertight {}, non-manifold edges {}, shells {}, flipped edges {}",
            report.is_watertight,
            report.non_manifold_edges,
            report.component_count,
            report.inconsistent_normals
        );
    }
    let t = std::time::Instant::now();
    let (groups, ids) = groups_of(&mesh);
    let result = reconstruct_solid(&mesh, &groups, &ids, &SolidParams::default());
    let volume: f64 = (0..mesh.triangle_count())
        .map(|t| {
            let [a, b, c] = mesh.triangle(t).map(|p| p.as_dvec3());
            a.dot(b.cross(c)) / 6.0
        })
        .sum();
    println!(
        "{}: {} triangles, {} groups, mesh volume {:.4}, {:.2} s",
        path.display(),
        mesh.triangle_count(),
        groups.len(),
        volume.abs(),
        t.elapsed().as_secs_f32()
    );
    match result {
        Err(e) => println!("refused:\n{e}"),
        Ok(solid) => {
            let r = &solid.report;
            println!(
                "{} faces {:?}, {} edges {:?}, {} vertices, genus {}\nmesh dev max {:.4} (group {}) rms {:.4}, vertex gap {:.2e}, edge gap {:.2e}, edge-mesh {:.4}\n{:?}\n{}",
                solid.faces.len(),
                r.surface_counts,
                solid.edges.len(),
                r.curve_counts,
                solid.vertices.len(),
                r.genus,
                r.mesh_max_dev,
                r.worst_group + 1,
                r.mesh_rms_dev,
                r.vertex_gap,
                r.edge_gap,
                r.edge_mesh_dev,
                r.relations,
                r.notes.join("\n")
            );
            if let Ok(dir) = std::env::var("SCANIMPROVER_STEP_DIR") {
                let stem = path.file_stem().unwrap().to_string_lossy().to_string();
                let out = std::path::Path::new(&dir).join(format!("{stem}.step"));
                std::fs::write(&out, generate_solid_step(&stem, &solid)).unwrap();
            }
        }
    }
}
