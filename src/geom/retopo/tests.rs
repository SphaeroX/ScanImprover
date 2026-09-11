//! Quality tests of the quad retopology on synthetic shapes.

use super::*;
use crate::rng::Rng;
use glam::Vec3;
use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Test shapes
// ---------------------------------------------------------------------------

fn icosphere(radius: f32, subdivisions: usize) -> Mesh {
    let t = (1.0 + 5.0f32.sqrt()) * 0.5;
    let mut pos: Vec<Vec3> = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ]
    .iter()
    .map(|&p| Vec3::from(p).normalize())
    .collect();
    let mut tris: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    for _ in 0..subdivisions {
        let mut cache: HashMap<(u32, u32), u32> = HashMap::new();
        let mut mid = |a: u32, b: u32, pos: &mut Vec<Vec3>| {
            let key = (a.min(b), a.max(b));
            *cache.entry(key).or_insert_with(|| {
                pos.push((pos[a as usize] + pos[b as usize]).normalize());
                pos.len() as u32 - 1
            })
        };
        let mut next = Vec::with_capacity(tris.len() * 4);
        for [a, b, c] in tris {
            let ab = mid(a, b, &mut pos);
            let bc = mid(b, c, &mut pos);
            let ca = mid(c, a, &mut pos);
            next.extend_from_slice(&[[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        tris = next;
    }
    Mesh::from_indexed(
        pos.iter().map(|p| (*p * radius).to_array()).collect(),
        tris.concat(),
    )
}

/// Parametric (u, v) grid, periodic in both directions.
fn periodic_grid(nu: usize, nv: usize, f: impl Fn(f32, f32) -> Vec3) -> Mesh {
    let mut pos = Vec::with_capacity(nu * nv);
    for j in 0..nv {
        for i in 0..nu {
            pos.push(f(i as f32 / nu as f32, j as f32 / nv as f32).to_array());
        }
    }
    let id = |i: usize, j: usize| ((j % nv) * nu + (i % nu)) as u32;
    let mut idx = Vec::with_capacity(nu * nv * 6);
    for j in 0..nv {
        for i in 0..nu {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            idx.extend_from_slice(&[a, b, c, a, c, d]);
        }
    }
    Mesh::from_indexed(pos, idx)
}

fn torus(major: f32, minor: f32, nu: usize, nv: usize) -> Mesh {
    use std::f32::consts::TAU;
    periodic_grid(nu, nv, |u, v| {
        let (su, cu) = (u * TAU).sin_cos();
        let (sv, cv) = (v * TAU).sin_cos();
        Vec3::new(
            (major + minor * cv) * cu,
            (major + minor * cv) * su,
            minor * sv,
        )
    })
}

/// Axis-aligned cube of edge length `size`, every face an m x m grid
/// (diagonals alternate so the triangulation has no preferred direction).
fn box_mesh(size: f32, m: usize) -> Mesh {
    let h = size * 0.5;
    let coord = |i: usize| -h + size * i as f32 / m as f32;
    let mut corners: Vec<[f32; 3]> = Vec::new();
    // (normal axis, sign) for the six faces.
    for axis in 0..3 {
        for sign in [-1.0f32, 1.0] {
            let (u_ax, v_ax) = ((axis + 1) % 3, (axis + 2) % 3);
            let point = |i: usize, j: usize| {
                let mut p = [0.0f32; 3];
                p[axis] = sign * h;
                p[u_ax] = coord(i);
                p[v_ax] = coord(j);
                p
            };
            for j in 0..m {
                for i in 0..m {
                    let (a, b, c, d) = (
                        point(i, j),
                        point(i + 1, j),
                        point(i + 1, j + 1),
                        point(i, j + 1),
                    );
                    let quad = if (i + j) % 2 == 0 {
                        [a, b, c, a, c, d]
                    } else {
                        [a, b, d, b, c, d]
                    };
                    if sign > 0.0 {
                        corners.extend_from_slice(&quad);
                    } else {
                        corners.extend(quad.chunks(3).flat_map(|t| [t[0], t[2], t[1]]));
                    }
                }
            }
        }
    }
    Mesh::from_corners(&corners)
}

/// Closed cylinder along z with flat caps.
fn capped_cylinder(radius: f32, height: f32, sectors: usize, rings: usize) -> Mesh {
    use std::f32::consts::TAU;
    let rim = |s: usize, z: f32, r: f32| {
        let (sn, cs) = (TAU * (s % sectors) as f32 / sectors as f32).sin_cos();
        [r * cs, r * sn, z]
    };
    let mut corners: Vec<[f32; 3]> = Vec::new();
    for k in 0..rings {
        let (z0, z1) = (
            height * k as f32 / rings as f32,
            height * (k + 1) as f32 / rings as f32,
        );
        for s in 0..sectors {
            let (a, b, c, d) = (
                rim(s, z0, radius),
                rim(s + 1, z0, radius),
                rim(s + 1, z1, radius),
                rim(s, z1, radius),
            );
            corners.extend_from_slice(&[a, b, c, a, c, d]);
        }
    }
    // Caps as concentric rings.
    let cap_rings = (radius / (height / rings as f32)).ceil().max(2.0) as usize;
    for (z, up) in [(0.0f32, false), (height, true)] {
        for k in 0..cap_rings {
            let (r0, r1) = (
                radius * k as f32 / cap_rings as f32,
                radius * (k + 1) as f32 / cap_rings as f32,
            );
            for s in 0..sectors {
                let (a, b, c, d) = (
                    rim(s, z, r0),
                    rim(s + 1, z, r0),
                    rim(s + 1, z, r1),
                    rim(s, z, r1),
                );
                let tris: Vec<[[f32; 3]; 3]> = if k == 0 {
                    vec![[a, c, d]]
                } else {
                    vec![[a, c, d], [a, b, c]]
                };
                for [p, q, r] in tris {
                    // Ring order (a, c, d) winds counter-clockwise seen from +z.
                    if up {
                        corners.extend_from_slice(&[p, r, q]);
                    } else {
                        corners.extend_from_slice(&[p, q, r]);
                    }
                }
            }
        }
    }
    Mesh::from_corners(&corners)
}

// ---------------------------------------------------------------------------
// Quality measures
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct Quality {
    quads: usize,
    triangles: usize,
    faces: usize,
    /// Every edge has at most two faces, used once in each direction.
    manifold: bool,
    /// Every edge has exactly two faces.
    closed: bool,
    /// Fraction of vertices with valence 4 (interior vertices only).
    valence4: f32,
    /// Output vertices / face centres to input surface.
    mean_out_to_in: f32,
    max_out_to_in: f32,
    /// Input vertices to output surface.
    max_in_to_out: f32,
    /// Where the two maxima occur (diagnostics, printed via `Debug`).
    #[allow(dead_code)]
    worst_out: Vec3,
    #[allow(dead_code)]
    worst_in: Vec3,
}

fn polygons(m: &Mesh) -> Vec<Vec<u32>> {
    let q = m.quad_count();
    let mut faces: Vec<Vec<u32>> = (0..q)
        .map(|k| {
            let i = &m.indices[6 * k..6 * k + 6];
            vec![i[0], i[1], i[2], i[5]]
        })
        .collect();
    for t in 2 * q..m.triangle_count() {
        faces.push(m.indices[3 * t..3 * t + 3].to_vec());
    }
    faces
}

fn assess(input: &Mesh, out: &Mesh) -> Quality {
    let faces = polygons(out);
    let mut directed: HashMap<(u32, u32), u32> = HashMap::new();
    for f in &faces {
        for c in 0..f.len() {
            *directed.entry((f[c], f[(c + 1) % f.len()])).or_insert(0) += 1;
        }
    }
    let manifold = directed.values().all(|&c| c == 1);
    let closed = manifold
        && directed
            .keys()
            .all(|&(a, b)| directed.contains_key(&(b, a)));
    let mut valence = vec![0u32; out.vertex_count()];
    let mut boundary = vec![false; out.vertex_count()];
    for &(a, b) in directed.keys() {
        if a < b || !directed.contains_key(&(b, a)) {
            valence[a as usize] += 1;
            valence[b as usize] += 1;
        }
        if !directed.contains_key(&(b, a)) {
            boundary[a as usize] = true;
            boundary[b as usize] = true;
        }
    }
    let interior: Vec<usize> = (0..out.vertex_count()).filter(|&v| !boundary[v]).collect();
    let valence4 =
        interior.iter().filter(|&&v| valence[v] == 4).count() as f32 / interior.len().max(1) as f32;

    let bvh_in = Bvh::new(&input.positions, &input.indices);
    let mut samples: Vec<Vec3> = out.positions.iter().map(|&p| Vec3::from(p)).collect();
    for f in &faces {
        samples.push(
            f.iter()
                .map(|&v| Vec3::from(out.positions[v as usize]))
                .sum::<Vec3>()
                / f.len() as f32,
        );
    }
    let d: Vec<f32> = samples
        .iter()
        .map(|&p| bvh_in.closest_distance(p))
        .collect();
    let worst_out = samples[(0..d.len())
        .max_by(|&a, &b| d[a].total_cmp(&d[b]))
        .unwrap_or(0)];
    let bvh_out = Bvh::new(&out.positions, &out.indices);
    // Only vertices that belong to the input surface.
    let (max_in_to_out, worst_in) = input
        .indices
        .iter()
        .map(|&v| {
            let p = Vec3::from(input.positions[v as usize]);
            (bvh_out.closest_distance(p), p)
        })
        .fold((0.0f32, Vec3::ZERO), |a, b| if b.0 > a.0 { b } else { a });
    let quads = out.quad_count();
    Quality {
        quads,
        triangles: out.triangle_count() - 2 * quads,
        faces: faces.len(),
        manifold,
        closed,
        valence4,
        mean_out_to_in: d.iter().sum::<f32>() / d.len() as f32,
        max_out_to_in: d.iter().copied().fold(0.0, f32::max),
        max_in_to_out,
        worst_out,
        worst_in,
    }
}

fn run(mesh: &Mesh, params: RetopoParams) -> (RetopoOutput, Quality) {
    let out = quad_retopology(mesh, &params, &|_, _| {}).expect("retopology succeeds");
    let q = assess(mesh, &out.mesh);
    eprintln!("{:?}\n{q:?}", out.stats);
    (out, q)
}

fn params(target_faces: usize) -> RetopoParams {
    RetopoParams {
        target_faces,
        ..RetopoParams::default()
    }
}

fn assert_count_near(q: &Quality, target: usize) {
    let ratio = q.faces as f32 / target as f32;
    assert!(
        (0.6..=1.5).contains(&ratio),
        "{} faces for target {target}",
        q.faces
    );
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn sphere_becomes_a_closed_pure_quad_mesh() {
    let sphere = icosphere(10.0, 5);
    let target = 1200;
    let (out, q) = run(&sphere, params(target));
    assert_eq!(q.triangles, 0, "pure quad mode leaves no triangles");
    assert!(q.manifold && q.closed, "{q:?}");
    assert_count_near(&q, target);
    let h = out.stats.edge_length;
    // Flat quads on a curved surface: the face centres sag a little.
    assert!(q.max_out_to_in < 0.1 * h, "{q:?}");
    assert!(q.mean_out_to_in < 0.02 * h, "{q:?}");
    assert!(q.max_in_to_out < 0.3 * h, "no holes: {q:?}");
    assert!(q.valence4 > 0.8, "{q:?}");
}

#[test]
fn torus_is_nearly_regular() {
    let t = torus(20.0, 7.0, 160, 64);
    let target = 1500;
    let (out, q) = run(&t, params(target));
    assert_eq!(q.triangles, 0);
    assert!(q.manifold && q.closed, "{q:?}");
    assert_count_near(&q, target);
    assert!(q.max_in_to_out < 0.3 * out.stats.edge_length, "{q:?}");
    assert!(q.valence4 > 0.85, "{q:?}");
}

#[test]
fn quad_dominant_mode_on_the_torus() {
    let t = torus(20.0, 7.0, 160, 64);
    let target = 1500;
    let p = RetopoParams {
        pure_quads: false,
        ..params(target)
    };
    let (_, q) = run(&t, p);
    assert!(q.manifold, "{q:?}");
    assert_count_near(&q, target);
    assert!(
        q.quads as f32 > 0.85 * q.faces as f32,
        "quad dominant: {q:?}"
    );
}

#[test]
fn box_keeps_its_sharp_edges() {
    let b = box_mesh(20.0, 40);
    let target = 600;
    let (out, q) = run(&b, params(target));
    assert!(q.manifold && q.closed, "{q:?}");
    assert_count_near(&q, target);
    let h = out.stats.edge_length;
    // Rounded edges or corners would leave the input edges far from the output.
    assert!(q.max_in_to_out < 0.1 * h, "sharp edges preserved: {q:?}");
    assert!(q.valence4 > 0.8, "{q:?}");
    // Edge flow is aligned with the box axes.
    let aligned = edge_alignment(&out.mesh, |_| [Vec3::X, Vec3::Y, Vec3::Z]);
    assert!(aligned > 0.9, "{aligned}");
}

#[test]
fn cylinder_edges_follow_the_principal_directions() {
    let c = capped_cylinder(8.0, 30.0, 128, 60);
    let target = 1000;
    let (out, q) = run(&c, params(target));
    assert!(q.manifold && q.closed, "{q:?}");
    assert_count_near(&q, target);
    // The rims are followed; a quad next to a singularity can still cut a
    // rim locally (known limitation), which bounds the worst case.
    assert!(
        q.max_in_to_out < 0.3 * out.stats.edge_length,
        "rims preserved: {q:?}"
    );
    assert!(q.mean_out_to_in < 0.02 * out.stats.edge_length, "{q:?}");
    // On the wall the principal directions are the axis and the circumference.
    let aligned = edge_alignment(&out.mesh, |mid| {
        if mid.z < 1.0 || mid.z > 29.0 {
            return [Vec3::ZERO; 3];
        }
        let radial = Vec3::new(mid.x, mid.y, 0.0).normalize();
        [Vec3::Z, Vec3::Z.cross(radial), Vec3::ZERO]
    });
    assert!(aligned > 0.9, "{aligned}");
}

#[test]
fn open_hemisphere_keeps_a_clean_boundary() {
    let sphere = icosphere(10.0, 5);
    let mut idx = Vec::new();
    for t in 0..sphere.triangle_count() {
        let [a, b, c] = sphere.triangle(t);
        if (a + b + c).z > 0.0 {
            idx.extend_from_slice(&sphere.indices[3 * t..3 * t + 3]);
        }
    }
    let half = Mesh::from_indexed(sphere.positions.clone(), idx);
    let target = 600;
    let (out, q) = run(&half, params(target));
    assert!(q.manifold && !q.closed, "{q:?}");
    assert_eq!(q.triangles, 0);
    assert_count_near(&q, target);
    assert!(q.max_in_to_out < 0.3 * out.stats.edge_length, "{q:?}");
}

#[test]
fn noisy_scan_still_gives_a_manifold_quad_mesh() {
    let mut sphere = icosphere(10.0, 6);
    let mut rng = Rng::new(7);
    for p in sphere.positions.iter_mut() {
        let v = Vec3::from(*p);
        let r = 1.0 + (rng.f32() - 0.5) * 0.01;
        *p = (v * r).to_array();
    }
    sphere.recompute_normals();
    let target = 800;
    let (_, q) = run(&sphere, params(target));
    assert!(q.manifold && q.closed, "{q:?}");
    assert_eq!(q.triangles, 0);
    assert_count_near(&q, target);
    assert!(q.valence4 > 0.75, "{q:?}");
}

#[test]
fn face_count_follows_the_target() {
    let sphere = icosphere(10.0, 5);
    let (_, small) = run(&sphere, params(400));
    let (_, large) = run(&sphere, params(2400));
    assert_count_near(&small, 400);
    assert_count_near(&large, 2400);
}

#[test]
fn tiny_input_is_rejected() {
    let m = Mesh::from_indexed(
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        vec![0, 1, 2],
    );
    assert!(quad_retopology(&m, &RetopoParams::default(), &|_, _| {}).is_err());
}

/// Fraction of polygon edges whose direction is within ~15° of one of the
/// reference directions returned for the edge midpoint (zero vectors are
/// ignored; edges without any reference are skipped).
fn edge_alignment(m: &Mesh, dirs: impl Fn(Vec3) -> [Vec3; 3]) -> f32 {
    let mut total = 0;
    let mut good = 0;
    for f in polygons(m) {
        for c in 0..f.len() {
            let a = Vec3::from(m.positions[f[c] as usize]);
            let b = Vec3::from(m.positions[f[(c + 1) % f.len()] as usize]);
            let refs = dirs((a + b) * 0.5);
            if refs.iter().all(|r| *r == Vec3::ZERO) {
                continue;
            }
            total += 1;
            let d = (b - a).normalize_or_zero();
            if refs.iter().any(|r| d.dot(*r).abs() > 0.966) {
                good += 1;
            }
        }
    }
    good as f32 / total.max(1) as f32
}

/// Timing on a real mesh: `SCANIMPROVER_RETOPO_MESH=path cargo test --release
/// retopo_timing -- --ignored --nocapture`.
#[test]
#[ignore]
fn retopo_timing() {
    let Ok(path) = std::env::var("SCANIMPROVER_RETOPO_MESH") else {
        return;
    };
    let path = std::path::PathBuf::from(path);
    let bytes = std::fs::read(&path).unwrap();
    let mut mesh = crate::io::load_any(&path, &bytes).unwrap();
    // Optional 1-to-4 midpoint subdivisions to emulate dense scans.
    let subdiv: usize = std::env::var("SCANIMPROVER_RETOPO_SUBDIV")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    for _ in 0..subdiv {
        let mut pos = mesh.positions.clone();
        let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
        let mut idx = Vec::with_capacity(mesh.indices.len() * 4);
        for t in mesh.indices.chunks_exact(3) {
            let mut m = [0u32; 3];
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                m[k] = *mid.entry((a.min(b), a.max(b))).or_insert_with(|| {
                    let p = (Vec3::from(pos[a as usize]) + Vec3::from(pos[b as usize])) * 0.5;
                    pos.push(p.to_array());
                    pos.len() as u32 - 1
                });
            }
            idx.extend_from_slice(&[
                t[0], m[0], m[2], m[0], t[1], m[1], m[2], m[1], t[2], m[0], m[1], m[2],
            ]);
        }
        mesh = Mesh::from_indexed(pos, idx);
    }
    let target = std::env::var("SCANIMPROVER_RETOPO_TARGET")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let start = std::time::Instant::now();
    let last = std::sync::Mutex::new(start);
    let out = quad_retopology(&mesh, &params(target), &|f, stage| {
        let mut l = last.lock().unwrap();
        eprintln!(
            "{:>6.2}s  {:>3.0}%  {stage}",
            l.elapsed().as_secs_f32(),
            f * 100.0
        );
        *l = std::time::Instant::now();
    })
    .unwrap();
    eprintln!(
        "{} tris -> {:?} in {:.2}s",
        mesh.triangle_count(),
        out.stats,
        start.elapsed().as_secs_f32()
    );
    let q = assess(&mesh, &out.mesh);
    eprintln!("{q:?}");
    if let Ok(dst) = std::env::var("SCANIMPROVER_RETOPO_OUT") {
        crate::io::save_any(std::path::Path::new(&dst), &out.mesh).unwrap();
    }
}
