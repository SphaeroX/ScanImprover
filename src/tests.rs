#[cfg(test)]
mod tests {
    use crate::geom::boundary::{find_boundary_edges, generate_hole_mask};
    use crate::geom::bvh::{closest_point_triangle, Bvh};
    use crate::geom::fitting::{fit_circle, fit_plane};
    use crate::geom::symmetry::{
        detect_symmetry, detect_symmetry_from_line, refine_symmetry, SymPlane,
    };
    use crate::geom::distance::deviation;
    use crate::io;
    use crate::mesh::Mesh;
    use crate::rng::Rng;
    use crate::decimate;
    use glam::Vec3;
    use std::path::Path;

    fn box_mesh(cx: f32, cy: f32, cz: f32, sx: f32, sy: f32, sz: f32) -> Mesh {
        let (x0, x1) = (cx - sx * 0.5, cx + sx * 0.5);
        let (y0, y1) = (cy - sy * 0.5, cy + sy * 0.5);
        let (z0, z1) = (cz - sz * 0.5, cz + sz * 0.5);
        let v: Vec<[f32; 3]> = vec![
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
        ];
        let idx: Vec<u32> = vec![
            0, 1, 2, 0, 2, 3,
            4, 6, 5, 4, 7, 6,
            0, 3, 7, 0, 7, 4,
            1, 5, 6, 1, 6, 2,
            0, 4, 5, 0, 5, 1,
            3, 2, 6, 3, 6, 7,
        ];
        Mesh::from_indexed(v, idx)
    }

    fn sphere_mesh(radius: f32, rings: usize, sectors: usize) -> Mesh {
        let mut v = Vec::new();
        for r in 0..=rings {
            let phi = std::f32::consts::PI * r as f32 / rings as f32;
            for s in 0..=sectors {
                let theta = 2.0 * std::f32::consts::PI * s as f32 / sectors as f32;
                v.push([
                    radius * phi.sin() * theta.cos(),
                    radius * phi.cos(),
                    radius * phi.sin() * theta.sin(),
                ]);
            }
        }
        let mut idx = Vec::new();
        for r in 0..rings {
            for s in 0..sectors {
                let a = (r * (sectors + 1) + s) as u32;
                let b = a + sectors as u32 + 1;
                idx.push(a);
                idx.push(b);
                idx.push(a + 1);
                idx.push(b);
                idx.push(b + 1);
                idx.push(a + 1);
            }
        }
        Mesh::from_indexed(v, idx)
    }

    fn merge(a: &Mesh, b: &Mesh) -> Mesh {
        let mut v = a.positions.clone();
        let mut idx = a.indices.clone();
        let off = a.positions.len() as u32;
        v.extend_from_slice(&b.positions);
        idx.extend(b.indices.iter().map(|i| i + off));
        Mesh::from_indexed(v, idx)
    }

    #[test]
    fn weld_box_corners() {
        let mut raw = Vec::new();
        for t in 0..12 {
            for k in 0..3 {
                raw.push([t as f32 * 3.0 + k as f32, 0.0, 0.0]);
            }
        }
        let m = Mesh::from_corners(&raw);
        assert_eq!(m.triangle_count(), 12);
        assert_eq!(m.vertex_count(), 36);
        for n in &m.normals {
            let l = Vec3::from(*n).length();
            assert!((l - 1.0).abs() < 1e-5, "normal not unit: {l}");
        }
    }

    #[test]
    fn stl_roundtrip() {
        let m = box_mesh(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let bytes = crate::io::stl::save(&m, Path::new("test_part"));
        let loaded = crate::io::stl::load(&bytes).unwrap();
        assert_eq!(loaded.triangle_count(), m.triangle_count());
        let bb1 = m.bbox();
        let bb2 = loaded.bbox();
        assert!((bb1.min - bb2.min).length() < 1e-5);
        assert!((bb1.max - bb2.max).length() < 1e-5);
    }

    #[test]
    fn stl_ascii_load() {
        let text = "solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid x\n";
        let m = crate::io::stl::load(text.as_bytes()).unwrap();
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(m.vertex_count(), 3);
    }

    #[test]
    fn ply_roundtrip() {
        let m = sphere_mesh(5.0, 12, 24);
        let bytes = crate::io::ply::save(&m);
        let loaded = crate::io::ply::load(&bytes).unwrap();
        assert_eq!(loaded.triangle_count(), m.triangle_count());
        assert_eq!(loaded.vertex_count(), m.vertex_count());
        assert!((loaded.surface_area() - m.surface_area()).abs() < 1e-3);
    }

    #[test]
    fn obj_roundtrip() {
        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let text = crate::io::obj::save(&m);
        let loaded = crate::io::obj::load(text.as_bytes()).unwrap();
        assert_eq!(loaded.triangle_count(), m.triangle_count());
        assert!((loaded.bbox().diagonal() - m.bbox().diagonal()).abs() < 1e-4);
    }

    #[test]
    fn bvh_distance_matches_bruteforce() {
        let m = sphere_mesh(3.0, 10, 20);
        let bvh = Bvh::new(&m.positions, &m.indices);
        let mut rng = Rng::new(99);
        let pts = m.sample_surface(200, &mut rng);
        for p in pts {
            let p = Vec3::from(p);
            let d_bvh = bvh.closest_distance(p);
            let mut best = f32::MAX;
            for t in 0..m.triangle_count() {
                let [a, b, c] = m.triangle(t);
                let (_, d2) = closest_point_triangle(p, a, b, c);
                best = best.min(d2.sqrt());
            }
            assert!(
                (d_bvh - best).abs() < 1e-4,
                "bvh {d_bvh} vs brute {best}"
            );
        }
    }

    #[test]
    fn bvh_distance_off_surface() {
        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let bvh = Bvh::new(&m.positions, &m.indices);
        let d = bvh.closest_distance(Vec3::new(5.0, 0.0, 0.0));
        assert!((d - 4.0).abs() < 1e-5, "distance to box face: {d}");
        let d = bvh.closest_distance(Vec3::new(1.5, 1.5, 1.5));
        assert!((d - 0.5 * (3.0f32).sqrt()).abs() < 1e-5);
    }

    #[test]
    fn bvh_ray_cast_hits() {
        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let bvh = Bvh::new(&m.positions, &m.indices);
        let hit = bvh.ray_cast(Vec3::new(-5.0, 0.0, 0.0), Vec3::X, 100.0);
        assert!(hit.is_some());
        let (t, _) = hit.unwrap();
        assert!((t - 4.0).abs() < 1e-4, "hit at t={t}");
        assert!(bvh
            .ray_cast(Vec3::new(-5.0, 0.0, 0.0), -Vec3::X, 100.0)
            .is_none());
    }

    #[test]
    fn plane_fit_noisy() {
        let mut rng = Rng::new(7);
        let mut pts = Vec::new();
        for _ in 0..1000 {
            let x = (rng.f32() - 0.5) * 10.0;
            let y = (rng.f32() - 0.5) * 10.0;
            let z = (rng.f32() - 0.5) * 0.002;
            pts.push([x, y, z]);
        }
        let f = fit_plane(&pts).unwrap();
        assert!(f.normal.z.abs() > 0.999, "normal {:?}", f.normal);
        assert!(f.max_dev < 0.002);
        assert!(f.point.z.abs() < 0.01);
    }

    #[test]
    fn circle_fit_synthetic() {
        let mut pts = Vec::new();
        let n = 720;
        for i in 0..n {
            let a = i as f32 / n as f32 * 2.0 * std::f32::consts::PI;
            let (c, s) = (a.cos(), a.sin());
            pts.push([5.0 * c + 1.0, 5.0 * s + 2.0, 0.5]);
        }
        let c = fit_circle(&pts).unwrap();
        assert!((c.radius - 5.0).abs() < 1e-3, "radius {}", c.radius);
        assert!((c.center.x - 1.0).abs() < 1e-3, "center {:?}", c.center);
        assert!((c.center.y - 2.0).abs() < 1e-3);
        assert!((c.center.z - 0.5).abs() < 1e-3);
        assert!(c.radial_rms.sqrt() < 1e-3);
    }

    fn symmetric_test_part() -> Mesh {
        let base = box_mesh(0.0, 0.0, 0.0, 4.0, 2.0, 2.0);
        let bump_l = box_mesh(1.5, 1.25, 0.8, 0.5, 0.5, 0.5);
        let bump_r = box_mesh(-1.5, 1.25, 0.8, 0.5, 0.5, 0.5);
        merge(&merge(&base, &bump_l), &bump_r)
    }

    #[test]
    fn symmetry_detect_finds_x_plane() {
        let m = symmetric_test_part();
        let bvh = Bvh::new(&m.positions, &m.indices);
        let (plane, rms) = detect_symmetry(&m, &bvh).expect("no symmetry found");
        assert!(plane.normal.x.abs() > 0.99, "normal {:?}", plane.normal);
        assert!(plane.point.x.abs() < 0.05, "point {:?}", plane.point);
        assert!(rms < 0.02, "rms {rms}");
    }

    #[test]
    fn symmetry_refine_from_tilted_init() {
        let m = symmetric_test_part();
        let bvh = Bvh::new(&m.positions, &m.indices);
        let init = SymPlane {
            normal: Vec3::new(0.99, 0.05, 0.1).normalize(),
            point: Vec3::new(0.2, 0.0, 0.0),
        };
        let (plane, rms) = refine_symmetry(&m, &bvh, &init);
        assert!(plane.normal.x.abs() > 0.99, "normal {:?}", plane.normal);
        assert!(plane.point.x.abs() < 0.05, "point {:?}", plane.point);
        assert!(rms < 0.02, "rms {rms}");
    }

    #[test]
    fn symmetry_from_line_finds_exact_plane() {
        let m = symmetric_test_part();
        let bvh = Bvh::new(&m.positions, &m.indices);
        let a = Vec3::new(0.0, 1.0, 0.5);
        let b = Vec3::new(0.0, -1.0, -0.5);
        let (plane, rms) = detect_symmetry_from_line(&m, &bvh, a, b, None)
            .expect("symmetry from line failed");
        assert!(plane.normal.x.abs() > 0.99, "normal {:?}", plane.normal);
        assert!(plane.point.x.abs() < 0.05, "point {:?}", plane.point);
        assert!(rms < 0.05, "rms {rms}");
    }

    #[test]
    fn symmetry_from_line_with_exclusion_mask() {
        let base = box_mesh(0.0, 0.0, 0.0, 4.0, 2.0, 2.0);
        let bump_l = box_mesh(1.5, 1.25, 0.8, 0.5, 0.5, 0.5);
        let bump_r = box_mesh(-1.5, 1.25, 0.8, 0.5, 0.5, 0.5);
        let sym_part = merge(&merge(&base, &bump_l), &bump_r);
        let sym_tri_count = sym_part.triangle_count();

        let asym_protrusion = box_mesh(1.8, -0.8, 0.0, 0.8, 0.8, 0.8);
        let m = merge(&sym_part, &asym_protrusion);
        let total_tris = m.triangle_count();

        let mut mask = vec![0u8; total_tris];
        for t in sym_tri_count..total_tris {
            mask[t] = 1;
        }

        let bvh = Bvh::new(&m.positions, &m.indices);
        let a = Vec3::new(0.0, 1.0, 0.5);
        let b = Vec3::new(0.0, -1.0, -0.5);
        let (plane, rms) = detect_symmetry_from_line(&m, &bvh, a, b, Some(&mask))
            .expect("symmetry with mask failed");
        assert!(plane.normal.x.abs() > 0.99, "normal {:?}", plane.normal);
        assert!(plane.point.x.abs() < 0.05, "point {:?}", plane.point);
        assert!(rms < 0.05, "rms {rms}");
    }

    #[test]
    fn boundary_hole_detection_watertight_and_open() {
        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let b_edges = find_boundary_edges(&m);
        assert_eq!(b_edges.len(), 0, "watertight box should have 0 boundary edges");
        let mask = generate_hole_mask(&m, 2);
        assert!(mask.iter().all(|&v| v == 0), "watertight mask should be empty");

        // Open mesh: remove 2 triangles (indices 0..6) from box
        let mut open_indices = m.indices.clone();
        open_indices.drain(0..6);
        let open_m = Mesh::from_indexed(m.positions.clone(), open_indices);
        let open_edges = find_boundary_edges(&open_m);
        assert!(!open_edges.is_empty(), "open mesh must have boundary edges");
        let open_mask = generate_hole_mask(&open_m, 2);
        assert!(open_mask.iter().any(|&v| v > 0), "open mesh mask should have masked triangles");
    }

    #[test]
    fn symmetry_with_automatic_hole_exclusion() {
        let sym_part = symmetric_test_part();
        let mut hole_indices = Vec::new();
        for t in 0..sym_part.triangle_count() {
            let [p0, p1, p2] = sym_part.triangle(t);
            let center_x = (p0.x + p1.x + p2.x) / 3.0;
            // Remove a patch on the +X bump
            if center_x > 1.4 && center_x < 1.7 && p0.y > 1.2 {
                continue;
            }
            hole_indices.push(sym_part.indices[3 * t]);
            hole_indices.push(sym_part.indices[3 * t + 1]);
            hole_indices.push(sym_part.indices[3 * t + 2]);
        }
        let mesh_with_hole = Mesh::from_indexed(sym_part.positions.clone(), hole_indices);
        let bvh = Bvh::new(&mesh_with_hole.positions, &mesh_with_hole.indices);
        let hole_mask = generate_hole_mask(&mesh_with_hole, 2);

        let a = Vec3::new(0.0, 1.0, 0.5);
        let b = Vec3::new(0.0, -1.0, -0.5);
        let (plane, rms) = detect_symmetry_from_line(&mesh_with_hole, &bvh, a, b, Some(&hole_mask))
            .expect("symmetry with hole mask failed");
        assert!(plane.normal.x.abs() > 0.99, "normal {:?}", plane.normal);
        assert!(plane.point.x.abs() < 0.05, "point {:?}", plane.point);
        assert!(rms < 0.05, "rms {rms}");
    }

    #[test]
    fn decimate_sphere_bounded_error() {
        let m = sphere_mesh(10.0, 48, 96);
        let tris_before = m.triangle_count();
        let (simplified, est_err) = decimate::decimate(&m, 0.1, 0.05, false).unwrap();
        assert!(simplified.triangle_count() < tris_before / 2);
        assert!(est_err <= 0.05 + 1e-6, "estimate {est_err}");
        let bvh = Bvh::new(&simplified.positions, &simplified.indices);
        let dev = deviation(&m.positions, &bvh);
        assert!(
            dev.max_dev < est_err * 5.0,
            "max deviation {} vs estimate {}",
            dev.max_dev,
            est_err
        );
        let acc = 100.0 * (1.0 - dev.max_dev / m.bbox().diagonal());
        assert!(acc > 99.0);
    }

    #[test]
    fn auto_decimate_hits_target() {
        let m = sphere_mesh(10.0, 48, 96);
        let tris_before = m.triangle_count();
        let target = 0.05f32;
        let (simplified, est, dev, _heat, iterations) =
            crate::worker::auto_decimate(&m, target, false).unwrap();
        assert!(iterations >= 1 && iterations <= 5);
        assert!(
            dev.max_dev <= target * 1.6,
            "measured {} vs target {}",
            dev.max_dev,
            target
        );
        assert!(
            simplified.triangle_count() < tris_before * 4 / 5,
            "not decimated enough: {} -> {}",
            tris_before,
            simplified.triangle_count()
        );
        let est_ok = est <= target + 1e-5;
        assert!(est_ok, "estimate {est} above target {target}");
    }

    #[test]
    fn transform_rotates_features() {
        let m = box_mesh(0.0, 0.0, 0.0, 4.0, 2.0, 8.0);
        let q = glam::Quat::from_axis_angle(Vec3::X, std::f32::consts::FRAC_PI_2);
        let mut m2 = m.clone();
        m2.transform(q, Vec3::new(5.0, 0.0, 0.0));
        let bb = m2.bbox();
        assert!((bb.center().x - 5.0).abs() < 1e-5);
        assert!((bb.extent().x - 4.0).abs() < 1e-4);
        assert!((bb.extent().y - 8.0).abs() < 1e-4);
        assert!((bb.extent().z - 2.0).abs() < 1e-4);
        for n in &m2.normals {
            assert!((Vec3::from(*n).length() - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn screen_ray_hits_box_center() {
        use crate::camera::Camera;
        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let bvh = Bvh::new(&m.positions, &m.indices);
        let mut cam = Camera::default();
        cam.aspect = 4.0 / 3.0;
        cam.fit(&m.bbox());
        let (ro, rd) = cam.screen_ray(400.0, 300.0, 800.0, 600.0);
        let hit = bvh.ray_cast(ro, rd, cam.far);
        assert!(hit.is_some(), "center ray missed: ro {ro:?} rd {rd:?}");
        let (t, _) = hit.unwrap();
        let p = ro + rd * t;
        assert!(
            p.x.abs() <= 1.001 && p.y.abs() <= 1.001 && p.z.abs() <= 1.001,
            "hit {p:?} not on the box surface"
        );
    }

    #[test]
    fn screen_ray_corners_miss_box() {
        use crate::camera::Camera;
        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let bvh = Bvh::new(&m.positions, &m.indices);
        let mut cam = Camera::default();
        cam.aspect = 1.0;
        cam.fit(&m.bbox());
        cam.set_view(crate::camera::ViewDir::Iso);
        let (ro, rd) = cam.screen_ray(1.0, 1.0, 800.0, 600.0);
        assert!(bvh.ray_cast(ro, rd, cam.far).is_none());
        let (ro, rd) = cam.screen_ray(400.0, 300.0, 800.0, 600.0);
        assert!(bvh.ray_cast(ro, rd, cam.far).is_some());
    }

    #[test]
    fn brush_selects_under_cursor() {
        use crate::camera::Camera;
        use crate::pick;
        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let bvh = Bvh::new(&m.positions, &m.indices);
        let mut cam = Camera::default();
        cam.target = Vec3::ZERO;
        cam.distance = 20.0;
        cam.aspect = 4.0 / 3.0;
        cam.orient = glam::Quat::from_rotation_arc(Vec3::Z, -Vec3::X);
        let (ro, rd) = cam.screen_ray(400.0, 300.0, 800.0, 600.0);
        let (t, tri) = bvh.ray_cast(ro, rd, cam.far).expect("center ray should hit");
        let hit = pick::Hit { pos: ro + rd * t, tri };
        let mut sel = vec![0u8; m.triangle_count()];
        pick::brush(&m, &cam, &hit, 40.0, 600.0, true, &mut sel);
        assert!(
            sel.iter().filter(|&&v| v > 0).count() >= 1,
            "brush selected nothing"
        );
        let hit_face_sel = sel[tri as usize] > 0;
        assert!(hit_face_sel, "face directly under cursor must be selected");
    }

    #[test]
    fn io_dispatch_unknown() {
        let err = match io::load_any(Path::new("foo.txt"), b"x") {
            Err(e) => e,
            Ok(_) => panic!("expected failure"),
        };
        assert!(err.contains("Unsupported"));
    }
}
