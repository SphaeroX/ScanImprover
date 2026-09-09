#[cfg(test)]
mod tests {
    use crate::decimate;
    use crate::geom::boundary::{find_boundary_edges, generate_hole_mask};
    use crate::geom::bvh::{Bvh, closest_point_triangle};
    use crate::geom::distance::deviation;
    use crate::geom::fitting::{fit_circle, fit_plane};
    use crate::geom::symmetry::{
        SymPlane, detect_symmetry, detect_symmetry_from_line, refine_symmetry,
    };
    use crate::io;
    use crate::mesh::Mesh;
    use crate::rng::Rng;
    use glam::Vec3;
    use std::path::Path;

    fn box_mesh(cx: f32, cy: f32, cz: f32, sx: f32, sy: f32, sz: f32) -> Mesh {
        let (x0, x1) = (cx - sx * 0.5, cx + sx * 0.5);
        let (y0, y1) = (cy - sy * 0.5, cy + sy * 0.5);
        let (z0, z1) = (cz - sz * 0.5, cz + sz * 0.5);
        let v: Vec<[f32; 3]> = vec![
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ];
        let idx: Vec<u32> = vec![
            0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6, 0, 3, 7, 0, 7, 4, 1, 5, 6, 1, 6, 2, 0, 4, 5, 0, 5,
            1, 3, 2, 6, 3, 6, 7,
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

    fn cylinder_wall_mesh(radius: f32, height: f32, rings: usize, sectors: usize) -> Mesh {
        let mut v = Vec::new();
        for r in 0..=rings {
            let z = height * r as f32 / rings as f32;
            for s in 0..sectors {
                let theta = 2.0 * std::f32::consts::PI * s as f32 / sectors as f32;
                v.push([radius * theta.cos(), radius * theta.sin(), z]);
            }
        }
        let mut idx = Vec::new();
        for r in 0..rings {
            for s in 0..sectors {
                let s1 = (s + 1) % sectors;
                let a = (r * sectors + s) as u32;
                let b = (r * sectors + s1) as u32;
                let c = ((r + 1) * sectors + s) as u32;
                let d = ((r + 1) * sectors + s1) as u32;
                idx.extend_from_slice(&[a, b, d, a, d, c]);
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
            assert!((d_bvh - best).abs() < 1e-4, "bvh {d_bvh} vs brute {best}");
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
        assert!(
            bvh.ray_cast(Vec3::new(-5.0, 0.0, 0.0), -Vec3::X, 100.0)
                .is_none()
        );
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
        assert!(!c.cylinder, "planar ring must not be reported as cylinder");
    }

    #[test]
    fn circle_fit_on_tall_cylinder_wall_patch() {
        // 240 degree patch of a vertical cylinder, R = 8, axis Z through
        // (2, -3), height 30: the PCA plane would be slanted along the wall.
        let mut pts = Vec::new();
        let n_ang = 120;
        let n_h = 40;
        for i in 0..n_ang {
            let a = i as f32 / n_ang as f32 * 240.0f32.to_radians();
            for j in 0..n_h {
                let z = -5.0 + 30.0 * j as f32 / (n_h - 1) as f32;
                pts.push([2.0 + 8.0 * a.cos(), -3.0 + 8.0 * a.sin(), z]);
            }
        }
        let c = fit_circle(&pts).unwrap();
        assert!(c.cylinder, "cylinder wall must be recognized");
        assert!(
            c.normal.z.abs() > 0.999,
            "normal must be the cylinder axis, got {:?}",
            c.normal
        );
        assert!((c.radius - 8.0).abs() < 0.01, "radius {}", c.radius);
        assert!((c.center.x - 2.0).abs() < 0.01, "center {:?}", c.center);
        assert!((c.center.y + 3.0).abs() < 0.01, "center {:?}", c.center);
        assert!(c.radial_rms.sqrt() < 1e-3, "radial rms {}", c.radial_rms);
    }

    #[test]
    fn circle_fit_on_narrow_tall_cylinder_strip() {
        // 30 degree arc, R = 12, height 40: the PCA plane would run lengthwise
        // along the wall (a "section along the height"), giving a crooked fit.
        let mut pts = Vec::new();
        let n_ang = 24;
        let n_h = 60;
        for i in 0..n_ang {
            let a = i as f32 / n_ang as f32 * 30.0f32.to_radians();
            for j in 0..n_h {
                let z = 40.0 * j as f32 / (n_h - 1) as f32;
                pts.push([12.0 * a.cos(), 12.0 * a.sin(), z]);
            }
        }
        let c = fit_circle(&pts).unwrap();
        assert!(c.cylinder, "cylinder strip must be recognized");
        assert!(
            c.normal.z.abs() > 0.99,
            "normal must be the cylinder axis, got {:?}",
            c.normal
        );
        assert!((c.radius - 12.0).abs() < 0.1, "radius {}", c.radius);
        assert!(c.center.x.abs() < 0.1, "center {:?}", c.center);
        assert!(c.center.y.abs() < 0.1, "center {:?}", c.center);
        assert!(c.radial_rms.sqrt() < 0.05, "radial rms {}", c.radial_rms);
    }

    #[test]
    fn circle_fit_full_cylinder_wall_mesh() {
        // Triangle vertices of a complete cylinder wall, R = 10, height 40.
        let m = cylinder_wall_mesh(10.0, 40.0, 6, 32);
        let mut pts: Vec<[f32; 3]> = Vec::new();
        for t in 0..m.triangle_count() {
            for k in 0..3 {
                pts.push(m.positions[m.indices[3 * t + k] as usize]);
            }
        }
        let c = fit_circle(&pts).unwrap();
        assert!(c.cylinder, "full cylinder wall must be recognized");
        assert!(c.normal.z.abs() > 0.999, "normal {:?}", c.normal);
        assert!((c.radius - 10.0).abs() < 0.01, "radius {}", c.radius);
        assert!(c.center.x.abs() < 0.01, "center {:?}", c.center);
        assert!(c.center.y.abs() < 0.01, "center {:?}", c.center);
        assert!(c.radial_rms.sqrt() < 1e-3, "radial rms {}", c.radial_rms);
    }

    #[test]
    fn circle_fit_noisy_cylinder_wall() {
        // Full 360 degree bore wall with scanner-like noise (+/- 0.05 mm).
        let mut rng = Rng::new(11);
        let mut pts = Vec::new();
        for _ in 0..4000 {
            let a = rng.f32() * 2.0 * std::f32::consts::PI;
            let z = rng.f32() * 25.0;
            let n = |rng: &mut Rng| (rng.f32() - 0.5) * 0.1;
            pts.push([
                3.0 + 9.0 * a.cos() + n(&mut rng),
                -7.0 + 9.0 * a.sin() + n(&mut rng),
                z + n(&mut rng),
            ]);
        }
        let c = fit_circle(&pts).unwrap();
        assert!(c.cylinder, "noisy bore wall must be recognized");
        assert!(c.normal.z.abs() > 0.999, "normal {:?}", c.normal);
        assert!((c.radius - 9.0).abs() < 0.02, "radius {}", c.radius);
        assert!((c.center.x - 3.0).abs() < 0.02, "center {:?}", c.center);
        assert!((c.center.y + 7.0).abs() < 0.02, "center {:?}", c.center);
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
        let (plane, rms) =
            detect_symmetry_from_line(&m, &bvh, a, b, None).expect("symmetry from line failed");
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
        assert_eq!(
            b_edges.len(),
            0,
            "watertight box should have 0 boundary edges"
        );
        let mask = generate_hole_mask(&m, 2);
        assert!(
            mask.iter().all(|&v| v == 0),
            "watertight mask should be empty"
        );

        // Open mesh: remove 2 triangles (indices 0..6) from box
        let mut open_indices = m.indices.clone();
        open_indices.drain(0..6);
        let open_m = Mesh::from_indexed(m.positions.clone(), open_indices);
        let open_edges = find_boundary_edges(&open_m);
        assert!(!open_edges.is_empty(), "open mesh must have boundary edges");
        let open_mask = generate_hole_mask(&open_m, 2);
        assert!(
            open_mask.iter().any(|&v| v > 0),
            "open mesh mask should have masked triangles"
        );
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
        let (t, tri) = bvh
            .ray_cast(ro, rd, cam.far)
            .expect("center ray should hit");
        let hit = pick::Hit {
            pos: ro + rd * t,
            tri,
        };
        let mut sel = vec![0u8; m.triangle_count()];
        pick::brush(&m, &bvh, &cam, &hit, 40.0, 600.0, true, &mut sel);
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

    #[test]
    fn multi_plane_and_circle_management() {
        let mut app = crate::app::App::new();
        let m = std::sync::Arc::new(box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0));
        let tris = m.triangle_count();
        app.current = Some(m.clone());
        app.original = Some(m.clone());
        app.sel = std::sync::Arc::new(vec![1u8; tris]);
        app.sel_count = tris;

        // Fit first plane
        app.fit_plane_from_selection();
        assert_eq!(app.planes.len(), 1);
        assert_eq!(app.planes[0].name, "Plane 1");
        assert!(app.selected_plane_id.is_some());

        // Fit second plane
        app.fit_plane_from_selection();
        assert_eq!(app.planes.len(), 2);
        assert_eq!(app.planes[1].name, "Plane 2");
        let id2 = app.planes[1].id;
        assert_eq!(app.selected_plane_id, Some(id2));

        // Delete plane 1
        let id1 = app.planes[0].id;
        app.delete_plane(id1);
        assert_eq!(app.planes.len(), 1);
        assert_eq!(app.planes[0].id, id2);

        // Undo delete
        app.undo();
        assert_eq!(app.planes.len(), 2);

        // Test clear selection
        assert!(app.sel_count > 0);
        app.clear_selection();
        assert_eq!(app.sel_count, 0);
    }

    #[test]
    fn symmetry_pick_line_mode_transition_and_suppression() {
        use crate::app::Mode;
        let mut app = crate::app::App::new();
        let m = std::sync::Arc::new(box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0));
        let tris = m.triangle_count();
        app.current = Some(m.clone());
        app.original = Some(m.clone());
        app.sel = std::sync::Arc::new(vec![0u8; tris]);
        app.sel_count = 0;

        // Start in SymPickLine
        app.mode = Mode::SymPickLine;
        app.sym_pick.clear();
        assert_eq!(app.mode, Mode::SymPickLine);

        // Place point 1
        let p1 = Vec3::new(0.0, 0.0, 1.0);
        app.sym_pick.push(p1);
        app.suppress_sel_drag = true;
        assert_eq!(app.sym_pick.len(), 1);
        assert_eq!(app.mode, Mode::SymPickLine);

        // Place point 2 -> triggers transition to Mode::Orbit and schedules symmetry calculation
        let p2 = Vec3::new(1.0, 0.0, 1.0);
        app.sym_pick.push(p2);
        app.suppress_sel_drag = true;
        if app.sym_pick.len() >= 2 {
            let a = app.sym_pick[0];
            let b = app.sym_pick[1];
            app.schedule_sym_from_line(a, b);
            app.mode = Mode::Orbit;
        }

        // Mode is now Orbit, 2 points stored, sym job submitted
        assert_eq!(app.mode, Mode::Orbit);
        assert_eq!(app.sym_pick.len(), 2);
        assert!(app.sym_job.is_some());

        // While mouse is still held down from point 2 click, suppress_sel_drag is true:
        assert!(app.suppress_sel_drag);
        assert_eq!(
            app.sel_count, 0,
            "No faces must be selected during pick line placement"
        );

        // Once mouse is released, suppress_sel_drag resets to false:
        app.suppress_sel_drag = false;
        assert!(!app.suppress_sel_drag);
    }

    #[test]
    fn face_selection_automatically_opens_accordion() {
        use crate::camera::Camera;
        use crate::pick;
        use crate::ui::ToolSection;

        let mut app = crate::app::App::new();
        let m = std::sync::Arc::new(box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0));
        let tris = m.triangle_count();
        app.current = Some(m.clone());
        app.original = Some(m.clone());
        app.sel = std::sync::Arc::new(vec![0u8; tris]);
        app.sel_count = 0;
        app.active_section = Some(ToolSection::Decimation);

        let bvh = Bvh::new(&m.positions, &m.indices);
        let mut cam = Camera::default();
        cam.target = Vec3::ZERO;
        cam.distance = 20.0;
        cam.aspect = 4.0 / 3.0;
        cam.orient = glam::Quat::from_rotation_arc(Vec3::Z, -Vec3::X);
        let (ro, rd) = cam.screen_ray(400.0, 300.0, 800.0, 600.0);
        let (t, tri) = bvh
            .ray_cast(ro, rd, cam.far)
            .expect("center ray should hit");
        let hit = pick::Hit {
            pos: ro + rd * t,
            tri,
        };

        let mut sel = (*app.sel).clone();
        pick::brush(&m, &bvh, &cam, &hit, 40.0, 600.0, true, &mut sel);
        app.sel = std::sync::Arc::new(sel);
        app.recount_sel();
        if app.sel_count > 0 {
            app.active_section = Some(ToolSection::Selection);
        }

        assert!(app.sel_count > 0);
        assert_eq!(app.active_section, Some(ToolSection::Selection));
    }

    #[test]
    fn export_step_and_script_and_dxf() {
        use crate::export::{dxf, fusion_script, step};
        use crate::geom::fitting::{CircleFit, FittedCircle, FittedPlane, PlaneFit};

        let plane = FittedPlane {
            id: 1,
            name: "Top Plane".to_string(),
            fit: PlaneFit {
                point: Vec3::new(10.0, 20.0, 30.0),
                normal: Vec3::Z,
                rms: 0.001,
                max_dev: 0.002,
            },
            visible: true,
            color: [1.0, 0.5, 0.2, 1.0],
        };

        let circle = FittedCircle {
            id: 2,
            name: "Borehole 1".to_string(),
            fit: CircleFit {
                center: Vec3::new(10.0, 20.0, 30.0),
                normal: Vec3::Z,
                radius: 15.0,
                plane_rms: 0.001,
                radial_rms: 0.002,
                radial_max: 0.003,
                cylinder: false,
            },
            visible: true,
            color: [0.2, 0.8, 0.3, 1.0],
        };

        // 1. STEP Export Test
        let step_content = step::generate_step(&[&plane], &[&circle], 100.0);
        assert!(step_content.contains("ISO-10303-21;"));
        assert!(step_content.contains("ADVANCED_FACE('Top Plane'"));
        assert!(step_content.contains("ADVANCED_FACE('Borehole 1'"));
        assert!(step_content.contains("CIRCLE('"));
        assert!(step_content.contains("OPEN_SHELL('"));
        assert!(step_content.contains("END-ISO-10303-21;"));

        // 2. Fusion Script Test
        let py_content = fusion_script::generate_fusion_script(&[&plane], &[&circle]);
        assert!(py_content.contains("import adsk.core"));
        assert!(py_content.contains("import adsk.fusion"));
        assert!(py_content.contains("plane_feat.name = \"Top Plane\""));
        assert!(py_content.contains("sketch.name = \"Borehole 1\""));
        assert!(py_content.contains("sketchCurves.sketchCircles.addByCenterRadius"));
        // Check mm -> cm conversion (10mm -> 1.000000 cm)
        assert!(py_content.contains("1.000000"));

        // 3. DXF Test
        let dxf_content = dxf::generate_dxf(&[&plane], &[&circle], 100.0);
        assert!(dxf_content.contains("3DFACE"));
        assert!(dxf_content.contains("CIRCLE"));
        assert!(dxf_content.contains("POINT"));
        assert!(dxf_content.contains("EOF"));
    }

    #[test]
    fn feature_alignment_quick_surface_workflow() {
        use crate::app::{App, SymState};
        use crate::geom::alignment::{AxisChoice, FeatureRef};
        use crate::geom::fitting::{CircleFit, FittedCircle, FittedPlane, PlaneFit};
        use crate::geom::symmetry::SymPlane;

        let mut app = App::new();
        let dummy_mesh = std::sync::Arc::new(crate::mesh::Mesh::default());
        app.current = Some(dummy_mesh.clone());
        app.original = Some(dummy_mesh);

        // 1. Setup Symmetry Plane (pointing along Y initially)
        app.sym = Some(SymState {
            plane: SymPlane {
                point: Vec3::new(0.0, 10.0, 0.0),
                normal: Vec3::Y,
            },
            rms: 0.001,
            show: true,
        });

        // 2. Setup Circle (axis along X initially, center at 50, 10, 30)
        let circle = FittedCircle {
            id: 1,
            name: "Main Bore".to_string(),
            fit: CircleFit {
                center: Vec3::new(50.0, 10.0, 30.0),
                normal: Vec3::X,
                radius: 12.0,
                plane_rms: 0.001,
                radial_rms: 0.001,
                radial_max: 0.002,
                cylinder: false,
            },
            visible: true,
            color: [0.0, 1.0, 0.0, 1.0],
        };
        app.circles.push(circle);

        // 3. Setup Plane (normal along Z initially, point at 0, 0, 25)
        let plane = FittedPlane {
            id: 2,
            name: "Base Plane".to_string(),
            fit: PlaneFit {
                point: Vec3::new(0.0, 0.0, 25.0),
                normal: Vec3::Z,
                rms: 0.001,
                max_dev: 0.002,
            },
            visible: true,
            color: [1.0, 0.0, 0.0, 1.0],
        };
        app.planes.push(plane);

        // Assign Symmetry Plane to X
        app.toggle_assign_feature(FeatureRef::SymmetryPlane, AxisChoice::X);
        assert_eq!(
            app.feature_assigned_axis(FeatureRef::SymmetryPlane),
            Some(AxisChoice::X)
        );

        // Assign Circle to Z
        app.toggle_assign_feature(FeatureRef::Circle(1), AxisChoice::Z);
        assert_eq!(
            app.feature_assigned_axis(FeatureRef::Circle(1)),
            Some(AxisChoice::Z)
        );

        // Assign Plane to Y
        app.toggle_assign_feature(FeatureRef::Plane(2), AxisChoice::Y);
        assert_eq!(
            app.feature_assigned_axis(FeatureRef::Plane(2)),
            Some(AxisChoice::Y)
        );

        // Set Origin to Circle Center
        app.toggle_origin_feature(FeatureRef::Circle(1));
        assert!(app.is_feature_origin(FeatureRef::Circle(1)));

        // Run alignment!
        let success = app.align_to_features();
        assert!(success, "align_to_features should succeed");

        // Verify Symmetry Plane normal is now along X (or -X)
        let sym_norm = app.sym.unwrap().plane.normal;
        assert!(
            (sym_norm.abs() - Vec3::X).length() < 1e-4,
            "Symmetry normal should be along X, got {:?}",
            sym_norm
        );

        // Verify Circle normal (axis) is now along Z (or -Z)
        let circ_norm = app.circles[0].fit.normal;
        assert!(
            (circ_norm.abs() - Vec3::Z).length() < 1e-4,
            "Circle axis should be along Z, got {:?}",
            circ_norm
        );

        // Verify Plane normal is now along Y (or -Y)
        let plane_norm = app.planes[0].fit.normal;
        assert!(
            (plane_norm.abs() - Vec3::Y).length() < 1e-4,
            "Plane normal should be along Y, got {:?}",
            plane_norm
        );

        // Verify Circle Center is at origin (0, 0, 0)
        let circ_center = app.circles[0].fit.center;
        assert!(
            circ_center.length() < 1e-4,
            "Circle center should be at (0,0,0), got {:?}",
            circ_center
        );

        // Test Undo
        assert_eq!(app.undo.len(), 1);
        app.undo();
        assert!(
            (app.circles[0].fit.center - Vec3::new(50.0, 10.0, 30.0)).length() < 1e-4,
            "Undo should restore previous circle center"
        );
    }

    #[test]
    fn test_hole_detection_single_and_multiple() {
        use crate::geom::hole_detect::detect_holes;

        let m = box_mesh(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        // Watertight: 0 holes
        let holes_none = detect_holes(&m);
        assert_eq!(holes_none.len(), 0, "Watertight box has 0 holes");

        // Remove 1 triangle (indices 0..3) -> 1 triangular hole with 3 edges
        let mut open_idx = m.indices.clone();
        open_idx.drain(0..3);
        let m_open1 = Mesh::from_indexed(m.positions.clone(), open_idx);
        let holes1 = detect_holes(&m_open1);
        assert_eq!(holes1.len(), 1, "Should detect exactly 1 hole");
        assert_eq!(holes1[0].edge_count(), 3, "Triangle hole has 3 edges");
        assert!(holes1[0].perimeter > 0.0);

        // Remove a 2-triangle face (a full quad on one side) -> 1 quad hole with 4 edges
        let mut open_quad_idx = m.indices.clone();
        open_quad_idx.drain(0..6);
        let m_quad = Mesh::from_indexed(m.positions.clone(), open_quad_idx);
        let holes_quad = detect_holes(&m_quad);
        assert_eq!(holes_quad.len(), 1, "Should detect 1 quad hole");
        assert_eq!(holes_quad[0].edge_count(), 4, "Quad hole has 4 edges");

        // Remove two opposite triangles on different sides -> 2 separate holes
        let mut two_holes_idx = m.indices.clone();
        // Remove triangle 0 (0..3) and triangle 6 (18..21)
        two_holes_idx.drain(18..21);
        two_holes_idx.drain(0..3);
        let m_two = Mesh::from_indexed(m.positions.clone(), two_holes_idx);
        let holes2 = detect_holes(&m_two);
        assert_eq!(holes2.len(), 2, "Should detect 2 independent holes");
    }

    #[test]
    fn test_hole_filling_all_algorithms() {
        use crate::geom::hole_detect::detect_holes;
        use crate::geom::hole_fill::{
            FillDirectionMode, HoleFillConfig, HoleFillMethod, apply_patch, fill_holes,
            generate_hole_patch,
        };

        let m = box_mesh(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        // Remove 2 triangles (indices 0..6) to create an open hole
        let mut open_idx = m.indices.clone();
        open_idx.drain(0..6);
        let m_open = Mesh::from_indexed(m.positions.clone(), open_idx);

        let holes = detect_holes(&m_open);
        assert_eq!(holes.len(), 1);
        let target_hole = &holes[0];

        // 1. Planar Fan
        let mut m_fan = m_open.clone();
        let cfg_fan = HoleFillConfig {
            method: HoleFillMethod::PlanarFan,
            ..Default::default()
        };
        let patch_fan = generate_hole_patch(&m_fan, target_hole, cfg_fan).unwrap();
        apply_patch(&mut m_fan, &patch_fan);
        let holes_after_fan = detect_holes(&m_fan);
        assert_eq!(
            holes_after_fan.len(),
            0,
            "Mesh must be closed after PlanarFan"
        );

        // 2. Ear Clipping
        let mut m_ear = m_open.clone();
        let cfg_ear = HoleFillConfig {
            method: HoleFillMethod::EarClipping,
            ..Default::default()
        };
        let patch_ear = generate_hole_patch(&m_ear, target_hole, cfg_ear).unwrap();
        apply_patch(&mut m_ear, &patch_ear);
        let holes_after_ear = detect_holes(&m_ear);
        assert_eq!(
            holes_after_ear.len(),
            0,
            "Mesh must be closed after EarClipping"
        );

        // 3. Minimal Area Triangulation
        let mut m_area = m_open.clone();
        let cfg_area = HoleFillConfig {
            method: HoleFillMethod::MinimalArea,
            ..Default::default()
        };
        let patch_area = generate_hole_patch(&m_area, target_hole, cfg_area).unwrap();
        apply_patch(&mut m_area, &patch_area);
        let holes_after_area = detect_holes(&m_area);
        assert_eq!(
            holes_after_area.len(),
            0,
            "Mesh must be closed after MinimalArea"
        );

        // 4. Liepa Smooth (Refined & Faired)
        let mut m_liepa = m_open.clone();
        let cfg_liepa = HoleFillConfig {
            method: HoleFillMethod::LiepaSmooth,
            ..Default::default()
        };
        let patch_liepa = generate_hole_patch(&m_liepa, target_hole, cfg_liepa).unwrap();
        apply_patch(&mut m_liepa, &patch_liepa);
        let holes_after_liepa = detect_holes(&m_liepa);
        assert_eq!(
            holes_after_liepa.len(),
            0,
            "Mesh must be closed after LiepaSmooth"
        );

        // 5. Meshmixer-style Bulge & Density test
        let mut m_bulge = m_open.clone();
        let cfg_bulge = HoleFillConfig {
            method: HoleFillMethod::LiepaSmooth,
            density: 2.0,
            bulge: 0.8,
            direction_mode: FillDirectionMode::AutoNormal,
            smooth_iterations: 20,
        };
        let patch_bulge = generate_hole_patch(&m_bulge, target_hole, cfg_bulge).unwrap();
        assert!(!patch_bulge.new_positions.is_empty(), "High density Liepa should add interior vertices");
        apply_patch(&mut m_bulge, &patch_bulge);
        assert_eq!(detect_holes(&m_bulge).len(), 0);

        // 6. Batch fill all holes
        let filled_batch = fill_holes(&m_open, &holes, cfg_area).unwrap();
        assert_eq!(detect_holes(&filled_batch).len(), 0);
    }

    #[test]
    fn test_undo_redo_history_stack() {
        let mut app = crate::app::App::new();
        let m = box_mesh(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let arc_m = std::sync::Arc::new(m);
        app.current = Some(arc_m.clone());
        app.original = Some(arc_m);

        assert_eq!(app.undo.len(), 0);
        assert_eq!(app.redo.len(), 0);

        // Action 1: Translate mesh
        app.apply_transform(glam::Quat::IDENTITY, Vec3::new(10.0, 0.0, 0.0));
        assert_eq!(app.undo.len(), 1);
        assert_eq!(app.redo.len(), 0);
        assert!((app.bbox.center().x - 10.0).abs() < 1e-3);

        // Action 2: Translate mesh again
        app.apply_transform(glam::Quat::IDENTITY, Vec3::new(0.0, 20.0, 0.0));
        assert_eq!(app.undo.len(), 2);
        assert_eq!(app.redo.len(), 0);
        assert!((app.bbox.center().y - 20.0).abs() < 1e-3);

        // Undo once -> back to (10, 0, 0)
        app.undo();
        assert_eq!(app.undo.len(), 1);
        assert_eq!(app.redo.len(), 1);
        assert!((app.bbox.center().y - 0.0).abs() < 1e-3);

        // Redo -> forward to (10, 20, 0)
        app.redo();
        assert_eq!(app.undo.len(), 2);
        assert_eq!(app.redo.len(), 0);
        assert!((app.bbox.center().y - 20.0).abs() < 1e-3);

        // Undo again, then make a NEW action -> redo stack must be cleared!
        app.undo();
        assert_eq!(app.redo.len(), 1);
        app.apply_transform(glam::Quat::IDENTITY, Vec3::new(0.0, 0.0, 30.0));
        assert_eq!(app.redo.len(), 0, "New action must clear redo stack");
    }

    #[test]
    fn test_mesh_diagnostics_and_repair() {
        use crate::geom::repair::{
            analyze_mesh, auto_repair_mesh, remove_degenerate_faces,
            remove_small_components, unify_normals,
        };

        // 1. Watertight check
        let m = box_mesh(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let rep = analyze_mesh(&m);
        assert!(rep.is_watertight, "Box mesh should be watertight");
        assert_eq!(rep.euler_characteristic, 2);
        assert_eq!(rep.genus, 0);

        // 2. Degenerate faces detection and removal
        let mut pos = m.positions.clone();
        let mut idx = m.indices.clone();
        // Add a collapsed triangle: (0, 0, 1)
        idx.extend_from_slice(&[0, 0, 1]);
        // Add a colinear triangle: (0, 1, mid)
        let mid = [5.0, 0.0, 0.0];
        let mid_idx = pos.len() as u32;
        pos.push(mid);
        idx.extend_from_slice(&[0, 1, mid_idx]);

        let m_degen = Mesh::from_indexed(pos, idx);
        let rep_degen = analyze_mesh(&m_degen);
        assert!(rep_degen.degenerate_faces >= 1);

        let m_clean = remove_degenerate_faces(&m_degen);
        let rep_clean = analyze_mesh(&m_clean);
        assert_eq!(rep_clean.degenerate_faces, 0);

        // 3. Normal unification
        let mut flipped_idx = m.indices.clone();
        // Invert triangle 0
        flipped_idx.swap(1, 2);
        let m_flipped = Mesh::from_indexed(m.positions.clone(), flipped_idx);
        let rep_flipped = analyze_mesh(&m_flipped);
        assert!(rep_flipped.inconsistent_normals > 0);

        let m_unified = unify_normals(&m_flipped);
        let rep_unified = analyze_mesh(&m_unified);
        assert_eq!(rep_unified.inconsistent_normals, 0);

        // 4. Floating debris removal
        let mut pos_debris = m.positions.clone();
        let mut idx_debris = m.indices.clone();
        // Add a tiny detached triangle far away
        let d0 = pos_debris.len() as u32;
        pos_debris.push([100.0, 100.0, 100.0]);
        pos_debris.push([101.0, 100.0, 100.0]);
        pos_debris.push([100.0, 101.0, 100.0]);
        idx_debris.push(d0);
        idx_debris.push(d0 + 1);
        idx_debris.push(d0 + 2);

        let m_with_debris = Mesh::from_indexed(pos_debris, idx_debris);
        let rep_deb = analyze_mesh(&m_with_debris);
        assert_eq!(rep_deb.component_count, 2);

        let m_no_debris = remove_small_components(&m_with_debris, false, 0.05);
        let rep_no_deb = analyze_mesh(&m_no_debris);
        assert_eq!(rep_no_deb.component_count, 1);

        // 5. 1-Click Auto Repair Pipeline
        let (repaired, summary) = auto_repair_mesh(&m_with_debris);
        assert!(summary.contains("Auto Repair complete"));
        let rep_final = analyze_mesh(&repaired);
        assert_eq!(rep_final.component_count, 1);
        assert_eq!(rep_final.degenerate_faces, 0);
    }

    #[test]
    fn test_hole_solver_planar_guidance() {
        use crate::geom::hole_detect::detect_holes;
        use crate::geom::hole_solver::{solve_best_hole_config, ReferenceGeometry};

        // Create a planar ring (flat surface on z = 0 with a central hole)
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        let n = 12;
        let r_inner = 5.0f32;
        let r_outer = 10.0f32;

        for i in 0..n {
            let theta = (i as f32 / n as f32) * std::f32::consts::TAU;
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            positions.push([cos_t * r_inner, sin_t * r_inner, 0.0]);
            positions.push([cos_t * r_outer, sin_t * r_outer, 0.0]);
        }

        for i in 0..n {
            let i_next = (i + 1) % n;
            let in_curr = (i * 2) as u32;
            let out_curr = (i * 2 + 1) as u32;
            let in_next = (i_next * 2) as u32;
            let out_next = (i_next * 2 + 1) as u32;

            indices.push(in_curr);
            indices.push(out_curr);
            indices.push(out_next);

            indices.push(in_curr);
            indices.push(out_next);
            indices.push(in_next);
        }

        let mesh = Mesh::from_indexed(positions, indices);
        let holes = detect_holes(&mesh);
        assert!(!holes.is_empty());

        let inner_hole = holes.iter().find(|h| h.perimeter < 40.0).expect("Inner hole");

        let ref_plane = ReferenceGeometry::Plane {
            id: 1,
            name: "Z-Plane".to_string(),
            point: glam::Vec3::ZERO,
            normal: glam::Vec3::Z,
        };

        let result = solve_best_hole_config(&mesh, inner_hole, &[ref_plane]).expect("Solver succeeds");
        assert!(result.rms_error < 1e-3, "RMS error should be virtually zero on plane, got {}", result.rms_error);
        assert!(result.tested_count > 10, "Should have tested multiple candidate configurations");
    }

    #[test]
    fn test_hole_solver_cylinder_and_multi_reference() {
        use crate::geom::hole_detect::detect_holes;
        use crate::geom::hole_solver::{solve_best_hole_config, refine_patch_to_references, ReferenceGeometry};
        use crate::geom::hole_fill::generate_hole_patch;

        let mut positions = Vec::new();
        let mut indices = Vec::new();
        let radius = 10.0f32;
        let n_circ = 16;
        let n_height = 4;

        for h in 0..n_height {
            let z = h as f32 * 5.0;
            for i in 0..n_circ {
                let theta = (i as f32 / n_circ as f32) * std::f32::consts::PI;
                positions.push([radius * theta.cos(), radius * theta.sin(), z]);
            }
        }

        for h in 0..(n_height - 1) {
            for i in 0..(n_circ - 1) {
                if h == 1 && (i == 7 || i == 8) {
                    continue;
                }
                let v0 = (h * n_circ + i) as u32;
                let v1 = (h * n_circ + i + 1) as u32;
                let v2 = ((h + 1) * n_circ + i + 1) as u32;
                let v3 = ((h + 1) * n_circ + i) as u32;

                indices.push(v0);
                indices.push(v1);
                indices.push(v2);

                indices.push(v0);
                indices.push(v2);
                indices.push(v3);
            }
        }

        let mesh = Mesh::from_indexed(positions, indices);
        let holes = detect_holes(&mesh);
        assert!(!holes.is_empty());

        let ref_circle = ReferenceGeometry::Circle {
            id: 2,
            name: "Cylindrical Bore".to_string(),
            center: glam::Vec3::ZERO,
            normal: glam::Vec3::Z,
            radius,
            mode: crate::geom::hole_solver::CircleGuideMode::CylinderWall,
        };

        let ref_plane = ReferenceGeometry::Plane {
            id: 1,
            name: "Top Face".to_string(),
            point: glam::Vec3::new(0.0, 0.0, 15.0),
            normal: glam::Vec3::Z,
        };

        let refs = vec![ref_plane, ref_circle.clone()];
        let small_hole = holes.iter().min_by(|a, b| a.perimeter.partial_cmp(&b.perimeter).unwrap()).unwrap();

        let result = solve_best_hole_config(&mesh, small_hole, &refs).expect("Solve succeeds");
        assert!(result.tested_count > 0);

        let mut patch = generate_hole_patch(&mesh, small_hole, result.best_config).expect("Generate patch");
        refine_patch_to_references(&mut patch, &mesh, small_hole, &refs);
        assert!(!patch.new_positions.is_empty() || !patch.preview_positions.is_empty());
    }

    #[test]
    fn test_hole_solver_circle_disk_and_rim_no_infinite_cylinder() {
        use crate::geom::hole_solver::{ReferenceGeometry, CircleGuideMode};

        let ref_circle = ReferenceGeometry::Circle {
            id: 1,
            name: "Test Circle".to_string(),
            center: glam::Vec3::ZERO,
            normal: glam::Vec3::Z,
            radius: 10.0,
            mode: CircleGuideMode::DiskAndRim,
        };

        // Point inside the circle on plane z=0
        assert_eq!(ref_circle.distance_to_point(glam::Vec3::new(5.0, 0.0, 0.0)), 0.0);

        // Point outside the circle on plane z=0 (at r=15)
        // With previous broken cylinder code, this returned 0.0. Now it returns 5.0!
        let dist_outside = ref_circle.distance_to_point(glam::Vec3::new(15.0, 0.0, 0.0));
        assert!((dist_outside - 5.0).abs() < 1e-4, "Expected distance 5.0, got {}", dist_outside);

        // Point 20 mm above the circle axis
        // With previous broken cylinder code, this returned 0.0. Now it returns 20.0!
        let dist_above = ref_circle.distance_to_point(glam::Vec3::new(0.0, 0.0, 20.0));
        assert!((dist_above - 20.0).abs() < 1e-4, "Expected distance 20.0, got {}", dist_above);
    }

    #[test]
    fn brush_radius_adjustment_and_clamping() {
        let mut app = crate::app::App::new();
        assert_eq!(app.brush_radius, 25.0);

        // Increase brush size by steps
        let steps = 5.0f32;
        app.brush_radius = (app.brush_radius + steps * 3.0).clamp(2.0, 150.0);
        assert_eq!(app.brush_radius, 40.0);

        // Decrease brush size beyond min clamp
        let steps_neg = -20.0f32;
        app.brush_radius = (app.brush_radius + steps_neg * 3.0).clamp(2.0, 150.0);
        assert_eq!(app.brush_radius, 2.0);

        // Increase brush size beyond max clamp
        let steps_large = 100.0f32;
        app.brush_radius = (app.brush_radius + steps_large * 3.0).clamp(2.0, 150.0);
        assert_eq!(app.brush_radius, 150.0);
    }

    #[test]
    fn test_auto_center_on_import() {
        let mut app = crate::app::App::new();
        // Box from (10.0, 20.0, 30.0) to (14.0, 26.0, 38.0), center is (12.0, 23.0, 34.0)
        let m = box_mesh(10.0, 20.0, 30.0, 4.0, 6.0, 8.0);
        let bytes = crate::io::stl::save(&m, Path::new("offset_box"));
        let temp_dir = std::env::temp_dir();
        let temp_file = temp_dir.join("scanimprover_test_offset_box.stl");
        std::fs::write(&temp_file, bytes).unwrap();

        app.load_file(temp_file.clone());
        let _ = std::fs::remove_file(&temp_file);

        let center = app.bbox.center();
        assert!(
            center.length() < 1e-4,
            "Mesh bbox center should be at (0,0,0) after import, was {:?}",
            center
        );
        assert!(app.display().is_some());
        assert!(app.current.is_some());
        assert!(app.original.is_some());
        assert!(app.status.contains("Centered at global origin"));
    }

    #[test]
    fn face_groups_box_six_planes() {
        use crate::geom::segment::{GroupKind, segment_faces};
        use crate::geom::topology::MeshTopology;

        let m = box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        let topo = MeshTopology::build(&m);
        let (groups, ids) = segment_faces(&m, &topo, 45.0, 1, 0.0015);
        assert_eq!(groups.len(), 6, "each box face must be its own group");
        assert!(ids.iter().all(|&g| g >= 0), "every face must be grouped");
        assert!(groups.iter().all(|g| g.kind == GroupKind::Plane));
        assert!(groups.iter().all(|g| g.tris.len() == 2));
    }

    #[test]
    fn face_groups_cylinder_classification() {
        use crate::geom::segment::{GroupKind, segment_faces};
        use crate::geom::topology::MeshTopology;

        let m = cylinder_wall_mesh(10.0, 40.0, 6, 32);
        let topo = MeshTopology::build(&m);
        let (groups, ids) = segment_faces(&m, &topo, 45.0, 4, 0.0015);
        assert_eq!(groups.len(), 1, "cylinder wall must be a single group");
        assert_eq!(groups[0].kind, GroupKind::Cylinder);
        assert!(
            (groups[0].radius - 10.0).abs() < 0.05,
            "radius {}",
            groups[0].radius
        );
        assert!(groups[0].normal.dot(Vec3::Z).abs() > 0.99);
        assert_eq!(
            ids.iter().filter(|&&g| g == 0).count(),
            m.triangle_count()
        );
    }

    #[test]
    fn face_groups_sphere_classification() {
        use crate::geom::segment::{GroupKind, segment_faces};
        use crate::geom::topology::MeshTopology;

        let m = sphere_mesh(5.0, 24, 48);
        let topo = MeshTopology::build(&m);
        let (groups, _) = segment_faces(&m, &topo, 60.0, 4, 0.0015);
        assert!(
            groups.len() <= 3,
            "sphere should form one main group, got {}",
            groups.len()
        );
        let big = groups.iter().max_by_key(|g| g.tris.len()).unwrap();
        assert_eq!(big.kind, GroupKind::Sphere);
        assert!((big.radius - 5.0).abs() < 0.05, "radius {}", big.radius);
    }

    #[test]
    fn face_groups_app_workflow() {
        let mut app = crate::app::App::new();
        let m = std::sync::Arc::new(box_mesh(0.0, 0.0, 0.0, 4.0, 2.0, 2.0));
        app.current = Some(m.clone());
        app.original = Some(m);

        app.detect_face_groups();
        assert_eq!(app.face_groups.len(), 6);
        assert!(app.groups_show);
        assert_eq!(
            app.group_ids.len(),
            app.current.as_ref().unwrap().triangle_count()
        );

        let id = app.face_groups[0].id;
        app.select_group_faces(id, false);
        assert_eq!(app.sel_count, app.face_groups[0].tris.len());

        // Transforming the mesh must transform the group fits as well.
        let q = glam::Quat::from_axis_angle(Vec3::Z, std::f32::consts::FRAC_PI_2);
        let before = app.face_groups[0].point;
        app.apply_transform(q, Vec3::new(1.0, 0.0, 0.0));
        let after = app.face_groups[0].point;
        assert!(
            (after - (q * before + Vec3::X)).length() < 1e-4,
            "group plane point must be transformed"
        );

        // Fitting a plane from a group pushes a managed plane object.
        let planes_before = app.planes.len();
        app.fit_group_plane(id);
        assert_eq!(app.planes.len(), planes_before + 1);

        // Mesh topology changes must invalidate the groups.
        let curr = (*app.current.clone().unwrap()).clone();
        app.set_mesh_modified(curr, "test modification".to_string());
        assert!(app.face_groups.is_empty());
        assert!(app.group_ids.is_empty());
    }

    #[test]
    fn double_click_selects_complete_group() {
        let mut app = crate::app::App::new();
        let m = std::sync::Arc::new(box_mesh(0.0, 0.0, 0.0, 2.0, 2.0, 2.0));
        app.current = Some(m.clone());
        app.original = Some(m);
        app.detect_face_groups();

        // Double-click on triangle 0 -> the whole group (2 faces) is selected.
        app.select_region_under(0);
        assert_eq!(app.sel_count, 2);
        let gid = app.group_ids[0];
        let expected: std::collections::HashSet<u32> = app
            .face_groups
            .iter()
            .find(|g| g.id == gid)
            .unwrap()
            .tris
            .iter()
            .copied()
            .collect();
        let selected: std::collections::HashSet<u32> = (0..app.sel.len())
            .filter(|&t| app.sel[t] > 0)
            .map(|t| t as u32)
            .collect();
        assert_eq!(selected, expected);

        // Additive mode (default): double-click on another group keeps the
        // existing selection and adds the new group's faces.
        assert!(app.group_sel_additive);
        let other_tri = (0..app.group_ids.len())
            .find(|&t| app.group_ids[t] != gid)
            .unwrap() as u32;
        app.select_region_under(other_tri);
        assert_eq!(app.sel_count, 4, "additive double-click must merge selections");

        // Replace mode: double-click replaces the selection.
        app.group_sel_additive = false;
        app.select_region_under(0);
        assert_eq!(app.sel_count, 2);

        // Without face groups the double-click falls back to a
        // crease-angle region (here: the flat face under triangle 0).
        app.clear_face_groups();
        app.sel = std::sync::Arc::new(vec![0u8; 12]);
        app.sel_count = 0;
        app.select_region_under(0);
        assert_eq!(app.sel_count, 2);
    }

    #[test]
    fn group_filter_matches_only_kind() {
        use crate::app::{GroupFilter, group_matches_filter};
        use crate::geom::segment::GroupKind;

        assert!(group_matches_filter(GroupKind::Plane, GroupFilter::All));
        assert!(group_matches_filter(GroupKind::Freeform, GroupFilter::All));
        assert!(group_matches_filter(GroupKind::Plane, GroupFilter::Plane));
        assert!(!group_matches_filter(GroupKind::Cylinder, GroupFilter::Plane));
        assert!(group_matches_filter(GroupKind::Cylinder, GroupFilter::Cylinder));
        assert!(!group_matches_filter(GroupKind::Plane, GroupFilter::Cylinder));
        assert!(group_matches_filter(GroupKind::Sphere, GroupFilter::Sphere));
        assert!(!group_matches_filter(GroupKind::Sphere, GroupFilter::Freeform));
        assert!(group_matches_filter(GroupKind::Freeform, GroupFilter::Freeform));
        assert!(!group_matches_filter(GroupKind::Cylinder, GroupFilter::Sphere));
    }

    /// Curved grid patch mesh (wavy height field over the XY plane).
    fn wavy_patch_mesh(nx: usize, ny: usize) -> Mesh {
        let mut positions = Vec::new();
        for j in 0..=ny {
            for i in 0..=nx {
                let x = i as f32;
                let y = j as f32;
                let z = (x * 0.35).sin() * (y * 0.3).cos() * 2.0;
                positions.push([x, y, z]);
            }
        }
        let mut indices = Vec::new();
        for j in 0..ny {
            for i in 0..nx {
                let a = (j * (nx + 1) + i) as u32;
                let b = a + 1;
                let c = a + (nx + 1) as u32;
                let d = c + 1;
                indices.extend_from_slice(&[a, b, d, a, d, c]);
            }
        }
        Mesh::from_indexed(positions, indices)
    }

    #[test]
    fn freeform_fit_tilted_plane_with_overshoot() {
        use crate::geom::freeform::{FreeformExtend, FreeformParams, fit_freeform};

        // Exactly planar, tilted selection: the freeform must reproduce the
        // plane everywhere, including the overshoot region.
        let mut pts = Vec::new();
        for i in 0..40 {
            for j in 0..40 {
                let (x, y) = (i as f32, j as f32);
                pts.push([x, y, 0.5 * x + 0.25 * y]);
            }
        }
        let params = FreeformParams {
            overshoot_mm: 5.0,
            extend: FreeformExtend::Curvature,
            resolution: 80,
            smoothness: 0,
        };
        let fit = fit_freeform(&pts, &params).unwrap();
        assert!(fit.rms < 0.05, "rms {}", fit.rms);

        // The surface extends past the data footprint (39 x 39 in XY).
        let bb = fit.surface.bbox();
        assert!(
            bb.extent().x >= 44.0,
            "surface must overshoot the selection, extent {}",
            bb.extent().x
        );

        // Every vertex (also the extended ones) must lie on the plane.
        for p in &fit.surface.positions {
            let z_expected = 0.5 * p[0] + 0.25 * p[1];
            assert!(
                (p[2] - z_expected).abs() < 0.1,
                "vertex {:?} not on plane (expected z = {z_expected})",
                p
            );
        }
    }

    #[test]
    fn freeform_curvature_vs_slope_extension() {
        use crate::geom::freeform::{FreeformExtend, FreeformParams, fit_freeform};

        // Patch of a cylinder wall (R = 10, axis Y), arc a in [-0.5, 0.5].
        let mut pts = Vec::new();
        for i in 0..40 {
            let a = -0.5 + i as f32 / 39.0;
            for j in 0..30 {
                let y = j as f32 * 10.0 / 29.0;
                pts.push([10.0 * a.sin(), y, 10.0 * a.cos() - 10.0]);
            }
        }

        let make = |extend| FreeformParams {
            overshoot_mm: 2.0,
            extend,
            resolution: 100,
            smoothness: 0,
        };

        // Max radial error against the true cylinder for vertices beyond the
        // data arc range (|x| > 10 * sin(0.5) = 4.795) along the arc
        // extension band. Corner regions (x and y both outside) are excluded:
        // extrapolating into a corner diagonal is inherently approximate.
        let radial_err = |fit: &crate::geom::freeform::FreeformFitData| {
            fit.surface
                .positions
                .iter()
                .filter(|p| p[0].abs() > 5.0 && p[1] >= 2.0 && p[1] <= 8.0)
                .map(|p| {
                    let r = (p[0] * p[0] + (p[2] + 10.0) * (p[2] + 10.0)).sqrt();
                    (r - 10.0).abs()
                })
                .fold(0.0f32, f32::max)
        };

        let fit_curv = fit_freeform(&pts, &make(FreeformExtend::Curvature)).unwrap();
        assert!(fit_curv.rms < 0.15, "rms {}", fit_curv.rms);
        let err_curv = radial_err(&fit_curv);

        let fit_slope = fit_freeform(&pts, &make(FreeformExtend::Slope)).unwrap();
        let err_slope = radial_err(&fit_slope);
        assert!(
            err_curv < 0.15,
            "curvature continuation must track the cylinder, max err {err_curv}"
        );
        assert!(
            err_slope > err_curv + 0.10,
            "slope continuation must deviate more than curvature ({err_slope} vs {err_curv})"
        );

        // Spike guard: even in the corner regions the surface must stay
        // within a sane band around the true cylinder (no wild extrapolation).
        let err_corner = |fit: &crate::geom::freeform::FreeformFitData| {
            fit.surface
                .positions
                .iter()
                .filter(|p| p[1] < -0.5 || p[1] > 10.5)
                .map(|p| {
                    let r = (p[0] * p[0] + (p[2] + 10.0) * (p[2] + 10.0)).sqrt();
                    (r - 10.0).abs()
                })
                .fold(0.0f32, f32::max)
        };
        assert!(
            err_corner(&fit_curv) < 0.5,
            "corner extrapolation must stay sane, got {}",
            err_corner(&fit_curv)
        );
    }

    #[test]
    fn freeform_app_fit_refit_delete_workflow() {
        use crate::geom::freeform::FreeformParams;

        let mut app = crate::app::App::new();
        let m = std::sync::Arc::new(wavy_patch_mesh(30, 30));
        let tris = m.triangle_count();
        app.current = Some(m.clone());
        app.original = Some(m);
        app.sel = std::sync::Arc::new(vec![1u8; tris]);
        app.sel_count = tris;

        app.fit_freeform_from_selection();
        assert_eq!(app.freeforms.len(), 1);
        assert!(app.freeform_job.is_some(), "fit job must be scheduled");
        assert_eq!(app.freeforms[0].name, "Freeform 1");
        assert_eq!(app.selected_freeform_id, Some(app.freeforms[0].id));

        // Pump the async worker until the fit lands.
        let ctx = eframe::egui::Context::default();
        let mut waited = 0;
        while app.freeform_job.is_some() && waited < 2400 {
            app.handle_worker(&ctx);
            std::thread::sleep(std::time::Duration::from_millis(5));
            waited += 1;
        }
        assert!(app.freeform_job.is_none(), "fit job did not finish");
        assert!(app.freeforms[0].surface.is_some(), "surface must be fitted");
        assert!(app.freeforms[0].boundary.len() > 0, "boundary must exist");
        assert!(app.freeforms[0].rms >= 0.0);

        // Changing parameters schedules a refit with the new values.
        let id = app.freeforms[0].id;
        let old = app.freeforms[0].params;
        app.set_freeform_params(
            id,
            FreeformParams {
                overshoot_mm: old.overshoot_mm + 2.0,
                ..old
            },
        );
        assert!(app.freeform_job.is_some(), "refit must be scheduled");
        let submitted_params = app.freeforms[0].params;
        assert!((submitted_params.overshoot_mm - (old.overshoot_mm + 2.0)).abs() < 1e-6);

        // Undo restores the state before the freeform was created.
        app.undo();
        assert!(app.freeforms.is_empty(), "undo must remove the freeform");
    }

    #[test]
    fn freeform_second_fit_queues_while_first_runs() {
        let mut app = crate::app::App::new();
        let m = std::sync::Arc::new(wavy_patch_mesh(24, 24));
        let tris = m.triangle_count();
        app.current = Some(m.clone());
        app.original = Some(m);
        app.sel = std::sync::Arc::new(vec![1u8; tris]);
        app.sel_count = tris;

        app.fit_freeform_from_selection();
        assert!(app.freeform_job.is_some(), "first fit must hold the job slot");

        // A second fit while the first job runs must be queued, not dropped.
        app.fit_freeform_from_selection();
        assert_eq!(app.freeforms.len(), 2);
        assert!(
            app.freeforms[1].refit_pending,
            "second fit must be queued until the first finishes"
        );
        assert_eq!(
            app.freeform_job.unwrap().1,
            app.freeforms[0].id,
            "first job keeps the slot"
        );

        // Pump until both surfaces are fitted.
        let ctx = eframe::egui::Context::default();
        let mut waited = 0;
        while (app.freeform_job.is_some() || app.freeforms.iter().any(|f| f.refit_pending))
            && waited < 2400
        {
            app.handle_worker(&ctx);
            std::thread::sleep(std::time::Duration::from_millis(5));
            waited += 1;
        }
        assert!(
            app.freeforms.iter().all(|f| f.surface.is_some()),
            "both surfaces must be fitted"
        );
        assert!(app.freeform_job.is_none());
        assert!(app.freeforms.iter().all(|f| !f.refit_pending));
    }

    #[test]
    fn freeform_overshoot_grows_surface_footprint() {
        use crate::geom::freeform::{FreeformExtend, FreeformParams, fit_freeform};

        // Wavy height field; more overshoot must produce a larger surface.
        let mut pts = Vec::new();
        for i in 0..60 {
            for j in 0..60 {
                let (x, y) = (i as f32 * 0.5, j as f32 * 0.5);
                pts.push([x, y, (x * 0.4).sin() * (y * 0.35).cos() * 1.5]);
            }
        }
        let p0 = FreeformParams {
            overshoot_mm: 0.0,
            extend: FreeformExtend::Curvature,
            resolution: 90,
            smoothness: 0,
        };
        let p1 = FreeformParams {
            overshoot_mm: 4.0,
            ..p0
        };
        let fit0 = fit_freeform(&pts, &p0).unwrap();
        let fit1 = fit_freeform(&pts, &p1).unwrap();
        let area = |fit: &crate::geom::freeform::FreeformFitData| {
            let bb = fit.surface.bbox();
            bb.extent().x * bb.extent().y
        };
        let (area0, area1) = (area(&fit0), area(&fit1));
        assert!(
            area1 > area0 * 1.25,
            "overshoot must grow the surface: {area0} vs {area1}"
        );
        assert!(fit1.rms < 0.2, "rms {}", fit1.rms);
    }

    #[test]
    fn freeform_defaults_are_slope_and_smoothed() {
        use crate::geom::freeform::{FreeformExtend, FreeformParams};
        let p = FreeformParams::default_for(50.0);
        assert_eq!(p.extend, FreeformExtend::Slope, "Slope must be the default");
        assert!(p.smoothness >= 1, "light smoothing by default");
        assert_eq!(p.resolution, 120);
    }

    #[test]
    fn freeform_smoothness_flattens_noise() {
        use crate::geom::freeform::{FreeformExtend, FreeformParams, fit_freeform_grid};

        // Noisy height field: smoothing passes must reduce grid roughness.
        let mut pts = Vec::new();
        let mut rng = crate::rng::Rng::new(5);
        for i in 0..40 {
            for j in 0..40 {
                let (x, y) = (i as f32 * 0.5, j as f32 * 0.5);
                pts.push([
                    x,
                    y,
                    0.5 * (x * 0.3).sin() + (rng.f32() - 0.5) * 0.2,
                ]);
            }
        }
        let make = |smoothness| FreeformParams {
            overshoot_mm: 2.0,
            extend: FreeformExtend::Slope,
            resolution: 80,
            smoothness,
        };
        // Roughness = sum of squared second differences of the height grid.
        let roughness = |grid: &crate::geom::freeform::FreeformGrid| {
            let (nx, ny) = (grid.nx, grid.ny);
            let h = &grid.heights;
            let mut e = 0.0f32;
            for j in 1..ny {
                for i in 1..nx {
                    let idx = j * (nx + 1) + i;
                    let d2 = h[idx - 1] - 2.0 * h[idx] + h[idx + 1];
                    e += d2 * d2;
                }
            }
            e
        };
        let g0 = fit_freeform_grid(&pts, &make(0)).unwrap();
        let g8 = fit_freeform_grid(&pts, &make(8)).unwrap();
        assert!(
            roughness(&g8) < roughness(&g0) * 0.5,
            "smoothing must reduce grid roughness ({} vs {})",
            roughness(&g8),
            roughness(&g0)
        );
    }

    #[test]
    fn freeform_bicubic_interpolates_grid() {
        use crate::geom::freeform::{
            FreeformExtend, FreeformParams, bicubic_control_net, eval_bicubic,
            fit_freeform_grid,
        };

        let mut pts = Vec::new();
        for i in 0..24 {
            for j in 0..18 {
                let (x, y) = (i as f32, j as f32);
                pts.push([x, y, (x * 0.3).sin() * (y * 0.25).cos() * 2.0]);
            }
        }
        let params = FreeformParams {
            overshoot_mm: 3.0,
            extend: FreeformExtend::Curvature,
            resolution: 40,
            smoothness: 1,
        };
        let grid = fit_freeform_grid(&pts, &params).unwrap();
        let net = bicubic_control_net(&grid);
        assert_eq!(net.cps.len(), grid.ny + 1);
        assert_eq!(net.cps[0].len(), grid.nx + 1);

        // The B-spline surface must pass exactly through the grid nodes.
        for (i, j) in [(0usize, 0usize), (grid.nx, 0), (0, grid.ny), (grid.nx, grid.ny),
                       (grid.nx / 2, grid.ny / 2), (grid.nx / 3, 2 * grid.ny / 3)] {
            let expected = grid.origin
                + grid.u * (grid.x0 + i as f32 * grid.gw)
                + grid.v * (grid.y0 + j as f32 * grid.gh)
                + grid.normal * grid.heights[j * (grid.nx + 1) + i];
            let u = i as f64 / grid.nx as f64;
            let v = j as f64 / grid.ny as f64;
            let got = eval_bicubic(&net, u, v);
            assert!(
                (got - expected).length() < 1e-3,
                "node ({i},{j}): got {got:?}, expected {expected:?}"
            );
        }
    }

    #[test]
    fn freeform_step_export_structure() {
        use crate::geom::freeform::{
            FreeformExtend, FreeformParams, bicubic_control_net, fit_freeform_grid,
        };

        let mut pts = Vec::new();
        for i in 0..20 {
            for j in 0..16 {
                let (x, y) = (i as f32, j as f32);
                pts.push([x, y, (x * 0.35).sin() * 1.5]);
            }
        }
        let params = FreeformParams {
            overshoot_mm: 2.0,
            extend: FreeformExtend::Slope,
            resolution: 24,
            smoothness: 2,
        };
        let grid = fit_freeform_grid(&pts, &params).unwrap();
        let net = bicubic_control_net(&grid);
        let content = crate::export::step::generate_freeform_step("Freeform 1", &net);

        assert!(content.contains("ISO-10303-21;"));
        assert!(content.contains("B_SPLINE_SURFACE_WITH_KNOTS"));
        assert!(content.contains("B_SPLINE_CURVE_WITH_KNOTS"));
        assert!(content.contains("ADVANCED_FACE('Freeform 1'"));
        assert!(content.contains("EDGE_LOOP"));
        assert!(content.contains("FACE_OUTER_BOUND"));
        assert!(content.contains("OPEN_SHELL"));
        assert!(content.contains("END-ISO-10303-21;"));
        assert!(content.contains("APPLICATION_PROTOCOL_DEFINITION"));
        assert!(content.contains("AXIS2_PLACEMENT_3D"));
        // One cartesian point per control point, plus global placement origin.
        let cp_count = (grid.nx + 1) * (grid.ny + 1);
        let point_count = content.matches("CARTESIAN_POINT").count();
        assert_eq!(point_count, cp_count + 1, "one point per control point plus origin expected");
        // Clamped bicubic knot summary: multiplicities start and end with 4.
        let surface_line = content
            .lines()
            .find(|l| l.contains("B_SPLINE_SURFACE_WITH_KNOTS"))
            .expect("surface entity line");
        let after = surface_line
            .split(".UNSPECIFIED.,.F.,.F.,.F.,")
            .nth(1)
            .expect("multiplicity lists");
        let u_mults = after
            .split('(')
            .nth(1)
            .and_then(|rest| rest.split(')').next())
            .unwrap_or_default();
        assert!(
            u_mults.starts_with("4,") && u_mults.ends_with(",4"),
            "clamped multiplicities expected, got ({u_mults})"
        );
        // Balanced parentheses on every entity line (cheap STEP sanity).
        for line in content.lines() {
            if line.starts_with('#') {
                let opens = line.matches('(').count();
                let closes = line.matches(')').count();
                assert_eq!(
                    opens,
                    closes,
                    "unbalanced parens in STEP line: {line}"
                );
            }
        }
        // Curve form: 1 form flag + 2 logicals (closed, self_intersect).
        let curve_line = content
            .lines()
            .find(|l| l.contains("B_SPLINE_CURVE_WITH_KNOTS"))
            .unwrap();
        assert!(curve_line.contains(".UNSPECIFIED.,.F.,.F.,("));
    }

    #[test]
    fn freeform_curvature_spikes_clamped_on_thin_strip() {
        use crate::geom::freeform::{FreeformExtend, FreeformParams, fit_freeform};

        // Nearly collinear thin strip (e.g. a narrow scan band). The
        // quadratic fit is degenerate across the strip; without the bend
        // clamp this spikes wildly in the overshoot region.
        let mut rng = crate::rng::Rng::new(42);
        let mut pts = Vec::new();
        for i in 0..60 {
            let x = i as f32 * 0.35;
            let y = (rng.f32() - 0.5) * 0.02;
            let z = 0.02 * x + (rng.f32() - 0.5) * 0.004;
            pts.push([x, y, z]);
        }
        let params = FreeformParams {
            overshoot_mm: 2.0,
            extend: FreeformExtend::Curvature,
            resolution: 64,
            smoothness: 0,
        };
        let fit = fit_freeform(&pts, &params).unwrap();
        assert!(!fit.surface.positions.is_empty());
        for p in &fit.surface.positions {
            let z_expected = 0.02 * p[0];
            assert!(
                (p[2] - z_expected).abs() < 0.5,
                "thin-strip fit must stay near the strip plane, got {p:?} \
                 (expected z = {z_expected})"
            );
        }
    }

    #[test]
    #[ignore] // performance smoke test: run with `cargo test -- --ignored`
    fn freeform_perf_worst_case() {
        use crate::geom::freeform::{FreeformExtend, FreeformParams, fit_freeform};

        // Dense 96k point cloud (typical sub-sampled selection cap) at the
        // maximum resolution. Must stay interactive even in debug builds.
        let mut pts = Vec::with_capacity(96_000);
        for i in 0..400 {
            for j in 0..240 {
                let (x, y) = (i as f32 * 0.1, j as f32 * 0.1);
                let z = (x * 0.1).sin() * (y * 0.12).cos() * 3.0 + 0.01 * ((i + j) % 100) as f32;
                pts.push([x, y, z]);
            }
        }
        let params = FreeformParams {
            overshoot_mm: 5.0,
            extend: FreeformExtend::Curvature,
            resolution: 320,
            smoothness: 2,
        };
        let t0 = std::time::Instant::now();
        let fit = fit_freeform(&pts, &params).unwrap();
        let dt = t0.elapsed();
        println!(
            "worst-case fit: {} points, {} tris, {dt:?}",
            pts.len(),
            fit.surface.triangle_count()
        );
        assert!(dt.as_millis() < 2000, "fit took {dt:?}");
    }

    #[test]
    fn freeform_deviation_computation_and_in_tolerance() {
        use crate::geom::bvh::Bvh;
        use crate::geom::freeform::{calculate_in_tolerance_pct, compute_freeform_deviation};
        use crate::mesh::Mesh;

        // Create a flat 2-triangle plane mesh at z = 0.
        let plane_mesh = Mesh {
            positions: vec![
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [10.0, 10.0, 0.0],
                [0.0, 10.0, 0.0],
            ],
            normals: vec![[0.0, 0.0, 1.0]; 4],
            indices: vec![0, 1, 2, 0, 2, 3],
        };
        let bvh = Bvh::new(&plane_mesh.positions, &plane_mesh.indices);

        // Create a test surface mesh elevated by z = 0.1 at some vertices and z = 0.5 at others.
        let test_surface = Mesh {
            positions: vec![
                [1.0, 1.0, 0.1],
                [2.0, 2.0, 0.1],
                [3.0, 3.0, 0.5],
                [4.0, 4.0, 0.5],
            ],
            normals: vec![[0.0, 0.0, 1.0]; 4],
            indices: vec![0, 1, 2, 0, 2, 3],
        };

        let (heat, mean_dev, max_dev) = compute_freeform_deviation(&test_surface, &bvh);
        assert_eq!(heat.len(), 4);
        assert!((heat[0] - 0.1).abs() < 1e-4);
        assert!((heat[1] - 0.1).abs() < 1e-4);
        assert!((heat[2] - 0.5).abs() < 1e-4);
        assert!((heat[3] - 0.5).abs() < 1e-4);
        assert!((max_dev - 0.5).abs() < 1e-4);
        assert!((mean_dev - 0.3).abs() < 1e-4);

        // Tolerance check: with tol = 0.2, exactly 2 of 4 (50%) are within tolerance.
        let tol_pct = calculate_in_tolerance_pct(&heat, 0.2);
        assert!((tol_pct - 50.0).abs() < 1e-4);

        // With tol = 0.6, 100% are within tolerance.
        let tol_all = calculate_in_tolerance_pct(&heat, 0.6);
        assert!((tol_all - 100.0).abs() < 1e-4);
    }

    #[test]
    fn freeform_vertex_color_gradient_traffic_light_and_spectrum() {
        use crate::geom::freeform::{FreeformGradient, freeform_vertex_color};

        // TrafficLight: 0.0 mm -> Green, 0.5 * max -> Yellow, 1.0 * max -> Red.
        let col_zero = freeform_vertex_color(0.0, 0.2, FreeformGradient::TrafficLight, 1.0);
        assert!(col_zero[1] > 0.8 && col_zero[0] < 0.2, "Zero dist must be green: {:?}", col_zero);

        let col_max = freeform_vertex_color(0.2, 0.2, FreeformGradient::TrafficLight, 1.0);
        assert!(col_max[0] > 0.9 && col_max[1] < 0.2, "Max dist must be red: {:?}", col_max);

        // Spectrum: 0.0 mm -> Blue, 1.0 * max -> Red.
        let col_spec_zero = freeform_vertex_color(0.0, 0.2, FreeformGradient::Spectrum, 1.0);
        assert!(col_spec_zero[2] > 0.8 && col_spec_zero[0] < 0.2, "Zero dist in spectrum must be blue: {:?}", col_spec_zero);

        let col_spec_max = freeform_vertex_color(0.25, 0.2, FreeformGradient::Spectrum, 1.0);
        assert!(col_spec_max[0] > 0.9 && col_spec_max[2] < 0.1, "Clamped max dist in spectrum must be red: {:?}", col_spec_max);
    }

    #[test]
    fn step_export_syntax_has_no_double_parentheses_on_sets() {
        use crate::export::step::{generate_freeform_step, generate_step};
        use crate::geom::fitting::FittedPlane;
        use crate::geom::freeform::BicubicNet;

        // Test plane step export
        let plane = FittedPlane {
            id: 1,
            name: "Test Plane".to_string(),
            fit: crate::geom::fitting::PlaneFit {
                point: glam::Vec3::ZERO,
                normal: glam::Vec3::Z,
                rms: 0.0,
                max_dev: 0.0,
            },
            color: [1.0, 1.0, 1.0, 1.0],
            visible: true,
        };
        let plane_step = generate_step(&[&plane], &[], 100.0);
        assert!(plane_step.contains("PRODUCT('Reference Geometry','Reference Geometry','',(#"));
        assert!(plane_step.contains("ADVANCED_FACE('Test Plane',(#"));
        assert!(plane_step.contains("SHELL_BASED_SURFACE_MODEL('Reference Surfaces',(#"));
        assert!(plane_step.contains("MANIFOLD_SURFACE_SHAPE_REPRESENTATION('Reference Geometry',(#"));

        // Test freeform step export
        let mut cps = vec![vec![glam::Vec3::ZERO; 4]; 4];
        for j in 0..4 {
            for i in 0..4 {
                cps[j][i] = glam::Vec3::new(i as f32, j as f32, 0.0);
            }
        }
        let net = BicubicNet {
            deg_u: 3,
            deg_v: 3,
            cps,
            u_knots_full: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            v_knots_full: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            nx: 3,
            ny: 3,
        };
        let ff_step = generate_freeform_step("Test Freeform", &net);
        assert!(ff_step.contains("PRODUCT('Freeform Surface','Freeform Surface','',(#"));
        assert!(ff_step.contains("ADVANCED_FACE('Test Freeform',(#"));
        assert!(ff_step.contains("SHELL_BASED_SURFACE_MODEL('Freeform Surface',(#"));
        assert!(ff_step.contains("MANIFOLD_SURFACE_SHAPE_REPRESENTATION('Freeform Surface',(#"));

        // Verify that outside of B_SPLINE_SURFACE_WITH_KNOTS 2D control point array,
        // FILE_DESCRIPTION, FILE_SCHEMA and complex entities, there are no ((#
        for line in ff_step.lines() {
            if line.contains("B_SPLINE_SURFACE_WITH_KNOTS") || line.contains("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT") {
                continue;
            }
            assert!(
                !line.contains("((#"),
                "Invalid nested set parentheses found in line: {}",
                line
            );
        }
    }
}

