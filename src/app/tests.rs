//! Regression tests for document state handling in `App`.

use super::App;
use crate::geom::alignment::{AxisChoice, FeatureRef};
use crate::mesh::Mesh;
use glam::{Quat, Vec3};
use std::path::PathBuf;
use std::sync::Arc;

fn box_mesh(cx: f32, cy: f32, cz: f32, s: f32) -> Mesh {
    let h = s * 0.5;
    let p = |x: f32, y: f32, z: f32| [cx + x * h, cy + y * h, cz + z * h];
    let positions = vec![
        p(-1.0, -1.0, -1.0),
        p(1.0, -1.0, -1.0),
        p(1.0, 1.0, -1.0),
        p(-1.0, 1.0, -1.0),
        p(-1.0, -1.0, 1.0),
        p(1.0, -1.0, 1.0),
        p(1.0, 1.0, 1.0),
        p(-1.0, 1.0, 1.0),
    ];
    let indices = vec![
        0, 2, 1, 0, 3, 2, // -z
        4, 5, 6, 4, 6, 7, // +z
        0, 1, 5, 0, 5, 4, // -y
        2, 3, 7, 2, 7, 6, // +y
        1, 2, 6, 1, 6, 5, // +x
        0, 4, 7, 0, 7, 3, // -x
    ];
    Mesh::from_indexed(positions, indices)
}

fn app_with(mesh: Mesh) -> App {
    let mut app = App::new();
    app.install_loaded_mesh(PathBuf::from("test.stl"), mesh);
    app
}

fn select(app: &mut App, tris: &[usize]) {
    let n = app.current.as_ref().unwrap().triangle_count();
    let mut sel = vec![0u8; n];
    for &t in tris {
        sel[t] = 1;
    }
    app.sel = Arc::new(sel);
    app.recount_sel();
    app.mark_selection_changed();
}

#[test]
fn loading_a_new_mesh_resets_the_hidden_mask() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    select(&mut app, &[0, 1]);
    app.hide_selection();
    assert_eq!(app.hidden_regions.len(), 1);
    assert!(app.is_face_hidden(0));

    // A larger mesh replaces the document: nothing may stay hidden.
    let two = box_mesh(0.0, 0.0, 0.0, 2.0).combine(&box_mesh(5.0, 0.0, 0.0, 2.0));
    app.install_loaded_mesh(PathBuf::from("two.stl"), two);
    assert!(app.hidden_regions.is_empty());
    assert!(!app.any_face_hidden());
    assert_eq!(app.hidden_mask.len(), 24);
    assert_eq!(app.sel.len(), 24);
    assert!(app.undo.is_empty());
    assert!(app.align_slots.x.is_none());
}

#[test]
fn auto_center_on_load_can_be_turned_off() {
    let mut app = App::new();
    assert!(app.auto_center_on_load, "centering stays the default");
    app.install_loaded_mesh(PathBuf::from("a.stl"), box_mesh(10.0, 0.0, -4.0, 2.0));
    assert!(app.bbox.center().length() < 1e-5);

    app.auto_center_on_load = false;
    app.install_loaded_mesh(PathBuf::from("b.stl"), box_mesh(10.0, 0.0, -4.0, 2.0));
    assert!((app.bbox.center() - Vec3::new(10.0, 0.0, -4.0)).length() < 1e-5);
    assert!(app.status.contains("Original coordinates kept"), "{}", app.status);
}

#[test]
fn decimation_preview_does_not_inherit_hidden_faces() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    select(&mut app, &[0, 1, 2]);
    app.hide_selection();
    assert!(app.any_face_hidden());

    // Simulate a decimation preview with a different triangle set.
    app.preview = Some(Arc::new(box_mesh(0.0, 0.0, 0.0, 2.0)));
    app.update_hidden_mask();
    assert!(
        !app.any_face_hidden(),
        "hidden regions index the working mesh, not the preview"
    );

    app.apply_preview();
    assert!(
        app.hidden_regions.is_empty(),
        "hidden regions cannot survive a mesh replacement"
    );
    assert!(!app.any_face_hidden());
    assert_eq!(
        app.sel.len(),
        app.current.as_ref().unwrap().triangle_count()
    );
}

#[test]
fn deleting_a_hidden_region_reindexes_the_others() {
    let two = box_mesh(0.0, 0.0, 0.0, 2.0).combine(&box_mesh(5.0, 0.0, 0.0, 2.0));
    let mut app = app_with(two);
    // Region A: first 4 faces, region B: faces 12..16 of the second box.
    select(&mut app, &[0, 1, 2, 3]);
    app.hide_selection();
    select(&mut app, &[12, 13, 14, 15]);
    app.hide_selection();
    assert_eq!(app.hidden_regions.len(), 2);
    let a_id = app.hidden_regions[0].id;

    app.delete_hidden_region(a_id);
    assert_eq!(app.current.as_ref().unwrap().triangle_count(), 20);
    assert_eq!(app.hidden_regions.len(), 1, "the other region must survive");
    assert_eq!(app.hidden_regions[0].faces, vec![8, 9, 10, 11]);
    assert!(app.is_face_hidden(8) && app.is_face_hidden(11));
    assert!(!app.is_face_hidden(7) && !app.is_face_hidden(12));
}

#[test]
fn transform_keeps_hole_and_patch_data_in_sync() {
    // Open box (top removed) has one hole.
    let mut m = box_mesh(0.0, 0.0, 0.0, 2.0);
    m.indices.truncate(30);
    let mut app = app_with(Mesh::from_indexed(m.positions.clone(), m.indices.clone()));
    app.refresh_repair();
    assert_eq!(app.repair_holes.len(), 1);
    app.select_hole(Some(0));
    assert!(app.repair_preview_patch.is_some());
    let centroid_before = app.repair_holes[0].centroid;

    let rot = Quat::from_rotation_y(0.7);
    let trans = Vec3::new(3.0, -2.0, 1.0);
    app.apply_transform(rot, trans);
    let expected = rot * centroid_before + trans;
    assert!((app.repair_holes[0].centroid - expected).length() < 1e-4);

    // Recomputing from the transformed mesh must agree with the cached data.
    let fresh = crate::geom::hole_detect::detect_holes(app.current.as_ref().unwrap());
    assert!((fresh[0].centroid - app.repair_holes[0].centroid).length() < 1e-4);
    assert!((fresh[0].bbox.min - app.repair_holes[0].bbox.min).length() < 1e-4);
    // Patch preview vertices moved with the mesh.
    let patch = app.repair_preview_patch.as_ref().unwrap();
    for p in &patch.preview_positions {
        let v = Vec3::from(*p);
        assert!(
            v.distance(fresh[0].centroid) < 3.0,
            "patch vertex left behind: {v:?}"
        );
    }
    // The view follows the model.
    assert!((app.camera.target - (rot * Vec3::ZERO + trans)).length() < 1e-4);
}

#[test]
fn bridge_cluster_cache_follows_selection_changes() {
    let two = box_mesh(0.0, 0.0, 0.0, 2.0).combine(&box_mesh(5.0, 0.0, 0.0, 2.0));
    let mut app = app_with(two);
    select(&mut app, &[0, 1]);
    assert!(app.bridge_clusters().is_err(), "one cluster is not enough");
    select(&mut app, &[0, 1, 12, 13]);
    assert!(app.has_bridge_clusters(), "two clusters across the gap");
    let cached_gen = app.sel_generation;
    assert!(
        app.bridge_cluster_cache
            .as_ref()
            .is_some_and(|c| c.sel_generation == cached_gen)
    );
    app.clear_selection();
    assert!(!app.has_bridge_clusters());
}

#[test]
fn undo_restores_alignment_slots_and_deleted_features() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    select(&mut app, &[4, 5]);
    app.fit_plane_from_selection();
    let pid = app.planes[0].id;
    app.toggle_assign_feature(FeatureRef::Plane(pid), AxisChoice::Z);
    assert_eq!(app.align_slots.z, Some(FeatureRef::Plane(pid)));

    app.delete_plane(pid);
    assert!(app.planes.is_empty());
    assert!(
        app.align_slots.z.is_none(),
        "deleting a feature clears its slot"
    );

    app.undo();
    assert_eq!(app.planes.len(), 1);
    assert_eq!(
        app.align_slots.z,
        Some(FeatureRef::Plane(pid)),
        "undo restores the slot"
    );
}

#[test]
fn brush_stroke_records_one_snapshot_only_when_something_changes() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    app.camera.target = Vec3::ZERO;
    app.camera.distance = 10.0;
    app.camera.aspect = 1.0;
    app.camera.set_view(crate::camera::ViewDir::Front);
    let bvh = app.ensure_bvh().unwrap();
    let (ro, rd) = app.camera.screen_ray(400.0, 400.0, 800.0, 800.0);
    let (t, tri) = bvh.ray_cast(ro, rd, f32::INFINITY).unwrap();
    let hit = crate::pick::Hit {
        pos: ro + rd * t,
        tri,
    };

    app.stroke_snapshot_pending = true;
    assert!(app.brush_apply(&hit, 30.0, 800.0, true));
    assert_eq!(app.undo.len(), 1, "first change of the stroke snapshots");
    assert!(!app.stroke_snapshot_pending);
    // Painting the same faces again changes nothing and must not snapshot.
    app.stroke_snapshot_pending = true;
    assert!(!app.brush_apply(&hit, 30.0, 800.0, true));
    assert_eq!(app.undo.len(), 1);
    assert!(app.sel_count > 0);
}

// ---------------------------------------------------------------------------
// Worker-backed operations
// ---------------------------------------------------------------------------

/// Polls worker results until `done` holds (or fails after ~10 s).
fn pump_until(app: &mut App, mut done: impl FnMut(&App) -> bool) {
    let ctx = egui::Context::default();
    for _ in 0..2000 {
        app.handle_worker(&ctx);
        if done(app) {
            return;
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    panic!("timed out waiting for the worker");
}

/// Small flat grid; tests lower the async threshold instead of building a
/// mesh above it.
fn grid_mesh() -> Mesh {
    let n = 12usize;
    let mut positions = Vec::with_capacity((n + 1) * (n + 1));
    for j in 0..=n {
        for i in 0..=n {
            positions.push([i as f32, j as f32, 0.0]);
        }
    }
    let mut indices = Vec::with_capacity(n * n * 6);
    for j in 0..n {
        for i in 0..n {
            let a = (j * (n + 1) + i) as u32;
            let b = a + 1;
            let c = a + (n + 1) as u32;
            let d = c + 1;
            indices.extend_from_slice(&[a, b, d, a, d, c]);
        }
    }
    Mesh::from_indexed(positions, indices)
}

#[test]
fn open_file_async_loads_on_the_worker() {
    let mesh = box_mesh(10.0, 20.0, 30.0, 4.0);
    let bytes = crate::io::stl::save(&mesh, std::path::Path::new("async_box"));
    let path = std::env::temp_dir().join("scanimprover_async_open.stl");
    std::fs::write(&path, bytes).unwrap();

    let mut app = App::new();
    app.open_file_async(path.clone());
    assert!(app.load_job.is_some());
    assert!(app.is_editing());
    assert!(!app.worker.activities().is_empty());
    pump_until(&mut app, |a| a.load_job.is_none());
    let _ = std::fs::remove_file(&path);

    assert!(app.has_mesh());
    assert!(app.bbox.center().length() < 1e-4, "loaded mesh is centered");
    assert_eq!(app.current.as_ref().unwrap().triangle_count(), 12);
    assert!(app.status.contains("Loaded"));
    assert!(app.worker.activities().is_empty());
}

#[test]
fn open_file_async_reports_missing_file() {
    let mut app = App::new();
    app.open_file_async(PathBuf::from("/definitely/not/here.stl"));
    pump_until(&mut app, |a| a.load_job.is_none());
    assert!(app.status.contains("Load failed"), "{}", app.status);
    assert!(!app.has_mesh());
}

#[test]
fn large_mesh_analysis_and_edits_run_on_the_worker() {
    let mut app = app_with(grid_mesh());
    app.async_min_tris = 0;

    // Analysis is dispatched asynchronously and lands later.
    app.request_repair_analysis();
    assert!(app.analysis_job.is_some());
    assert!(app.repair_health.is_none());
    pump_until(&mut app, |a| a.analysis_job.is_none());
    let health = app.repair_health.as_ref().expect("analysis result applied");
    assert_eq!(health.hole_count, 1, "an open grid has one boundary loop");
    assert_eq!(app.repair_holes.len(), 1);

    // A mesh edit runs on the worker, then becomes an undoable step.
    let before = app.current.as_ref().unwrap().triangle_count();
    app.request_mesh_edit(super::MeshEditKind::RemoveDegenerate);
    assert!(app.edit_job.is_some());
    assert!(app.is_editing());
    // Further edits are refused while one is running.
    app.request_mesh_edit(super::MeshEditKind::UnifyNormals);
    assert!(app.status.contains("wait"));
    pump_until(&mut app, |a| a.edit_job.is_none());
    assert_eq!(app.undo.len(), 1);
    assert_eq!(app.current.as_ref().unwrap().triangle_count(), before);
    assert!(app.status.contains("degenerate"), "{}", app.status);
}

#[test]
fn mesh_edit_result_is_discarded_when_the_mesh_changed_meanwhile() {
    let mut app = app_with(grid_mesh());
    app.async_min_tris = 0;
    app.request_mesh_edit(super::MeshEditKind::RemoveDegenerate);
    assert!(app.edit_job.is_some());
    // The document changes before the worker finishes.
    app.apply_transform(Quat::IDENTITY, Vec3::new(1.0, 0.0, 0.0));
    let undo_after_transform = app.undo.len();
    pump_until(&mut app, |a| a.edit_job.is_none());
    assert_eq!(
        app.undo.len(),
        undo_after_transform,
        "stale edit must not be applied"
    );
    assert!(app.status.contains("discarded"), "{}", app.status);
}

#[test]
fn face_group_requests_are_coalesced_while_running() {
    let mut app = app_with(grid_mesh());
    app.async_min_tris = 0;
    app.request_face_groups();
    assert!(app.groups_job.is_some());
    app.request_face_groups();
    assert!(
        app.groups_rerun_pending,
        "second request waits for the first"
    );
    pump_until(&mut app, |a| {
        a.groups_job.is_none() && !a.groups_rerun_pending
    });
    assert_eq!(
        app.face_groups.len(),
        1,
        "a flat grid is a single planar group"
    );
    assert_eq!(
        app.face_groups[0].kind,
        crate::geom::segment::GroupKind::Plane
    );
    assert_eq!(
        app.group_ids.len(),
        app.current.as_ref().unwrap().triangle_count()
    );
}

#[test]
fn export_mesh_async_writes_a_loadable_file() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    let path = std::env::temp_dir().join("scanimprover_async_export.ply");
    app.export_mesh_async(path.clone());
    assert!(app.export_job.is_some());
    pump_until(&mut app, |a| a.export_job.is_none());
    assert!(app.status.starts_with("Exported"), "{}", app.status);
    let bytes = std::fs::read(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let back = crate::io::ply::load(&bytes).unwrap();
    assert_eq!(back.triangle_count(), 12);
}

#[test]
fn small_mesh_requests_run_synchronously() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    assert!(app.current.as_ref().unwrap().triangle_count() < app.async_min_tris);
    app.request_repair_analysis();
    assert!(app.analysis_job.is_none());
    assert!(app.repair_health.as_ref().is_some_and(|h| h.is_watertight));
    app.request_face_groups();
    assert!(app.groups_job.is_none());
    assert_eq!(app.face_groups.len(), 6);
    app.request_mesh_edit(super::MeshEditKind::UnifyNormals);
    assert!(app.edit_job.is_none());
    assert_eq!(app.undo.len(), 1);
}

#[test]
fn failed_task_clears_its_job_slot() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    let id = app.worker.submit_task("boom", |_| panic!("kaboom"));
    app.export_job = Some(id);
    pump_until(&mut app, |a| a.export_job.is_none());
    assert!(app.status.contains("failed"), "{}", app.status);
}

#[test]
fn grow_and_shrink_selection_returns_to_exact_initial_point() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    app.expand_angle_deg = 180.0;
    select(&mut app, &[0]);
    assert_eq!(app.sel_count, 1);
    assert_eq!(app.sel[0], 1);

    app.grow_selection();
    let count_1 = app.sel_count;
    assert!(count_1 > 1);

    app.grow_selection();
    let count_2 = app.sel_count;
    assert!(count_2 > count_1);

    app.shrink_selection();
    assert_eq!(app.sel_count, count_1);

    app.shrink_selection();
    assert_eq!(app.sel_count, 1);
    assert_eq!(app.sel[0], 1);
    assert_eq!(app.sel.iter().filter(|&&v| v > 0).count(), 1);

    app.shrink_selection();
    assert_eq!(app.sel_count, 1);
    assert_eq!(app.sel[0], 1);

    app.shrink_selection();
    assert_eq!(app.sel_count, 1);
    assert_eq!(app.sel[0], 1);
}

#[test]
fn grow_and_shrink_near_crease_does_not_drift() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    app.expand_angle_deg = 45.0;
    select(&mut app, &[0]);
    assert_eq!(app.sel_count, 1);

    app.grow_selection();
    assert_eq!(app.sel_count, 2);
    assert_eq!(app.sel[0], 1);
    assert_eq!(app.sel[1], 1);

    app.shrink_selection();
    assert_eq!(app.sel_count, 1);
    assert_eq!(app.sel[0], 1);

    app.shrink_selection();
    assert_eq!(app.sel_count, 1);
    assert_eq!(app.sel[0], 1);
}

#[test]
fn grow_session_undo_restores_initial_selection() {
    let mut app = app_with(box_mesh(0.0, 0.0, 0.0, 2.0));
    app.expand_angle_deg = 180.0;
    select(&mut app, &[0]);
    let undo_count_before = app.undo.len();

    app.grow_selection();
    app.grow_selection();
    assert_eq!(app.undo.len(), undo_count_before + 1);

    app.undo();
    assert_eq!(app.sel_count, 1);
    assert_eq!(app.sel[0], 1);
    assert!(app.sel_grow_base.is_none());
    assert!(app.sel_grow_history.is_empty());
}

#[test]
fn vertex_colors_selection_has_hard_edges_without_gradient() {
    let flat = Mesh::from_indexed(
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ],
        vec![0, 1, 2, 1, 3, 2],
    );
    let mut app = app_with(flat.clone());
    // Select triangle 0, leave triangle 1 unselected.
    select(&mut app, &[0]);

    let split = crate::render::CreaseSplit::build_per_face(&flat, crate::render::CREASE_COS);
    let colors = app.vertex_colors(&split.render_to_mesh, &split.render_to_face);

    assert_eq!(colors.len(), 6);
    // Triangle 0 (corners 0, 1, 2) is selected.
    // Triangle 1 (corners 3, 4, 5) is unselected.
    // Corners 0, 1, 2 must all have identical selected tint.
    assert_eq!(colors[0], colors[1]);
    assert_eq!(colors[1], colors[2]);

    // Corners 3, 4, 5 must all have identical unselected base color.
    assert_eq!(colors[3], colors[4]);
    assert_eq!(colors[4], colors[5]);

    // The selected color must differ from the unselected color (orange tinted).
    assert_ne!(colors[0], colors[3]);
}


