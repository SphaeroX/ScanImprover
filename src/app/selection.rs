//! Face selection, hidden regions and face groups.

use super::{App, GroupKind, HiddenRegion, axis_name, group_matches_filter, rotation_between};
use crate::geom::segment::segment_faces;
use crate::geom::topology::{flood_select_from, grow_selection, shrink_selection};
use crate::mesh::Aabb;
use crate::pick;
use glam::{Quat, Vec3};
use rayon::prelude::*;
use std::sync::Arc;

fn mix(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// Deviation heat map ramp (blue -> cyan -> green -> yellow -> red).
fn heat_color(t: f32) -> [f32; 3] {
    let stops = [
        [0.10, 0.20, 0.85],
        [0.00, 0.85, 0.90],
        [0.10, 0.90, 0.15],
        [1.00, 0.80, 0.00],
        [0.95, 0.10, 0.10],
    ];
    let x = (t.clamp(0.0, 1.0) * 4.0).min(3.999);
    let i = x as usize;
    mix(stops[i], stops[i + 1], x - i as f32)
}

impl App {
    // ----------------------------------------------------------------------
    // Selection basics
    // ----------------------------------------------------------------------

    /// True when the selection array matches the displayed mesh.
    pub(crate) fn selection_valid(&self) -> bool {
        self.display()
            .is_some_and(|m| self.sel.len() == m.triangle_count())
    }

    pub(crate) fn recount_sel(&mut self) {
        self.sel_count = self.sel.iter().filter(|&&v| v > 0).count();
        if self.sel_count == 0 {
            self.bridge_preview_active = false;
        }
        self.update_bridge_preview();
    }

    /// Corner points of every selected triangle.
    pub(crate) fn selection_points(&self) -> Option<Vec<[f32; 3]>> {
        let m = self.display()?;
        if !self.selection_valid() {
            return None;
        }
        let sel = &*self.sel;
        let mut pts = Vec::with_capacity(self.sel_count * 3);
        for t in 0..m.triangle_count() {
            if sel[t] > 0 {
                for k in 0..3 {
                    pts.push(m.positions[m.indices[3 * t + k] as usize]);
                }
            }
        }
        if pts.is_empty() { None } else { Some(pts) }
    }

    /// Bounding box of the selected triangles.
    pub(crate) fn selection_bbox(&self) -> Option<Aabb> {
        let pts = self.selection_points()?;
        let mut min = Vec3::splat(f32::MAX);
        let mut max = Vec3::splat(f32::MIN);
        for p in &pts {
            let v = Vec3::from(*p);
            min = min.min(v);
            max = max.max(v);
        }
        Some(Aabb { min, max })
    }

    pub(crate) fn clear_selection(&mut self) {
        self.reset_selection_grow_history();
        if self.sel_count == 0 && self.selection_valid() {
            return;
        }
        if self.selection_valid() {
            Arc::make_mut(&mut self.sel).fill(0);
        }
        self.sel_count = 0;
        self.bridge_preview_patch = None;
        self.bridge_status = None;
        self.bridge_preview_active = false;
        self.mark_selection_changed();
    }

    pub(crate) fn invert_selection(&mut self) {
        if !self.selection_valid() {
            return;
        }
        self.reset_selection_grow_history();
        self.push_snapshot();
        let hidden = self.hidden_mask.clone();
        let sel = Arc::make_mut(&mut self.sel);
        for (t, v) in sel.iter_mut().enumerate() {
            let is_hidden = t < hidden.len() && hidden[t];
            *v = if *v > 0 || is_hidden { 0 } else { 1 };
        }
        self.recount_sel();
        self.mark_selection_changed();
        self.status = format!("Selection inverted: {} faces", self.sel_count);
    }

    /// Replaces the whole selection array (used by group / region selects).
    fn commit_selection(&mut self, sel: Vec<u8>) {
        self.sel = Arc::new(sel);
        self.recount_sel();
        self.mark_selection_changed();
    }

    /// Applies one brush dab at `hit`. Records an undo snapshot on the first
    /// change of a stroke. Returns true when the selection changed.
    pub(crate) fn brush_apply(
        &mut self,
        hit: &pick::Hit,
        radius_px: f32,
        viewport_h: f32,
        add: bool,
    ) -> bool {
        if !self.selection_valid() {
            return false;
        }
        let Some(mesh) = self.display().cloned() else {
            return false;
        };
        let Some(bvh) = self.ensure_bvh() else {
            return false;
        };
        let (_, tris) = self.brush_query(&mesh, &bvh, hit, radius_px, viewport_h);
        let value = if add { 1u8 } else { 0u8 };
        let changes: Vec<u32> = tris
            .into_iter()
            .filter(|&t| (t as usize) < self.sel.len() && self.sel[t as usize] != value)
            .collect();
        if changes.is_empty() {
            return false;
        }
        self.reset_selection_grow_history();
        if self.stroke_snapshot_pending {
            self.push_snapshot();
            self.stroke_snapshot_pending = false;
        }
        let sel = Arc::make_mut(&mut self.sel);
        for t in changes {
            sel[t as usize] = value;
        }
        self.recount_sel();
        self.mark_selection_changed();
        true
    }

    /// Triangles under the brush, honouring hidden faces and (optionally)
    /// the connected-only restriction.
    pub(crate) fn brush_query(
        &mut self,
        mesh: &crate::mesh::Mesh,
        bvh: &crate::geom::bvh::Bvh,
        hit: &pick::Hit,
        radius_px: f32,
        viewport_h: f32,
    ) -> (f32, Vec<u32>) {
        let topo = if self.brush_connected {
            self.ensure_topology()
        } else {
            None
        };
        let hidden = &self.hidden_mask;
        let is_hidden = |t: u32| (t as usize) < hidden.len() && hidden[t as usize];
        match topo {
            Some(topo) => pick::query_brush_connected(
                mesh,
                bvh,
                &topo,
                &self.camera,
                hit,
                radius_px,
                viewport_h,
                is_hidden,
            ),
            None => pick::query_brush_triangles_filtered(
                mesh,
                bvh,
                &self.camera,
                hit,
                radius_px,
                viewport_h,
                is_hidden,
            ),
        }
    }

    pub(crate) fn grow_selection(&mut self) {
        if self.sel_count == 0 {
            return;
        }
        let Some(topo) = self.ensure_topology() else {
            return;
        };
        let angle_rad = self.expand_angle_deg.to_radians();
        let new_sel = grow_selection(&topo, &self.sel, angle_rad);
        if new_sel == *self.sel {
            return;
        }
        if self.sel_grow_base.is_none() {
            self.push_snapshot();
            self.sel_grow_base = Some(self.sel.clone());
        }
        self.sel_grow_history.push(self.sel.clone());
        self.commit_selection(new_sel);
        self.status = format!(
            "Selection expanded (crease threshold: {:.1}°): {} faces",
            self.expand_angle_deg, self.sel_count
        );
    }

    pub(crate) fn shrink_selection(&mut self) {
        if self.sel_count == 0 {
            return;
        }
        // 1. If we have a history of grown rings, unwind the most recent ring:
        if let Some(prev_sel) = self.sel_grow_history.pop() {
            self.sel = prev_sel;
            self.recount_sel();
            self.mark_selection_changed();
            self.status = format!("Selection shrunk: {} faces", self.sel_count);
            return;
        }

        // 2. If we grew from an initial selection and returned to it, stop:
        if self.sel_grow_base.is_some() {
            self.status = format!("Selection at initial state: {} faces", self.sel_count);
            return;
        }

        // 3. Fallback for manual selections: do not shrink single-face selections to nothing.
        if self.sel_count <= 1 {
            return;
        }
        if let Some(topo) = self.ensure_topology() {
            self.push_snapshot();
            let new_sel = shrink_selection(&topo, &self.sel);
            self.commit_selection(new_sel);
            self.status = format!("Selection shrunk: {} faces", self.sel_count);
        }
    }

    // ----------------------------------------------------------------------
    // Hidden regions
    // ----------------------------------------------------------------------

    /// Rebuilds the per-triangle hidden mask. Hidden regions refer to faces
    /// of the working mesh, so nothing is hidden while a decimation preview
    /// is displayed.
    pub(crate) fn update_hidden_mask(&mut self) {
        let nt = if self.preview.is_some() {
            0
        } else {
            self.current
                .as_ref()
                .map(|m| m.triangle_count())
                .unwrap_or(0)
        };
        let mut mask = vec![false; nt];
        for hr in self.hidden_regions.iter().filter(|hr| !hr.visible) {
            for &f in &hr.faces {
                if (f as usize) < nt {
                    mask[f as usize] = true;
                }
            }
        }
        self.hidden_mask = mask;
    }

    pub(crate) fn is_face_hidden(&self, tri: u32) -> bool {
        let t = tri as usize;
        t < self.hidden_mask.len() && self.hidden_mask[t]
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn any_face_hidden(&self) -> bool {
        self.hidden_mask.iter().any(|&h| h)
    }

    pub(crate) fn hide_selection(&mut self) {
        if self.sel_count == 0 || self.preview.is_some() {
            return;
        }
        let Some(m) = self.current.clone() else {
            return;
        };
        let nt = m.triangle_count();
        if self.sel.len() != nt {
            return;
        }
        let selected_faces: Vec<u32> = (0..nt as u32)
            .filter(|&t| self.sel[t as usize] > 0)
            .collect();
        if selected_faces.is_empty() {
            return;
        }
        let prospective_hidden = (0..nt)
            .filter(|&t| self.is_face_hidden(t as u32) || self.sel[t] > 0)
            .count();
        if prospective_hidden >= nt {
            self.status = "Cannot hide all faces of the mesh.".to_string();
            return;
        }

        self.push_snapshot();
        let hidden_tris = selected_faces.len();
        let id = self.next_obj_id;
        self.next_obj_id += 1;
        let name = format!("Hidden Region {}", self.hidden_regions.len() + 1);
        self.hidden_regions.push(HiddenRegion {
            id,
            name: name.clone(),
            visible: false,
            faces: selected_faces,
        });
        self.reset_selection();
        self.mark_visibility_changed();
        self.status = format!("Hidden {hidden_tris} faces into '{name}'.");
    }

    pub(crate) fn toggle_hidden_region_visibility(&mut self, id: u64) {
        let Some(i) = self.hidden_regions.iter().position(|r| r.id == id) else {
            return;
        };
        self.push_snapshot();
        let hr = &mut self.hidden_regions[i];
        hr.visible = !hr.visible;
        let vis = hr.visible;
        let name = hr.name.clone();
        self.mark_visibility_changed();
        self.status = if vis {
            format!("Showing '{name}'.")
        } else {
            format!("Hiding '{name}'.")
        };
    }

    pub(crate) fn restore_hidden_region(&mut self, id: u64) {
        let Some(i) = self.hidden_regions.iter().position(|r| r.id == id) else {
            return;
        };
        self.push_snapshot();
        let hr = self.hidden_regions.remove(i);
        self.mark_visibility_changed();
        self.status = format!("Restored '{}' back into the mesh.", hr.name);
    }

    pub(crate) fn restore_all_hidden_regions(&mut self) {
        if self.hidden_regions.is_empty() {
            return;
        }
        self.push_snapshot();
        self.hidden_regions.clear();
        self.mark_visibility_changed();
        self.status = "All hidden regions restored to mesh.".to_string();
    }

    /// Permanently deletes the faces of a hidden region. The other hidden
    /// regions are re-indexed so they survive the deletion.
    pub(crate) fn delete_hidden_region(&mut self, id: u64) {
        let Some(i) = self.hidden_regions.iter().position(|r| r.id == id) else {
            return;
        };
        let Some(m) = self.current.clone() else {
            return;
        };
        if self.preview.is_some() {
            self.status = "Discard or apply the decimation preview first.".to_string();
            return;
        }
        self.push_snapshot();
        let hr = self.hidden_regions.remove(i);
        let nt = m.triangle_count();
        let mut del_sel = vec![0u8; nt];
        for &f in &hr.faces {
            if (f as usize) < nt {
                del_sel[f as usize] = 1;
            }
        }
        // Old triangle index -> new index (kept triangles keep their order).
        let mut remap = vec![u32::MAX; nt];
        let mut next = 0u32;
        for t in 0..nt {
            if del_sel[t] == 0 {
                remap[t] = next;
                next += 1;
            }
        }
        let mut survivors: Vec<HiddenRegion> = std::mem::take(&mut self.hidden_regions);
        for region in &mut survivors {
            region.faces = region
                .faces
                .iter()
                .filter_map(|&f| remap.get(f as usize).copied())
                .filter(|&f| f != u32::MAX)
                .collect();
        }
        survivors.retain(|r| !r.faces.is_empty());

        let (kept_mesh, _) = m.split_by_selection(&del_sel);
        self.set_mesh_modified(kept_mesh, format!("Deleted '{}'.", hr.name));
        self.hidden_regions = survivors;
        self.mark_visibility_changed();
    }

    // ----------------------------------------------------------------------
    // Face groups
    // ----------------------------------------------------------------------

    /// Drops all face group data (e.g. because the mesh topology changed).
    pub(crate) fn invalidate_face_groups(&mut self) {
        self.face_groups.clear();
        self.group_ids.clear();
        self.selected_group = None;
        self.hover_group = None;
        self.mark_groups_changed();
    }

    /// The per-face group map changed: the render mesh is split along group
    /// boundaries, so it has to be rebuilt.
    pub(crate) fn mark_groups_changed(&mut self) {
        self.groups_generation += 1;
        self.mesh_dirty = true;
        self.aux_dirty = true;
    }

    pub(crate) fn detect_face_groups(&mut self) {
        let Some(m) = self.display().cloned() else {
            return;
        };
        let Some(topo) = self.ensure_topology() else {
            return;
        };
        let min_tris = self.group_min_tris.round().max(1.0) as usize;
        let (groups, ids) = segment_faces(
            &m,
            &topo,
            self.group_angle_deg,
            min_tris,
            self.group_fit_tol,
            self.group_feature_frac,
        );
        self.install_face_groups(groups, ids);
    }

    /// Stores a segmentation result and reports it in the status line.
    pub(crate) fn install_face_groups(
        &mut self,
        groups: Vec<crate::geom::segment::FaceGroup>,
        ids: Vec<i32>,
    ) {
        let planes = groups.iter().filter(|g| g.kind == GroupKind::Plane).count();
        let cylinders = groups
            .iter()
            .filter(|g| g.kind == GroupKind::Cylinder)
            .count();
        let spheres = groups
            .iter()
            .filter(|g| g.kind == GroupKind::Sphere)
            .count();
        let other = groups.len() - planes - cylinders - spheres;
        self.selected_group = None;
        self.hover_group = None;
        self.status = if groups.is_empty() {
            "No face groups found. Try a larger crease angle or smaller min faces.".to_string()
        } else {
            format!(
                "{} face groups: {} planes, {} cylinders, {} spheres, {} other",
                groups.len(),
                planes,
                cylinders,
                spheres,
                other
            )
        };
        self.face_groups = groups;
        self.group_ids = ids;
        self.groups_show = true;
        self.mark_groups_changed();
    }

    pub(crate) fn clear_face_groups(&mut self) {
        self.invalidate_face_groups();
        self.aux_dirty = true;
        self.status = "Face groups cleared.".to_string();
    }

    /// Selects the faces of a group. With `additive` the group's faces are
    /// merged into the current selection instead of replacing it.
    pub(crate) fn select_group_faces(&mut self, id: i32, additive: bool) {
        let Some(m) = self.display() else {
            return;
        };
        let tris = m.triangle_count();
        if self.group_ids.len() != tris {
            return;
        }
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else {
            return;
        };
        let kind = g.kind;
        let group_tris = g.tris.clone();
        let prev = if additive && self.sel.len() == tris {
            self.sel_count
        } else {
            0
        };
        self.reset_selection_grow_history();
        self.push_snapshot();
        let mut sel = if additive && self.sel.len() == tris {
            (*self.sel).clone()
        } else {
            vec![0u8; tris]
        };
        for &t in &group_tris {
            sel[t as usize] = 1;
        }
        self.commit_selection(sel);
        self.status = if additive && prev > 0 {
            format!(
                "Group #{} ({}): +{} faces added, {} total selected.",
                id + 1,
                kind.label(),
                self.sel_count - prev,
                self.sel_count
            )
        } else {
            format!(
                "Group #{} ({}): {} faces selected.",
                id + 1,
                kind.label(),
                self.sel_count
            )
        };
    }

    /// Selects the group's faces and pushes a fitted plane into the objects list.
    pub(crate) fn fit_group_plane(&mut self, id: i32) {
        self.select_group_faces(id, false);
        self.fit_plane_from_selection();
    }

    /// Selects the group's faces and fits a freeform surface onto them.
    pub(crate) fn fit_group_freeform(&mut self, id: i32) {
        self.select_group_faces(id, false);
        self.fit_freeform_from_selection();
    }

    pub(crate) fn align_group_axis(&mut self, id: i32, axis: Vec3) {
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else {
            return;
        };
        let q = rotation_between(g.normal, axis);
        let kind = g.kind;
        self.apply_transform(q, Vec3::ZERO);
        self.status = format!(
            "Group #{} ({}) aligned to {}.",
            id + 1,
            kind.label(),
            axis_name(axis)
        );
    }

    pub(crate) fn origin_on_group_plane(&mut self, id: i32) {
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else {
            return;
        };
        let d = g.point.dot(g.normal);
        self.apply_transform(Quat::IDENTITY, -g.normal * d);
        self.status = "Origin moved onto the group plane.".to_string();
    }

    pub(crate) fn origin_on_group_axis(&mut self, id: i32) {
        let Some(g) = self.face_groups.iter().find(|g| g.id == id) else {
            return;
        };
        let p = g.point - g.normal * g.point.dot(g.normal);
        self.apply_transform(Quat::IDENTITY, -p);
        self.status = "Origin moved onto the cylinder axis.".to_string();
    }

    /// Double-click pick: selects the complete face group under the cursor.
    /// If no face group exists there (or none were detected), falls back to a
    /// Meshmixer-style region select grown from the clicked triangle using the
    /// brush crease angle.
    pub(crate) fn select_region_under(&mut self, tri: u32) {
        let Some(m) = self.display() else {
            return;
        };
        let tris = m.triangle_count();
        if (tri as usize) < tris && self.group_ids.len() == tris {
            let gid = self.group_ids[tri as usize];
            if gid >= 0 && self.face_groups.iter().any(|g| g.id == gid) {
                self.select_group_faces(gid, self.group_sel_additive);
                return;
            }
        }
        let Some(topo) = self.ensure_topology() else {
            return;
        };
        let region = flood_select_from(&topo, tri, self.expand_angle_deg.to_radians());
        if region.is_empty() {
            return;
        }
        self.reset_selection_grow_history();
        self.push_snapshot();
        let additive = self.group_sel_additive && self.sel.len() == tris;
        let mut sel = if additive {
            (*self.sel).clone()
        } else {
            vec![0u8; tris]
        };
        let mut added = 0usize;
        for &t in &region {
            if sel[t as usize] == 0 {
                sel[t as usize] = 1;
                added += 1;
            }
        }
        self.commit_selection(sel);
        self.status = if additive {
            format!(
                "Region added (crease angle {:.1}°): +{} faces, {} total selected",
                self.expand_angle_deg, added, self.sel_count
            )
        } else {
            format!(
                "Region selected (crease angle {:.1}°): {} faces",
                self.expand_angle_deg, added
            )
        };
    }

    /// Color of a face group in the current coloring mode, `None` when the
    /// group is filtered out.
    fn group_color(&self, gid: i32) -> Option<[f32; 3]> {
        let g = self.face_groups.get(usize::try_from(gid).ok()?)?;
        if !group_matches_filter(g.kind, self.groups_filter) {
            return None;
        }
        Some(if self.groups_by_type {
            crate::geom::segment::KIND_COLORS[g.kind as usize]
        } else {
            crate::geom::segment::group_hue_color(gid)
        })
    }

    /// Final colors (linear RGBA) per render vertex: base color, face group
    /// color of the vertex's triangle, deviation heat, hovered group and
    /// selection tint. `render_to_mesh` / `render_to_face` map each render
    /// vertex to its mesh vertex and one of its triangles.
    pub(crate) fn vertex_colors(
        &self,
        render_to_mesh: &[u32],
        render_to_face: &[u32],
    ) -> Vec<[f32; 4]> {
        let base = [0.74f32, 0.76, 0.80];
        let Some(m) = self.display() else {
            return Vec::new();
        };
        let nv = m.vertex_count();
        let nt = m.triangle_count();
        let sel = self.build_vertex_selection().unwrap_or_default();
        let groups_on =
            self.groups_show && !self.face_groups.is_empty() && self.group_ids.len() == nt;
        let heat = self
            .heat
            .as_deref()
            .filter(|h| self.heat_on && h.len() == nv && self.heat_max > 0.0);
        let heat_scale = 1.0 / self.heat_max.max(1e-9);
        let hover = self.hover_group;
        render_to_mesh
            .par_iter()
            .zip(render_to_face.par_iter())
            .map(|(&mv, &face)| {
                let mv = mv as usize;
                let gid = if groups_on && (face as usize) < nt {
                    self.group_ids[face as usize]
                } else {
                    -1
                };
                let mut col = base;
                if gid >= 0
                    && let Some(c) = self.group_color(gid)
                {
                    col = c;
                }
                if let Some(h) = heat
                    && h[mv] >= 0.0
                {
                    col = heat_color((h[mv] * heat_scale).clamp(0.0, 1.0));
                }
                if gid >= 0 && hover == Some(gid) {
                    col = mix(col, [1.0, 1.0, 1.0], 0.45);
                }
                if sel.get(mv).is_some_and(|&s| s > 0.75) {
                    col = mix(col, [1.0, 0.45, 0.10], 0.55);
                }
                crate::render::srgb_to_linear([col[0], col[1], col[2], 1.0])
            })
            .collect()
    }

    /// Per-vertex GPU selection weights.
    pub(crate) fn build_vertex_selection(&self) -> Option<Vec<f32>> {
        let m = self.display()?;
        let nv = m.vertex_count();
        let mut sel = vec![0.0f32; nv];
        if self.sel.len() == m.triangle_count() {
            for t in 0..m.triangle_count() {
                if self.sel[t] > 0 && !self.is_face_hidden(t as u32) {
                    for k in 0..3 {
                        sel[m.indices[3 * t + k] as usize] = 1.0;
                    }
                }
            }
        }
        Some(sel)
    }
}
