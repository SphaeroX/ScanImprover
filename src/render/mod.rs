//! Bevy scene that mirrors the application state.
//!
//! The application (`crate::app::App`) is a Bevy resource. Every frame the
//! egui pass records what the viewport should show (camera, overlay lines,
//! fills, dirty flags) and [`sync_scene`] applies it to Bevy entities:
//!
//! * the scan mesh as two entities sharing one `Mesh` asset (front faces lit
//!   with vertex colors, back faces in the inside color),
//! * a wireframe line mesh,
//! * depth-tested overlay lines and translucent fills as per-frame meshes,
//! * always-on-top overlay lines via gizmos,
//! * a perspective camera whose viewport follows the egui central panel,
//!   with two camera-relative directional lights.

use crate::app::App as ScanApp;
use crate::mesh::Mesh as ScanMesh;
use bevy::asset::RenderAssetUsages;
use bevy::camera::Viewport;
use bevy::camera::visibility::RenderLayers;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use bevy::render::render_resource::{BlendState, Face};
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiGlobalSettings, PrimaryEguiContext};
use glam::Vec3 as GVec3;
use rayon::prelude::*;

/// A line-list / triangle-list vertex: position + RGBA color (sRGB, 0..1).
pub type LineVertex = [f32; 7];

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_scene)
            .add_systems(PostUpdate, sync_scene);
    }
}

#[derive(Component)]
struct ViewCamera;

/// Handles and entities of the scene objects driven by the application.
#[derive(Resource)]
struct SceneEntities {
    mesh: Handle<Mesh>,
    front: Entity,
    back: Entity,
    wire: Entity,
    wire_mesh: Handle<Mesh>,
    lines: Entity,
    lines_mesh: Handle<Mesh>,
    fills: Entity,
    fills_mesh: Handle<Mesh>,
    /// Mesh generation currently uploaded; `u64::MAX` = nothing.
    mesh_generation: u64,
    /// Crease-split render vertex -> source mesh vertex.
    render_to_mesh: Vec<u32>,
    /// Triangle corners as render vertex indices (all triangles).
    render_indices: Vec<u32>,
    /// Content of the overlay meshes currently uploaded.
    last_lines: Vec<LineVertex>,
    last_fills: Vec<LineVertex>,
    wire_valid: bool,
}

fn setup_scene(
    mut commands: Commands,
    mut egui_settings: ResMut<EguiGlobalSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut gizmo_config: ResMut<GizmoConfigStore>,
    scan: Res<ScanApp>,
) {
    // The egui context gets its own camera (spawned below) instead of the
    // scene camera, so the UI covers the whole window while the 3D view is
    // restricted to the central panel.
    egui_settings.auto_create_primary_context = false;

    let (config, _) = gizmo_config.config_mut::<DefaultGizmoConfigGroup>();
    config.line.width = 1.5;
    // Overlay gizmos always draw on top of the geometry.
    config.depth_bias = -1.0;

    let mesh = meshes.add(fill_mesh(&[[0.0f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; 3]));
    let dummy_line = [[0.0f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; 2];
    let dummy_tri = [[0.0f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; 3];
    let wire_mesh = meshes.add(line_mesh(&dummy_line));
    let lines_mesh = meshes.add(line_mesh(&dummy_line));
    let fills_mesh = meshes.add(fill_mesh(&dummy_tri));

    let front_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        perceptual_roughness: 0.62,
        metallic: 0.0,
        reflectance: 0.35,
        cull_mode: Some(Face::Back),
        ..default()
    });
    let back_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.93, 0.86, 0.48),
        perceptual_roughness: 0.8,
        metallic: 0.0,
        reflectance: 0.2,
        cull_mode: Some(Face::Front),
        double_sided: true,
        ..default()
    });
    let overlay_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        alpha_mode: AlphaMode::Blend,
        cull_mode: None,
        double_sided: true,
        ..default()
    });

    let front = commands
        .spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(front_material),
            Visibility::Hidden,
        ))
        .id();
    let back = commands
        .spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(back_material),
            Visibility::Hidden,
        ))
        .id();
    let wire = commands
        .spawn((
            Mesh3d(wire_mesh.clone()),
            MeshMaterial3d(overlay_material.clone()),
            Visibility::Hidden,
        ))
        .id();
    let lines = commands
        .spawn((
            Mesh3d(lines_mesh.clone()),
            MeshMaterial3d(overlay_material.clone()),
            Visibility::Hidden,
        ))
        .id();
    let fills = commands
        .spawn((
            Mesh3d(fills_mesh.clone()),
            MeshMaterial3d(overlay_material),
            Visibility::Hidden,
        ))
        .id();

    // Scene camera with camera-relative key and fill lights.
    let cam = &scan.camera;
    commands
        .spawn((
            ViewCamera,
            Camera3d::default(),
            Camera {
                order: 0,
                clear_color: ClearColorConfig::Custom(bg_color()),
                ..default()
            },
            Projection::Perspective(PerspectiveProjection {
                fov: cam.fov_y,
                aspect_ratio: 1.5,
                near: 0.1,
                far: 10_000.0,
                ..default()
            }),
            camera_transform(cam),
            Msaa::Sample4,
            Tonemapping::None,
            AmbientLight {
                color: Color::WHITE,
                brightness: 650.0,
                affects_lightmapped_meshes: true,
            },
        ))
        .with_children(|parent| {
            // Camera space: +X right, +Y up, +Z back (towards the viewer).
            for (dir, lux) in [
                (Vec3::new(0.35, 0.55, 0.75), 2600.0),
                (Vec3::new(-0.40, -0.30, 0.45), 1300.0),
            ] {
                parent.spawn((
                    DirectionalLight {
                        illuminance: lux,
                        ..default()
                    },
                    Transform::default().looking_to(-dir.normalize(), Vec3::Y),
                ));
            }
        });

    // UI camera: draws egui over the full window on top of the scene.
    commands.spawn((
        PrimaryEguiContext,
        Camera3d::default(),
        RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: bevy::camera::CameraOutputMode::Write {
                blend_state: Some(BlendState::ALPHA_BLENDING),
                clear_color: ClearColorConfig::None,
            },
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
        Msaa::Off,
        Tonemapping::None,
    ));

    commands.insert_resource(SceneEntities {
        mesh,
        front,
        back,
        wire,
        wire_mesh,
        lines,
        lines_mesh,
        fills,
        fills_mesh,
        mesh_generation: u64::MAX,
        render_to_mesh: Vec::new(),
        render_indices: Vec::new(),
        last_lines: Vec::new(),
        last_fills: Vec::new(),
        wire_valid: false,
    });
}

fn bg_color() -> Color {
    let (_, bottom) = crate::ui::theme::viewport_gradient();
    Color::srgb(bottom[0], bottom[1], bottom[2])
}

fn camera_transform(cam: &crate::camera::Camera) -> Transform {
    let eye = cam.eye();
    let up = cam.up();
    Transform::from_translation(Vec3::new(eye.x, eye.y, eye.z)).looking_at(
        Vec3::new(cam.target.x, cam.target.y, cam.target.z),
        Vec3::new(up.x, up.y, up.z),
    )
}

fn sync_scene(
    mut scan: ResMut<ScanApp>,
    mut scene: ResMut<SceneEntities>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut cameras: Query<(&mut Camera, &mut Transform, &mut Projection), With<ViewCamera>>,
    mut visibility: Query<&mut Visibility>,
    mut gizmos: Gizmos,
    window: Option<Single<&Window, With<PrimaryWindow>>>,
) {
    let scan = &mut *scan;
    let has_mesh = scan.has_mesh();

    // --- Mesh geometry / visibility / colors ---------------------------
    if scan.mesh_dirty {
        if let Some(m) = scan.display().cloned() {
            if scene.mesh_generation != scan.mesh_generation {
                let split = CreaseSplit::build(&m, CREASE_COS);
                let mut mesh = Mesh::new(
                    PrimitiveTopology::TriangleList,
                    RenderAssetUsages::default(),
                );
                let positions: Vec<[f32; 3]> = split
                    .render_to_mesh
                    .par_iter()
                    .map(|&v| m.positions[v as usize])
                    .collect();
                mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
                mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, split.normals.clone());
                let colors = expand(&split.render_to_mesh, &scan.vertex_colors());
                mesh.insert_attribute(
                    Mesh::ATTRIBUTE_COLOR,
                    VertexAttributeValues::Float32x4(colors),
                );
                mesh.insert_indices(Indices::U32(visible_indices(
                    &split.indices,
                    Some(&scan.hidden_mask),
                )));
                let _ = meshes.insert(&scene.mesh, mesh);
                scene.render_to_mesh = split.render_to_mesh;
                scene.render_indices = split.indices;
                scene.mesh_generation = scan.mesh_generation;
                scan.aux_dirty = false;
                scan.sel_dirty = false;
            } else if let Some(mut mesh) = meshes.get_mut(&scene.mesh) {
                let indices = visible_indices(&scene.render_indices, Some(&scan.hidden_mask));
                mesh.insert_indices(Indices::U32(indices));
            }
        }
        scan.mesh_dirty = false;
        scan.wire_dirty = true;
    }
    if (scan.aux_dirty || scan.sel_dirty)
        && has_mesh
        && scene.mesh_generation == scan.mesh_generation
    {
        if let Some(mut mesh) = meshes.get_mut(&scene.mesh) {
            let colors = expand(&scene.render_to_mesh, &scan.vertex_colors());
            mesh.insert_attribute(
                Mesh::ATTRIBUTE_COLOR,
                VertexAttributeValues::Float32x4(colors),
            );
        }
        scan.aux_dirty = false;
        scan.sel_dirty = false;
    }

    // --- Wireframe -------------------------------------------------------
    if scan.wire_dirty && scan.show_wireframe {
        if let Some(m) = scan.display().cloned() {
            if m.triangle_count() > crate::app::WIREFRAME_MAX_TRIS {
                scan.status = format!(
                    "Wireframe disabled: mesh too dense (> {}k triangles).",
                    crate::app::WIREFRAME_MAX_TRIS / 1000
                );
                scene.wire_valid = false;
            } else {
                let lines = wireframe_lines(&m, Some(&scan.hidden_mask), [0.25, 0.28, 0.33, 0.85]);
                scene.wire_valid = lines.len() >= 2;
                if scene.wire_valid {
                    let _ = meshes.insert(&scene.wire_mesh, line_mesh(&lines));
                }
            }
        }
        scan.wire_dirty = false;
    }

    // --- Per-frame overlays ---------------------------------------------
    // Meshes are only replaced when their content changed, and never with
    // empty geometry (the entity is hidden instead).
    let frame = &scan.frame;
    if frame.depth_lines.len() >= 2 && frame.depth_lines != scene.last_lines {
        let _ = meshes.insert(&scene.lines_mesh, line_mesh(&frame.depth_lines));
        scene.last_lines = frame.depth_lines.clone();
    }
    if frame.fills.len() >= 3 && frame.fills != scene.last_fills {
        let _ = meshes.insert(&scene.fills_mesh, fill_mesh(&frame.fills));
        scene.last_fills = frame.fills.clone();
    }
    for pair in frame.overlay_lines.chunks_exact(2) {
        let a = Vec3::new(pair[0][0], pair[0][1], pair[0][2]);
        let b = Vec3::new(pair[1][0], pair[1][1], pair[1][2]);
        gizmos.line(
            a,
            b,
            Color::srgba(pair[0][3], pair[0][4], pair[0][5], pair[0][6]),
        );
    }

    let set_vis = |visibility: &mut Query<&mut Visibility>, e: Entity, on: bool| {
        if let Ok(mut v) = visibility.get_mut(e) {
            let want = if on {
                Visibility::Inherited
            } else {
                Visibility::Hidden
            };
            if *v != want {
                *v = want;
            }
        }
    };
    let show_mesh = has_mesh && frame.show_mesh && scene.mesh_generation == scan.mesh_generation;
    set_vis(&mut visibility, scene.front, show_mesh);
    set_vis(&mut visibility, scene.back, show_mesh);
    set_vis(
        &mut visibility,
        scene.wire,
        has_mesh && frame.show_wireframe && scene.wire_valid,
    );
    set_vis(&mut visibility, scene.lines, frame.depth_lines.len() >= 2);
    set_vis(&mut visibility, scene.fills, frame.fills.len() >= 3);

    // --- Camera ------------------------------------------------------------
    if let Ok((mut camera, mut transform, mut projection)) = cameras.single_mut() {
        camera.clear_color = ClearColorConfig::Custom(bg_color());
        if let Some(window) = window.as_deref() {
            let scale = window.scale_factor();
            let r = frame.viewport_rect;
            let win = window.physical_size();
            if win.x > 0 && win.y > 0 && r.width() > 0.0 && r.height() > 0.0 {
                let x = ((r.min.x * scale).round().max(0.0) as u32).min(win.x - 1);
                let y = ((r.min.y * scale).round().max(0.0) as u32).min(win.y - 1);
                let w = ((r.width() * scale).round().max(1.0) as u32).min(win.x - x);
                let h = ((r.height() * scale).round().max(1.0) as u32).min(win.y - y);
                camera.viewport = Some(Viewport {
                    physical_position: UVec2::new(x, y),
                    physical_size: UVec2::new(w.max(1), h.max(1)),
                    ..default()
                });
            }
        }
        let cam = &scan.camera;
        *transform = camera_transform(cam);
        *projection = Projection::Perspective(PerspectiveProjection {
            fov: cam.fov_y,
            aspect_ratio: cam.aspect.max(1e-3),
            near: cam.near,
            far: cam.far,
            ..default()
        });
    }
}

/// Expands a per-mesh-vertex array to the crease-split render vertices.
fn expand<T: Copy + Send + Sync>(render_to_mesh: &[u32], per_vertex: &[T]) -> Vec<T> {
    render_to_mesh
        .par_iter()
        .map(|&mv| per_vertex[mv as usize])
        .collect()
}

/// Builds a line-list mesh from position + sRGB color vertices.
fn line_mesh(lines: &[LineVertex]) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::LineList, RenderAssetUsages::default());
    let n = lines.len() / 2 * 2;
    let positions: Vec<[f32; 3]> = lines[..n].iter().map(|v| [v[0], v[1], v[2]]).collect();
    let colors: Vec<[f32; 4]> = lines[..n]
        .iter()
        .map(|v| srgb_to_linear([v[3], v[4], v[5], v[6]]))
        .collect();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_COLOR,
        VertexAttributeValues::Float32x4(colors),
    );
    mesh
}

/// Builds a triangle-list mesh from position + sRGB color vertices.
fn fill_mesh(tris: &[LineVertex]) -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    let n = tris.len() / 3 * 3;
    let positions: Vec<[f32; 3]> = tris[..n].iter().map(|v| [v[0], v[1], v[2]]).collect();
    let colors: Vec<[f32; 4]> = tris[..n]
        .iter()
        .map(|v| srgb_to_linear([v[3], v[4], v[5], v[6]]))
        .collect();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_COLOR,
        VertexAttributeValues::Float32x4(colors),
    );
    mesh
}

/// Converts an sRGB color (as used by the overlay builders and the UI) to
/// the linear RGBA Bevy expects in vertex colors.
pub fn srgb_to_linear(c: [f32; 4]) -> [f32; 4] {
    let lin = |v: f32| {
        let v = v.clamp(0.0, 1.0);
        if v <= 0.04045 {
            v / 12.92
        } else {
            ((v + 0.055) / 1.055).powf(2.4)
        }
    };
    [lin(c[0]), lin(c[1]), lin(c[2]), c[3].clamp(0.0, 1.0)]
}

/// Index buffer content with hidden triangles removed.
pub fn visible_indices(indices: &[u32], hidden_mask: Option<&[bool]>) -> Vec<u32> {
    match hidden_mask {
        Some(mask) if mask.iter().any(|&h| h) => {
            let mut vis = Vec::with_capacity(indices.len());
            for (t, chunk) in indices.chunks_exact(3).enumerate() {
                if t < mask.len() && mask[t] {
                    continue;
                }
                vis.extend_from_slice(chunk);
            }
            vis
        }
        _ => indices.to_vec(),
    }
}

/// Unique mesh edges as line-list vertices (parallel sort based dedupe).
pub fn wireframe_lines(
    mesh: &ScanMesh,
    hidden_mask: Option<&[bool]>,
    color: [f32; 4],
) -> Vec<LineVertex> {
    let nt = mesh.triangle_count();
    let mut keys: Vec<u64> = (0..nt)
        .into_par_iter()
        .filter(|&t| !hidden_mask.is_some_and(|m| t < m.len() && m[t]))
        .flat_map_iter(|t| {
            let i0 = mesh.indices[3 * t];
            let i1 = mesh.indices[3 * t + 1];
            let i2 = mesh.indices[3 * t + 2];
            [edge_key(i0, i1), edge_key(i1, i2), edge_key(i2, i0)]
        })
        .collect();
    keys.par_sort_unstable();
    keys.dedup();
    keys.into_par_iter()
        .flat_map_iter(|k| {
            let a = (k >> 32) as usize;
            let b = (k & 0xffff_ffff) as usize;
            let pa = mesh.positions[a];
            let pb = mesh.positions[b];
            [
                [pa[0], pa[1], pa[2], color[0], color[1], color[2], color[3]],
                [pb[0], pb[1], pb[2], color[0], color[1], color[2], color[3]],
            ]
        })
        .collect()
}

#[inline]
fn edge_key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    ((lo as u64) << 32) | hi as u64
}

/// Cosine of the crease angle (30 degrees): adjacent faces meeting at a
/// sharper angle do not share a shading normal.
pub const CREASE_COS: f32 = 0.866;

/// Render vertices with normals split at crease edges.
///
/// For every mesh vertex the incident faces are grouped into "smoothing
/// groups": faces whose normals are within the crease angle of each other
/// (transitively) share one render vertex whose normal is the area-weighted
/// average of the group. Welded CAD-like meshes therefore get crisp edges
/// while smooth scan surfaces keep smooth shading.
pub struct CreaseSplit {
    pub render_to_mesh: Vec<u32>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

/// Per mesh vertex: the averaged normal of each smooth group and, per
/// incident corner, `(triangle, group)`.
type VertexGroups = (Vec<[f32; 3]>, Vec<(u32, u32)>);

impl CreaseSplit {
    pub fn build(mesh: &ScanMesh, crease_cos: f32) -> CreaseSplit {
        let nv = mesh.positions.len();
        let nt = mesh.triangle_count();
        if nv == 0 || nt == 0 {
            return CreaseSplit {
                render_to_mesh: (0..nv as u32).collect(),
                normals: mesh.normals.clone(),
                indices: mesh.indices.clone(),
            };
        }
        // Area-weighted face normals.
        let face_n: Vec<GVec3> = (0..nt)
            .into_par_iter()
            .map(|t| {
                let [a, b, c] = mesh.triangle(t);
                (b - a).cross(c - a)
            })
            .collect();
        // CSR vertex -> incident corners (triangle * 3 + k).
        let mut offsets = vec![0u32; nv + 1];
        for &i in &mesh.indices {
            offsets[i as usize + 1] += 1;
        }
        for v in 0..nv {
            offsets[v + 1] += offsets[v];
        }
        let mut fill = vec![0u32; nv];
        let mut corners = vec![0u32; mesh.indices.len()];
        for (ci, &v) in mesh.indices.iter().enumerate() {
            let v = v as usize;
            corners[offsets[v] as usize + fill[v] as usize] = ci as u32;
            fill[v] += 1;
        }
        // Per vertex: group incident faces, assign a group id to each corner.
        let per_vertex: Vec<VertexGroups> = (0..nv)
            .into_par_iter()
            .map(|v| {
                let cs = &corners[offsets[v] as usize..offsets[v + 1] as usize];
                let n = cs.len();
                let mut parent: Vec<u32> = (0..n as u32).collect();
                fn find(p: &mut [u32], mut x: u32) -> u32 {
                    while p[x as usize] != x {
                        let g = p[x as usize];
                        p[x as usize] = p[g as usize];
                        x = g;
                    }
                    x
                }
                let unit: Vec<GVec3> = cs
                    .iter()
                    .map(|&c| face_n[c as usize / 3].normalize_or_zero())
                    .collect();
                for i in 0..n {
                    for j in i + 1..n {
                        if unit[i].dot(unit[j]) >= crease_cos {
                            let (a, b) = (find(&mut parent, i as u32), find(&mut parent, j as u32));
                            if a != b {
                                parent[a as usize] = b;
                            }
                        }
                    }
                }
                let mut group_of_root: Vec<(u32, usize)> = Vec::new();
                let mut sums: Vec<GVec3> = Vec::new();
                let mut corner_groups = Vec::with_capacity(n);
                for (i, &c) in cs.iter().enumerate() {
                    let r = find(&mut parent, i as u32);
                    let g = match group_of_root.iter().find(|(root, _)| *root == r) {
                        Some((_, g)) => *g,
                        None => {
                            group_of_root.push((r, sums.len()));
                            sums.push(GVec3::ZERO);
                            sums.len() - 1
                        }
                    };
                    sums[g] += face_n[c as usize / 3];
                    corner_groups.push((c, g as u32));
                }
                let normals = sums
                    .iter()
                    .map(|s| {
                        if s.length_squared() > 1e-20 {
                            s.normalize().to_array()
                        } else {
                            [0.0, 1.0, 0.0]
                        }
                    })
                    .collect();
                (normals, corner_groups)
            })
            .collect();
        let mut base = vec![0u32; nv + 1];
        for v in 0..nv {
            base[v + 1] = base[v] + per_vertex[v].0.len() as u32;
        }
        let total = base[nv] as usize;
        let mut render_to_mesh = vec![0u32; total];
        let mut normals = vec![[0.0f32; 3]; total];
        let mut indices = vec![0u32; mesh.indices.len()];
        for (v, (group_normals, corner_groups)) in per_vertex.iter().enumerate() {
            for (g, n) in group_normals.iter().enumerate() {
                let rv = base[v] as usize + g;
                render_to_mesh[rv] = v as u32;
                normals[rv] = *n;
            }
            for &(corner, g) in corner_groups {
                indices[corner as usize] = base[v] + g;
            }
        }
        CreaseSplit {
            render_to_mesh,
            normals,
            indices,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube() -> ScanMesh {
        let c = |x: f32, y: f32, z: f32| [x, y, z];
        let v = [
            c(-1.0, -1.0, -1.0),
            c(1.0, -1.0, -1.0),
            c(1.0, 1.0, -1.0),
            c(-1.0, 1.0, -1.0),
            c(-1.0, -1.0, 1.0),
            c(1.0, -1.0, 1.0),
            c(1.0, 1.0, 1.0),
            c(-1.0, 1.0, 1.0),
        ];
        let idx = [
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6,
            5, 0, 4, 7, 0, 7, 3,
        ];
        ScanMesh::from_indexed(v.to_vec(), idx.to_vec())
    }

    #[test]
    fn wireframe_dedupes_shared_edges() {
        let m = ScanMesh::from_indexed(
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0],
            ],
            vec![0, 1, 2, 1, 3, 2],
        );
        let lines = wireframe_lines(&m, None, [1.0; 4]);
        assert_eq!(lines.len(), 10);
        let hidden = [true, false];
        let lines = wireframe_lines(&m, Some(&hidden), [1.0; 4]);
        assert_eq!(lines.len(), 6);
    }

    #[test]
    fn visible_indices_drops_hidden_triangles() {
        let idx = vec![0u32, 1, 2, 1, 3, 2];
        assert_eq!(visible_indices(&idx, None).len(), 6);
        assert_eq!(visible_indices(&idx, Some(&[false, false])).len(), 6);
        assert_eq!(visible_indices(&idx, Some(&[true, false])), vec![1, 3, 2]);
    }

    #[test]
    fn crease_split_separates_cube_faces_but_keeps_smooth_surfaces() {
        let cube = cube();
        assert_eq!(cube.vertex_count(), 8);
        let split = CreaseSplit::build(&cube, CREASE_COS);
        assert_eq!(split.render_to_mesh.len(), 24);
        assert_eq!(split.indices.len(), cube.indices.len());
        for chunk in split.indices.chunks_exact(3) {
            let n0 = GVec3::from(split.normals[chunk[0] as usize]);
            let n1 = GVec3::from(split.normals[chunk[1] as usize]);
            let n2 = GVec3::from(split.normals[chunk[2] as usize]);
            assert!(
                n0.dot(n1) > 0.999 && n1.dot(n2) > 0.999,
                "cube faces must be flat"
            );
            for &ri in chunk {
                assert!((split.render_to_mesh[ri as usize] as usize) < 8);
            }
        }
        let flat = ScanMesh::from_indexed(
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0],
            ],
            vec![0, 1, 2, 1, 3, 2],
        );
        let split = CreaseSplit::build(&flat, CREASE_COS);
        assert_eq!(split.render_to_mesh.len(), 4);
        assert_eq!(split.indices, flat.indices);
    }

    #[test]
    fn line_and_fill_meshes_drop_incomplete_primitives() {
        let v = [0.0f32, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let m = line_mesh(&[v, v, v]);
        assert_eq!(m.count_vertices(), 2);
        let m = fill_mesh(&[v, v, v, v]);
        assert_eq!(m.count_vertices(), 3);
        assert_eq!(srgb_to_linear([1.0, 0.0, 0.5, 0.3])[0], 1.0);
        assert!(srgb_to_linear([0.5, 0.0, 0.0, 1.0])[0] < 0.25);
    }
}
