# Changes: architecture, quality, performance and UI pass (September 2026)

This document lists every fix and improvement made in the refactoring pass.
Functionality was kept; behaviour only changed where a bug was fixed or where
it is called out below as an addition.

## Bug fixes

| Area | Problem | Fix |
| --- | --- | --- |
| Hidden regions | Opening a new file kept the hidden-face mask of the previous mesh, so random faces of the new mesh were invisible and unpickable. | Loading resets hidden regions and the mask (`install_loaded_mesh`). |
| Hidden regions | The decimation preview used the hidden mask of the working mesh (different triangle set), hiding arbitrary preview faces; applying a preview kept stale regions. | The mask is empty while a preview is shown; applying a preview drops the regions and says so in the status line. |
| Hidden regions | Deleting one hidden region silently restored all others. | Remaining regions are re-indexed through the deletion and stay hidden. |
| Transforms | Rotating / translating the mesh left the hole list, hole-fill preview and bridge preview at their old positions. | Holes (centroid, normal, bbox) and patch previews are transformed with the mesh; the camera target follows so the model stays framed. |
| Camera | The `Z` view preset rendered the model upside down (up vector was -Y). | Presets derive a level horizon from the up axis. |
| Camera | Orbit accumulated roll. | Turntable model (yaw around the up axis, pitch around the camera's right vector): no roll, no pole clamp, the view can swing over the top and horizontal drags keep following the mouse while upside down. |
| Camera | View preset transitions were abrupt (0.28 s). | 0.45 s ease-out transitions. |
| Performance | The selection HUD ran the full bridge cluster detection (and rebuilt the mesh topology when it was not cached) every frame while anything was selected. | Cluster detection is cached per selection / mesh generation. |
| Performance | Hover highlighting re-uploaded the whole per-vertex attribute buffer (16 bytes per vertex) on every mouse move. | Hover is drawn as an overlay; selection weights live in a separate 4-byte buffer; heat / group attributes are uploaded only when they change. |
| Performance | Toggling a hidden region re-uploaded the full vertex buffer. | Vertex buffer is reused when only visibility changed (mesh generation counter). |
| Undo | Every left click in the viewport pushed an undo snapshot, even when nothing was selected. | A brush stroke snapshots once, on its first real change. |
| Undo | Alignment slot assignments were not part of snapshots; deleting a feature left dangling slot references. | Snapshots include the slots; deleting a feature clears its slots (restored on undo). |
| Rendering | Depth-tested overlay lines were drawn opaque, ignoring their alpha. | Line pipelines blend; fills get a depth bias to avoid z-fighting on the surface. |
| Rendering | Depth precision: near/far were fixed at 0.01 / 1e7. | Near/far are fitted to the scene every frame. |
| Repair | The health report always showed 0 non-manifold vertices. | Real fan-connectivity check per vertex (parallel). |
| Worker | A failed job left its spinner running forever; a panic in a job killed the worker thread. | Failures clear the job slot; panics are caught and reported. |
| Loading | Stale state survived a new file (pick-line mode, wheel accumulator, alignment slots, solver status). | All document state is reset on load. |
| Theme | Border colors were built as invalid premultiplied RGBA and rendered as bright additive lines. | Proper premultiplied constants; contrast is now tested. |
| UI text | Several glyphs (arrows, boxes, cross marks) are not in the bundled font and rendered as boxes. | Painted disclosure arrows; text labels elsewhere. |

## Architecture

- `src/app.rs` (3,500 lines) split into `src/app/`: `history` (undo/redo with one capture/restore path), `session` (loading, transforms, caches, worker results), `features` (planes, circles, freeforms, symmetry, alignment slots), `selection` (brush, hidden regions, face groups), `repair` (holes, bridge, clean-up), `tasks` (worker-backed operations), `viewport` (input and overlays), `overlay` (primitive builders).
- UI split into `theme`, `toolbar`, `status_bar`, `shortcuts`, `gizmo`, `object_browser`, `sections`, `repair`, `bridge`, `alignment`, `accordion`; the object browser is a docked right panel.
- Empty leftovers of the reverted "Fit to Object" feature removed; unused `pan_drag` / `zoom` / `look_from` removed.
- `mark_mesh_changed` / `mark_selection_changed` / `mark_visibility_changed` centralise cache invalidation; `mesh_generation` and `sel_generation` key the GPU and cluster caches.
- Alignment slots got helper methods (`axis_of`, `set_axis`, `remove_feature`).
- Settings (`src/settings.rs`): panel widths, theme, up axis, view toggles, brush options and open sections persist between runs in a small key=value file in the user's configuration directory.
- CI runs `cargo test` before building release binaries. Clippy is clean.

## Asynchronous operations and feedback

- Worker: two threads, every job tracked as an *activity* with label, start time and optional progress / stage; the UI is woken up as soon as a result is ready.
- New worker-backed operations: file loading (with stages), mesh analysis (holes + health), face-group segmentation (requests while running are coalesced), mesh edits (auto repair, unify normals, remove debris, remove degenerate faces, fill all holes), hole solver, mesh export.
- Operations run on the worker above 200,000 triangles (loading and export always); results are discarded if the mesh changed in the meantime; a second edit is refused while one runs.
- Feedback: activity feed with progress bars in the status bar, spinners on section headers, a busy pill in the viewport while the document is being loaded or edited, disabled buttons while editing.
- Large meshes build their spatial index and topology in the background right after loading.

## Camera and navigation (additions)

- Turntable orbit around the point under the cursor, exact panning at the grabbed depth, zoom towards the cursor, animated view presets (Top / Front / Right / Iso, Bottom / Back / Left), fit to selection, selectable Y or Z up axis.
- Navigation gizmo in the viewport corner (click an axis to look along it, click again to flip).
- Keyboard: `F` fit, `G` grid, `Ctrl+O` open, plus the existing `1`–`4`, `H`, `.` / `,`, `Ctrl+Z` / `Ctrl+Y`.
- Drag and drop of mesh files.

## Renderer: Bevy

- The custom wgpu renderer and the eframe shell were replaced by [Bevy 0.19](https://bevy.org) with `bevy_egui` for the existing egui interface. The application state is a Bevy resource; the egui pass records what the viewport should show and a `PostUpdate` system mirrors it into Bevy entities (scan mesh, wireframe, overlay line / fill meshes, gizmos, camera with viewport, lights).
- Shading: vertex normals are split at crease edges (30 degrees) when building the render mesh, so flat faces of welded CAD-like meshes are flat and curved scan surfaces stay smooth; lighting is Bevy's PBR with two camera-relative directional lights and ambient light, MSAA 4x, no tonemapping (accurate colors). Back faces render in the inside color via a second entity with front-face culling.
- Face-group / heat / selection coloring moved from the shader to per-vertex colors computed on the CPU (`App::vertex_colors`).
- `--screenshot <file.png>` renders a few frames to a file and exits, for automated visual checks.
- Dependencies are built optimised in debug profiles (`[profile.dev.package."*"] opt-level = 3`) so the debug build stays interactive.
- Rendering is reactive (`WinitSettings::desktop_app`): frames are drawn on input, window events and egui repaint requests (animations, worker results) instead of continuously, which keeps an idle app at near-zero GPU / battery use.
- The window starts maximized. The camera viewport is derived from egui's pixels-per-point (display scale times interface zoom), so the scene always fills the viewport.
- `--demo <out.mp4> <part> <scan> <large>` (`src/demo.rs`) records a scripted feature tour: synthetic pointer / keyboard input on a fixed 30 fps clock, cursor and captions drawn over the interface, every frame streamed to `ffmpeg`. `--screenshots <dir> ...` runs a shorter script and saves the README stills; `examples/fetch.sh` downloads the public test meshes they use.

## Rendering and design

- 4x MSAA, themed background, ground grid with major lines and colored axes.
- Interface font: Noto Sans (regular for text, semibold for section titles and captions, Noto Sans Symbols as fallback), bundled under `assets/fonts` (SIL Open Font License).
- WCAG 2.1 AA conformant dark and light palettes (every text token reaches 4.5:1 on every surface; enforced by `ui::theme::tests::palettes_are_wcag_aa_conformant`), switchable in the toolbar.
- Flat panel layout: captions instead of nested boxes, faint borders, several tool sections can be open at once.
- Crease-split normals, wireframe edge extraction and overlay mesh builders are unit tested.

## Algorithms

- BVH: binned SAH build (parallel for large subtrees), near-first traversal, flat node layout. Closest-point and ray queries are roughly twice as fast; the test suite went from 21 s to 11 s.
- STL: parallel binary parsing and writing; welding uses a parallel sort while keeping the original vertex numbering.
- Segmentation (face groups): rewritten for scans. Normals are smoothed over a quarter of a *feature size* (new slider, default 4 % of the model size, never across creases); a face is a crease face when most of the rim of its feature-size neighbourhood bends away by more than half the crease angle, so fillets and small features separate surfaces while noise and gentle waviness do not; detection runs at three scales so narrow features (ribs, rims) get their own region; crease bands are given back to the region whose anchor plane they lie on; regions split by an absorbed band are reunited; tiny noise islands join their neighbours. On coarse CAD meshes this reduces to the classic crease-angle growing. Plane classification uses trimmed statistics (best 95 % of the points) and the cylinder test tolerates noisy normals. The old neighbour-to-neighbour crease test put an entire scan into one group (median dihedral 2.5°).
- Face group colors are exact per face: the render mesh is split along group boundaries instead of blending colors at shared vertices (which produced gray speckle on thin groups).
- Segmentation: primitive classification of regions runs in parallel.
- Circle / cylinder fitting: axis candidate scoring runs in parallel.
- Hole filling: Liepa-style edge flipping after refinement improves triangle quality (minimum angle) before fairing; loop edges are never flipped and orientation is preserved.
- Repair: non-manifold vertex detection.
- Brush: optional "connected faces only" mode (on by default) so the brush does not bleed through thin walls or onto neighbouring surfaces.

## Tests

75 tests before, 131 after. New coverage: camera (turntable, presets, orbit pivot, zoom anchor, fit, transitions), BVH (structure, brute-force comparisons, sphere queries, degenerate input), renderer (shader parsing, uniform layout, wireframe dedupe, headless render), worker (activity feed, panics), app regressions for all bugs above, async task layer (load, analysis, edits, coalescing, discard on change, export, failure), I/O edge cases (OBJ, PLY, STL), distance, decimation, alignment slots, overlay grid, connected brush, non-manifold vertices, edge flips, weld numbering, palette contrast, settings round trip.
