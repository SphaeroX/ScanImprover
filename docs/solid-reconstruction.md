# Solid reconstruction from face groups (experimental)

Issue #6: turn a closed, segmented scan into a parametric B-Rep solid that
CAD systems can open as STEP. This page summarises related work, explains
the approach implemented in `src/geom/solid/` and lists its limits.

## Related work

Every learned method below hands the step of intersecting faces into edges
and corners to a geometry library or a solver: PyMesh for Point2CAD,
OpenCASCADE for NVDNet and BrepGen, Gurobi for ComplexGen. None produces a
watertight solid without one. Classic reverse-engineering pipelines and
Fusion's prismatic mesh conversion get the topology from region adjacency
instead. ScanImprover already has that from its face groups.

| Method | Year | Input | Output | Topology from | Runtime deps | Fit for this app |
|---|---|---|---|---|---|---|
| [Efficient RANSAC](https://onlinelibrary.wiley.com/doi/abs/10.1111/j.1467-8659.2007.01016.x) (Schnabel, Wahl, Klein) | 2007 | point cloud | planes, spheres, cylinders, cones, tori + leftover points; no B-Rep | – | none (CPU) | fitting step only; we already segment |
| [Benkő, Martin, Várady](https://www.sciencedirect.com/science/article/abs/pii/S0010448501001002) | 2001 | segmented points | consistent B-Rep with blends | region adjacency + surface intersection | CAD kernel | closest classic precedent |
| Geomagic Design X | – | mesh | regions → auto surface (NURBS patches) or feature tree | curvature regions | commercial | reference workflow |
| [Fusion "Convert Mesh" (prismatic)](https://help.autodesk.com/cloudhelp/ENU/Fusion-Mesh/files/MESH-CONVERT-TO-SOLID.htm) | – | mesh + face groups | one B-Rep face per face group; solid if watertight | face groups | commercial | same idea; exact surface types not documented |
| QuickSurface / Mesh2Surface | – | mesh | primitives with constraints (parallel, perpendicular, coincident), solid built interactively | user | commercial | constraint ideas |
| [ParSeNet](https://arxiv.org/abs/2003.12181) | 2020 | point cloud | segments + primitives / B-spline patches | none | trained net, GPU | fitting ideas only |
| [HPNet](https://arxiv.org/abs/2105.10620) | 2021 | point cloud | primitive segmentation | none | trained net, GPU | – |
| [ComplexGen](https://arxiv.org/abs/2205.14573) | 2022 | point cloud | corners, curves, patches | predicted chain complex + global optimisation | GPU nets, Gurobi licence, ~10 min/model | too heavy |
| [SED-Net](https://dl.acm.org/doi/10.1145/3588432.3591522) | 2023 | point cloud | surfaces + edges | edge detection branch | trained net, GPU | – |
| [Point2CAD](https://arxiv.org/abs/2312.04962) | 2024 | segmented point cloud | clipped surface meshes, polyline edges, corners | pairwise *mesh* intersection of extended surfaces | PyMesh, GPU Docker | closest in spirit; no STEP |
| [NVDNet](https://arxiv.org/abs/2406.05261) | 2024 | point cloud / distance field | parametric surfaces, curves, vertices | Voronoi-cell adjacency + OpenCASCADE intersection | GPU, OCC | adjacency rule like ours |
| [BrepGen](https://arxiv.org/abs/2401.15563) | 2024 | – (generative) | freeform B-Reps | merged latent tree nodes | CUDA diffusion, pythonOCC | not reconstruction |

Why the app builds the B-Rep from the mesh, without a kernel or a network:

* **The mesh already contains the topology.** A closed mesh segmented into
  face groups tells us exactly which faces touch, along which boundary, and
  where three or more meet. The learned methods above spend most of their
  effort guessing this, or recover it by intersecting extended surfaces and
  then pruning (Point2CAD, NVDNet). Here the mesh boundary between two
  groups also picks the right branch of an intersection curve and seeds its
  computation.
* **No new dependencies.** OpenCASCADE is a large C++ dependency and the
  learned methods need GPUs and trained weights. Everything here is about
  2,500 lines of Rust on the existing `glam`/`rayon` stack.
* **Deterministic and explainable.** Every step is a least-squares fit or a
  closed-form intersection, and a failure can be traced to named groups.

## Approach

Input: a closed, consistently oriented, single-shell mesh and its face
groups (per-triangle group ids). Holes must be closed first. The existing
hole fill does this today, and the face-group-guided hole fill (#5) is
designed to produce such meshes.

1. **Topology** (`topo.rs`).
   * Check the mesh: closed, 2-manifold, one shell. An inside-out mesh is
     flipped.
   * Ungrouped triangles join the neighbouring group they share the most
     edges with.
   * Each connected group region becomes a face and must be a disk with
     holes (V − E + F = 2 − loops).
   * Chains of mesh edges between two faces become edges, and mesh vertices
     where more than two faces meet become vertices. A closed edge with no
     corner (for example a cylinder meeting a plane) gets one seam vertex.
   * Loops follow the mesh orientation, so the face is always on the left
     and every edge is used once in each direction.
2. **Surfaces** (`surface.rs`). The segmentation's classification picks the
   primitive, which is then refitted robustly: least squares, refitted on
   the points within 3 MADs.
   * Plane: PCA.
   * Cylinder and sphere: Levenberg–Marquardt, seeded from the face group.
   * Freeform groups: first try a cone (seeded from the normal covariance
     and the intersection of the normal lines). Otherwise use a bicubic
     B-spline patch from the existing freeform fit, extended 20 % beyond the
     group so the neighbouring faces intersect it inside the patch.
   * Analytic surfaces are unbounded, which provides the extrapolation past
     noisy boundaries.
3. **Relationships** (`regularize.rs`). Within the snap angle (default 2°):
   * Plane normals and axes are merged into parallel sets.
   * Sets that are nearly perpendicular are made exactly perpendicular.
   * Axes of cylinders and cones closer than 3× the fit tolerance become
     one shared axis, and spheres centred on such an axis are moved onto it.
   * Every changed surface is refitted with the snapped parameters held
     fixed.
4. **Vertices.** Each vertex is the point common to all its faces' surfaces
   closest to the mesh corner, found by minimum-norm Gauss–Newton on the
   signed distances. With four or more surfaces it is the least-squares
   compromise.
5. **Edges** (`curve.rs`).
   * Plane–plane: **line**.
   * Any two coaxial surfaces of revolution: **circle**. This covers a plane
     perpendicular to the axis, cylinder, cone, and a sphere centred on the
     axis. It is solved as an intersection of 2D profiles.
   * Plane–cylinder at an angle: **ellipse**. When the plane is parallel to
     the axis, the lines along the cylinder.
   * Every other pair, including anything with a B-spline face: a cubic
     **B-spline** interpolating points of the exact intersection. The mesh
     boundary is resampled by arc length and each sample is projected onto
     both surfaces.
   * An analytic curve that strays too far from the mesh boundary falls
     back to the B-spline.
6. **Validation.** Loops close up, every edge is used exactly twice with
   opposite senses, and V − E + 2F − L = 2 − 2·genus (the genus comes from
   the mesh). The panel reports:
   * vertex gaps (distance of a vertex from its surfaces);
   * edge gaps;
   * mesh boundary vs edge curve;
   * the deviation of every group's mesh vertices from its surface.
7. **STEP** (`src/export/step_solid.rs`). AP214 `MANIFOLD_SOLID_BREP` →
   `CLOSED_SHELL` → `ADVANCED_FACE`, on `PLANE`, `CYLINDRICAL_SURFACE`,
   `CONICAL_SURFACE`, `SPHERICAL_SURFACE` or `B_SPLINE_SURFACE_WITH_KNOTS`.
   * Edges are `EDGE_CURVE`s on `LINE`, `CIRCLE`, `ELLIPSE` or
     `B_SPLINE_CURVE_WITH_KNOTS`.
   * `same_sense` records whether the surface's own normal points out of the
     solid; a bore has a cylinder facing inwards.
   * Cylinders between two circles carry no seam edge. Readers add one,
     which is how CAD systems commonly write them.

Anything that cannot be represented is refused with a message naming the
groups involved, rather than written as a broken file. Examples:
* an open mesh;
* a group wrapping around a handle;
* a freeform group that is not a height field;
* neighbours whose surfaces do not meet, such as parallel planes with a
  missing step face.

## Validation status

The unit tests reconstruct synthetic meshes after running the real
segmentation with its default settings (`src/geom/solid/tests.rs`):
* a box;
* a capped cylinder;
* a noisy, rotated box;
* a plate with a hole (genus 1);
* a cone frustum;
* a box with a freeform top.

The resulting STEP files were loaded with OpenCASCADE 7.8 (`cadquery-ocp`):
`STEPControl_Reader`, then `BRepCheck_Analyzer`, `ShapeAnalysis_Shell` and
volume properties. Every file is one valid closed solid with no free edges,
and the volumes match the analytic values:

| Part | Faces | Edges | Vertices | OCC volume | Exact volume |
|---|---|---|---|---|---|
| box 20×10×6 | 6 planes | 12 lines | 8 | 1200.0000 | 1200 |
| cylinder r 8, h 20 | 2 planes + cylinder | 2 circles | 2 | 4021.2386 | 4021.2386 |
| noisy rotated box (±0.02 jitter) | 6 planes | 12 lines | 8 | 2518.87 | 2520 |
| plate with Ø10 hole | 6 planes + cylinder | 12 lines + 2 circles | 10 | 4928.7611 | 4928.7611 |
| cone frustum | 2 planes + cone | 2 circles | 2 | 2575.6033 | 2575.6033 |
| box with freeform top | 5 planes + B-spline | 8 lines + 4 B-splines | 8 | 3199.9988 | 3200 |

Two public sample meshes (`examples/fetch.sh`) with the default face-group
settings:

* **fandisk** (12,946 triangles, a CAD part with fillets) becomes a solid in
  0.15 s:
  * 18 faces: 10 planes, 3 cylinders, 1 sphere and 4 B-spline faces;
  * 45 edges: 17 lines, 9 circles and 19 B-splines;
  * 30 vertices, genus 0.

  The mesh vertices deviate at most 0.09 mm from their surfaces (RMS
  0.0085). OpenCASCADE loads it as one valid solid with volume 20.218,
  against 20.243 for the mesh (−0.12 %). OCC widens some vertex tolerances,
  up to 0.13, at corners where four or more surfaces meet and along the
  B-spline edges.
* **rocker-arm** (an organic casting) is refused: its largest group wraps
  around a hole of the part, so it would have to be split into several
  faces.

To repeat the check:

```bash
SCANIMPROVER_STEP_DIR=/tmp/solids cargo test --release dump_step -- --ignored
# any mesh file, default settings, prints the report or the refusal
SCANIMPROVER_SOLID_MESH=part.stl SCANIMPROVER_STEP_DIR=/tmp/solids \
  cargo test --release reconstruct_mesh_file -- --ignored --nocapture
```

## Limitations

* The mesh must be one closed, manifold shell. Hollow parts with inner
  shells are refused.
* One face per connected group region. A group that wraps around a handle
  (common on organic parts) is refused rather than split automatically.
* Tori and other surfaces of revolution have no analytic type yet. Fillets
  are exact only when the segmentation reports them as cylinders; otherwise
  they become B-spline faces.
* B-spline faces come from the height-field freeform fit, so a group that
  folds over its fit plane is refused. The patch is also a smoothing
  approximation: its accuracy is reported but not guaranteed to reach the
  analytic tolerance.
* Corners where four or more surfaces meet use a least-squares vertex, and
  gaps are reported. The STEP uncertainty is set from the largest gap.
* Edges carry no p-curves, and closed periodic faces carry no seam edges.
  OpenCASCADE rebuilds both on import. Other readers have not been tested
  (Fusion 360 and SolidWorks were not available here).

## Future work

* Torus fitting (fillets), and blend recognition between neighbouring
  faces (Benkő et al.).
* Splitting faces that wrap around handles, and supporting several shells.
* Trimmed B-spline faces fitted directly over the face boundary instead of a
  height-field patch, with continuity constraints to analytic neighbours.
* More relationships: equal radii, symmetric features, and planes at
  standard angles or aligned with the coordinate system.
* A viewport preview of the trimmed faces, and a per-face deviation colour
  map.
