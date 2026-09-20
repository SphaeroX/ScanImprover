# Automated quad retopology (experimental)

Issue #3 asks for automated quad retopology of dense scan meshes: fewer
polygons, edge loops that follow the surface curvature, and output that is
easier to take into CAD (Fusion 360 converts quad meshes to T-Splines /
B-Rep much better than triangle soups). This document compares the
candidate methods, explains the choice and lists the known limitations of
the implementation in `src/geom/retopo/`.

## Requirements in this project

* Input: triangle scans of 100k–2M triangles, noisy, often with holes,
  sometimes non-manifold. Output must be usable for reverse engineering:
  mostly quads, manifold, close to the scan, aligned with sharp edges.
* Pure Rust. The only native code is the vendored meshoptimizer. No Eigen,
  Boost, Gurobi or other large C++ dependencies; small pure-Rust crates only
  when clearly justified.
* Runs on the worker thread in seconds to tens of seconds, parallel with rayon.
* License compatible with shipping inside the application.

## Candidates

| Method | Quality (singularities / edge flow) | Robustness on noisy scans | Speed at ~1M tris | Reference code, license, dependencies | Pure-Rust integration cost |
|---|---|---|---|---|---|
| **Instant Meshes** (Jakob, Tarini, Panozzo, Sorkine-Hornung, SIGGRAPH Asia 2015) | Quad-dominant; more singularities than global methods (the paper calls this its main limitation). Extrinsic smoothing aligns edges with principal curvature; snaps to sharp features. | Very good: purely local smoothing, works on range scans and point clouds; extraction can leave small holes and irregular faces that need clean-up. | Linear time, ~1–3 s (paper: 372M triangles in 9 min). | BSD-3; Eigen, TBB, nanogui (UI only). | **Low.** No linear solver, ~2k lines; parallel Gauss–Seidel maps directly onto rayon. |
| **QuadriFlow** (Huang, Zhou, Nießner, Shewchuk, Guibas, SGP 2018) | About 4x fewer singularities than Instant Meshes; watertight, rarely non-manifold. Loses detail at low target density. | Needs a watertight manifold input (ships its own manifold conversion). Blender's "Quad" remesher; its manual advises against it for cleaning up meshes. | 5 s for 0.86M faces, 20 s for 2.4M (single thread). | MIT; Boost, Eigen, LEMON (Gurobi only for benchmarks). | **Medium.** Instant Meshes plus a min-cost-flow solver (network simplex) and a local SAT-based fix-up. |
| **Mixed-Integer Quadrangulation** (Bommes, Zimmer, Kobbelt, SIGGRAPH 2009) + QEx extraction (Ebke et al. 2013) | Few singularities, good curvature alignment (global parameterization). | Weak: needs clean manifold input and cut graphs; noise produces spurious singularities. | Minutes (libigl MIQ > 2 min on QuadriFlow's test set). | CoMISo GPL, libQEx GPLv3; sparse Cholesky + greedy mixed-integer rounding. | **High**, and GPL code cannot be ported. |
| **QuadWild** (Pietroni et al., SIGGRAPH 2021) + **Bi-MDF** quantization (Heistermann, Warnett, Bommes, SIGGRAPH 2023) | Best of the group for CAD-like shapes: feature-line driven patch layout, very few singularities. | Designed for clean CAD meshes; dihedral-threshold feature detection picks up noise on scans. 98.8% success on Thingi10k. | Minutes per model (Bi-MDF cut the quantization from ~1 h to ~17 s on a 300-model set). | GPL-3.0; VCG, CG3Lib, libigl, optional Gurobi (Bi-MDF via libSatsuma/LEMON). | **Very high** (tracing, patch decomposition, flow quantization), and GPL. |
| **Exoside QuadRemesher**, ZBrush ZRemesher, Houdini (Labs node uses QuadRemesher) | Industry reference for artist-like edge loops. | Good in practice. | Seconds. | Proprietary. | Not possible. |
| **Neural cross fields**: NeurCross (Dong et al., SIGGRAPH 2025), CrossGen (2025) | Good principal-curvature alignment and singularity placement (authors' claims); CrossGen still needs QEx/QuadWild to extract the mesh. | Authors claim robustness to noise. | GPU; NeurCross is per-shape optimisation (orders of magnitude slower than CrossGen). | AGPL-3.0 / PyTorch. | Not viable (GPU, Python, license). |

Other findings:

* The only Rust quad remesher on crates.io is `quadrs` (MIT, "experimental",
  based on Instant Meshes, ~100 downloads). It was not adopted: unaudited, no
  feature constraints, and our pipeline needs tight integration with the
  existing BVH, decimator and worker progress anyway.
* `remesh` / `baby_shark` only do isotropic triangle remeshing.
* Recent work on robust extraction (N. Ray, "On Quad Mesh Extraction From
  Messy Grid Preserving Maps", 2025) and new field formulations (integrable
  odeco fields, 2026) are interesting follow-ups for a global method, but
  all still depend on a global parameterization solve.

## Decision

**A pure-Rust field-aligned remesher in the Instant Meshes family**, with
the improvements that fit into that framework:

* It is the only candidate that is robust on noisy, open, non-manifold scans
  *and* runs in seconds without a linear or integer solver, which keeps the
  integration pure Rust and parallel.
* BSD-3 allows reimplementing it; the implementation is written from the
  paper and the published formulas, not copied.
* Extrinsic 4-RoSy smoothing already makes the edge flow follow the principal
  curvature directions on curved regions (cylinders, tori) and hard
  constraints align it with sharp creases, which is what CAD reconstruction
  needs most.
* QuadriFlow's global min-cost-flow singularity reduction is the natural next
  step and can be added on top of the same fields later (see Future work).
  Global methods (MIQ, QuadWild) give nicer layouts on clean CAD meshes but
  are GPL, need solvers we do not have, and are fragile on raw scans.

## Implementation (`src/geom/retopo/`)

1. **Work mesh** (`prep.rs`). The lattice scale `s` follows from the target
   face count (`h = sqrt(area / faces)`; `s = 2h` in pure-quad mode). Dense
   scans are decimated with the vendored meshoptimizer to about `s/3.5` edge
   length (which also removes sub-lattice noise), coarse input is refined by
   conforming long-edge splitting until no edge exceeds `s/2`.
2. **Features.** Edges with a dihedral angle above the crease angle and open
   boundary edges are feature edges; short isolated chains (< `s`) are
   dropped as noise. Vertices on a feature get a hard orientation constraint
   along it and a lattice *line* through them; feature corners whose edges
   meet at right angles become lattice *points*. Corners where the directions
   disagree (e.g. three edges of a box corner) stay free and become
   singularities.
3. **Hierarchy** (`hierarchy.rs`): greedy vertex pairing by
   `(n_i·n_j)·max(A_i/A_j, A_j/A_i)`, graph colouring per level.
4. **Fields** (`field.rs`): extrinsic 4-RoSy orientation field and 4-PoSy
   position field, solved coarsest level first and refined, each level with
   parallel Gauss–Seidel sweeps (one rayon pass per colour).
5. **Extraction** (`extract.rs`): edges of the work mesh are classified by the
   integer lattice offset of their endpoints. Zero-offset edges collapse
   (cheapest first, union-find, refusing merges between clusters already
   joined by a lattice edge), unit offsets become edges. Cluster positions are
   weighted towards members close to their lattice point. Dangling edges and
   edges spanning a nearly collinear vertex are removed, then faces are traced
   counter-clockwise; clockwise loops (outer boundaries) and large loops that
   do not cover surface are rejected.
6. **Clean-up** (`post.rs`): adjacent triangles merge into well-shaped quads.
   In pure-quad mode (default) the extraction runs at `2h` and one linear
   subdivision step turns every n-gon into n quads, so the result has no
   triangles at all; otherwise polygons are split into quads plus at most one
   triangle. Vertices are projected onto the input with the BVH and relaxed
   tangentially (feature and boundary vertices stay put).

### Quads in the application

`Mesh` stays a triangle mesh; an optional `QuadLayout` marks the first `k`
triangle pairs as quads (`(a,b,c)+(a,c,d)`) together with a checksum of the
index buffer. Every existing algorithm keeps working on the triangles; any
operation that rewrites the triangles invalidates the layout automatically.
While it is valid, the wireframe hides quad diagonals and OBJ/PLY export
writes real 4-sided faces. STL is triangles only by definition.

The tool lives in the *Experimental* section: target face count (automatic
default from the mesh size), sharp-edge alignment with a crease angle,
pure-quad mode and relaxation passes. It runs on the worker (with progress in
the panel and the activity feed) and the result replaces the working mesh as
one undoable step.

## Results

Synthetic shapes (`src/geom/retopo/tests.rs`, pure-quad mode unless noted).
`h` is the target edge length; distances are the largest distance from an
input vertex to the output surface (holes or cut features show up here) and
the mean distance from output vertices / face centres to the input.

| Shape (input tris) | Target | Faces | Tris | Closed | Valence 4 | max in→out | mean out→in |
|---|---|---|---|---|---|---|---|
| Sphere (20k) | 1200 | 1276 quads | 0 | yes | 94.8% | 0.05 h | 0.012 h |
| Torus (20k) | 1500 | 1592 quads | 0 | yes | 92.5% | 0.10 h | 0.016 h |
| Torus, quad-dominant | 1500 | 1510 quads | 96 | yes | 92.0% | 0.09 h | 0.017 h |
| Box with sharp edges (19k) | 600 | 600 quads | 0 | yes | 98.7% | 0.02 h | 0.0001 h |
| Capped cylinder (23k) | 1000 | 1054 quads | 0 | yes | 96.3% | 0.27 h | 0.009 h |
| Open hemisphere (10k) | 600 | 635 quads | 0 | open (boundary kept) | 96.7% | 0.18 h | 0.012 h |
| Noisy sphere, ±0.5% radius (82k) | 800 | 816 quads | 0 | yes | 91.8% | 0.12 h | 0.017 h |

On the box more than 90% of the edges are parallel to the box axes, and on the
cylinder wall more than 90% follow the axis or the circumference (principal
directions); both are asserted by the tests.

Example meshes (`examples/fetch.sh`, automatic target, macOS laptop, release
build, total time including BVH and clean-up):

| Mesh | Input tris | Output | Time |
|---|---|---|---|
| fandisk (CAD) | 12,946 | 680 quads | 0.10 s |
| rocker-arm | 20,088 | 1,078 quads, closed | 0.11 s |
| stanford-bunny (open scan) | 69,451 | 3,698 quads | 0.18 s |
| armadillo | 99,976 | 5,206 quads | 0.23 s |
| armadillo, subdivided twice | 1,599,616 | 21,449 quads | 1.4 s |

Dense inputs are decimated to the working resolution first, so the run time
is dominated by that step and grows only mildly with the input size.

## Known limitations / future work

* **Singularities**: more irregular vertices than QuadriFlow or global
  methods. Future: QuadriFlow-style min-cost-flow consistency/singularity
  optimisation on the position field (pure Rust network simplex).
* **Holes**: the extraction can leave small holes where the fields are
  inconsistent (near singularities); holes of up to 16 edges that cover
  surface are closed by the clean-up, larger ones (and genuine scan holes)
  stay open. On CAD parts with thin walls (fandisk) a few holes remain.
* **Local feature cuts**: a quad next to a singularity can still cut across
  a sharp edge (worst case on the cylinder rim: 0.27 h).
* **Thin features** narrower than about one lattice cell (twice the edge
  length in pure-quad mode) are lost (e.g. the armadillo's claws at 5k quads).
* **Curvature alignment** comes from extrinsic smoothing only; there is no
  explicit principal-curvature term (noisy scans make curvature estimates
  unreliable). Nearly flat or umbilic regions have arbitrary orientation.
* **Features** are detected with a single dihedral threshold on the work
  mesh; very noisy scans may need a higher crease angle, rounded (filleted)
  edges are treated as smooth surface.
* **Uniform density**: no adaptive sizing by curvature yet.
* **Quad structure** is lost by any later edit of the triangles (repair,
  decimation, hole fill); OBJ import fans quads into triangles.
