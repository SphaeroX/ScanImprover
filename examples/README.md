# Example meshes

Run `./fetch.sh` to download the meshes used for the README screenshots and
the demo video (about 8 MB). They come from Alec Jacobson's
[common 3D test models](https://github.com/alecjacobson/common-3d-test-models)
collection and are **not** part of this repository; each model keeps the
license of its original source.

| File | Triangles | Origin | Used for |
| --- | --- | --- | --- |
| `fandisk.obj` | 12,946 | CAD part (Pratt & Whitney / Hugues Hoppe) | face groups, plane fit, alignment |
| `rocker-arm.obj` | 20,088 | Rocker arm (INRIA GAMMA) | overview, light theme |
| `stanford-bunny.obj` | 69,451 | Stanford 3D Scanning Repository | repair (holes) |
| `armadillo.obj` | 99,976 | Stanford 3D Scanning Repository | symmetry, decimation |

The Stanford models are provided for research and non-commercial use with
attribution to the Stanford Computer Graphics Laboratory.

```bash
./examples/fetch.sh
cargo run -- --screenshots screenshots examples/fandisk.obj examples/stanford-bunny.obj examples/armadillo.obj examples/rocker-arm.obj
cargo run -- --demo demo.mp4 examples/fandisk.obj examples/stanford-bunny.obj examples/armadillo.obj
```
