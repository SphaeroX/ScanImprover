#!/bin/sh
# Downloads the example meshes used for the README screenshots and the demo
# video from the "common 3D test models" collection
# (https://github.com/alecjacobson/common-3d-test-models). The files are not
# part of this repository; see examples/README.md for their origins.
set -e
cd "$(dirname "$0")"
BASE=https://raw.githubusercontent.com/alecjacobson/common-3d-test-models/master/data
for f in fandisk.obj rocker-arm.obj stanford-bunny.obj armadillo.obj; do
  if [ ! -f "$f" ]; then
    echo "fetching $f"
    curl -fsSL "$BASE/$f" -o "$f"
  fi
done
echo "done"
