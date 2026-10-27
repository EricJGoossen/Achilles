#!/usr/bin/env bash
# The contact solver demo: a passive humanoid robot toppling onto a ground
# plane (examples/robot_on_plane/robot_on_plane.arow). Builds an optimized `achilles` into
# build-release/ -- a debug build is far too slow to run in real time with
# collision on -- and runs it with semi-implicit Euler, the only integrator
# with collision so far. Extra arguments are passed through, e.g. --headless.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target achilles -j"$(nproc)"

exec "$BUILD_DIR/achilles" \
    "$ROOT_DIR/examples/robot_on_plane/robot_on_plane.arow" \
    "$ROOT_DIR/examples/robot_on_plane/sim_config.yaml" \
    --integrator euler --dt 0.004 \
    "$@"
