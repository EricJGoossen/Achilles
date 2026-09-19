#!/usr/bin/env bash
# Configures/builds achilles and runs it, visually, against the demo scene
# (examples/two_joint_arm.arow + examples/sim_config.yaml) -- the quickest
# way to see the OpenGL viewer working. Left-drag orbits the camera,
# scroll zooms; close the window to stop.
#
# Usage: scripts/run-example.sh [extra achilles args, e.g. --headless]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja
cmake --build "$BUILD_DIR" --target achilles -j"$(nproc)"

exec "$BUILD_DIR/achilles" \
    "$ROOT_DIR/examples/two_joint_arm.arow" \
    "$ROOT_DIR/examples/sim_config.yaml" \
    "$@"
