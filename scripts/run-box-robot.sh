#!/usr/bin/env bash
# The actuator demo: a four-wheeled box robot driving itself around a loop
# of waypoints (examples/box_robot). Its controller is the example's own
# program, examples/box_robot/box_robot.cpp, built as the box_robot target.
# Builds an optimized binary into build-release/ -- a debug build is far too
# slow to run in real time with collision on. Extra arguments are passed
# through, e.g. --headless or --seconds 20.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target box_robot -j"$(nproc)"

exec "$BUILD_DIR/box_robot" \
    "$ROOT_DIR/examples/box_robot/box_robot.arow" \
    "$ROOT_DIR/examples/box_robot/sim_config.yaml" \
    "$@"
