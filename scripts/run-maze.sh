#!/usr/bin/env bash
# The sensing demo: a robot finding its way out of a maze from its own
# sensors (examples/maze). Its controller is the example's own
# program, examples/maze/maze.cpp, built as the maze target.
# Builds an optimized binary into build-release/ -- a debug build is far too
# slow to run in real time with collision on. Extra arguments are passed
# through, e.g. --headless or --seed 7.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target maze -j"$(nproc)"

exec "$BUILD_DIR/maze" \
    "$ROOT_DIR/examples/maze/maze.arow" \
    "$ROOT_DIR/examples/maze/sim_config.yaml" \
    "$@"
