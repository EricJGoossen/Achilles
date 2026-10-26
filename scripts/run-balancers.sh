#!/usr/bin/env bash
# The balancing demo: eight two-wheeled robots balancing themselves with
# different gains, and a box thrown at one (examples/balancers). Its own
# controller, examples/balancers/balancers.cpp, builds as the balancers target.
# Builds an optimized binary into build-release/ -- a debug build is far too
# slow to run in real time with collision on. Extra arguments are passed
# through, e.g. --headless, --seconds 20 or --csv pitch.csv.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target balancers -j"$(nproc)"

exec "$BUILD_DIR/balancers" \
    "$ROOT_DIR/examples/balancers/balancers.arow" \
    "$ROOT_DIR/examples/balancers/sim_config.yaml" \
    "$@"
