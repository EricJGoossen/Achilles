#!/usr/bin/env bash
# The domino demo: a row of ten dominoes, each 1.25 times the size of the
# one before, the first knocking over the second and so on down to the
# biggest -- contact carrying momentum down a chain (examples/dominoes). Its
# own program, examples/dominoes/dominoes.cpp, builds as the dominoes target.
# It opens a window each for the chain through its camera and every domino's
# tilt on one plot; close them to stop. Builds an optimized binary into
# build-release/ -- a debug build is far too slow to run in real time with
# collision on. Extra arguments are passed through, e.g. --headless.
# The build runs at most JOBS (default 6) compile jobs at once: an unbounded
# one can run the machine out of memory.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target dominoes -j"${JOBS:-6}"

exec "$BUILD_DIR/dominoes" \
    "$ROOT_DIR/examples/dominoes/dominoes.arow" \
    "$ROOT_DIR/examples/dominoes/sim_config.yaml" \
    "$@"
