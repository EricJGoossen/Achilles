#!/usr/bin/env bash
# The ball-on-plate demo: a plate tilting under a ball to keep it balanced
# -- catching it as it rolls off a steep start, then rolling it around a
# figure-8 -- through a delayed, noisy sensor (examples/ball_balance). Its
# own controller, examples/ball_balance/ball_balance.cpp, builds as the
# ball_balance target. It opens a window each for the plate and ball
# (plate_camera.arow), the ball's true pose and the plate's tilt about each
# pivot; close them to stop.
# The build runs at most JOBS (default 6) compile jobs at once: an unbounded
# one can run the machine out of memory.
# Builds an optimized binary into build-release/ -- a debug build is far too
# slow to run in real time with collision on. Extra arguments are passed
# through, e.g. --headless, --seconds 20 or --csv ball.csv.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target ball_balance -j"${JOBS:-6}"

exec "$BUILD_DIR/ball_balance" \
    "$ROOT_DIR/examples/ball_balance/ball_balance.arow" \
    "$ROOT_DIR/examples/ball_balance/sim_config.yaml" \
    "$@"
