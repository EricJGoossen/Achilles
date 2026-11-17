#!/usr/bin/env bash
# The actuator demo: a four-wheeled robot driving itself around a loop
# of waypoints (examples/waypoint_robot). Its controller is the example's own
# program, examples/waypoint_robot/waypoint_robot.cpp, built as the waypoint_robot target.
# It opens a window each for the loop from above (waypoint_robot_camera.arow), the
# chassis's pose and its front wheels' speeds; close them to stop.
# The build runs at most JOBS (default 6) compile jobs at once: an unbounded
# one can run the machine out of memory.
# Builds an optimized binary into build-release/ -- a debug build is far too
# slow to run in real time with collision on. Extra arguments are passed
# through, e.g. --headless or --seconds 20.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target waypoint_robot -j"${JOBS:-6}"

exec "$BUILD_DIR/waypoint_robot" \
    "$ROOT_DIR/examples/waypoint_robot/waypoint_robot.arow" \
    "$ROOT_DIR/examples/waypoint_robot/sim_config.yaml" \
    "$@"
