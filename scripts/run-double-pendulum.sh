#!/usr/bin/env bash
# The double pendulum demo: the two-joint arm swinging freely under gravity,
# chaotic, run with RK4 at a 2 ms step, which keeps it swinging essentially
# indefinitely without visibly bleeding energy (examples/two_joint_arm). Its
# own program, examples/two_joint_arm/two_joint_arm.cpp, builds as the
# two_joint_arm target. It opens a window each for the arm through its
# camera, both joints' angles and both joints' rates; close them to stop.
# Builds an optimized binary into build-release/. Extra arguments are passed
# through, e.g. --headless, --seconds 20, or --integrator euler to watch a
# cruder integrator damp the swing out.
# The build runs at most JOBS (default 6) compile jobs at once: an unbounded
# one can run the machine out of memory.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-release"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target two_joint_arm -j"${JOBS:-6}"

exec "$BUILD_DIR/two_joint_arm" \
    "$ROOT_DIR/examples/two_joint_arm/two_joint_arm.arow" \
    "$ROOT_DIR/examples/two_joint_arm/sim_config.yaml" \
    "$@"
