#!/usr/bin/env bash
# Runs the long-horizon validation of algorithms::aba's own dynamics
# against the independently-derived closed-form double-pendulum model in
# tests/examples_two_joint_arm.cpp -- for minutes of simulated time rather
# than the two seconds the ordinary CI test covers.
#
# This is deliberately NOT a ctest case (the underlying test carries gtest's
# DISABLED_ prefix, so `ctest` skips it); it's a validation you run by hand
# when you want to convince yourself the engine still tracks the reference
# far out, not a gate on every build.
#
# What it checks, and why it isn't the obvious thing: the double pendulum is
# chaotic, so comparing *trajectories* over minutes is guaranteed to fail --
# a one-ulp difference grows exponentially, and the engine and the reference
# separate no matter how correct both are. So the assertion is instead
# per-tick: at every state the sim visits, does ABA's own qdd match the
# closed form's qdd for that same state? That error never accumulates, and
# a few minutes of chaotic wandering covers far more of configuration space
# than any short trajectory does. Trajectory divergence is printed too, but
# purely as an observable -- seeing where chaos takes over is the point, not
# a failure.
#
# Usage:
#   scripts/validate-long-horizon.sh [seconds] [integrator]
#     seconds     default 300
#     integrator  euler | verlet | midpoint | implicit | rk4  (default rk4 --
#                 the one this exists to double-check, since it's the one
#                 that doesn't visibly look wrong when watched)
set -euo pipefail

SECONDS_TO_RUN="${1:-300}"
INTEGRATOR="${2:-rk4}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G Ninja >/dev/null
cmake --build "$BUILD_DIR" --target examples_two_joint_arm -j"$(nproc)"

ACHILLES_LONG_HORIZON_SECONDS="$SECONDS_TO_RUN" \
ACHILLES_LONG_HORIZON_INTEGRATOR="$INTEGRATOR" \
    exec "$BUILD_DIR/tests/examples_two_joint_arm" \
    --gtest_also_run_disabled_tests \
    --gtest_filter='*DISABLED_LongHorizonDynamicsMatchClosedFormWithoutDrift'
