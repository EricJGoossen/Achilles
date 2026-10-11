#!/usr/bin/env bash
# Achilles half of the MuJoCo comparison -- run inside the devcontainer
# after bench/build.sh. Writes into bench/results/:
#   achilles_throughput.jsonl   one line per (build, scene, integrator, N)
#   traces/<scene>_<integrator>.csv   single-robot state traces for
#                                     compare_mujoco.py
#
# Usage: bench/run_achilles.sh [throughput|traces|all]   (default all)
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT_DIR/bench/results"
CONFIG="$ROOT_DIR/examples/sim_config.yaml"
WHAT="${1:-all}"
DT=0.002

# Accuracy traces: every scene. Throughput: THROUGHPUT_SCENES.
SCENES=(
    "$ROOT_DIR/examples/two_joint_arm.arow"
    "$ROOT_DIR/examples/three_joint_arm.arow"
    "$ROOT_DIR/examples/asymmetric_two_joint_arm.arow"
    "$ROOT_DIR/bench/scenes/two_joint_arm_small_swing.arow"
)
THROUGHPUT_SCENES=(
    "$ROOT_DIR/examples/two_joint_arm.arow"
    "$ROOT_DIR/examples/three_joint_arm.arow"
    "$ROOT_DIR/bench/scenes/seven_joint_arm.arow"
)
# See bench/build.sh: float with the project's own flags (SSE2 on x86-64),
# float with -march=native, double with -march=native.
BUILDS=(build-bench build-bench-native build-bench-native-double)
BATCHES=(1 4 16 64 256 1024 4096)

mkdir -p "$OUT/traces"

if [[ "$WHAT" == throughput || "$WHAT" == all ]]; then
    : > "$OUT/achilles_throughput.jsonl"
    for build in "${BUILDS[@]}"; do
        for scene in "${THROUGHPUT_SCENES[@]}"; do
            for integrator in euler rk4; do
                for n in "${BATCHES[@]}"; do
                    # ~1M robot-steps per trial keeps every point to a
                    # second or two of wall time.
                    steps=$(( 1000000 / n ))
                    (( steps > 20000 )) && steps=20000
                    (( steps < 200 )) && steps=200
                    line=$("$ROOT_DIR/$build/achilles_bench" throughput \
                        "$scene" "$CONFIG" "$integrator" "$DT" "$n" "$steps" 3)
                    line="${line%\}}, \"build\": \"$build\", \"scene\": \"$(basename "$scene" .arow)\"}"
                    echo "$line" | tee -a "$OUT/achilles_throughput.jsonl"
                done
            done
        done
    done
fi

if [[ "$WHAT" == traces || "$WHAT" == all ]]; then
    for scene in "${SCENES[@]}"; do
        for integrator in euler rk4; do
            name="$(basename "$scene" .arow)_$integrator"
            # 20 s at 2 ms.
            "$ROOT_DIR/build-bench-native/achilles_bench" trace \
                "$scene" "$CONFIG" "$integrator" "$DT" 10000 \
                "$OUT/traces/$name.csv"
            echo "trace $name"
        done
    done
fi
