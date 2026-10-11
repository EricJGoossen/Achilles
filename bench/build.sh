#!/usr/bin/env bash
# Builds bench/achilles_bench three ways, in Release, logging compiled out:
#   build-bench/                float, the project's own flags (SSE2 on
#                               x86-64 -> 4 lanes)
#   build-bench-native/         float, -march=native (AVX2 -> 8 lanes)
#   build-bench-native-double/  double, -march=native (AVX2 -> 4 lanes) --
#                               the same precision as MuJoCo
# Reuses build/_deps/*-src when present so no re-download is needed; never
# touches build/ itself.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DEP_ARGS=()
for dep in eigen yaml-cpp glfw; do
    src="$ROOT_DIR/build/_deps/$dep-src"
    if [[ -d "$src" ]]; then
        upper="$(echo "$dep" | tr "[:lower:]" "[:upper:]")"
        DEP_ARGS+=("-DFETCHCONTENT_SOURCE_DIR_$upper=$src")
    fi
done

build() {
    local dir="$1"; shift
    cmake -S "$ROOT_DIR/bench" -B "$ROOT_DIR/$dir" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release -DACHILLES_ENABLE_LOGGING=OFF \
        "${DEP_ARGS[@]}" "$@" >/dev/null
    cmake --build "$ROOT_DIR/$dir" --target achilles_bench
}

build build-bench -DACHILLES_BENCH_NATIVE=OFF -DACHILLES_DOUBLE_PRECISION=OFF
build build-bench-native -DACHILLES_BENCH_NATIVE=ON \
    -DACHILLES_DOUBLE_PRECISION=OFF
build build-bench-native-double -DACHILLES_BENCH_NATIVE=ON \
    -DACHILLES_DOUBLE_PRECISION=ON
