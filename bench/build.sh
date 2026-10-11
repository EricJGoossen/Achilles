#!/usr/bin/env bash
# Builds bench/achilles_bench twice, in Release: once with the project's own
# flags (build-bench/, SSE2 -> 2 double lanes) and once with -march=native
# (build-bench-native/, AVX2 -> 4 double lanes). Reuses build/_deps/*-src
# when present so no re-download is needed; never touches build/ itself.
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
        -DCMAKE_BUILD_TYPE=Release "${DEP_ARGS[@]}" "$@" >/dev/null
    cmake --build "$ROOT_DIR/$dir" --target achilles_bench
}

build build-bench -DACHILLES_BENCH_NATIVE=OFF
build build-bench-native -DACHILLES_BENCH_NATIVE=ON
