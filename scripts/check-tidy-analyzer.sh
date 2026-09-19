#!/usr/bin/env bash
# Runs the Clang Static Analyzer checks (.clang-tidy-analyzer) in isolation
# from scripts/check-tidy.sh's main AST-matcher lint pass.
#
# Why a separate script rather than a flag on check-tidy.sh: the analyzer
# is symbolic execution, not a lexical/AST-matcher pass -- far more
# expensive per translation unit than every other clang-tidy check
# combined. Keeping it as its own executable means it gets its own cache
# (an ordinary .clang-tidy edit doesn't invalidate analyzer results, and
# vice versa) and its own CI job, so the fast routine lint and the slow
# analyzer pass never block or invalidate each other. See
# .clang-tidy-analyzer for why this checker subset specifically.
#
# Mirrors check-tidy.sh's translation-unit discovery, fingerprint cache,
# and parallel-execution model; see that script's comments for the
# reasoning behind each. Does NOT repeat check-tidy.sh's orphaned-header
# invariant check -- that enforces coverage (every header reachable from a
# real TU), already asserted once by the main script, and this script
# analyzes the exact same TU list so there's nothing more to enforce here.
#
# Usage: scripts/check-tidy-analyzer.sh [build-dir]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-$ROOT_DIR/build}"
COMPILE_DB="$BUILD_DIR/compile_commands.json"
CACHE_FILE="$BUILD_DIR/.check-tidy-analyzer-cache"
CONFIG_FILE="$ROOT_DIR/.clang-tidy-analyzer"

if ! command -v clang-tidy >/dev/null 2>&1; then
    echo "clang-tidy not found in PATH." >&2
    exit 127
fi

if [ ! -f "$COMPILE_DB" ]; then
    echo "compile_commands.json not found at $COMPILE_DB -- configure/build the project first." >&2
    exit 1
fi

# Unlike check-tidy.sh, there's no src-vs-tests config split to worry about
# here: --config-file below pins every TU to the exact same analyzer
# config regardless of directory, so one header filter covers all of them.
HEADER_FILTER="^${ROOT_DIR}/(src|include|tests)/.*"
EIGEN_INCLUDE_DIR="$BUILD_DIR/_deps/eigen-src"
GOOGLETEST_INCLUDE_DIR="$BUILD_DIR/_deps/googletest-src/googletest/include"
YAML_CPP_INCLUDE_DIR="$BUILD_DIR/_deps/yaml-cpp-src/include"
GLFW_INCLUDE_DIR="$BUILD_DIR/_deps/glfw-src/include"
DEP_ARGS=(-x c++ -std=c++20 -I"$ROOT_DIR/src" -I"$ROOT_DIR/include" -isystem"$EIGEN_INCLUDE_DIR" -isystem"$GOOGLETEST_INCLUDE_DIR" -isystem"$YAML_CPP_INCLUDE_DIR" -isystem"$GLFW_INCLUDE_DIR")

# Hashes .clang-tidy-analyzer and this script itself, so a config or flag
# change invalidates every cache entry at once, the same as a source edit
# would. Deliberately independent of check-tidy.sh's own CONFIG_HASH (its
# .clang-tidy / tests/.clang-tidy inputs) -- the two caches don't share
# invalidation triggers, since neither run's outcome depends on the
# other's config.
CONFIG_HASH="$(cat "$CONFIG_FILE" "${BASH_SOURCE[0]}" | sha256sum | awk '{print $1}')"

# See check-tidy.sh for why this is exact (via the preprocessor) rather
# than a guess, and why third-party headers are excluded from hashing.
project_deps_of() {
    clang++ "${DEP_ARGS[@]}" -M "$1" 2>/dev/null \
        | sed 's/^[^:]*://' \
        | tr -d '\\' \
        | tr -s ' \t\n' '\n' \
        | sed '/^$/d' \
        | grep -E "^${ROOT_DIR}/(src|include|tests)/" \
        | sort -u
}

fingerprint_of() {
    {
        echo "$CONFIG_HASH"
        project_deps_of "$1" | xargs -r sha256sum
    } | sha256sum | awk '{print $1}'
}

declare -A CACHE
if [ -f "$CACHE_FILE" ]; then
    while IFS=$'\t' read -r unit hash; do
        [ -n "$unit" ] && CACHE["$unit"]="$hash"
    done < "$CACHE_FILE"
fi
declare -A NEW_CACHE

STATUS=0
JOBS="${CHECK_TIDY_JOBS:-$(nproc 2>/dev/null || echo 4)}"
# A fresh, unique directory per invocation -- see check-tidy.sh's identical
# comment for why a fixed name under $BUILD_DIR isn't safe here.
RESULT_DIR="$(mktemp -d "$BUILD_DIR/.check-tidy-analyzer-parallel.XXXXXX")"
trap 'rm -rf "$RESULT_DIR"' EXIT

mapfile -t SRC_FILES < <(find "$ROOT_DIR/src" -name '*.cpp' | sort)
# Non-recursive, mirroring tests/CMakeLists.txt's own glob (see
# TESTING.md §1): tests/support/ holds shared headers only, never its own
# test executables.
mapfile -t TEST_FILES < <(find "$ROOT_DIR/tests" -maxdepth 1 -name '*.cpp' | sort)
ALL_UNITS=("${SRC_FILES[@]}" "${TEST_FILES[@]}")

# Phase 1: resolve which units are already cached as passing (cheap -- no
# clang-tidy invocation involved) versus which actually need to be checked.
TO_RUN_FILES=()
TO_RUN_FPS=()
SKIPPED_COUNT=0
for file in "${ALL_UNITS[@]}"; do
    rel="${file#"$ROOT_DIR"/}"
    fp="$(fingerprint_of "$file")"

    if [ "${CACHE[$rel]:-}" = "$fp" ]; then
        NEW_CACHE["$rel"]="$fp"
        SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
        continue
    fi

    TO_RUN_FILES+=("$file")
    TO_RUN_FPS+=("$fp")
done
CHECKED_COUNT="${#TO_RUN_FILES[@]}"

# Phase 2: run the actual clang-tidy invocations, up to JOBS concurrently.
# See check-tidy.sh for why results are buffered per-file and printed back
# in order, and why the invocation is wrapped in `|| rc=$?` rather than
# checked via a bare `if` after the fact.
run_check() {
    local file="$1" idx="$2"
    local rc=0
    clang-tidy -p "$BUILD_DIR" --quiet \
        --config-file="$CONFIG_FILE" \
        --header-filter="$HEADER_FILTER" \
        --warnings-as-errors='*' \
        "$file" > "$RESULT_DIR/$idx.log" 2>&1 || rc=$?
    echo "$rc" > "$RESULT_DIR/$idx.status"
}

if [ "${#TO_RUN_FILES[@]}" -gt 0 ]; then
    echo "-- clang-tidy-analyzer: checking ${#TO_RUN_FILES[@]} translation unit(s) (${JOBS} parallel jobs) --"
    running=0
    for i in "${!TO_RUN_FILES[@]}"; do
        run_check "${TO_RUN_FILES[$i]}" "$i" &
        running=$((running + 1))
        if [ "$running" -ge "$JOBS" ]; then
            wait -n
            running=$((running - 1))
        fi
    done
    wait

    for i in "${!TO_RUN_FILES[@]}"; do
        rel="${TO_RUN_FILES[$i]#"$ROOT_DIR"/}"
        cat "$RESULT_DIR/$i.log"
        if [ "$(cat "$RESULT_DIR/$i.status")" = "0" ]; then
            NEW_CACHE["$rel"]="${TO_RUN_FPS[$i]}"
        else
            STATUS=1
        fi
    done
fi

{
    for unit in "${!NEW_CACHE[@]}"; do
        printf '%s\t%s\n' "$unit" "${NEW_CACHE[$unit]}"
    done
} > "$CACHE_FILE"

echo "check-tidy-analyzer: ${CHECKED_COUNT} checked, ${SKIPPED_COUNT} unchanged since last clean run"

exit $STATUS
