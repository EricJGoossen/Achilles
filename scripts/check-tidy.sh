#!/usr/bin/env bash
# Fails if clang-tidy reports any warning under src/, include/, examples/,
# or tests/.
#
# This only lints the real src/*.cpp, examples/*/*.cpp, and tests/*.cpp
# translation units (via
# the project's compile database) -- it does NOT compile each header
# standalone. That relies on an invariant this script also enforces: every
# header under include/ must be reachable (via #include, transitively) from
# at least one src/*.cpp file, and every header under tests/support/ must
# be reachable from at least one tests/*.cpp file, so clang-tidy's
# --header-filter actually reaches it from a real TU. If a header falls out
# of its reachable set (e.g. a new header nothing includes yet), this
# script fails loudly on the "orphaned header" check below rather than
# silently skipping it.
#
# A tests/*.cpp file picks up tests/.clang-tidy (its nearest config, found
# by walking up from the file) instead of the root .clang-tidy -- that's
# plain clang-tidy config resolution, not something this script arranges.
#
# Checking every file on every run is slow, so a persistent cache in the
# build directory skips re-running clang-tidy on a .cpp file whose
# relevant content hasn't changed since it last passed clean. "Relevant
# content" is the file itself plus every project header it transitively
# includes (via `clang++ -M`) plus .clang-tidy and this script, so the
# cache invalidates itself whenever anything that could change the
# check's outcome changes -- not just the file itself. A file is cached
# ONLY when it passes: a file that's currently failing is always re-run
# and re-reported, every time, until it's actually fixed -- never
# silently skipped just because nobody happened to touch it in this
# particular session.
#
# For a quick check while iterating, --since REV runs only the files whose
# own content or any project header they include changed since git
# revision REV (committed, staged, unstaged or untracked) -- every file,
# if a tidy config or this script changed. Files it skips keep their cache
# entries, so it never marks anything clean it didn't check; the full run
# stays the gate.
#
# Usage: scripts/check-tidy.sh [--since REV] [build-dir]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SINCE=""
BUILD_DIR="$ROOT_DIR/build"
while [ "$#" -gt 0 ]; do
    case "$1" in
        --since)
            if [ "$#" -lt 2 ]; then
                echo "--since needs a git revision." >&2
                exit 2
            fi
            SINCE="$2"
            shift 2
            ;;
        *)
            BUILD_DIR="$1"
            shift
            ;;
    esac
done
COMPILE_DB="$BUILD_DIR/compile_commands.json"
CACHE_FILE="$BUILD_DIR/.check-tidy-cache"

if ! command -v clang-tidy >/dev/null 2>&1; then
    echo "clang-tidy not found in PATH." >&2
    exit 127
fi

if [ ! -f "$COMPILE_DB" ]; then
    echo "compile_commands.json not found at $COMPILE_DB -- configure/build the project first." >&2
    exit 1
fi

# A tests/*.cpp TU resolves tests/.clang-tidy (a much broader `*,`-style
# check list than the root config's curated allowlist), and clang-tidy
# picks the active check set from the *main* file being analyzed, not per
# header. Reusing one shared header-filter would mean every include/
# header gets re-litigated under that broader ruleset the moment any
# tests/*.cpp file reaches it -- re-flagging things root's curated list
# deliberately never enabled, for a reason that has nothing to do with the
# header's own quality. So each file type gets a filter scoped to its own
# tree: a src/*.cpp check still opens up to src/include (a src/*.cpp
# reaching into tests/ would be a layering bug worth seeing), and a
# tests/*.cpp check is scoped to tests/ alone -- shared headers stay
# governed by whichever config the src/*.cpp side already uses.
SRC_HEADER_FILTER="^${ROOT_DIR}/(src|include)/.*"
# examples/*/*.cpp resolve the root .clang-tidy, same as src/*.cpp, so they
# share its include/ scope plus their own example directory.
EXAMPLE_HEADER_FILTER="^${ROOT_DIR}/(src|include|examples)/.*"
TEST_HEADER_FILTER="^${ROOT_DIR}/tests/.*"
GOOGLETEST_INCLUDE_DIR="$BUILD_DIR/_deps/googletest-src/googletest/include"
YAML_CPP_INCLUDE_DIR="$BUILD_DIR/_deps/yaml-cpp-src/include"
GLFW_INCLUDE_DIR="$BUILD_DIR/_deps/glfw-src/include"
DEP_ARGS=(-x c++ -std=c++20 -I"$ROOT_DIR/src" -I"$ROOT_DIR/include" -isystem"$GOOGLETEST_INCLUDE_DIR" -isystem"$YAML_CPP_INCLUDE_DIR" -isystem"$GLFW_INCLUDE_DIR")

# Hashes both .clang-tidy configs (root and tests/ -- a tests/*.cpp file
# resolves tests/.clang-tidy as its nearest config, not the root one) and
# this script itself, so a config or flag change invalidates every cache
# entry at once, the same as a source edit would.
CONFIG_HASH="$(cat "$ROOT_DIR/.clang-tidy" "$ROOT_DIR/tests/.clang-tidy" "${BASH_SOURCE[0]}" | sha256sum | awk '{print $1}')"

# Prints one project header/source path per line that $1 transitively
# includes (via the preprocessor, so it's exact, not a guess), restricted
# to files under src/, include/, examples/, or tests/ -- third-party headers (yaml-cpp,
# xsimd, googletest) are deliberately excluded: they're pinned by
# CMakeLists.txt, not edited here, and hashing them on every run would be
# pure overhead.
project_deps_of() {
    clang++ "${DEP_ARGS[@]}" -M "$1" 2>/dev/null \
        | sed 's/^[^:]*://' \
        | tr -d '\\' \
        | tr -s ' \t\n' '\n' \
        | sed '/^$/d' \
        | grep -E "^${ROOT_DIR}/(src|include|examples|tests)/" \
        | sort -u
}

# A file's fingerprint is the config hash plus the content hash of every
# file project_deps_of finds -- so it changes if the file itself changes,
# if any header it includes changes, or if .clang-tidy/this script change.
fingerprint_of() {
    {
        echo "$CONFIG_HASH"
        printf '%s\n' "$@" | xargs -r sha256sum
    } | sha256sum | awk '{print $1}'
}

# --since REV: every project path changed since REV, as absolute paths. A
# changed tidy config or this script means every file is affected.
declare -A CHANGED
SINCE_ALL=0
if [ -n "$SINCE" ]; then
    if ! git -C "$ROOT_DIR" rev-parse --verify --quiet "$SINCE^{commit}" >/dev/null; then
        echo "--since: '$SINCE' isn't a git revision." >&2
        exit 2
    fi
    while IFS= read -r path; do
        [ -n "$path" ] || continue
        CHANGED["$ROOT_DIR/$path"]=1
        case "$path" in
            .clang-tidy | tests/.clang-tidy | scripts/check-tidy.sh) SINCE_ALL=1 ;;
        esac
    done < <(
        git -C "$ROOT_DIR" diff --name-only "$SINCE"
        git -C "$ROOT_DIR" ls-files --others --exclude-standard
    )
fi

# Whether any of the given paths changed since --since's revision (always,
# without --since).
affected() {
    if [ -z "$SINCE" ] || [ "$SINCE_ALL" = "1" ]; then
        return 0
    fi
    local path
    for path in "$@"; do
        if [ -n "${CHANGED[$path]:-}" ]; then
            return 0
        fi
    done
    return 1
}

declare -A CACHE
if [ -f "$CACHE_FILE" ]; then
    while IFS=$'\t' read -r unit hash; do
        [ -n "$unit" ] && CACHE["$unit"]="$hash"
    done < "$CACHE_FILE"
fi
declare -A NEW_CACHE

STATUS=0
CHECKED_COUNT=0
SKIPPED_COUNT=0

# clang-tidy invocations are independent per translation unit and dominate
# the runtime, so they run concurrently (bounded by JOBS) instead of one at
# a time. Fingerprinting stays sequential -- it's just `clang++ -M` plus
# hashing, not a full clang-tidy run, so it's cheap relative to the analysis
# itself and doesn't need parallelizing.
# One clang-tidy over a sim-heavy file takes 1-2 GB, so by default run as
# many at once as both the cores and the available memory allow (2 GB
# each) -- one per core alone outruns memory on a 16-core, 16 GB machine,
# and the kernel kills or swaps jobs instead. CHECK_TIDY_JOBS overrides it.
default_jobs() {
    local cores available_kb by_memory
    cores="$(nproc 2>/dev/null || echo 4)"
    available_kb="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
    by_memory=$((available_kb / (2 * 1024 * 1024)))
    if [ "$by_memory" -lt 1 ]; then
        by_memory=1
    fi
    if [ "$by_memory" -lt "$cores" ]; then
        echo "$by_memory"
    else
        echo "$cores"
    fi
}
JOBS="${CHECK_TIDY_JOBS:-$(default_jobs)}"
# A fresh, unique directory per invocation (not a fixed name under
# $BUILD_DIR) -- two check-tidy.sh runs against the same build dir (e.g.
# a local run overlapping with CI, or just two terminals) would otherwise
# race on the same result files, each clobbering the other's in-flight
# output.
RESULT_DIR="$(mktemp -d "$BUILD_DIR/.check-tidy-parallel.XXXXXX")"
trap 'rm -rf "$RESULT_DIR"' EXIT

mapfile -t SRC_FILES < <(find "$ROOT_DIR/src" -name '*.cpp' | sort)
mapfile -t EXAMPLE_FILES < <(find "$ROOT_DIR/examples" -name '*.cpp' | sort)
# Non-recursive, mirroring tests/CMakeLists.txt's own glob (see
# TESTING.md §1): tests/support/ holds shared headers only, never its own
# test executables.
mapfile -t TEST_FILES < <(find "$ROOT_DIR/tests" -maxdepth 1 -name '*.cpp' | sort)

ALL_UNITS=()
ALL_FILTERS=()
for src in "${SRC_FILES[@]}"; do
    ALL_UNITS+=("$src")
    ALL_FILTERS+=("$SRC_HEADER_FILTER")
done
for example_src in "${EXAMPLE_FILES[@]}"; do
    ALL_UNITS+=("$example_src")
    ALL_FILTERS+=("$EXAMPLE_HEADER_FILTER")
done
for test_src in "${TEST_FILES[@]}"; do
    ALL_UNITS+=("$test_src")
    ALL_FILTERS+=("$TEST_HEADER_FILTER")
done

# Phase 1: resolve which units are already cached as passing (cheap -- no
# clang-tidy invocation involved) versus which actually need to be checked.
TO_RUN_FILES=()
TO_RUN_FILTERS=()
TO_RUN_FPS=()
for i in "${!ALL_UNITS[@]}"; do
    file="${ALL_UNITS[$i]}"
    rel="${file#"$ROOT_DIR"/}"
    mapfile -t deps < <(project_deps_of "$file")
    if ! affected "$file" "${deps[@]}"; then
        if [ -n "${CACHE[$rel]:-}" ]; then
            NEW_CACHE["$rel"]="${CACHE[$rel]}"
        fi
        SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
        continue
    fi
    fp="$(fingerprint_of "${deps[@]}")"

    if [ "${CACHE[$rel]:-}" = "$fp" ]; then
        NEW_CACHE["$rel"]="$fp"
        SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
        continue
    fi

    TO_RUN_FILES+=("$file")
    TO_RUN_FILTERS+=("${ALL_FILTERS[$i]}")
    TO_RUN_FPS+=("$fp")
done
CHECKED_COUNT="${#TO_RUN_FILES[@]}"

# Phase 2: run the actual clang-tidy invocations, up to JOBS concurrently.
# Each writes its output/exit status to its own file (indexed by position
# in TO_RUN_FILES) instead of the terminal, so results are printed back out
# in a fixed order below -- grouped by file, not interleaved by whichever
# job happens to finish first.
run_check() {
    local file="$1" header_filter="$2" idx="$3"
    local rc=0
    # `set -e` (inherited from the parent script) would otherwise kill this
    # background subshell the instant clang-tidy exits non-zero, skipping
    # the status write below and silently losing the result for that file.
    clang-tidy -p "$BUILD_DIR" --quiet \
        --header-filter="$header_filter" \
        --warnings-as-errors='*' \
        "$file" > "$RESULT_DIR/$idx.log" 2>&1 || rc=$?
    echo "$rc" > "$RESULT_DIR/$idx.status"
}

if [ "${#TO_RUN_FILES[@]}" -gt 0 ]; then
    echo "-- clang-tidy: checking ${#TO_RUN_FILES[@]} translation unit(s) (${JOBS} parallel jobs) --"
    running=0
    for i in "${!TO_RUN_FILES[@]}"; do
        run_check "${TO_RUN_FILES[$i]}" "${TO_RUN_FILTERS[$i]}" "$i" &
        running=$((running + 1))
        if [ "$running" -ge "$JOBS" ]; then
            # `wait -n`'s own exit status is the finished job's exit status
            # (run_check always exits 0 itself, but a stray failure here
            # shouldn't kill the whole script under `set -e` -- every
            # actual clang-tidy failure is already captured per-file in
            # its .status file below).
            wait -n || true
            running=$((running - 1))
        fi
    done
    wait || true

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

if [ -n "$SINCE" ]; then
    echo "check-tidy: ${CHECKED_COUNT} checked, ${SKIPPED_COUNT} unchanged since last clean run or since ${SINCE}"
else
    echo "check-tidy: ${CHECKED_COUNT} checked, ${SKIPPED_COUNT} unchanged since last clean run"
fi

# Enforce the invariant this whole approach depends on: every header must
# be reachable from some src/*.cpp, or clang-tidy never actually sees it.
mapfile -t ALL_HEADERS < <(find "$ROOT_DIR/include" -name '*.hpp' | sed "s#^${ROOT_DIR}/include/##" | sort -u)
mapfile -t REACHABLE < <(
    for src in "${SRC_FILES[@]}"; do
        project_deps_of "$src"
    done | grep -E "^${ROOT_DIR}/include/" | sed "s#^${ROOT_DIR}/include/##" | sort -u
)
mapfile -t ORPHANED < <(comm -23 <(printf '%s\n' "${ALL_HEADERS[@]}") <(printf '%s\n' "${REACHABLE[@]}"))
if [ "${#ORPHANED[@]}" -gt 0 ]; then
    echo "check-tidy: the following header(s) aren't reachable from any src/*.cpp file," >&2
    echo "so clang-tidy never actually checks them -- #include them (directly or" >&2
    echo "transitively) from a real .cpp, or this check can't see them:" >&2
    printf '  include/%s\n' "${ORPHANED[@]}" >&2
    STATUS=1
fi

# Same invariant, for tests/support/ against tests/*.cpp -- a shared test
# fake/archetype/builder nothing under tests/ actually includes would
# otherwise never get linted either.
mapfile -t ALL_TEST_HEADERS < <(find "$ROOT_DIR/tests/support" -name '*.hpp' 2>/dev/null | sed "s#^${ROOT_DIR}/tests/support/##" | sort -u)
if [ "${#ALL_TEST_HEADERS[@]}" -gt 0 ]; then
    mapfile -t TEST_REACHABLE < <(
        for test_src in "${TEST_FILES[@]}"; do
            project_deps_of "$test_src"
        done | grep -E "^${ROOT_DIR}/tests/support/" | sed "s#^${ROOT_DIR}/tests/support/##" | sort -u
    )
    mapfile -t TEST_ORPHANED < <(comm -23 <(printf '%s\n' "${ALL_TEST_HEADERS[@]}") <(printf '%s\n' "${TEST_REACHABLE[@]}"))
    if [ "${#TEST_ORPHANED[@]}" -gt 0 ]; then
        echo "check-tidy: the following tests/support/ header(s) aren't reachable from" >&2
        echo "any tests/*.cpp file, so clang-tidy never actually checks them -- #include" >&2
        echo "them (directly or transitively) from a real tests/*.cpp, or this check" >&2
        echo "can't see them:" >&2
        printf '  tests/support/%s\n' "${TEST_ORPHANED[@]}" >&2
        STATUS=1
    fi
fi

exit $STATUS
