#!/usr/bin/env bash
# Fails if clang-tidy reports any warning under src/ or include/.
#
# This only lints the real src/*.cpp translation units (via the project's
# compile database) -- it does NOT compile each header standalone. That
# relies on an invariant this script also enforces: every header under
# include/ must be reachable (via #include, transitively) from at least
# one src/*.cpp file, so clang-tidy's --header-filter actually reaches it
# from a real TU. If a header falls out of that reachable set (e.g. a new
# header nothing includes yet), this script fails loudly on the "orphaned
# header" check below rather than silently skipping it.
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
# Usage: scripts/check-tidy.sh [build-dir]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-$ROOT_DIR/build}"
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

HEADER_FILTER="^${ROOT_DIR}/(src|include)/.*"
EIGEN_INCLUDE_DIR="$BUILD_DIR/_deps/eigen-src"
DEP_ARGS=(-x c++ -std=c++20 -I"$ROOT_DIR/src" -I"$ROOT_DIR/include" -isystem"$EIGEN_INCLUDE_DIR")

# Hashes .clang-tidy and this script itself, so a config or flag change
# invalidates every cache entry at once, the same as a source edit would.
CONFIG_HASH="$(cat "$ROOT_DIR/.clang-tidy" "${BASH_SOURCE[0]}" | sha256sum | awk '{print $1}')"

# Prints one project header/source path per line that $1 transitively
# includes (via the preprocessor, so it's exact, not a guess), restricted
# to files under src/ or include/ -- third-party headers (Eigen, xsimd)
# are deliberately excluded: they're pinned by CMakeLists.txt, not edited
# here, and hashing them on every run would be pure overhead.
project_deps_of() {
    clang++ "${DEP_ARGS[@]}" -M "$1" 2>/dev/null \
        | sed 's/^[^:]*://' \
        | tr -d '\\' \
        | tr -s ' \t\n' '\n' \
        | sed '/^$/d' \
        | grep -E "^${ROOT_DIR}/(src|include)/" \
        | sort -u
}

# A file's fingerprint is the config hash plus the content hash of every
# file project_deps_of finds -- so it changes if the file itself changes,
# if any header it includes changes, or if .clang-tidy/this script change.
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
CHECKED_COUNT=0
SKIPPED_COUNT=0

# Runs clang-tidy on $1 (a src/*.cpp file) against the real compile
# database, unless its fingerprint is already cached as passing.
check_unit() {
    local file="$1"
    local rel="${file#"$ROOT_DIR"/}"
    local fp
    fp="$(fingerprint_of "$file")"

    if [ "${CACHE[$rel]:-}" = "$fp" ]; then
        NEW_CACHE["$rel"]="$fp"
        SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
        return
    fi

    CHECKED_COUNT=$((CHECKED_COUNT + 1))
    if clang-tidy -p "$BUILD_DIR" --quiet \
        --header-filter="$HEADER_FILTER" \
        --warnings-as-errors='*' \
        "$file"; then
        NEW_CACHE["$rel"]="$fp"
    else
        STATUS=1
    fi
}

mapfile -t SRC_FILES < <(find "$ROOT_DIR/src" -name '*.cpp' | sort)
if [ "${#SRC_FILES[@]}" -gt 0 ]; then
    echo "-- clang-tidy: src/*.cpp translation units --"
    for src in "${SRC_FILES[@]}"; do
        check_unit "$src"
    done
fi

{
    for unit in "${!NEW_CACHE[@]}"; do
        printf '%s\t%s\n' "$unit" "${NEW_CACHE[$unit]}"
    done
} > "$CACHE_FILE"

echo "check-tidy: ${CHECKED_COUNT} checked, ${SKIPPED_COUNT} unchanged since last clean run"

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

exit $STATUS
