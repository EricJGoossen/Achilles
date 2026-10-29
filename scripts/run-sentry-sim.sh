#!/usr/bin/env bash
# The sentry co-simulation (examples/sentry): ARUW's sentry on the ARC 2026
# field, its real MCB firmware (aruw-mcb's hosted build) and its real Jetson
# software (Project Otto) running against hardware emulated by Achilles.
#
# Everything runs in one Docker container (built from examples/sentry/docker/
# on top of .devcontainer's image) with the three checkouts mounted, as your
# user. The checkouts default to Achilles's siblings; set MCB_DIR / OTTO_DIR to
# use others.
#
#   scripts/run-sentry-sim.sh setup     image, container, submodules, patches, builds, Otto's env
#   scripts/run-sentry-sim.sh build     rebuild the sim and the firmware (run does this too)
#   scripts/run-sentry-sim.sh run [options]
#       --viewer             open Achilles's viewer (X11; WSLg works)
#       --seconds N          stop after N simulated seconds (default: until Ctrl-C)
#       --otto-config FILE   an Otto config layered over the sentry's, e.g.
#                            examples/sentry/routes/field_tour.yaml
#       --no-build           skip the incremental rebuild
#       anything else goes to sentry_sim (--autonav-at S, --calibrate-at S, --seed N)
#   scripts/run-sentry-sim.sh plot [RUN_DIR]   truth vs the MCB's estimate (default: latest run)
#   scripts/run-sentry-sim.sh shell     a shell in the container
#   scripts/run-sentry-sim.sh down      remove the container
#
# Runs go to build-sentry-sim/runs/<time>/ (build-sentry-sim/runs/latest):
# sim.log, mcb.log, otto.log, pose_log.csv, otto_logs/.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=achilles-sentry-sim
CONTAINER=achilles-sentry-sim
# Paths inside the container.
WS=/ws
ACH=$WS/Achilles
MCB=$WS/aruw-mcb
OTTO=$WS/project-otto
WORK=$ACH/build-sentry-sim
SIM_BIN=$WORK/sentry_sim/sentry_sim
MCB_ELF=$MCB/aruw-mcb-project/build/sim/scons-release/TARGET_SENTRY_ACHLYS/aruw-mcb.elf
MCB_PATCH=examples/sentry/patches/aruw-mcb-hosted-hardware-bridge.patch
OTTO_PATCH=examples/sentry/patches/project-otto-sim-hooks.patch

die() {
  echo "run-sentry-sim: $*" >&2
  exit 1
}

# ---------------------------------------------------------------------------
# Inside the container
# ---------------------------------------------------------------------------

inside_build() {
  export UV_CACHE_DIR=$WORK/uv-cache UV_PYTHON_INSTALL_DIR=$WORK/uv-python
  if [[ ! -f $WORK/sentry_sim/CMakeCache.txt ]]; then
    cmake -S "$ACH" -B "$WORK/sentry_sim" -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_COMPILER=clang++ >"$WORK/cmake-configure.log" 2>&1 ||
      { tail -n 30 "$WORK/cmake-configure.log"; die "configuring Achilles failed"; }
  fi
  echo "building sentry_sim..."
  cmake --build "$WORK/sentry_sim" --target sentry_sim -j"$(nproc)" >"$WORK/sim-build.log" 2>&1 ||
    { grep -E "error" -A3 "$WORK/sim-build.log" | head -n 40; die "building sentry_sim failed (see build-sentry-sim/sim-build.log)"; }

  echo "building the MCB firmware (SENTRY_ACHLYS, hosted)..."
  (cd "$MCB/aruw-mcb-project" && scons build-sim robot=SENTRY_ACHLYS -j"$(nproc)") >"$WORK/mcb-build.log" 2>&1 ||
    { grep -E " error|Error [0-9]" -A3 "$WORK/mcb-build.log" | head -n 40; die "building the firmware failed (see build-sentry-sim/mcb-build.log)"; }
}

inside_otto_env() {
  export UV_CACHE_DIR=$WORK/uv-cache UV_PYTHON_INSTALL_DIR=$WORK/uv-python
  echo "setting up Otto's environment..."
  (cd "$OTTO" && uv sync --frozen --quiet)
  # project_otto.main imports both even with the camera stubbed out.
  if ! "$OTTO/.venv/bin/python" -c "import pyrealsense2, gi" 2>/dev/null; then
    (cd "$OTTO" && uv pip install --quiet "pyrealsense2==2.56.*" "PyGObject==3.42.1")
  fi
}

inside_run() {
  local viewer=0 seconds=0 otto_config="" build=1
  local sim_args=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --viewer) viewer=1 ;;
      --seconds) seconds="$2"; shift ;;
      --otto-config) otto_config="$2"; shift ;;
      --no-build) build=0 ;;
      *) sim_args+=("$1") ;;
    esac
    shift
  done
  [[ $build == 1 ]] && inside_build

  # Globals: cleanup runs from the EXIT trap, after this function has returned.
  run_dir="$WORK/runs/$(date +%Y-%m-%d_%H-%M-%S)"
  mkdir -p "$run_dir"
  ln -sfn "$(basename "$run_dir")" "$WORK/runs/latest"
  printf '%s\n' "${sim_args[@]}" >"$run_dir/sim_args"
  [[ -n $otto_config ]] && cp "$otto_config" "$run_dir/otto_config.yaml"

  # Leftovers from a run that didn't get to clean up.
  pkill -f "$SIM_BIN" 2>/dev/null || true
  pkill -f "$MCB_ELF" 2>/dev/null || true
  pkill -f run_otto.py 2>/dev/null || true

  # Each in its own session: Ctrl-C reaches only this script, which stops them in order.
  local headless=(--headless)
  [[ $viewer == 1 ]] && headless=()
  sim_pid="" mcb_pid="" otto_pid=""
  # Each child leads its own process group (setsid), so a signal to -pid reaches its children too.
  stop_group() {
    local pid=$1 signal=$2 grace_tenths=$3
    [[ -n $pid ]] || return 0
    kill "-$signal" -- "-$pid" 2>/dev/null || return 0
    for _ in $(seq "$grace_tenths"); do
      kill -0 -- "-$pid" 2>/dev/null || return 0
      sleep 0.1
    done
    kill -KILL -- "-$pid" 2>/dev/null || true
  }
  cleanup() {
    trap - INT TERM EXIT
    # Otto first: it aborts if its serial devices vanish under it.
    stop_group "$otto_pid" INT 30
    stop_group "$mcb_pid" TERM 20
    stop_group "$sim_pid" TERM 20
    wait 2>/dev/null || true
    echo "run-sentry-sim: logs in build-sentry-sim/runs/$(basename "$run_dir")"
  }
  trap cleanup INT TERM EXIT

  setsid "$SIM_BIN" "$ACH/examples/sentry/field.arow" "$ACH/examples/sentry/sim_config.yaml" \
    "${headless[@]}" --run-dir "$run_dir" "${sim_args[@]}" \
    > >(tee -i "$run_dir/sim.log") 2>&1 &
  sim_pid=$!
  disown "$sim_pid" # stopped by cleanup; no job notices
  for _ in $(seq 100); do
    [[ -S $run_dir/mcb.sock ]] && break
    sleep 0.1
  done
  [[ -S $run_dir/mcb.sock ]] || die "the sim didn't open its MCB socket (see $run_dir/sim.log)"

  ARUW_SIM_HARDWARE_SOCKET=$run_dir/mcb.sock setsid "$MCB_ELF" >"$run_dir/mcb.log" 2>&1 &
  mcb_pid=$!
  disown "$mcb_pid"

  local otto_args=(--otto-dir "$OTTO" --run-dir "$run_dir" --silent)
  [[ -n $otto_config ]] && otto_args+=(--extra-config "$run_dir/otto_config.yaml")
  setsid "$OTTO/.venv/bin/python" -X faulthandler "$ACH/examples/sentry/otto_bridge/run_otto.py" \
    "${otto_args[@]}" >"$run_dir/otto.log" 2>&1 &
  otto_pid=$!
  disown "$otto_pid"

  echo "run-sentry-sim: running in build-sentry-sim/runs/$(basename "$run_dir") (Ctrl-C to stop)"
  # The sim runs until stopped (--seconds is watched here, so Otto can be stopped before the sim
  # takes its devices away) or until its viewer is closed.
  while kill -0 "$sim_pid" 2>/dev/null; do
    if [[ $seconds != 0 ]] &&
      awk -v limit="$seconds" '/^t=/ { t = $2 + 0 } END { exit !(t >= limit) }' "$run_dir/sim.log"; then
      break
    fi
    sleep 0.5
  done
}

inside_plot() {
  export UV_CACHE_DIR=$WORK/uv-cache UV_PYTHON_INSTALL_DIR=$WORK/uv-python
  local run_dir=${1:-$WORK/runs/latest}
  run_dir=$(readlink -f "$run_dir")
  local plot_args=("$run_dir/pose_log.csv" "$run_dir/plot.png")
  local autonav_at
  autonav_at=$(grep -A1 -x -- "--autonav-at" "$run_dir/sim_args" 2>/dev/null | tail -n 1 || true)
  [[ -n $autonav_at ]] && plot_args+=(--autonav-at "$autonav_at")
  [[ -f $run_dir/otto_config.yaml ]] && plot_args+=(--route "$run_dir/otto_config.yaml")
  uv run --quiet --no-project --with matplotlib --with pyyaml \
    python "$ACH/examples/sentry/tools/plot_run.py" "${plot_args[@]}"
  echo "run-sentry-sim: wrote ${run_dir#"$ACH"/}/plot.png"
}

if [[ ${1:-} == --inside ]]; then
  shift
  cmd=${1:-}
  shift || true
  case "$cmd" in
    build) inside_build ;;
    otto-env) inside_otto_env ;;
    run) inside_run "$@" ;;
    plot) inside_plot "$@" ;;
    *) die "unknown inside command '$cmd'" ;;
  esac
  exit 0
fi

# ---------------------------------------------------------------------------
# On the host
# ---------------------------------------------------------------------------

MCB_DIR=$(realpath -m "${MCB_DIR:-$ROOT_DIR/../aruw-mcb}")
OTTO_DIR=$(realpath -m "${OTTO_DIR:-$ROOT_DIR/../project-otto}")
MOUNTS="$ROOT_DIR:$MCB_DIR:$OTTO_DIR"

ensure_image() {
  if [[ ${REBUILD_IMAGE:-0} == 1 ]] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "building the $IMAGE image..."
    docker build -t achilles-dev "$ROOT_DIR/.devcontainer"
    docker build -t "$IMAGE" --build-arg BASE=achilles-dev "$ROOT_DIR/examples/sentry/docker"
  fi
}

ensure_container() {
  local state
  state=$(docker inspect -f '{{.State.Running}} {{index .Config.Labels "sentry-sim.mounts"}} {{.Image}}' \
    "$CONTAINER" 2>/dev/null || true)
  local image_id
  image_id=$(docker image inspect -f '{{.Id}}' "$IMAGE")
  if [[ -n $state && $state != *" $MOUNTS $image_id" ]]; then
    docker rm -f "$CONTAINER" >/dev/null
    state=""
  fi
  if [[ -z $state ]]; then
    [[ -d $MCB_DIR && -d $OTTO_DIR ]] || die "need aruw-mcb at $MCB_DIR and project-otto at $OTTO_DIR (or set MCB_DIR / OTTO_DIR)"
    mkdir -p "$ROOT_DIR/build-sentry-sim"
    docker run -d --init --name "$CONTAINER" --label "sentry-sim.mounts=$MOUNTS" \
      --user "$(id -u):$(id -g)" --network host \
      -v "$ROOT_DIR:$ACH" -v "$MCB_DIR:$MCB" -v "$OTTO_DIR:$OTTO" \
      -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY="${DISPLAY:-:0}" \
      -w "$ACH" "$IMAGE" sleep infinity >/dev/null
  elif [[ $state == false* ]]; then
    docker start "$CONTAINER" >/dev/null
  fi
}

in_container() {
  local tty=(-i)
  [[ -t 0 && -t 1 ]] && tty=(-it)
  docker exec "${tty[@]}" -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER" "$ACH/scripts/run-sentry-sim.sh" --inside "$@"
}

# The checkout's own patch, applied unless it already is.
apply_patch() {
  local dir=$1 patch=$ROOT_DIR/$2
  if git -C "$dir" apply --check "$patch" 2>/dev/null; then
    echo "applying $(basename "$patch") to $dir"
    git -C "$dir" apply "$patch"
  elif ! git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
    echo "run-sentry-sim: warning: $(basename "$patch") neither applies to $dir nor is applied; leaving it" >&2
  fi
}

# A host path under one of the mounted checkouts, as the container sees it.
container_path() {
  local path
  path=$(realpath "$1")
  case "$path" in
    "$ROOT_DIR"/*) echo "$ACH/${path#"$ROOT_DIR"/}" ;;
    "$MCB_DIR"/*) echo "$MCB/${path#"$MCB_DIR"/}" ;;
    "$OTTO_DIR"/*) echo "$OTTO/${path#"$OTTO_DIR"/}" ;;
    *) die "$1 must be inside $ROOT_DIR, $MCB_DIR or $OTTO_DIR" ;;
  esac
}

cmd=${1:-}
shift || true
case "$cmd" in
  setup)
    git -C "$MCB_DIR" submodule update --init segger-rtt
    git -C "$OTTO_DIR" submodule update --init --recursive
    apply_patch "$MCB_DIR" "$MCB_PATCH"
    apply_patch "$OTTO_DIR" "$OTTO_PATCH"
    ensure_image
    ensure_container
    in_container build
    in_container otto-env
    echo "run-sentry-sim: ready; try: scripts/run-sentry-sim.sh run --viewer --otto-config examples/sentry/routes/field_tour.yaml"
    ;;
  build)
    ensure_container
    in_container build
    ;;
  run)
    args=()
    while [[ $# -gt 0 ]]; do
      if [[ $1 == --otto-config ]]; then
        args+=("$1" "$(container_path "$2")")
        shift
      else
        args+=("$1")
      fi
      shift
    done
    ensure_container
    in_container run "${args[@]}"
    ;;
  plot)
    ensure_container
    if [[ $# -gt 0 ]]; then
      in_container plot "$(container_path "$1")"
    else
      in_container plot
    fi
    ;;
  shell)
    ensure_container
    docker exec -it "$CONTAINER" bash
    ;;
  down)
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    ;;
  *)
    sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
    exit 1
    ;;
esac
