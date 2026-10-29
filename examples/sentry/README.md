# Sentry co-simulation (work in progress)

ARUW's sentry (aruw-mcb `SENTRY_ACHLYS`) on the ARC 2026 field, running its **real MCB
firmware** (aruw-mcb hosted build) and its **real Jetson software** (Project Otto's
`MainApplication`) against hardware emulated by Achilles. The repo split is kept: each program
is its own process from its own checkout, connected the way the robot wires them.

```
aruw-mcb (hosted) <-- CAN frames, UART bytes, IMU register bursts, sim clock --> sentry_sim
                      (Unix SOCK_SEQPACKET, <run-dir>/mcb.sock)
Project Otto      <-- MCB UART  (pty <run-dir>/mcb_uart)
                  <-- 2x LD19   (ptys <run-dir>/lidar_front, lidar_rear)
                  <-- camera: blank 60 Hz frames (otto_bridge/blank_camera.py; not modelled yet)
```

## Pieces

- `sentry_sim.cpp` — physics (500 Hz) + hardware emulation (1 kHz traffic), real-time paced.
  Writes `<run-dir>/pose_log.csv`: truth vs the MCB's estimate, overheard on its UART.
- `hw/` — DJI motors (C620/M3508, GM6020 current mode), turret-major pulley encoder + Lamprey,
  turret-MCB IMU boards (Mahony, CAN frames), MPU6500/ISM330 register bursts, DR16 remote with an
  operator script (IMU calibrate at `--calibrate-at`, autonav at `--autonav-at`), referee system,
  LD19 packets, DJI-serial framing/CRCs.
- `tools/generate_scene.py` — generates `field.arow`, `sentry.arow`, `sensors/*.arow`.
- `otto_bridge/run_otto.py` — runs Otto's own app with the sim's device paths and injected
  camera/detector/video factories.
- `patches/` — the changes the sim needs in aruw-mcb and project-otto (`setup` applies them).
- `routes/` — Otto configs that send the sentry around the field.
- `tools/plot_run.py` — plots a run's `pose_log.csv`.
- `docker/` — the container `scripts/run-sentry-sim.sh` runs everything in.

## Running

`scripts/run-sentry-sim.sh` does it all in one Docker container (built from `docker/` on top of
`.devcontainer`'s image), with the three checkouts mounted and running as your user. It expects
aruw-mcb and project-otto next to Achilles; set `MCB_DIR` / `OTTO_DIR` otherwise.

```bash
# Once: image + container, submodules, applies patches/ to the two checkouts (unless already
# applied), builds the sim and the firmware, sets up Otto's virtualenv.
scripts/run-sentry-sim.sh setup

# The dev loop: rebuilds whatever changed (sim and firmware), then runs the sim, the MCB and Otto.
# Ctrl-C stops all three (Otto first).
scripts/run-sentry-sim.sh run --viewer --otto-config examples/sentry/routes/field_tour.yaml
scripts/run-sentry-sim.sh run --seconds 120              # headless, stops itself
scripts/run-sentry-sim.sh plot                            # truth vs estimate for the latest run
```

- **Where it goes**: Otto's targeting state machine drives to `initial_push_setpoint`, then loops
  through `center_path_setpoints`, one every `setpoint_interval_duration`. Copy
  `routes/field_tour.yaml` and edit the points (field metres; obstacles in
  `tools/generate_scene.py`). `--otto-config` takes any Otto config, layered over the sentry's.
- **Timing**: IMU calibration is requested at 3 s and takes until ~18 s; autonav comes on at 25 s
  (`--calibrate-at`, `--autonav-at`; any other option goes to `sentry_sim`).
- **Runs** land in `build-sentry-sim/runs/<time>/` (`runs/latest`): `sim.log` (the status line
  printed each second), `mcb.log`, `otto.log` (crashes only; Otto's own logs are in
  `otto_logs/`), `pose_log.csv`, `plot.png`.
- **Changes**: edits to the sim, the firmware (aruw-mcb) or Otto take effect on the next `run`;
  Otto is plain Python. `scripts/run-sentry-sim.sh shell` opens a shell in the container.
- **Viewer**: `--viewer` opens Achilles's window over X11 (WSLg's works). Closing it ends the run.
  The camera orbits the world origin, the field's red-side corner (drag to orbit, scroll to zoom;
  scroll out ~15 m to see the field). The yellow lines are the viewer's joint skeleton: the planar
  chassis joint hangs off the origin.
- `ARUW_SIM_DEBUG=1` on the MCB prints per-second CAN delivery counts.

Notes:

- The sim holds its clock at zero until the MCB connects, so the processes can start in any order.
- The sim and Otto must share a machine (the sim's ptys are Otto's devices); the MCB needs only
  the socket.
- Otto needs `pyrealsense2` and `PyGObject` beyond its locked dependencies (and `libusb-1.0-0`,
  `libgirepository1.0-dev`, `libcairo2-dev`, `gir1.2-gstreamer-1.0`): `project_otto.main` imports
  both even with the camera stubbed out. The container has them.
- On WSL2 the wall clock can step back by seconds. Otto's clock is `CLOCK_REALTIME`, so it drops
  MCB odometry ("not newer than existing latest timestamp") until the clock catches up; the blank
  camera paces on the monotonic clock so the main loop keeps running through it.

## Modelling notes

- Chassis is a planar joint; X-drive omni wheels are an explicit traction model (static friction
  solved per step, capped at μN; near-free rollers). Masses/heights not in the firmware are
  estimates.
- The MCB's clocks start at zero when its process starts, as the board's do at boot, then follow
  the sim's time stamps.

## Firmware behaviour the sim shows (left as is)

- Starting off, the wheels break traction: `ChassisAutoNavController` ramps translation by
  `MAX_TRANSLATION_ACCELERATION` (1.0) per call, not per second, so it commands a speed step. The
  wheel odometry overcounts each start by ~0.25 m (μ = 0.8 here).
- Yaw drifts by turret major's gyro bias (~0.24°/s with the sim's biases) once autonav starts: the
  ISM330s never apply their calibrated offsets. `AbstractIMU::updateImuMeasurement` replaces
  `imuData.gyroOffsetRaw` with the average of `curImuData.gyroOffsetRaw`, which `ISM330` never
  sets, so the offset is zeroed on every update (calibration included). Syncing `curImuData`'s
  offsets from `imuData`'s, as an experiment, took the drift to ~0.02°/s. While the IMU calibrate
  command stays in `CALIBRATION_SUCCESS` it re-zeroes yaw every tick, which hides the drift until
  autonav takes over.
- Lidars ride on turret major: the sentry MCB reports turret major's pose as "chassis" to Otto
  (`SentryTransformAdapter::getVisionWorldToChassis`).
- Not emulated: cap bank / power sensor (offline → firmware doesn't throttle), flywheels,
  agitator, camera.
