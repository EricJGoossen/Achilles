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
- `patches/` — the changes the sim needs in aruw-mcb and project-otto (apply with `git am`).

## Running (manual for now)

```bash
cmake --build build-release --target sentry_sim
# aruw-mcb, with patches/aruw-mcb-hosted-hardware-bridge.patch applied (needs gcc-11, libgtest-dev):
(cd ../aruw-mcb/aruw-mcb-project && scons build-sim robot=SENTRY_ACHLYS)
./build-release/sentry_sim examples/sentry/field.arow examples/sentry/sim_config.yaml --headless --run-dir /tmp/sentry-sim &
ARUW_SIM_HARDWARE_SOCKET=/tmp/sentry-sim/mcb.sock ../aruw-mcb/aruw-mcb-project/build/sim/scons-release/TARGET_SENTRY_ACHLYS/aruw-mcb.elf &
../project-otto/.venv/bin/python examples/sentry/otto_bridge/run_otto.py --otto-dir ../project-otto --run-dir /tmp/sentry-sim
```

`ARUW_SIM_DEBUG=1` on the MCB prints per-second CAN delivery counts.

The sim holds its clock at zero until the MCB connects, so the processes can start in any order.

Setup notes:

- aruw-mcb needs its `segger-rtt` submodule; its devcontainer (`.devcontainer/`) builds the sim
  as is. The sim and Otto must share a machine/container (the sim's ptys are Otto's devices); the
  MCB only needs the socket, so it can run in its own container with the run dir on a shared mount.
- Otto: `git submodule update --init --recursive`, `uv sync --frozen`, then
  `uv pip install "pyrealsense2==2.56.*" PyGObject==3.42.1` (needs `libusb-1.0-0`,
  `libgirepository1.0-dev`, `libcairo2-dev`, `gir1.2-gstreamer-1.0`): `project_otto.main`
  imports both even with the camera stubbed out.
- On WSL2 the wall clock can step back by seconds. Otto's clock is `CLOCK_REALTIME`, so it drops
  MCB odometry ("not newer than existing latest timestamp") until the clock catches up; the blank
  camera paces on the monotonic clock so the main loop keeps running through it.

## Modelling notes

- Chassis is a planar joint; X-drive omni wheels are an explicit traction model (static friction
  solved per step, capped at μN; near-free rollers). Masses/heights not in the firmware are
  estimates.
- Starting off, the wheels break traction: `ChassisAutoNavController` ramps translation by
  `MAX_TRANSLATION_ACCELERATION` (1.0) per call, not per second, so it commands a speed step. The
  wheel odometry overcounts each start by ~0.25 m (μ = 0.8 here).
- Lidars ride on turret major: the sentry MCB reports turret major's pose as "chassis" to Otto
  (`SentryTransformAdapter::getVisionWorldToChassis`).
- Not emulated: cap bank / power sensor (offline → firmware doesn't throttle), flywheels,
  agitator, camera.
