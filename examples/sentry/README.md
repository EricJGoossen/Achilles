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

## Modelling notes

- Chassis is a planar joint; X-drive omni wheels are an explicit traction model (slip damper
  capped by friction, near-free rollers). Masses/heights not in the firmware are estimates.
- Lidars ride on turret major: the sentry MCB reports turret major's pose as "chassis" to Otto
  (`SentryTransformAdapter::getVisionWorldToChassis`).
- Not emulated: cap bank / power sensor (offline → firmware doesn't throttle), flywheels,
  agitator, camera.
