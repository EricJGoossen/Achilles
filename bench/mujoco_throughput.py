"""MuJoCo half of the throughput comparison (see bench/README.md).

Times the same scenes achilles_bench does, at the same dt and batch sizes, two
ways -- both single-threaded, both with the stepping loop in C, never Python:

  merged   one MjModel holding N independent copies of the robot side by
           side, advanced with mj_step(m, d, nstep=steps).
  rollout  one single-robot MjModel, N independent initial states advanced
           by mujoco.rollout (one MjData, so one thread).

Writes bench/results/mujoco_throughput.jsonl.
"""

import json
import os
import time

import mujoco
import numpy as np
from mujoco import rollout

import arow_to_mjcf as a2m

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONFIG = os.path.join(ROOT, "examples", "sim_config.yaml")
SCENES = ["two_joint_arm", "three_joint_arm"]
INTEGRATORS = {"euler": "Euler", "rk4": "RK4"}
BATCHES = [1, 4, 16, 64, 256, 1024, 4096]
DT = 0.002
TRIALS = 3


def steps_for(n: int) -> int:
    return int(min(20000, max(200, 1_000_000 // n)))


def best_of(fn, trials=TRIALS):
    times = []
    for _ in range(trials):
        t0 = time.perf_counter()
        fn()
        times.append(time.perf_counter() - t0)
    times.sort()
    return times[0], times[len(times) // 2]


def bench_merged(scene, integ, n, steps):
    m = mujoco.MjModel.from_xml_string(a2m.to_mjcf(scene, DT, integ, copies=n))
    d = mujoco.MjData(m)
    q0, qd0 = a2m.initial_state(scene, copies=n)
    d.qpos[:] = q0
    d.qvel[:] = qd0
    mujoco.mj_step(m, d, nstep=max(1, steps // 10))  # warm-up
    best, median = best_of(lambda: mujoco.mj_step(m, d, nstep=steps))
    finite = bool(np.all(np.isfinite(d.qpos)))
    return best, median, finite


def bench_rollout(scene, integ, n, steps):
    m = mujoco.MjModel.from_xml_string(a2m.to_mjcf(scene, DT, integ, copies=1))
    d = mujoco.MjData(m)
    q0, qd0 = a2m.initial_state(scene)
    d.qpos[:] = q0
    d.qvel[:] = qd0
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    s0 = np.empty(mujoco.mj_stateSize(m, spec))
    mujoco.mj_getState(m, d, s0, spec)
    init = np.tile(s0, (n, 1))
    out = np.empty((n, steps, s0.size))
    rollout.rollout(m, d, init, nstep=max(1, steps // 10))  # warm-up
    best, median = best_of(
        lambda: rollout.rollout(m, d, init, nstep=steps, state=out)
    )
    finite = bool(np.all(np.isfinite(out[:, -1])))
    return best, median, finite


def main():
    out_path = os.path.join(HERE, "results", "mujoco_throughput.jsonl")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        for scene_name in SCENES:
            scene = a2m.load_scene(
                os.path.join(ROOT, "examples", scene_name + ".arow"), CONFIG
            )
            for ours, integ in INTEGRATORS.items():
                for n in BATCHES:
                    steps = steps_for(n)
                    for mode, fn in (("merged", bench_merged),
                                     ("rollout", bench_rollout)):
                        best, median, finite = fn(scene, integ, n, steps)
                        row = {
                            "engine": "mujoco",
                            "mujoco_version": mujoco.__version__,
                            "mode": mode,
                            "scene": scene_name,
                            "integrator": ours,
                            "dt": float(np.float32(DT)),
                            "copies": n,
                            "steps": steps,
                            "trials": TRIALS,
                            "best_s": best,
                            "median_s": median,
                            "batch_steps_per_s": steps / best,
                            "robot_steps_per_s": steps * n / best,
                            "finite": finite,
                        }
                        print(json.dumps(row), flush=True)
                        f.write(json.dumps(row) + "\n")


if __name__ == "__main__":
    main()
