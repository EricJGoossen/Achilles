"""Accuracy comparison: achilles traces vs MuJoCo from identical initial scenes.

Reads bench/results/traces/<scene>_<integrator>.csv (bench/run_achilles.sh)
and, for each, measures three things:

  1. Instantaneous dynamics. At every state achilles visited, set MuJoCo to
     the same (q, qd), run mj_forward, and compare qacc against achilles' own
     ABA qdd. No integration involved, so this error never accumulates -- it
     is the direct "same equations of motion?" check.
  2. Engine vs engine. Step MuJoCo with the matching integrator at the same
     (float32-rounded) dt from the same initial state, and track the
     end-effector position difference over time as a percent of arm reach.
  3. Engine vs ground truth. A MuJoCo RK4 run at dt/100 stands in for the
     exact solution; both engines' production-dt runs are measured against
     it, so (2) can be read as "who is closer to the truth" rather than just
     "do they agree".

The end effector is the last link's center of mass; reach is the distance
from the base to it with the chain fully extended.

Writes bench/results/accuracy_summary.json, accuracy_summary.md and
error_<scene>.png plots.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import arow_to_mjcf as a2m  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONFIG = os.path.join(ROOT, "examples", "sim_config.yaml")
RESULTS = os.path.join(HERE, "results")
SCENES = {
    "two_joint_arm": os.path.join(ROOT, "examples", "two_joint_arm.arow"),
    "three_joint_arm": os.path.join(ROOT, "examples", "three_joint_arm.arow"),
    "asymmetric_two_joint_arm": os.path.join(
        ROOT, "examples", "asymmetric_two_joint_arm.arow"
    ),
    "two_joint_arm_small_swing": os.path.join(
        HERE, "scenes", "two_joint_arm_small_swing.arow"
    ),
}
INTEGRATORS = {"euler": "Euler", "rk4": "RK4"}
CHECKPOINTS = [1.0, 2.0, 5.0, 10.0, 20.0]
TRUTH_SUBSTEPS = 100


def load_trace(path, scene):
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["t"]
    n = len(scene.joints)
    q = np.empty((t.size, n))
    qd = np.empty((t.size, n))
    qdd = np.empty((t.size, n))
    for j, joint in enumerate(scene.joints):
        quat_vec = np.stack([data[f"j{j}_qx"], data[f"j{j}_qy"],
                             data[f"j{j}_qz"]], axis=1)
        q[:, j] = 2.0 * np.arctan2(quat_vec @ joint.axis, data[f"j{j}_qw"])
        qd[:, j] = data[f"j{j}_v{joint.slot}"]
        qdd[:, j] = data[f"j{j}_a{joint.slot}"]
    return t, np.unwrap(q, axis=0), qd, qdd


def make_model(scene, dt, integ):
    m = mujoco.MjModel.from_xml_string(a2m.to_mjcf(scene, dt, integ))
    m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
    return m, mujoco.MjData(m)


def simulate(m, d, q0, qd0, nsamples, substeps):
    d.qpos[:] = q0
    d.qvel[:] = qd0
    d.time = 0.0
    q = np.empty((nsamples, m.nq))
    q[0] = d.qpos
    for k in range(1, nsamples):
        mujoco.mj_step(m, d, nstep=substeps)
        q[k] = d.qpos
    return q


def tip_and_energy(m, d, q, qd=None):
    """End-effector world position and (if qd given) total energy per row."""
    tip = np.empty((q.shape[0], 3))
    energy = np.empty(q.shape[0])
    last = m.nbody - 1
    for k in range(q.shape[0]):
        d.qpos[:] = q[k]
        d.qvel[:] = 0.0 if qd is None else qd[k]
        mujoco.mj_forward(m, d)
        tip[k] = d.xipos[last]
        energy[k] = d.energy[0] + d.energy[1]
    return tip, energy


def reach_of(scene):
    tail = scene.joints[-1]
    return sum(np.linalg.norm(j.pos) for j in scene.joints[1:]) + np.linalg.norm(
        tail.com
    )


def max_up_to(t, err, tc):
    mask = t <= tc + 1e-9
    return float(np.max(err[mask])) if np.any(mask) else float("nan")


def first_exceed(t, err, thresh):
    idx = np.nonzero(err > thresh)[0]
    return float(t[idx[0]]) if idx.size else None


def main():
    summary = {}
    for scene_name, arow in SCENES.items():
        scene = a2m.load_scene(arow, CONFIG)
        reach = reach_of(scene)
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
        for ax, (ours, integ) in zip(axes, INTEGRATORS.items()):
            path = os.path.join(RESULTS, "traces", f"{scene_name}_{ours}.csv")
            t, q_a, qd_a, qdd_a = load_trace(path, scene)
            dt = float(t[1] - t[0])
            q0, qd0 = a2m.initial_state(scene)
            assert np.allclose(q_a[0], q0) and np.allclose(qd_a[0], qd0)

            # 1. Instantaneous dynamics at every visited state.
            m, d = make_model(scene, dt, integ)
            qacc = np.empty_like(qdd_a)
            for k in range(t.size):
                d.qpos[:] = q_a[k]
                d.qvel[:] = qd_a[k]
                mujoco.mj_forward(m, d)
                qacc[k] = d.qacc
            acc_err = np.max(np.abs(qdd_a - qacc), axis=1)
            acc_scale = np.sqrt(np.mean(np.sum(qacc**2, axis=1)))
            acc_rel = np.linalg.norm(qdd_a - qacc, axis=1) / np.maximum(
                np.linalg.norm(qacc, axis=1), 1e-12
            )

            # 2. MuJoCo, same integrator and dt.
            q_m = simulate(m, d, q0, qd0, t.size, 1)
            # 3. Ground truth: MuJoCo RK4 at dt / TRUTH_SUBSTEPS.
            mt, dtr = make_model(scene, dt, "RK4")
            mt.opt.timestep = dt / TRUTH_SUBSTEPS
            q_t = simulate(mt, dtr, q0, qd0, t.size, TRUTH_SUBSTEPS)

            tip_a, e_a = tip_and_energy(m, d, q_a, qd_a)
            tip_m, _ = tip_and_energy(m, d, q_m)
            tip_t, _ = tip_and_energy(m, d, q_t)
            err_am = 100 * np.linalg.norm(tip_a - tip_m, axis=1) / reach
            err_at = 100 * np.linalg.norm(tip_a - tip_t, axis=1) / reach
            err_mt = 100 * np.linalg.norm(tip_m - tip_t, axis=1) / reach
            # Energy drift as a percent of the system's potential-energy span
            # (initial energy is ~0 for the horizontal release, so a percent
            # of E(0) would be meaningless).
            # Serial chain: body i's CoM can sit at most r_i from the base,
            # so its potential energy spans 2 m_i g r_i (straight up vs down).
            g = np.linalg.norm(scene.gravity)
            pe_span, lever = 0.0, 0.0
            for i, j in enumerate(scene.joints):
                if i > 0:
                    lever += np.linalg.norm(j.pos)
                pe_span += 2 * j.mass * g * (lever + np.linalg.norm(j.com))
            e_drift = 100 * np.max(np.abs(e_a - e_a[0])) / pe_span

            key = f"{scene_name}/{ours}"
            summary[key] = {
                "dt": dt,
                "duration_s": float(t[-1]),
                "qacc_max_abs_err": float(np.max(acc_err)),
                "qacc_rms_scale": float(acc_scale),
                "qacc_rel_err_median": float(np.median(acc_rel)),
                "qacc_rel_err_max": float(np.max(acc_rel)),
                "tip_err_pct_vs_mujoco": {
                    str(tc): max_up_to(t, err_am, tc) for tc in CHECKPOINTS
                },
                "tip_err_pct_achilles_vs_truth": {
                    str(tc): max_up_to(t, err_at, tc) for tc in CHECKPOINTS
                },
                "tip_err_pct_mujoco_vs_truth": {
                    str(tc): max_up_to(t, err_mt, tc) for tc in CHECKPOINTS
                },
                "first_t_over_1pct_vs_mujoco": first_exceed(t, err_am, 1.0),
                "energy_drift_pct_of_pe_span": float(e_drift),
            }
            print(key, json.dumps(summary[key]), flush=True)

            floor = 1e-14
            ax.semilogy(t, np.maximum(err_am, floor), label="achilles vs MuJoCo")
            ax.semilogy(t, np.maximum(err_at, floor), label="achilles vs truth",
                        alpha=0.8)
            ax.semilogy(t, np.maximum(err_mt, floor), label="MuJoCo vs truth",
                        alpha=0.8, linestyle="--")
            ax.set_title(f"{scene_name} -- {ours}, dt={dt:g}s")
            ax.set_xlabel("simulated time (s)")
            ax.grid(True, which="both", alpha=0.3)
        axes[0].set_ylabel("end-effector error (% of reach)")
        axes[0].legend(loc="lower right", fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(RESULTS, f"error_{scene_name}.png"), dpi=120)
        plt.close(fig)

    with open(os.path.join(RESULTS, "accuracy_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    lines = [
        "| scene / integrator | max qacc rel err | tip err vs MuJoCo @1s / 5s / 20s (% reach) | achilles vs truth @5s | MuJoCo vs truth @5s | energy drift (% PE span) |",
        "|---|---|---|---|---|---|",
    ]
    for key, s in summary.items():
        am = s["tip_err_pct_vs_mujoco"]
        lines.append(
            f"| {key} | {s['qacc_rel_err_max']:.1e} | "
            f"{am['1.0']:.1e} / {am['5.0']:.1e} / {am['20.0']:.1e} | "
            f"{s['tip_err_pct_achilles_vs_truth']['5.0']:.1e} | "
            f"{s['tip_err_pct_mujoco_vs_truth']['5.0']:.1e} | "
            f"{s['energy_drift_pct_of_pe_span']:.2e} |"
        )
    with open(os.path.join(RESULTS, "accuracy_summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
