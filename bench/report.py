"""Merges achilles_throughput.jsonl + mujoco_throughput.jsonl into
bench/results/throughput_summary.md and throughput.png.

Speedup is achilles robot-steps/s divided by the *faster* of MuJoCo's two
single-threaded modes (merged model vs rollout) at the same scene,
integrator, dt and batch size.
"""

import json
import os
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")


def load(name):
    with open(os.path.join(RESULTS, name)) as f:
        return [json.loads(line) for line in f if line.strip()]


def main():
    rows = defaultdict(dict)  # (scene, integrator, n) -> {label: robot steps/s}
    for r in load("achilles_throughput.jsonl"):
        label = "achilles_avx2" if r["build"] == "build-bench-native" else "achilles_sse2"
        rows[(r["scene"], r["integrator"], r["copies"])][label] = r["robot_steps_per_s"]
    for r in load("mujoco_throughput.jsonl"):
        rows[(r["scene"], r["integrator"], r["copies"])]["mujoco_" + r["mode"]] = r[
            "robot_steps_per_s"
        ]

    out = [
        "Robot-steps per second, single thread, dt = 2 ms (higher is better).",
        "Speedup = achilles (AVX2 build) / best single-threaded MuJoCo mode.",
        "",
    ]
    groups = sorted({(s, i) for s, i, _ in rows})
    fig, axes = plt.subplots(1, len(groups), figsize=(4.2 * len(groups), 4), sharey=True)
    for ax, (scene, integ) in zip(axes, groups):
        out += [
            f"### {scene} / {integ}",
            "",
            "| N robots | achilles AVX2 | achilles SSE2 | MuJoCo merged | MuJoCo rollout | speedup |",
            "|---:|---:|---:|---:|---:|---:|",
        ]
        ns = sorted(n for s, i, n in rows if (s, i) == (scene, integ))
        series = defaultdict(list)
        for n in ns:
            r = rows[(scene, integ, n)]
            mj = max(r.get("mujoco_merged", 0), r.get("mujoco_rollout", 0))
            sp = r.get("achilles_avx2", 0) / mj if mj else float("nan")
            out.append(
                f"| {n} | {r.get('achilles_avx2', 0):,.0f} | {r.get('achilles_sse2', 0):,.0f} | "
                f"{r.get('mujoco_merged', 0):,.0f} | {r.get('mujoco_rollout', 0):,.0f} | {sp:.2f}x |"
            )
            for k in ("achilles_avx2", "achilles_sse2", "mujoco_merged", "mujoco_rollout"):
                series[k].append(r.get(k, float("nan")))
        out.append("")
        for k, v in series.items():
            ax.loglog(ns, v, marker="o", label=k.replace("_", " "))
        ax.set_title(f"{scene} / {integ}")
        ax.set_xlabel("robots per batch (N)")
        ax.grid(True, which="both", alpha=0.3)
    axes[0].set_ylabel("robot-steps / s")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "throughput.png"), dpi=120)

    with open(os.path.join(RESULTS, "throughput_summary.md"), "w") as f:
        f.write("\n".join(out))
    print("\n".join(out))


if __name__ == "__main__":
    main()
