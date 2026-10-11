"""Merges achilles_throughput.jsonl + mujoco_throughput.jsonl into
bench/results/throughput_summary.md and throughput.png.

Speedup is achilles robot-steps/s divided by the *faster* of MuJoCo's two
single-threaded modes (merged model vs rollout) at the same scene,
integrator, dt and batch size -- for achilles' float (AVX2) build, and
separately for its double (AVX2) build, which runs at MuJoCo's own
precision.
"""

import json
import os
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")

# bench/build.sh's build directories -> report labels.
BUILD_LABELS = {
    "build-bench-native": "achilles_f32_avx2",
    "build-bench-native-double": "achilles_f64_avx2",
    "build-bench": "achilles_f32_sse2",
}
SERIES = [
    "achilles_f32_avx2",
    "achilles_f64_avx2",
    "achilles_f32_sse2",
    "mujoco_merged",
    "mujoco_rollout",
]


def load(name):
    with open(os.path.join(RESULTS, name)) as f:
        return [json.loads(line) for line in f if line.strip()]


def main():
    rows = defaultdict(dict)  # (scene, integrator, n) -> {label: robot steps/s}
    for r in load("achilles_throughput.jsonl"):
        label = BUILD_LABELS[r["build"]]
        rows[(r["scene"], r["integrator"], r["copies"])][label] = r["robot_steps_per_s"]
    for r in load("mujoco_throughput.jsonl"):
        if "error" in r:  # see mujoco_throughput.py: e.g. model too large
            continue
        rows[(r["scene"], r["integrator"], r["copies"])]["mujoco_" + r["mode"]] = r[
            "robot_steps_per_s"
        ]

    out = [
        "Robot-steps per second, single thread, dt = 2 ms (higher is better).",
        "Speedup = achilles / the faster single-threaded MuJoCo mode at the",
        "same N; f32 = achilles' default float build, f64 = its double build",
        "(MuJoCo's own precision). Both achilles columns are -march=native",
        "(AVX2: 8 float or 4 double lanes); SSE2 is the project's default flags.",
        "",
    ]
    groups = sorted({(s, i) for s, i, _ in rows})
    fig, axes = plt.subplots(
        2, (len(groups) + 1) // 2, figsize=(4.4 * ((len(groups) + 1) // 2), 8),
        sharey=False,
    )
    axes = axes.flatten()
    for ax, (scene, integ) in zip(axes, groups):
        out += [
            f"### {scene} / {integ}",
            "",
            "| N robots | achilles f32 AVX2 | achilles f64 AVX2 | achilles f32 SSE2 "
            "| MuJoCo merged | MuJoCo rollout | speedup f32 | speedup f64 |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        ns = sorted(n for s, i, n in rows if (s, i) == (scene, integ))
        series = defaultdict(list)

        def cell(r, k):
            # n/a: no measurement, e.g. a MuJoCo merged model too large to
            # compile (see mujoco_throughput.py).
            return f"{r[k]:,.0f}" if k in r else "n/a"

        for n in ns:
            r = rows[(scene, integ, n)]
            mj = max(r.get("mujoco_merged", 0), r.get("mujoco_rollout", 0))
            sp32 = r.get("achilles_f32_avx2", 0) / mj if mj else float("nan")
            sp64 = r.get("achilles_f64_avx2", 0) / mj if mj else float("nan")
            out.append(
                f"| {n} | {cell(r, 'achilles_f32_avx2')} "
                f"| {cell(r, 'achilles_f64_avx2')} "
                f"| {cell(r, 'achilles_f32_sse2')} "
                f"| {cell(r, 'mujoco_merged')} "
                f"| {cell(r, 'mujoco_rollout')} "
                f"| {sp32:.2f}x | {sp64:.2f}x |"
            )
            for k in SERIES:
                series[k].append(r.get(k, float("nan")))
        out.append("")
        for k in SERIES:
            ax.loglog(ns, series[k], marker="o", label=k.replace("_", " "))
        ax.set_title(f"{scene} / {integ}")
        ax.set_xlabel("robots per batch (N)")
        ax.set_ylabel("robot-steps / s")
        ax.grid(True, which="both", alpha=0.3)
    for ax in axes[len(groups):]:
        ax.set_visible(False)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "throughput.png"), dpi=120)

    with open(os.path.join(RESULTS, "throughput_summary.md"), "w") as f:
        f.write("\n".join(out))
    print("\n".join(out))


if __name__ == "__main__":
    main()
