"""
Plots a sentry co-sim run from its pose_log.csv: the path on the field (truth and
the MCB's own estimate) and the estimate's position and yaw errors over time.

    python plot_run.py RUN_DIR/pose_log.csv OUT.png [--autonav-at S] [--route OTTO_CONFIG]

scripts/run-sentry-sim.sh plot runs it on a run directory.
"""

import argparse
import csv
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_scene import field_pieces, field_zones  # noqa: E402


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def route_points(path):
    """The push setpoint and the patrol loop from an Otto config, if it sets them."""
    import yaml

    with open(path) as f:
        strategy = ((yaml.safe_load(f) or {}).get("autonav") or {}).get("motion_strategy") or {}
    points = []
    if "initial_push_setpoint" in strategy:
        points.append(strategy["initial_push_setpoint"])
    points += strategy.get("center_path_setpoints") or []
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pose_log")
    parser.add_argument("out")
    parser.add_argument("--autonav-at", type=float, default=25.0)
    parser.add_argument("--route", help="the Otto config the run used, to mark its setpoints")
    args = parser.parse_args()

    # A run still being written can end in a partial line.
    rows = [r for r in csv.DictReader(open(args.pose_log)) if None not in r.values()]
    if not rows:
        raise SystemExit(f"{args.pose_log} has no rows yet")
    true_x = [float(r["true_x"]) for r in rows]
    true_y = [float(r["true_y"]) for r in rows]
    est = [r for r in rows if r["mcb_x"]]
    t = [float(r["time"]) for r in est]

    fig = plt.figure(figsize=(14, 8.5), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.5, 1])
    ax_map = fig.add_subplot(grid[:, 0])
    ax_pos = fig.add_subplot(grid[0, 1])
    ax_yaw = fig.add_subplot(grid[1, 1])

    for name, x0, y0, x1, y1, _height, colour in field_zones():
        if name.startswith("bumpy_road"):  # drivable floor: hatched, not filled
            ax_map.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, hatch="..", lw=0,
                                       edgecolor="#9aa0a6"))
        else:
            ax_map.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=colour, alpha=0.35, lw=0))
    # Obstacles shaded by height: the lidars (~0.34 m up) see over the High Ground's deck.
    for _name, x0, y0, x1, y1, _z0, z1, shape, _colour in field_pieces():
        shade = 0.78 - 0.5 * min(z1, 1.0)
        ax_map.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=(shade, shade, shade + 0.04), lw=0,
                                   hatch="//" if shape != "box" else None))
    if args.route:
        points = route_points(args.route)
        for i, (x, y) in enumerate(points):
            ax_map.plot(x, y, "x", color="#cf222e", ms=10, mew=2, label="Otto setpoints" if i == 0 else None)
            ax_map.annotate(str(i), (x, y), textcoords="offset points", xytext=(6, 6), color="#cf222e")
    ax_map.plot(true_x, true_y, color="#1f6feb", lw=2.0, label="true path")
    # Before calibration finishes the estimate sits at the firmware's initial position.
    settled = [r for r in est if float(r["time"]) >= min(15.0, args.autonav_at)]
    ax_map.plot([float(r["mcb_x"]) for r in settled], [float(r["mcb_y"]) for r in settled], color="#d1661b",
                lw=1.4, ls="--", label="MCB estimate")
    ax_map.plot(true_x[0], true_y[0], "o", color="#2da44e", ms=9, label="start")
    ax_map.plot(true_x[-1], true_y[-1], "s", color="#1f6feb", ms=8, label="end (true)")
    ax_map.set_aspect("equal")
    ax_map.set_xlim(-0.3, 12.3)
    ax_map.set_ylim(-0.3, 8.3)
    ax_map.set_xlabel("x (m)")
    ax_map.set_ylabel("y (m)")
    ax_map.set_title("ARC 2026 field")
    ax_map.legend(loc="upper right", fontsize=9)

    position_error = [math.hypot(float(r["mcb_x"]) - float(r["true_x"]), float(r["mcb_y"]) - float(r["true_y"]))
                      for r in est]
    yaw_error = [math.degrees(wrap(float(r["mcb_yaw"]) - float(r["true_yaw"]))) for r in est]
    for ax in (ax_pos, ax_yaw):
        ax.axvline(args.autonav_at, color="#888", ls=":", lw=1)
        ax.grid(alpha=0.3)
    ax_pos.plot(t, position_error, color="#d1661b")
    ax_pos.set_ylabel("position error (m)")
    ax_pos.set_title("MCB estimate vs truth (dotted: autonav on)")
    late = [e for e, tt in zip(position_error, t) if tt > 5.0]
    ax_pos.set_ylim(0, max(0.5, max(late, default=0.0) * 1.2))
    ax_yaw.plot(t, yaw_error, color="#8250df")
    ax_yaw.set_ylabel("yaw error (deg)")
    ax_yaw.set_xlabel("sim time (s)")

    paths = int(rows[-1]["autonav_paths"])
    fig.suptitle(f"{float(rows[-1]['time']):.0f} s run, {paths} autonav paths from Otto", fontsize=11)
    fig.savefig(args.out, dpi=110)


if __name__ == "__main__":
    main()
