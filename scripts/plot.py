#!/usr/bin/env python3
"""Plots columns from a CSV log written by achilles' util::CsvLogger (e.g.
the --energy-log file achilles itself writes -- see src/main.cpp).

Generic over any such CSV, not just energy: it just plots whichever named
columns you ask for against an x-axis column (defaulting to the first
column, "time" for an energy log).

Usage:
  python3 scripts/plot.py energy.csv
  python3 scripts/plot.py energy.csv --columns kinetic potential total
  python3 scripts/plot.py energy.csv --out energy.png
"""
import argparse
import csv
import sys

import matplotlib.pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument(
        "--x", default=None, help="column to use as the x-axis (default: the first column)"
    )
    parser.add_argument(
        "--columns", nargs="*", default=None,
        help="columns to plot (default: every column but the x-axis)"
    )
    parser.add_argument(
        "--out", default=None, help="save to this file instead of opening a window"
    )
    args = parser.parse_args()

    with open(args.csv_path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(v) for v in row] for row in reader if row]

    if not rows:
        sys.exit(f"{args.csv_path} has no data rows")

    x_name = args.x or header[0]
    if x_name not in header:
        sys.exit(f"no such column '{x_name}' -- available columns: {header}")
    x_index = header.index(x_name)
    x = [row[x_index] for row in rows]

    columns = args.columns or [name for name in header if name != x_name]
    for name in columns:
        if name not in header:
            sys.exit(f"no such column '{name}' -- available columns: {header}")
        index = header.index(name)
        plt.plot(x, [row[index] for row in rows], label=name)

    plt.xlabel(x_name)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    if args.out:
        plt.savefig(args.out)
    else:
        plt.show()


if __name__ == "__main__":
    main()
