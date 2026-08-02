#!/usr/bin/env python3
"""Render a heading_probe run: the turn rate against heading error, and the step responses.

The scatter is the plot the identification is read off. A proportional loop shows a straight
line through the origin that flattens into two horizontal shelves at the turn-rate limit; the
error where it flattens is e_sat. A rate-limited platform shows only the shelves, which is why
such a run reports SATURATION_DOMINATED and carries no usable gain.

    python3 ../tools/plot_heading_probe.py probe-out probe-out/probe_plot.png
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def read_meta(path):
    meta = {}
    if not Path(path).exists():
        return meta
    for row in read_csv(path):
        try:
            meta[row["key"]] = float(row["value"])
        except (ValueError, KeyError):
            meta[row["key"]] = row.get("value", "")
    return meta


def main():
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "probe-out")
    out_png = Path(sys.argv[2] if len(sys.argv) > 2 else out_dir / "probe_plot.png")

    samples = read_csv(out_dir / "probe.csv")
    if not samples:
        sys.exit("probe.csv is empty")
    model = read_meta(out_dir / "identification.csv")

    steps = {}
    for row in samples:
        if row.get("phase") == "WARMUP":
            continue
        err = row.get("heading_err_rad")
        rate = row.get("yaw_rate_rps") or row.get("yaw_rate_fd_rps")
        if err in (None, "") or rate in (None, ""):
            continue
        index = int(row["step_index"])
        steps.setdefault(index, {"t": [], "err": [], "rate": [], "step_deg": float(row["step_deg"])})
        entry = steps[index]
        entry["t"].append(float(row["elapsed_s"]))
        entry["err"].append(float(err))
        entry["rate"].append(float(rate))
    if not steps:
        sys.exit("probe.csv has no usable step samples")

    gain = model.get("K_rps_per_rad") or 0.0
    omega_max = model.get("omega_max_rps") or 0.0
    e_sat = model.get("e_sat_rad") or 0.0
    verdict = model.get("verdict", "")

    fig = plt.figure(figsize=(14, 6), constrained_layout=True)
    grid = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.1])
    fig.suptitle(f"Heading-loop identification — verdict {verdict}", fontsize=14)

    # Turn rate against heading error: the identification plot.
    ax = fig.add_subplot(grid[0, 0])
    all_err = [e for s in steps.values() for e in s["err"]]
    all_rate = [r for s in steps.values() for r in s["rate"]]
    ax.scatter(all_err, all_rate, s=4, alpha=0.35, color="#4269d0", label="samples")
    if gain > 0.0:
        span = max(abs(min(all_err)), abs(max(all_err)))
        xs = [-span + 2.0 * span * i / 200.0 for i in range(201)]
        ax.plot(xs, [max(-omega_max, min(omega_max, gain * x)) for x in xs], color="#ff725c",
                linewidth=1.8, label=f"fit: K={gain:.3f}, sat at {omega_max:.3f}")
        for sign in (1.0, -1.0):
            ax.axvline(sign * e_sat, color="#a3770a", linestyle=":", linewidth=1.1)
        ax.plot([], [], color="#a3770a", linestyle=":", label=f"e_sat = {math.degrees(e_sat):.1f} deg")
    if omega_max > 0.0:
        for sign in (1.0, -1.0):
            ax.axhline(sign * omega_max, color="#8a8a8a", linestyle="--", linewidth=1.0)
        ax.plot([], [], color="#8a8a8a", linestyle="--", label="turn-rate limit")
    ax.set_xlabel("heading error (rad)")
    ax.set_ylabel("yaw rate (rad/s)")
    ax.set_title("Turn rate vs heading error")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    # Per-step error decay, each step normalised to its own start so shapes overlay.
    ax = fig.add_subplot(grid[0, 1])
    for index in sorted(steps):
        entry = steps[index]
        t0 = entry["t"][0]
        ax.plot([ti - t0 for ti in entry["t"]], entry["err"], linewidth=1.3,
                label=f"{entry['step_deg']:+.0f} deg")
    ax.axhline(0.0, color="#8a8a8a", linewidth=1.0)
    ax.set_xlabel("time since step (s)")
    ax.set_ylabel("heading error (rad)")
    ax.set_title("Step responses")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncol=2)

    fig.savefig(out_png, dpi=150)
    print(f"wrote {out_png}")


if __name__ == "__main__":
    main()
