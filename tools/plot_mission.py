#!/usr/bin/env python3
"""Plot a recorded autopilot mission: the vehicle track vs the planned waypoints.

Usage: plot_mission.py <mission-out-dir> [output.png]

Reads track.csv, waypoints.csv, and planned_path.csv as written by mission_runner and
renders:
  - the ground track overlaid on the ideal planned Dubins route, with the waypoints, their
    capture gates, and required arrival attitudes,
  - north/east position components over time,
  - speed over ground over time,
  - depth / altitude-above-sea-floor over time (when the platform reports them).
"""
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "mission-out")
    out_png = Path(sys.argv[2] if len(sys.argv) > 2 else out_dir / "mission_plot.png")

    track = read_csv(out_dir / "track.csv")
    waypoints = read_csv(out_dir / "waypoints.csv")
    planned_file = out_dir / "planned_path.csv"
    planned = read_csv(planned_file) if planned_file.exists() else []
    if not track:
        sys.exit("track.csv is empty")

    # Local equirectangular projection about the first fix (meters east/north).
    lat0 = float(track[0]["lat_deg"])
    lon0 = float(track[0]["lon_deg"])
    m_per_deg_lat = 111132.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))

    def to_local(lat, lon):
        return (lon - lon0) * m_per_deg_lon, (lat - lat0) * m_per_deg_lat

    t = [float(r["elapsed_s"]) for r in track]
    east, north = zip(*[to_local(float(r["lat_deg"]), float(r["lon_deg"])) for r in track])
    speed = [float(r["speed_mps"]) for r in track]
    depth = [float(r["depth_m"]) if r.get("depth_m") not in (None, "") else None for r in track]
    alt_asf = [float(r["alt_asf_m"]) if r.get("alt_asf_m") not in (None, "") else None
               for r in track]
    has_depth = any(d is not None for d in depth)

    def optional_column(name):
        values = [float(r[name]) if r.get(name) not in (None, "") else None for r in track]
        return values if any(v is not None for v in values) else None

    yaw_rate = optional_column("yaw_rate_rps")
    xte = optional_column("cross_track_error_m")
    # meta.csv carries the configuration that produced the run, which is what turns the
    # turn-rate trace into a comparison against the platform limit and the planned demand.
    meta = {}
    meta_file = out_dir / "meta.csv"
    if meta_file.exists():
        for r in read_csv(meta_file):
            try:
                meta[r["key"]] = float(r["value"])
            except (ValueError, KeyError):
                pass
    omega_max = meta.get("max_turn_rate_rps")
    turn_radius = meta.get("turn_radius_m")
    cruise = meta.get("cruising_speed_mps")
    planned_omega = cruise / turn_radius if cruise and turn_radius else None
    capture = meta.get("pos_capture_m")
    wp_e, wp_n, wp_r, wp_yaw = [], [], [], []
    for r in waypoints:
        e, n = to_local(float(r["lat_deg"]), float(r["lon_deg"]))
        wp_e.append(e)
        wp_n.append(n)
        wp_r.append(float(r["capture_radius_m"]))
        yaw = r.get("arrival_yaw_rad", "")
        wp_yaw.append(float(yaw) if yaw not in (None, "") else None)

    rows = 2 + (1 if yaw_rate else 0) + (1 if xte else 0) + (1 if has_depth else 0)
    fig = plt.figure(figsize=(14, 3.0 * rows), constrained_layout=True)
    grid = fig.add_gridspec(rows, 2, width_ratios=[1.4, 1.0])
    fig.suptitle("Autopilot waypoint mission — simulated vehicle", fontsize=14)

    # Ground track vs planned waypoints.
    ax = fig.add_subplot(grid[:, 0])
    ax.plot(east, north, color="#4269d0", linewidth=1.6, label="vehicle track", zorder=3)
    ax.plot(east[0], north[0], marker="^", color="#3ca951", markersize=10,
            linestyle="none", label="start", zorder=5)
    ax.plot(east[-1], north[-1], marker="s", color="#ff725c", markersize=8,
            linestyle="none", label="end", zorder=5)
    if planned:
        pe, pn = zip(*[to_local(float(r["lat_deg"]), float(r["lon_deg"])) for r in planned])
        ax.plot(pe, pn, linestyle="--", color="#9c6b4e", linewidth=1.2, alpha=0.9,
                label="planned Dubins route", zorder=2)
    else:
        ax.plot(wp_e, wp_n, linestyle="--", color="#9c6b4e", linewidth=1.0, alpha=0.7,
                label="planned route (waypoint order)", zorder=2)
    span = max(max(wp_n) - min(wp_n), max(wp_e) - min(wp_e), 1.0)
    arrow_len = 0.06 * span
    has_attitude = False
    for i, (e, n, r, yaw) in enumerate(zip(wp_e, wp_n, wp_r, wp_yaw)):
        ax.add_patch(plt.Circle((e, n), r, facecolor="none", edgecolor="#a3770a",
                                linewidth=1.4, zorder=4))
        # Alternate label offsets so dense waypoint rows stay legible.
        ax.annotate(f"WP{i + 1}", (e, n), textcoords="offset points",
                    xytext=(8, 8 if i % 2 == 0 else -16), fontsize=10, color="#6f6f6f")
        if yaw is not None:
            has_attitude = True
            ax.annotate("", xy=(e + arrow_len * math.sin(yaw), n + arrow_len * math.cos(yaw)),
                        xytext=(e, n),
                        arrowprops=dict(arrowstyle="-|>", color="#6cc5b0", linewidth=1.6),
                        zorder=6)
    ax.plot([], [], color="#a3770a", label="capture radius")
    if has_attitude:
        ax.plot([], [], color="#6cc5b0", label="required arrival attitude")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Ground track vs planned waypoints")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    # Position components over time.
    ax = fig.add_subplot(grid[0, 1])
    ax.plot(t, north, color="#4269d0", linewidth=1.4, label="north (m)")
    ax.plot(t, east, color="#ff725c", linewidth=1.4, label="east (m)")
    ax.set_xlabel("elapsed time (s)")
    ax.set_ylabel("position (m)")
    ax.set_title("Position over time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    # Speed over time.
    ax = fig.add_subplot(grid[1, 1])
    ax.plot(t, speed, color="#3ca951", linewidth=1.4)
    ax.set_xlabel("elapsed time (s)")
    ax.set_ylabel("speed over ground (m/s)")
    ax.set_title("Speed over time")
    ax.grid(True, alpha=0.3)

    next_row = 2

    # Turn rate against the platform limit and the planned arc's demand: the gap between the
    # two is the turn authority left for disturbance rejection.
    if yaw_rate:
        ax = fig.add_subplot(grid[next_row, 1])
        next_row += 1
        tw = [ti for ti, w in zip(t, yaw_rate) if w is not None]
        ax.plot(tw, [w for w in yaw_rate if w is not None], color="#4269d0", linewidth=1.2)
        for level, color, label in ((omega_max, "#ff725c", "platform limit"),
                                    (planned_omega, "#8a8a8a", "planned arc demand")):
            if level:
                ax.axhline(level, color=color, linestyle="--", linewidth=1.1, label=label)
                ax.axhline(-level, color=color, linestyle="--", linewidth=1.1)
        ax.set_xlabel("elapsed time (s)")
        ax.set_ylabel("yaw rate (rad/s)")
        title = "Turn rate vs available authority"
        if not meta:
            title += " (no meta.csv: reference lines unavailable)"
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        if omega_max or planned_omega:
            ax.legend(loc="best", fontsize=9)

    # Signed cross-track error, starboard-positive, as the planner itself measured it.
    if xte:
        ax = fig.add_subplot(grid[next_row, 1])
        next_row += 1
        tx = [ti for ti, e in zip(t, xte) if e is not None]
        ax.plot(tx, [e for e in xte if e is not None], color="#3ca951", linewidth=1.2)
        ax.axhline(0.0, color="#8a8a8a", linewidth=1.0)
        if capture:
            ax.axhline(capture, color="#a3770a", linestyle=":", linewidth=1.1,
                       label="capture gate half-width")
            ax.axhline(-capture, color="#a3770a", linestyle=":", linewidth=1.1)
            ax.legend(loc="best", fontsize=9)
        ax.set_xlabel("elapsed time (s)")
        ax.set_ylabel("cross-track error (m)")
        ax.set_title("Cross-track error (starboard positive)")
        ax.grid(True, alpha=0.3)

    # Depth / altitude above sea floor over time.
    if has_depth:
        ax = fig.add_subplot(grid[next_row, 1])
        td = [ti for ti, d in zip(t, depth) if d is not None]
        ax.plot(td, [d for d in depth if d is not None], color="#4269d0", linewidth=1.4,
                label="depth (m)")
        if any(a is not None for a in alt_asf):
            ta = [ti for ti, a in zip(t, alt_asf) if a is not None]
            ax.plot(ta, [a for a in alt_asf if a is not None], color="#a3770a",
                    linewidth=1.4, label="altitude above sea floor (m)")
        ax.invert_yaxis()  # depth grows downward
        ax.set_xlabel("elapsed time (s)")
        ax.set_ylabel("meters (depth axis down)")
        ax.set_title("Depth / height above floor over time")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=9)

    fig.savefig(out_png, dpi=150)
    print(f"wrote {out_png}")


if __name__ == "__main__":
    main()
