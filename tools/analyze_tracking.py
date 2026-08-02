#!/usr/bin/env python3
"""Objective path-tracking metrics for a mission_runner recording.

Reports cross-track behaviour conditioned on path geometry, so corner-cutting on arcs is
distinguishable from overshoot and from straight-line disturbance rejection, and compares the
turn rate actually used against both the platform limit and the planned arc's demand.

Standard library only: this gates CI and runs on a boat.

Exit codes: 0 all gates passed, 1 a gate failed, 2 the inputs cannot be evaluated.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

EARTH_RADIUS_M = 6378137.0

# Used only when a recording predates meta.csv. Mirrors the shipped config/autopilot.yaml;
# the provenance is always printed so a reader knows the thresholds were not from the run.
FALLBACK_META = {
    "turn_radius_m": 1.25 * 3.0 / 0.2618,
    "min_turn_radius_m": 3.0 / 0.2618,
    "max_turn_rate_rps": 0.2618,
    "cruising_speed_mps": 3.0,
    "sample_step_m": 2.0,
    "pos_capture_m": 2.5,
}

# Fractions of the planned curvature that separate arcs from straights. Samples between the
# two are transitions and are excluded from both populations.
ARC_CURVATURE_FRAC = 0.5
STRAIGHT_CURVATURE_FRAC = 0.1

MOVING_SPEED_MPS = 0.5
# The guard exists to catch recordings whose plan is not what was flown, which happens when a
# route replans mid-mission and the from-start preview cannot contain the extra legs. It must
# still admit a legitimate disturbance run: a lateral current makes the vehicle crab and cover
# measurably more ground over the same plan. Measured separation is wide - a 0.5 m/s current
# gives ~1.07, while a depth-rate-limited spiral's loop-back passes give ~2.1.
PLAN_LENGTH_RATIO_GATE = (0.90, 1.20)


def parse_float(text):
    """A CSV cell as a float, or None when the source had not reported yet."""
    if text is None or text == "":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def percentile(values, q):
    """Linear-interpolated percentile of an unsorted sequence; None when empty."""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = q * (len(ordered) - 1)
    low = int(math.floor(pos))
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (pos - low)


def mean(values):
    return sum(values) / len(values) if values else None


def wrap_pi(angle):
    return math.remainder(angle, 2.0 * math.pi)


def project(lat_deg, lon_deg, lat0_deg, lon0_deg):
    """Local equirectangular east/north metres about a reference fix."""
    east = math.radians(lon_deg - lon0_deg) * EARTH_RADIUS_M * math.cos(math.radians(lat0_deg))
    north = math.radians(lat_deg - lat0_deg) * EARTH_RADIUS_M
    return east, north


def load_meta(directory):
    """meta.csv as a dict, falling back to the shipped defaults. Returns (meta, from_run)."""
    path = Path(directory) / "meta.csv"
    if not path.exists():
        return dict(FALLBACK_META), False
    meta = dict(FALLBACK_META)
    with path.open() as handle:
        for row in csv.DictReader(handle):
            value = parse_float(row.get("value"))
            if value is not None:
                meta[row["key"]] = value
            elif row.get("value"):
                meta[row["key"]] = row["value"]
    return meta, True


def load_track(directory):
    """track.csv rows with numeric fields parsed; unknown/absent columns become None."""
    path = Path(directory) / "track.csv"
    if not path.exists():
        raise FileNotFoundError(str(path))
    rows = []
    with path.open() as handle:
        for raw in csv.DictReader(handle):
            row = {
                "elapsed_s": parse_float(raw.get("elapsed_s")),
                "lat_deg": parse_float(raw.get("lat_deg")),
                "lon_deg": parse_float(raw.get("lon_deg")),
                "yaw_rad": parse_float(raw.get("yaw_rad")),
                "speed_mps": parse_float(raw.get("speed_mps")),
                "yaw_rate_rps": parse_float(raw.get("yaw_rate_rps")),
                "cross_track_error_m": parse_float(raw.get("cross_track_error_m")),
                "waypoint_index": parse_float(raw.get("waypoint_index")),
            }
            if row["lat_deg"] is None or row["lon_deg"] is None:
                continue
            rows.append(row)
    return rows


def numeric_curvature(points, index):
    """Signed curvature from the circle through three consecutive plan vertices.

    Only used for recordings that predate the planner-emitted kappa column; it is noisier at
    segment boundaries but reproduces the same conditioned statistics.
    """
    if index == 0 or index + 1 >= len(points):
        return 0.0
    (x0, y0), (x1, y1), (x2, y2) = points[index - 1], points[index], points[index + 1]
    a = math.hypot(x1 - x0, y1 - y0)
    b = math.hypot(x2 - x1, y2 - y1)
    c = math.hypot(x2 - x0, y2 - y0)
    if a < 1e-9 or b < 1e-9 or c < 1e-9:
        return 0.0
    cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    return 2.0 * cross / (a * b * c)


def load_plan(directory, lat0, lon0):
    """planned_path.csv as vertices with local x/y, arc length, signed curvature and tangent.

    Curvature and tangent come from the planner when the columns are present (exact), and are
    derived from the polyline otherwise (legacy recordings).
    """
    path = Path(directory) / "planned_path.csv"
    if not path.exists():
        return []
    raw_rows = []
    with path.open() as handle:
        for row in csv.DictReader(handle):
            lat = parse_float(row.get("lat_deg"))
            lon = parse_float(row.get("lon_deg"))
            if lat is None or lon is None:
                continue
            raw_rows.append(
                {
                    "lat": lat,
                    "lon": lon,
                    "s_m": parse_float(row.get("s_m")),
                    "kappa": parse_float(row.get("kappa_1pm")),
                    "az": parse_float(row.get("az_rad")),
                    "leg": parse_float(row.get("leg_index")),
                }
            )
    if not raw_rows:
        return []

    coords = [project(r["lat"], r["lon"], lat0, lon0) for r in raw_rows]
    plan = []
    running_s = 0.0
    for i, (x, y) in enumerate(coords):
        if i > 0:
            running_s += math.hypot(x - coords[i - 1][0], y - coords[i - 1][1])
        row = raw_rows[i]
        if row["az"] is not None:
            azimuth = row["az"]
        else:
            j = min(i + 1, len(coords) - 1)
            k = max(i - 1, 0)
            azimuth = math.atan2(coords[j][0] - coords[k][0], coords[j][1] - coords[k][1])
        plan.append(
            {
                "x": x,
                "y": y,
                "s_m": row["s_m"] if row["s_m"] is not None else running_s,
                "kappa": row["kappa"] if row["kappa"] is not None else numeric_curvature(coords, i),
                "az": azimuth,
                "leg": int(row["leg"]) if row["leg"] is not None else 0,
                "polyline_s": running_s,
            }
        )
    return plan


def signed_cross_track(x, y, vertex):
    """Starboard-positive offset from the plan tangent, matching the planner's own formula."""
    tx = math.sin(vertex["az"])
    ty = math.cos(vertex["az"])
    return ty * (x - vertex["x"]) - tx * (y - vertex["y"])


def project_track(track, plan, meta):
    """Attach a geometric cross-track error and the local plan geometry to each track row.

    The search window mirrors the planner's own progress search (a bounded look-back and
    look-ahead around a monotonic pointer) so the estimate is directly comparable with the
    planner-reported error. Global nearest-neighbour is wrong on any self-overlapping path.
    """
    if not plan:
        return track, False

    step = max(0.5, meta.get("sample_step_m", 2.0))
    back_vertices = 2  # the planner looks back 2 * sampleStepM, i.e. two vertices
    diverged = False
    consecutive_far = 0
    far_limit = max(4.0 * step, 3.0 * meta.get("turn_radius_m", 25.0))

    # Seed at the vertex nearest the first fix, searched only over the opening of the plan so
    # a route that revisits its start cannot seed at the wrong end.
    seed_span = max(2, len(plan) // 20)
    pointer = min(range(seed_span), key=lambda i: (track[0]["x"] - plan[i]["x"]) ** 2 + (track[0]["y"] - plan[i]["y"]) ** 2)

    for row in track:
        x, y = row["x"], row["y"]
        speed = row["speed_mps"] or 0.0
        ahead_m = max(6.0 * step, 3.0 * max(speed, 1.0))
        ahead_vertices = max(2, int(round(ahead_m / step)))

        lo = max(0, pointer - back_vertices)
        hi = min(len(plan) - 1, pointer + ahead_vertices)
        if row["waypoint_index"] is not None:
            leg = int(row["waypoint_index"])
            if leg > plan[pointer]["leg"]:
                # A real leg boundary: the planner restarts its pointer here, so follow it
                # rather than letting the monotonic window drag across the transition.
                for idx in range(len(plan)):
                    if plan[idx]["leg"] == leg:
                        pointer = idx
                        lo = idx
                        hi = min(len(plan) - 1, idx + ahead_vertices)
                        break

        best_idx = pointer
        best_d2 = float("inf")
        for idx in range(lo, hi + 1):
            d2 = (x - plan[idx]["x"]) ** 2 + (y - plan[idx]["y"]) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = idx
        pointer = best_idx

        if math.sqrt(best_d2) > far_limit:
            consecutive_far += 1
            if consecutive_far > 10:
                diverged = True
        else:
            consecutive_far = 0

        vertex = plan[best_idx]
        row["geom_xte_m"] = signed_cross_track(x, y, vertex)
        row["plan_kappa"] = vertex["kappa"]
        row["plan_s_m"] = vertex["s_m"]
        row["plan_leg"] = vertex["leg"]

    return track, diverged


def classify_segments(kappa, turn_radius_m):
    """'arc', 'straight' or 'transition' from the planned curvature at a sample."""
    if kappa is None or turn_radius_m <= 0.0:
        return "transition"
    reference = 1.0 / turn_radius_m
    magnitude = abs(kappa)
    if magnitude >= ARC_CURVATURE_FRAC * reference:
        return "arc"
    if magnitude <= STRAIGHT_CURVATURE_FRAC * reference:
        return "straight"
    return "transition"


def path_length(points):
    total = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
    return total


def histogram(values, reference, bins=12, top=1.2):
    """Counts of |value|/reference in equal bins over [0, top], plus an overflow bin."""
    if reference <= 0.0:
        return []
    width = top / bins
    counts = [0] * (bins + 1)
    for value in values:
        ratio = abs(value) / reference
        index = int(ratio / width)
        counts[min(index, bins)] += 1
    return counts


def analyse(directory):
    """Compute every metric for one recording; returns a dict of named values plus context."""
    meta, meta_from_run = load_meta(directory)
    track = load_track(directory)
    if not track:
        raise ValueError("track.csv has no usable rows")

    lat0 = track[0]["lat_deg"]
    lon0 = track[0]["lon_deg"]
    for row in track:
        row["x"], row["y"] = project(row["lat_deg"], row["lon_deg"], lat0, lon0)

    plan = load_plan(directory, lat0, lon0)
    plan_has_kappa = bool(plan) and any(abs(v["kappa"]) > 0.0 for v in plan)
    track, diverged = project_track(track, plan, meta)

    turn_radius = meta.get("turn_radius_m", FALLBACK_META["turn_radius_m"])
    min_radius = meta.get("min_turn_radius_m", FALLBACK_META["min_turn_radius_m"])
    omega_max = meta.get("max_turn_rate_rps", FALLBACK_META["max_turn_rate_rps"])
    margin_m = turn_radius - min_radius

    flown = path_length([(r["x"], r["y"]) for r in track])
    planned = path_length([(v["x"], v["y"]) for v in plan]) if plan else 0.0
    ratio = flown / planned if planned > 1e-6 else None
    geometry_valid = (
        plan_has_kappa
        and not diverged
        and ratio is not None
        and PLAN_LENGTH_RATIO_GATE[0] <= ratio <= PLAN_LENGTH_RATIO_GATE[1]
    )

    moving = [r for r in track if (r["speed_mps"] or 0.0) > MOVING_SPEED_MPS]
    reported_rates = [r["yaw_rate_rps"] for r in moving if r["yaw_rate_rps"] is not None]
    omega_source = "reported" if len(reported_rates) >= 0.99 * max(len(moving), 1) else "differenced"
    if omega_source == "differenced":
        for i, row in enumerate(track):
            if i == 0 or row["elapsed_s"] is None or track[i - 1]["elapsed_s"] is None:
                row["omega"] = None
                continue
            dt = row["elapsed_s"] - track[i - 1]["elapsed_s"]
            row["omega"] = wrap_pi(row["yaw_rad"] - track[i - 1]["yaw_rad"]) / dt if dt > 0 else None
    else:
        for row in track:
            row["omega"] = row["yaw_rate_rps"]

    arc_radial = []
    arc_radial_port = []
    arc_radial_stbd = []
    arc_omega = []
    straight_xte = []
    exec_xte_arc = []
    disagreement = []
    settle_distances = []

    # A straight only counts toward steady state once the vehicle has had room to recover from
    # the preceding turn; the recovery itself is reported separately as a settle distance.
    settle_window_m = 2.0 * turn_radius
    last_arc_end_s = None
    awaiting_settle = False

    for row in moving:
        kappa = row.get("plan_kappa")
        segment = classify_segments(kappa, turn_radius)
        exec_xte = row["cross_track_error_m"]
        geom_xte = row.get("geom_xte_m")
        # The planner's own error is authoritative; the geometric one is the cross-check.
        xte = exec_xte if exec_xte is not None else geom_xte
        if exec_xte is not None and geom_xte is not None:
            disagreement.append(exec_xte - geom_xte)
        if xte is None:
            continue
        plan_s = row.get("plan_s_m")
        if segment == "arc":
            last_arc_end_s = plan_s
            awaiting_settle = True
        if segment == "arc" and geometry_valid:
            radial = math.copysign(1.0, kappa) * xte
            arc_radial.append(radial)
            (arc_radial_port if kappa > 0.0 else arc_radial_stbd).append(radial)
            if row["omega"] is not None:
                arc_omega.append(row["omega"])
        elif segment == "straight" and geometry_valid:
            if last_arc_end_s is not None and plan_s is not None:
                since_arc = plan_s - last_arc_end_s
                # Armed once when the arc ends, so this is recovery distance, not distance
                # travelled along the leg.
                if awaiting_settle and abs(xte) < 0.5:
                    settle_distances.append(since_arc)
                    awaiting_settle = False
                if since_arc < settle_window_m:
                    continue
            straight_xte.append(xte)
        if segment == "arc" and exec_xte is not None:
            exec_xte_arc.append(abs(exec_xte))

    all_omega = [r["omega"] for r in moving if r["omega"] is not None]
    saturated = sum(1 for w in all_omega if abs(w) >= 0.95 * omega_max)
    over_limit = sum(1 for w in all_omega if abs(w) > omega_max * 1.0001)
    deep_in_margin = sum(1 for r in arc_radial if r <= -0.95 * margin_m) if margin_m > 0 else 0

    return {
        "directory": str(directory),
        "meta_from_run": meta_from_run,
        "plan_has_kappa": plan_has_kappa,
        "geometry_valid": geometry_valid,
        "diverged": diverged,
        "omega_source": omega_source,
        "turn_radius_m": turn_radius,
        "min_turn_radius_m": min_radius,
        "margin_m": margin_m,
        "omega_max_rps": omega_max,
        "planned_omega_rps": meta.get("cruising_speed_mps", 3.0) / turn_radius if turn_radius > 0 else None,
        "samples": len(track),
        "moving_samples": len(moving),
        "flown_m": flown,
        "planned_m": planned,
        "flown_over_planned": ratio,
        "arc_samples": len(arc_radial),
        "arc_radial_mean": mean(arc_radial),
        "arc_radial_p1": percentile(arc_radial, 0.01),
        "arc_radial_p10": percentile(arc_radial, 0.10),
        "arc_radial_p50": percentile(arc_radial, 0.50),
        "arc_radial_min": min(arc_radial) if arc_radial else None,
        "arc_radial_port_mean": mean(arc_radial_port),
        "arc_radial_stbd_mean": mean(arc_radial_stbd),
        "arc_margin_consumed_frac": deep_in_margin / len(arc_radial) if arc_radial else None,
        "arc_median_radius_m": (
            percentile([meta.get("cruising_speed_mps", 3.0) / abs(w) for w in arc_omega if abs(w) > 1e-6], 0.5)
            if arc_omega
            else None
        ),
        "arc_omega_median": percentile([abs(w) for w in arc_omega], 0.5) if arc_omega else None,
        "arc_omega_p95": percentile([abs(w) for w in arc_omega], 0.95) if arc_omega else None,
        "straight_samples": len(straight_xte),
        "straight_xte_mean": mean(straight_xte),
        "straight_xte_absmean": mean([abs(v) for v in straight_xte]),
        "straight_xte_p90": percentile([abs(v) for v in straight_xte], 0.90),
        "straight_xte_max": max([abs(v) for v in straight_xte]) if straight_xte else None,
        "settle_distance_median_m": percentile(settle_distances, 0.5),
        "settle_distance_max_m": max(settle_distances) if settle_distances else None,
        "saturated_frac": saturated / len(all_omega) if all_omega else None,
        "over_limit_samples": over_limit,
        "exec_xte_arc_p90": percentile(exec_xte_arc, 0.90),
        "exec_xte_arc_max": max(exec_xte_arc) if exec_xte_arc else None,
        "exec_vs_geom_rms": (
            math.sqrt(sum(d * d for d in disagreement) / len(disagreement)) if disagreement else None
        ),
        "omega_histogram": histogram(all_omega, omega_max),
    }


def fmt(value, digits=3):
    return "n/a" if value is None else f"{value:.{digits}f}"


def report(result):
    """Print the metrics table and every caveat that limits how it may be read."""
    print(f"=== {result['directory']} ===")
    if not result["meta_from_run"]:
        print("  NOTE: no meta.csv - thresholds come from built-in shipped-config defaults")
    if not result["plan_has_kappa"]:
        print("  NOTE: planned_path.csv has no curvature column - derived from the polyline")
    print(
        f"  R_plan={fmt(result['turn_radius_m'],2)} m  R_min={fmt(result['min_turn_radius_m'],2)} m  "
        f"margin={fmt(result['margin_m'],2)} m  omega_max={fmt(result['omega_max_rps'],4)} rad/s"
    )
    print(
        f"  flown={fmt(result['flown_m'],1)} m  planned={fmt(result['planned_m'],1)} m  "
        f"ratio={fmt(result['flown_over_planned'])}"
    )
    if result["diverged"]:
        print("  PLAN DIVERGED: the track left the previewed plan; geometric metrics suppressed")
    if not result["geometry_valid"]:
        print(
            "  GEOMETRIC GATES REFUSED: the flown/planned ratio is outside "
            f"[{PLAN_LENGTH_RATIO_GATE[0]}, {PLAN_LENGTH_RATIO_GATE[1]}] or the plan carries no curvature."
        )
        print("  Only the planner-reported cross-track error is meaningful for this run.")
    print(f"  omega source: {result['omega_source']}")
    if result["omega_source"] == "differenced":
        print("    WARNING: differenced yaw aliases against the publisher and overshoots the")
        print("    platform limit; do not gate on turn rate from this recording.")

    print(f"  -- arcs (n={result['arc_samples']}) --")
    print(
        f"    radial offset  mean={fmt(result['arc_radial_mean'])}  p10={fmt(result['arc_radial_p10'])}  "
        f"p50={fmt(result['arc_radial_p50'])}  min={fmt(result['arc_radial_min'])}   (negative = inside the arc)"
    )
    print(
        f"    port mean={fmt(result['arc_radial_port_mean'])}  stbd mean={fmt(result['arc_radial_stbd_mean'])}"
    )
    consumed = result["arc_margin_consumed_frac"]
    print(f"    samples at >=95% of the turn-radius margin: {'n/a' if consumed is None else f'{100*consumed:.1f}%'}")
    print(
        f"    median arc radius={fmt(result['arc_median_radius_m'],2)} m  "
        f"median |omega|={fmt(result['arc_omega_median'],4)}  p95={fmt(result['arc_omega_p95'],4)}  "
        f"(planned {fmt(result['planned_omega_rps'],4)})"
    )
    print(f"  -- straights, steady state (n={result['straight_samples']}) --")
    print(
        f"    xte mean={fmt(result['straight_xte_mean'])}  |xte| mean={fmt(result['straight_xte_absmean'])}  "
        f"p90={fmt(result['straight_xte_p90'])}  max={fmt(result['straight_xte_max'])}"
    )
    print(
        f"    post-turn settle to |xte|<0.5 m: median={fmt(result['settle_distance_median_m'],1)} m  "
        f"max={fmt(result['settle_distance_max_m'],1)} m"
    )
    print(f"  -- turn-rate utilisation (moving samples) --")
    sat = result["saturated_frac"]
    print(f"    at >=95% omega_max: {'n/a' if sat is None else f'{100*sat:.1f}%'}   above omega_max: {result['over_limit_samples']} samples")
    counts = result["omega_histogram"]
    if counts:
        total = max(sum(counts), 1)
        bins = len(counts) - 1
        for i, count in enumerate(counts):
            label = f">{1.2:.1f}" if i == bins else f"{i*1.2/bins:.1f}-{(i+1)*1.2/bins:.1f}"
            bar = "#" * int(round(40.0 * count / total))
            print(f"      |w|/wmax {label:>9}  {count:6d}  {bar}")
    if result["exec_xte_arc_p90"] is not None:
        print(
            f"  -- planner-reported |xte| on arcs --  p90={fmt(result['exec_xte_arc_p90'])}  "
            f"max={fmt(result['exec_xte_arc_max'])}"
        )
    if result["exec_vs_geom_rms"] is not None:
        print(f"  exec vs geometric xte RMS disagreement: {fmt(result['exec_vs_geom_rms'])} m")


def evaluate_gates(result):
    """Acceptance gates for a post-refactor run. Returns a list of (name, ok, detail)."""
    margin = result["margin_m"]
    planned_omega = result["planned_omega_rps"]
    checks = []

    def check(name, value, ok, detail):
        checks.append((name, bool(ok) if value is not None else None, detail))

    check("0 geometry evaluable", True, result["geometry_valid"], "flown/planned in range and plan has curvature")
    check("0 omega reported", True, result["omega_source"] == "reported", f"source={result['omega_source']}")

    mean_radial = result["arc_radial_mean"]
    check("1 arc mean offset", mean_radial, mean_radial is not None and abs(mean_radial) <= 0.50, "|mean| <= 0.50 m")
    p10 = result["arc_radial_p10"]
    check("1 arc p10 offset", p10, p10 is not None and p10 >= -1.00, "p10 >= -1.00 m")
    low = result["arc_radial_min"]
    check("1 arc min offset", low, low is not None and low >= -1.20, "min >= -1.20 m")
    port = result["arc_radial_port_mean"]
    stbd = result["arc_radial_stbd_mean"]
    check(
        "1 port/stbd symmetry",
        None if port is None or stbd is None else port - stbd,
        port is not None and stbd is not None and abs(port - stbd) <= 0.30,
        "|port - stbd| <= 0.30 m",
    )

    consumed = result["arc_margin_consumed_frac"]
    check("2 margin consumed", consumed, consumed is not None and consumed <= 0.0, "no sample at >=95% of margin")
    p1 = result["arc_radial_p1"]
    check("2 arc p1 vs margin", p1, p1 is not None and p1 >= -0.6 * margin, f"p1 >= {-0.6 * margin:.2f} m")

    median_omega = result["arc_omega_median"]
    check(
        "3 arc turn rate",
        median_omega,
        median_omega is not None and planned_omega is not None and abs(median_omega - planned_omega) <= 0.10 * planned_omega,
        f"median |omega| within 10% of {planned_omega:.4f}" if planned_omega else "n/a",
    )
    check("3 no over-limit samples", result["over_limit_samples"], result["over_limit_samples"] == 0, "0 samples above omega_max")

    s_mean = result["straight_xte_mean"]
    check("4 straight mean", s_mean, s_mean is not None and abs(s_mean) <= 0.10, "|mean| <= 0.10 m")
    s_p90 = result["straight_xte_p90"]
    check("4 straight p90", s_p90, s_p90 is not None and s_p90 <= 0.20, "p90 <= 0.20 m")
    s_max = result["straight_xte_max"]
    check("4 straight max", s_max, s_max is not None and s_max <= 0.50, "max <= 0.50 m")
    return checks


def load_expectations(path):
    """key,min,max rows describing the range a metric must fall in."""
    rows = []
    with Path(path).open() as handle:
        for row in csv.DictReader(handle):
            rows.append((row["key"], parse_float(row.get("min")), parse_float(row.get("max"))))
    return rows


def evaluate_expectations(result, expectations):
    checks = []
    for key, low, high in expectations:
        value = result.get(key)
        if value is None or not isinstance(value, (int, float)):
            checks.append((key, None, "metric absent"))
            continue
        ok = (low is None or value >= low) and (high is None or value <= high)
        checks.append((key, ok, f"{fmt(value,4)} in [{fmt(low,4)}, {fmt(high,4)}]"))
    return checks


def report_checks(title, checks):
    print()
    print(f"=== {title} ===")
    failed = 0
    for name, ok, detail in checks:
        if ok is None:
            mark = "SKIP"
            failed += 1
        elif ok:
            mark = "PASS"
        else:
            mark = "FAIL"
            failed += 1
        print(f"  [{mark}] {name:<26} {detail}")
    print(f"  {len(checks) - failed}/{len(checks)} passed")
    return failed


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("directory", help="a mission_runner output directory")
    parser.add_argument("--gate", action="store_true", help="apply the acceptance gates and fail on any breach")
    parser.add_argument("--expect", help="a key,min,max CSV of metric ranges this run must satisfy")
    args = parser.parse_args()

    try:
        result = analyse(Path(args.directory))
    except (FileNotFoundError, ValueError) as exc:
        print(f"cannot evaluate {args.directory}: {exc}", file=sys.stderr)
        return 2
    report(result)

    failures = 0
    if args.expect:
        try:
            expectations = load_expectations(args.expect)
        except (OSError, KeyError) as exc:
            print(f"cannot read expectations {args.expect}: {exc}", file=sys.stderr)
            return 2
        failures += report_checks("expectations", evaluate_expectations(result, expectations))
    if args.gate:
        failures += report_checks("acceptance gates", evaluate_gates(result))

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
