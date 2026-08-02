# Autopilot tools

These tools are the UMAA *consumer* side of the autopilot's services, built on the shared
clients in `clients/` (`WaypointMissionClient`, `VectorCommandClient`, `ConstraintsClient`,
`OperationalModeClient`) and bus monitors in `monitors/`. They link only `autopilot::core`
(config + route/guidance/zone slice) and talk to the autopilot purely over DDS — the tools
are staged to move into their own repositories. Binaries build to `build/tools/`.

## `mission_console`

Live mission-control web GUI. The C++ backend bridges the DDS bus to a single-page
browser app (no external web dependencies — works on an air-gapped network):

```bash
# from the build directory, alongside a running autopilot (same YAML/domain):
./tools/mission_console autopilot.yaml 8080 web
# then open http://localhost:8080/
```

- Subscribes to the three SA navigation reports plus the waypoint command ack / status /
  execution-status topics, and streams a JSON state snapshot to the browser over
  server-sent events (~5 Hz). REST endpoints: `GET /api/state`, `GET /api/stream`,
  `POST /api/mission`, `POST /api/mission/cancel`, `POST /api/preview`, `POST /api/mode`,
  `POST /api/vector`, `POST /api/vector/cancel`, `POST /api/rc`.
- The console commands under its **own UMAA identity** (`console:` block in the YAML):
  outgoing commands carry `source.id = console.source_id` and
  `source.parentID = console.platform_id`. Keep the platform id different from
  `identity.platform_id` to act as a REMOTE operator (the shipped default) or set it equal
  to act as the onboard autonomy.
- The chart plots the vehicle (heading, trail) on a local-tangent-plane graticule with
  pan/zoom, the active mission's waypoints (capture gates, arrival-attitude arrows,
  completed waypoints faded, the current one pulsing), the ideal planned Dubins route, and
  an animated dashed line for the active leg. The right panel carries position / heading /
  speed / depth / altitude-above-floor readouts plus execution details (distance to
  waypoint, cross-track error, waypoints remaining) and the command status history.
- **New mission**: click the chart to drop waypoints; click a waypoint (marker or list row)
  to edit its speed, capture radius, optional arrival heading, and optional elevation
  (depth or above-sea-floor). The ideal Dubins route for the draft is previewed live from
  the vehicle's current pose (`POST /api/preview` -> `DubinsPathPlanner::previewRoute`).
- The **EXECUTE** button publishes the route as a UMAA large list plus the referencing
  command, then turns into **CANCEL** (which disposes the command instance — the UMAA
  cancellation request) until the session reaches a terminal state. Chips above it show
  the live command status and whether the provider's command acknowledgement was received.
- Clicking a waypoint mid-mission shows its parameters, the session's command status/ack,
  and — for the current waypoint — the live achieved flags from the execution status
  report.

### Constraints panel

When the autopilot's `identity.constraints_source_id` is configured, the console is also a
full consumer of its MM constraint services (`ConstraintsClient`): specialization payloads
plus ConditionalControl Add/Delete and ActiveConstraints commands out; the ConditionalReport
(the authoritative constraint list), the standing ActiveConstraints acknowledgement (the
authoritative *applied* set — this is how a restarted console recovers the active set), and
the per-conditional state reports back. REST: `POST /api/constraints` (create; a body with
`id` is an upsert/edit), `DELETE /api/constraints/<uuid>`, `POST /api/constraints/active`
(`{"ids": ["<uuid>", ...]}` — the full applied set).

- **Zones**: the *+ Keep-in* / *+ Keep-out* buttons enter zone-draw mode — click the chart
  to add vertices; double-click, press Enter, or click the first vertex to close (Esc
  cancels; self-intersecting rings are rejected). Keep-in areas render green, keep-out red,
  inactive ones faded/dashed, violated ones flashing. Click a zone (chart or list) to edit
  its name and ceiling/floor bounds — each bound carries its own frame picker (depth, or
  altitude above the sea floor), so a zone can span e.g. from the surface (depth 0) down to
  5 m above the sea floor.
- **Speed / depth limits**: *+ Speed* / *+ Depth* create at-most/at-least value constraints
  edited in the same panel.
- Constraints are **global** (they outlive any one vector/waypoint session) and **toggled**
  via the per-row checkboxes: toggles apply immediately (300 ms debounce), show *pending…*
  until the autopilot's acknowledgement echoes the commanded set, and the row flags
  **VIOLATED** from the autopilot's ConditionalStateReport (mirrored by the execute dock's
  `CONSTRAINT VIOLATED` chip). An *active set unknown* banner shows until the first
  acknowledgement arrives.

### Operational mode panel

When the autopilot's `identity.operational_mode_control_source_id` is configured, the top
panel shows the vehicle's reported mode (MANUAL / STANDBY / REMOTE / AUTONOMOUS — STALE
when the 1 Hz report stops) and commands STANDBY / REMOTE / AUTONOMOUS via
`POST /api/mode`. MANUAL is platform-owned and never commandable. With implicit
transitions enabled the mode also follows commands automatically (sending a mission or
vector from the console flips the vehicle into REMOTE; the onboard autonomy's commands
flip it into AUTONOMOUS), and an implicitly-entered mode falls back to STANDBY once its
commands finish.

### Vector panel & bus activity

Compose a vector setpoint (heading °T, speed, optional depth/above-floor elevation,
optional timeout that auto-completes the command via its UMAA end time) and
Execute/Cancel it. The *Bus activity* list and the chart also show **every** waypoint
mission and vector command observed on the DDS bus — whoever commanded it — classified
and colored per source: own (green), onboard autonomy (teal), remote operator (magenta).

### Remote control (WASD)

*RC mode* drives the vehicle with the keyboard through rolling vector-command updates
(~5 Hz): `W` = along the current vehicle heading, `W+A`/`W+D` = ∓/± 45°, `A`/`D` = ∓/± 90°,
`S` = stop, `R`/`F` = speed up/down (clamped to platform limits), `Shift`/`Ctrl` = move
up/down (frame-aware), `Z` = toggle depth ↔ above-sea-floor, `Esc` exits. Safety is
layered: every update carries a 2 s end time (a dead console cannot leave the vehicle
driving), the backend stops the vehicle after 1 s without browser heartbeats, and
releasing all direction keys commands zero speed. The *Heading up* checkbox rotates the
chart so the vehicle's heading points up — much easier to steer from the vehicle's
perspective.

Screenshots (recorded against the sim vehicle): `../docs/console/`.

The backend serves static files from the web root passed as the third argument (CMake
copies `tools/web/` next to the build output). It uses the vendored single-header
`third-party/httplib` (HTTP/SSE) and `third-party/nlohmann` (JSON).

## `mission_runner`

End-to-end waypoint mission driver: the UMAA *consumer* side of the autopilot's Global
Waypoint control service. It publishes a `GlobalWaypointCommandType` addressed to the
autopilot's waypoint provider (`identity.waypoint_source_id` from the YAML) together with its
large-list route, then records the vehicle's Global Pose track and the command status until
the mission reaches a terminal state (COMPLETED / FAILED / CANCELED, or a 20-minute timeout).

The route is a closed loop with turns in both directions, laid out relative to the sim
vehicle's configured start position.

```bash
# terminal 1 (from the build directory): the autopilot with the sim vehicle
./autopilot autopilot.yaml

# terminal 2: run the built-in mission and record outputs
./tools/mission_runner autopilot.yaml mission-out

# ... or fly a custom route (local tangent-plane CSV, one waypoint per line:
# east_m,north_m,speed_mps,capture_radius_m[,arrival_yaw_rad][,elev_value_m,elev_frame]
# where elev_frame is `depth` or `asf`; a malformed row rejects the whole file, and
# the autopilot rejects non-positive or above-platform-max speeds)
./tools/mission_runner autopilot.yaml mission-out my-mission.csv
```

Outputs in the chosen directory:

- `track.csv` — one row per new pose fix (~20 Hz). Columns are append-only, so older
  recordings still parse; read by header name, never by position.
  - pose/speed, from the SA Global Pose and Speed reports:
    `elapsed_s, lat_deg, lon_deg, yaw_rad, speed_mps, depth_m, alt_asf_m`
  - `wall_utc_s` — epoch seconds at full sub-second precision, for aligning a recording
    with vehicle-side logs.
  - from the SA Velocity report: `yaw_rate_rps, vel_north_mps, vel_east_mps, vel_down_mps,
    vel_age_s`. Prefer `yaw_rate_rps` over differencing `yaw_rad`: the 20 Hz poll beats
    against the 20 Hz publisher, and differenced yaw shows peaks near 200% of the platform
    turn-rate limit that are sampling artifacts, not motion.
  - from the waypoint execution status report (filtered to this session):
    `waypoint_index, cross_track_error_m, distance_to_waypoint_m, distance_remaining_m,
    cumulative_distance_m, waypoints_remaining, exec_age_s`. `cross_track_error_m` is the
    planner's own error against the *live* leg and is authoritative; `planned_path.csv` is
    re-planned from the start pose and diverges once the route replans.
  - Empty fields mean the source had not reported yet; the `*_age_s` columns expose
    carry-forward staleness when a platform publishes slower than the pose rate.
- `waypoints.csv` — the commanded route: `index, lat_deg, lon_deg, capture_radius_m,
  arrival_yaw_rad, elev_value_m, elev_frame`.
- `planned_path.csv` — the ideal planned Dubins route: `lat_deg, lon_deg, s_m, kappa_1pm,
  az_rad, leg_index`. Curvature is exact from the planned geometry (`+1/rho` on a port
  turn, `0` on straights and the final-approach runway), so an analysis never has to
  differentiate the polyline; `leg_index` joins to `track.csv`'s `waypoint_index`.
- `meta.csv` — `key,value` record of the configuration that produced the run (turn radius,
  the kinematic minimum radius the margin is measured against, capabilities, tracking gains,
  sim currents). Without it a recording cannot be judged against any threshold after the fact.
- `status.log` — command status transitions and the final state.

Exit code 0 iff the command COMPLETED.

> Start a **fresh autopilot for every recording**. `planned_path.csv` is planned from the
> configured `vehicle_control.sim.initial_*` pose, so a run that begins where a previous
> mission left the vehicle is not comparable with its own plan. `analyze_tracking.py`
> detects this and refuses the geometric metrics rather than reporting nonsense.

## `heading_probe`

Identifies the platform's inner heading loop by commanding heading steps over the UMAA Global
Vector control service and measuring the response. **Run this before tuning the path tracker**:
its reported time constant is what sizes the tracker's curvature feedforward, and guessing it
is how a tracker ends up either under-turning or cutting corners on every arc.

```bash
./tools/heading_probe autopilot.yaml probe-out
./tools/heading_probe autopilot.yaml probe-out --speed=1.5 --schedule="20,-20,40,-40" \
    --dwell-max=30 --verify-tol=0.10
python3 ../tools/plot_heading_probe.py probe-out probe-out/probe_plot.png
```

Options: `--speed` (default `cruising_speed_mps`), `--schedule` (signed degree deltas, default
`5,-5,10,-10,20,-20,40,-40,90,-90,5,-5`), `--dwell-max`, `--settle`, `--warmup`, `--depth`/`--asf`,
`--verify-tol`, `--dry-run` (print the time and ground-track estimate and exit).

The schedule is a list of *deltas* applied to the current commanded heading, so it alternates
direction (real hulls are not symmetric), sums to zero, and re-measures the small-signal gain at
the end to expose drift. A step ends when the error has held inside its tolerance for two
seconds, or at `--dwell-max`; a fixed dwell cannot work when the turn rate is the unknown being
measured. Every command carries a 5 s end time, so a probe that dies stops the vehicle.

Before running against a real vehicle: no zones or constraints may be active, because vector-mode
zone guidance rewrites the commanded heading and invalidates the identification. `--dry-run`
reports how much water the schedule needs.

Outputs `probe.csv` (per-sample) and `identification.csv`:

- `K_rps_per_rad`, `K_r2`, `K_port_*`/`K_stbd_*`, `asymmetry_frac`
- `omega_max_rps`, `omega_max_cv`, `omega_max_windows`
- `e_sat_rad` — the error at which the loop saturates, `omega_max / K`
- `heading_loop_tau_s` — `1/K`, which is the value to configure
- `decay_tau_s` and `model_consistency` (`decay_tau * K`; 1.0 means the loop really is the
  first-order servo the tracker assumes, so a large deviation means actuator lag the
  feedforward will not capture)
- `omega_max_ratio_to_config`, `R_min_measured_m`, `verdict`

`verdict` is `OK`, or:

- `SATURATION_DOMINATED` — nearly every sample sits against the rate limit, so there is no
  linear region to fit. Trust `omega_max_rps` and `e_sat_knee_rad`; the gain fields are written
  **blank rather than zero** so a non-identifiable run cannot be pasted into config as `0`.
  The in-tree simulated vehicle is a proportional servo with lag (`heading_gain_rps_per_rad`,
  `heading_lag_s`), not a pure rate limiter, so a schedule with steps inside its linear region
  (`e_sat` is about 17 degrees at the shipped gain) identifies cleanly; only a schedule of large
  steps saturates. A pure rate limiter is still reachable by setting the gain to `1 / dt`.
- `UNRELIABLE_OMEGA_MAX` — fewer than three large steps saturated, or they disagreed by more
  than 15%. Add 40-degree-or-larger steps.

What to do with the measured `heading_loop_tau_s`, and the rest of the bring-up order, is in
[docs/tuning.md](../docs/tuning.md).

Exit code 0 on success, 2 when `--verify-tol` is set and the measured turn rate disagrees with
`platform_capabilities.surface.max_turn_rate_rps` by more than that fraction, 1 on IO/DDS
failure or an aborted command. **Reconcile a `--verify-tol` failure before tuning anything**:
every planned turn radius is derived from the configured rate, so the whole plan is wrong until
the config matches the platform.

## `analyze_tracking.py`

Objective tracking metrics for a recording (standard library only, no matplotlib):

```bash
python3 ../tools/analyze_tracking.py mission-out
python3 ../tools/analyze_tracking.py mission-out --gate                # apply acceptance gates
python3 ../tools/analyze_tracking.py mission-out --expect bounds.csv   # assert metric ranges
```

Exit code 0 when every requested check passed, 1 on a breach, 2 when the inputs cannot be
evaluated. `--gate` and `--expect` are what make it usable as a CI or campaign decision;
without them it only reports.

Cross-track behaviour is conditioned on the planned geometry, because arcs and straights fail
differently. On arcs it reports a **radial offset** (`sign(kappa) * cross_track_error`), which
is negative when the vehicle is cutting *inside* the planned arc and positive when it
overshoots, and compares the standing offset against the available turn-radius margin
(`turn_radius_m - min_turn_radius_m`). Straights are reported as steady state only, excluding
`2 * turn_radius_m` of recovery after each turn; that recovery is reported separately as a
settle distance. Arc samples are also split port/starboard so a control bias shows up as an
asymmetry instead of averaging away.

Two guards decide whether the geometric numbers may be trusted at all:

- **`flown / planned` length ratio.** Outside `[0.90, 1.20]` every geometric metric is
  suppressed. A depth-rate-limited spiral flies loop-back passes that the from-start preview
  cannot contain (ratio ~2.1), so its only meaningful error is the planner's own
  `cross_track_error_m`.
- **Turn-rate source.** `yaw_rate_rps` is used when recorded. Falling back to differencing
  `yaw_rad` is reported loudly, because the 20 Hz poll beats against the 20 Hz publisher and
  produces peaks near 200% of the platform limit that are sampling artifacts.

Each archived recording carries an `expectations.csv` locking its measured values, so CI can
verify that a change to the analysis still reproduces known-good data.

## `plot_mission.py`

Renders the recorded mission (requires python3 + matplotlib):

```bash
python3 ../tools/plot_mission.py mission-out mission-out/mission_plot.png
```

Produces a figure with the ground track vs the planned waypoints (with capture radii), the
north/east position components over time, and speed over ground over time.
