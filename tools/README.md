# Autopilot tools

Both tools are the UMAA *consumer* side of the autopilot's Global Waypoint control service,
built on the shared `WaypointMissionClient` (command + large-list route out; ack, command
status, and execution status back) and `MissionRoute` (waypoint construction) sources in
this directory.

## `mission_console`

Live mission-control web GUI. The C++ backend bridges the DDS bus to a single-page
browser app (no external web dependencies — works on an air-gapped network):

```bash
# alongside a running autopilot (same YAML/domain):
./apps/autopilot/mission_console autopilot.yaml 8080 web
# then open http://localhost:8080/
```

- Subscribes to the three SA navigation reports plus the waypoint command ack / status /
  execution-status topics, and streams a JSON state snapshot to the browser over
  server-sent events (~5 Hz). REST endpoints: `GET /api/state`, `GET /api/stream`,
  `POST /api/mission`, `POST /api/mission/cancel`, `POST /api/preview`.
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
./apps/autopilot/autopilot autopilot.yaml

# terminal 2: run the built-in mission and record outputs
./apps/autopilot/mission_runner autopilot.yaml mission-out

# ... or fly a custom route (local tangent-plane CSV, one waypoint per line:
# east_m,north_m,speed_mps,capture_radius_m[,arrival_yaw_rad][,elev_value_m,elev_frame]
# where elev_frame is `depth` or `asf`)
./apps/autopilot/mission_runner autopilot.yaml mission-out my-mission.csv
```

Outputs in the chosen directory:

- `track.csv` — `elapsed_s, lat_deg, lon_deg, yaw_rad, speed_mps, depth_m, alt_asf_m`
  sampled from the SA Global Pose / Speed reports.
- `waypoints.csv` — the commanded route: `index, lat_deg, lon_deg, capture_radius_m,
  arrival_yaw_rad, elev_value_m, elev_frame`.
- `planned_path.csv` — the ideal planned Dubins route (`lat_deg, lon_deg` samples), for
  comparing the executed track against the plan.
- `status.log` — command status transitions and the final state.

Exit code 0 iff the command COMPLETED.

## `plot_mission.py`

Renders the recorded mission (requires python3 + matplotlib):

```bash
python3 ../apps/autopilot/tools/plot_mission.py mission-out mission-out/mission_plot.png
```

Produces a figure with the ground track vs the planned waypoints (with capture radii), the
north/east position components over time, and speed over ground over time.
