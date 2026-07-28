# Autopilot tools

Both tools are the UMAA *consumer* side of the autopilot's Global Waypoint control service,
built on the shared `WaypointMissionClient` (command + large-list route out; ack, command
status, and execution status back) and the autopilot library's `MissionRoute` (waypoint
construction). The console additionally consumes the autopilot's MM constraint services
through `ConstraintsClient` in this directory.

## `mission_console`

Live mission-control web GUI. The C++ backend bridges the DDS bus to a single-page
browser app (no external web dependencies — works on an air-gapped network):

```bash
# alongside a running autopilot (same YAML/domain):
./mission_console autopilot.yaml 8080 web
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
`id` is an upsert/edit), `DELETE /api/constraints/<uuid>`, `POST /api/constraints/active`.

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
./mission_runner autopilot.yaml mission-out

# ... or fly a custom route (local tangent-plane CSV, one waypoint per line:
# east_m,north_m,speed_mps,capture_radius_m[,arrival_yaw_rad][,elev_value_m,elev_frame]
# where elev_frame is `depth` or `asf`)
./mission_runner autopilot.yaml mission-out my-mission.csv
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
python3 ../tools/plot_mission.py mission-out mission-out/mission_plot.png
```

Produces a figure with the ground track vs the planned waypoints (with capture radii), the
north/east position components over time, and speed over ground over time.
