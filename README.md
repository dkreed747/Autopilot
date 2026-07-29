# Autopilot

UMAA-native autopilot for uncrewed marine vehicles. Consumes UMAA MO Global
Waypoint / Global Vector commands over CycloneDDS, plans and tracks true Dubins
routes, enforces MM constraint/conditional safety services with a violation FSM
and safe-mode strategies, and runs an MM operational-mode FSM — all against a
swappable vehicle-control strategy (a kinematic sim ships in-tree). Built on the
[umaa-cpp](umaa-cpp/) SDK (pinned submodule).

## Quickstart

```sh
git submodule update --init --recursive
cmake --preset amd64-debug            # or amd64-release; both build in build/
cmake --build --preset amd64-debug
ctest --preset amd64-debug            # full suite: autopilot + SDK tests

cd build
export CYCLONEDDS_URI=file://$PWD/../umaa-cpp/config/cyclonedds-loopback.xml
./autopilot autopilot.yaml                          # terminal 1: the autopilot (sim vehicle)
./tools/mission_console autopilot.yaml 8080 web     # terminal 2: web console -> http://localhost:8080
./tools/mission_runner autopilot.yaml out           # terminal 3: fly the built-in demo mission
```

`cyclonedds-loopback.xml` keeps DDS discovery on localhost — use it for every
local run so peers find each other without multicast.

## Features

| Feature | Where | Try it |
|---|---|---|
| UMAA waypoint missions: large-list routes, gate (not bubble) capture, miss/replan budgets | `src/umaa/WaypointControlServiceProvider.cpp`, `src/guidance/DubinsPathPlanner.cpp` | `./tools/mission_runner autopilot.yaml out my.csv` |
| UMAA vector commands: heading/speed/elevation setpoints w/ tolerance tracking | `src/umaa/VectorControlServiceProvider.cpp` | console **Vector** panel, or `POST /api/vector` |
| True Dubins planning + PI cross-track tracking (`planner.xte.*`; `ki=0` = pure P) | `src/guidance/DubinsPath*.cpp`, `src/guidance/CrossTrackController.cpp` | set `vehicle_control.sim.current_east_mps: 0.3`, compare `ki: 0` vs `0.05` |
| Zone-aware planning: direct solve → Dubins-RRT* detour, mid-route replan on constraint change | `src/guidance/DubinsRrtStar.cpp` | draw a keep-out across an active route in the console |
| MM constraints: water zones / speed / depth, standing active set, per-conditional state reports | `src/safety/ConstraintSupervisor.cpp` | console **Constraints** panel |
| Violation FSM: MONITORING → RECOVERING (grace) → SAFE_MODE (latched), recovery drive-out | `src/safety/ConstraintSupervisor.cpp`, `src/safety/RecoveryGuidance.cpp` | activate a keep-out you are inside |
| Safe-mode strategies: Safe Return Path mission or zero-speed hold | `src/safety/SafeModeStrategyFactory.cpp`, `src/safety/SafeReturnPath.cpp` | `safety.safe_mode.strategy` |
| Output clamps: dynamic constraints + static limits + capabilities, most-restrictive-wins | `src/safety/ConstraintClamp.cpp` | activate a speed constraint below the commanded speed |
| Operational modes: MANUAL/STANDBY/REMOTE/AUTONOMOUS, LOCAL-vs-REMOTE classing by `parentID`, implicit transitions, idle revert, hold-at-ISSUED policy | `src/modes/OperationalModeManager.cpp` | console **Mode** panel; `operational_mode.*` |
| Driving-resource arbitration: class-aware priorities, safe mode above all | `src/modes/DrivingResourceArbiter.cpp` | send a vector during a mission |
| Failsafes: nav-staleness zero-speed hold, non-finite sample screening, MANUAL suppression | `src/core/AutopilotBrain.cpp` | `loop.nav_staleness_timeout_ms` |
| Web mission console: chart, mission editor w/ live Dubins preview, constraints/mode/vector panels, WASD RC with layered deadman | `tools/mission_console.cpp`, `tools/web/` | [tools/README.md](tools/README.md) |
| Headless mission runner + plotting | `tools/mission_runner.cpp`, `tools/plot_mission.py` | `python3 tools/plot_mission.py out/` |

## Architecture

One control loop (`AutopilotApp::step`, `loop.control_period_ms`) cycles DDS
consumers/providers; guidance recomputes per Global Pose sample
(`AutopilotBrain::onNavUpdate`). The sim vehicle integrates on its own thread.
Layers:

- **core/** — app wiring (`AutopilotApp`), control funnel + mode dispatch
  (`AutopilotBrain::emitControl` is the single actuation path; clamps and
  MANUAL suppression live there), shared nav state.
- **guidance/** — Dubins solver/planner/RRT*, cross-track PI, route/tolerance
  utilities.
- **safety/** — constraint supervisor + violation FSM, zone map/geometry
  (signed clearance, no polygon offsetting), recovery, safe-mode strategies.
- **modes/** — operational-mode FSM + provider, driving-resource arbiter.
- **umaa/** — UMAA service providers/observers (waypoint, vector, platform
  reports, nav observers).
- **vehicle/** — `IVehicleControl` strategy; `SimVehicleControl` is the
  in-tree implementation. Real platforms implement `IVehicleControl`
  (init/shutdown, `sendControlVector`, `isManualEngaged`) and are selected via
  `vehicle_control.type`.

The mission tools link only `autopilot::core` (config + route/guidance/zone
slice) — they talk to the app purely over DDS and are staged to move into their
own repositories.

## Configuration

One YAML for app and tools: [`config/autopilot.yaml`](config/autopilot.yaml) —
the inline comments are the reference. Loading fail-fasts on malformed values
(ranges, UUIDs, enums) and warns on suspicious ones (`src/config/ConfigValidation.cpp`).
Identity UUIDs must be well-formed; `console.platform_id` different from
`identity.platform_id` makes console commands REMOTE. The copy in `build/` is
refreshed on every build.

## Tests

```sh
ctest --preset amd64-debug          # or amd64-release
./build/autopilot_test --gtest_filter='DubinsPathPlannerTest.*'
```

GTest/GMock, ~180 cases across 22 suites: planner/solver/RRT* (incl. randomized
and drift/PI regression), zone geometry/map, constraint clamp/supervisor,
safety end-to-end (recovery, SRP, safe-mode), operational modes/gating/arbiter,
sim vehicle, config loader/validation.

## Deployment

CI publishes three images from one multi-target [`Dockerfile`](Dockerfile):

| Image | Contents | Default command |
|---|---|---|
| `…/autopilot` | autonomy stack only | `bin/autopilot autopilot.yaml` |
| `…/autopilot/mission-console` | web console + SPA (port 8080, `CONSOLE_PORT` override) | `entrypoint-console.sh` |
| `…/autopilot/mission-runner` | one-shot mission driver (exit 0 iff COMPLETED) | writes `mission-out/` volume |

> The app image no longer bundles the console — compose replaces the old
> all-in-one image.

```sh
docker compose up -d                       # autopilot only
docker compose --profile console up -d     # + web console on :8080
docker compose --profile runner up         # + one-shot demo mission
```

[`docker-compose.yml`](docker-compose.yml) runs a bridge network with multicast
off; the tools unicast-peer at the `autopilot` service (the topology CI
smoke-tests). All services share `./config/autopilot.yaml` (read-only mount).

> **Never run two autopilot instances with the same `identity.*` on one DDS
> domain** (e.g. a stray local binary plus the container): both publish nav and
> consume commands under identical IDs, and missions fail in confusing ways.
> Compose's single `autopilot` service enforces this; for local runs make sure
> the previous instance is dead first.
For a console on another host: run the console image there with
`CYCLONEDDS_URI` peers pointing at the vehicle's IP (and the vehicle side
peering back or listening with multicast off), publish 8080.

Local image build: `docker build --target autopilot .` (or `mission-console` /
`mission-runner`; needs the submodule in the context).

## CI

`.gitlab-ci.yml`: `build-test` (full build + ctest in the dev image) →
`build-images` (Kaniko, three `--target` runs, shared cache) →
`smoke-autopilot` / `smoke-console` / `smoke-runner` (the split images
round-trip constraints, modes, vectors, and a full mission over unicast-peer
DDS on the per-build network). Override `BASE_IMAGE_TAG` to test against an
unmerged dev-container image.

## Maintaining

- Standards + workflow: [`CLAUDE.md`](CLAUDE.md). Format with
  `git ls-files '*.cpp' '*.hpp' | grep -vE '^(third-party|umaa-cpp)/' | xargs clang-format -i`.
- Logging: log4cxx via the SDK (`log4cxx.xml` resolved from the working
  directory; `UMAA_LOG_*` macros).
- All binaries resolve `autopilot.yaml`, `CYCLONE_QOS_PROFILES.xml`,
  `log4cxx.xml`, and the console's `web/` root relative to the working
  directory (the images symlink them at `/opt/autopilot`).
- SDK: pinned `umaa-cpp` submodule; `-DAUTOPILOT_USE_SYSTEM_UMAA_CPP=ON`
  builds against an installed SDK instead (tests require the submodule).
- Recorded reference runs live in `docs/mission-results/` (each has the input
  `mission.csv` plus recorded track/status); re-run them with `mission_runner`
  to regression-check guidance changes.
