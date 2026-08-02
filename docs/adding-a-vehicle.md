# Adding a vehicle-control strategy

A vehicle-control strategy is the hardware abstraction the autopilot actuates through. Everything
upstream — planning, zone avoidance, constraint clamping, arbitration, operational modes — is
already done by the time your code is called. Guidance produces a `ControlVector`;
`AutopilotBrain::emitControl` is the single actuation path in the system; it ends in your
`sendControlVector`.

Only one strategy ships: `SimVehicleControl`, a kinematic simulator. It is the worked example
throughout, but note the one place it is *not* representative — see
[navigation publication](#navigation-publication-read-this-one).

## The interface

`include/autopilot/vehicle/IVehicleControl.hpp`, five methods:

| Method | Called from | Must do | Must not |
|---|---|---|---|
| `bool initialize()` | app startup, once, before the loop | open the link; return false to abort startup | assume navigation is flowing yet |
| `void shutdown()` | end of `run()`, on the control-loop thread, and from your own destructor | join every thread you started; be idempotent | be called from `stop()` — see [threading](#threading-and-shutdown) |
| `bool sendControlVector(const ControlVector&)` | every guidance recompute, i.e. once per pose sample | actuate | block; the caller is inside the recompute |
| `bool isManualEngaged() const` | every control tick, and again before every actuation | be cheap, non-blocking, `const` | lie — `true` suppresses **all** output, including safe mode |
| `void onOperationalModeChanged(OperationalMode)` | on each reported mode change, control-loop thread | optional: arm/disarm actuators when leaving STANDBY | assume it fires before the first `sendControlVector` |

`shutdown()` and `onOperationalModeChanged()` have empty defaults; the other three are pure.

**The platform owns MANUAL.** Returning `true` from the very first `isManualEngaged()` poll boots
the autopilot into MANUAL. While it is true, nothing is actuated — not vector commands, not
waypoint following, not a safe-mode maneuver. That is the correct behaviour for a hardware
override, and it means the method must be honest.

`sendControlVector`'s return value is currently discarded by the brain. Return `false` on a
rejected link write anyway; it is the right contract and the brain will grow a use for it.

## Units, frames and conventions

`include/autopilot/guidance/ControlVector.hpp`:

| Field | Unit | Frame / convention |
|---|---|---|
| `headingRad` | rad | **true north, azimuth, clockwise-positive**, wrapped to `[-pi, pi]`. Not magnetic. Not math-frame. |
| `speedMps` | m/s | **over ground.** The autopilot commands SOG because that is what `SpeedReport` carries; a platform servoing through-water must convert. May be negative if you declare `max_reverse_speed_mps`. |
| `elevationM` | m, optional | interpreted per `elevationFrame`. `nullopt` means hold current / don't care. |
| `elevationFrame` | enum | `DEPTH` (positive **down**, 0 at surface) and `ALTITUDE_ASF` (positive **up**, 0 at the sea floor) are enforced by the autopilot; `ALTITUDE_MSL`, `ALTITUDE_AGL`, `ALTITUDE_GEODETIC` are not convertible and are refused when a limit is set |
| `pitchRad` | rad, optional | written by nobody and read by nobody today. Treat as reserved. |

A sign error on `headingRad` is the classic integration bug. If your platform's heading is
counterclockwise-positive or referenced to math-frame east, convert at the boundary and say so in
a comment.

**What you are guaranteed:** finiteness is screened before you see it; speed and elevation are
already clamped by active constraints, static limits and platform capabilities with the most
restrictive winning; nothing arrives while you report MANUAL. When the autopilot cannot bound an
elevation it does not send one — `elevationM` arrives as `nullopt` (hold current) rather than
unchecked.

**What you are not guaranteed:** no rate limit on successive setpoints beyond the planner's own
geometry, and no promise a setpoint repeats. Hold the last one.

**Elevation frames you cannot service should be rejected, not approximated.** `DEPTH` and
`ALTITUDE_ASF` are both first-class: they are interconvertible through the seafloor reference
`depth + altitudeASF` from your own pose, so `constraints.max_depth_m` bounds an ASF setpoint and
`constraints.min_altitude_asf_m` bounds a DEPTH one. `ALTITUDE_MSL`, `ALTITUDE_AGL` and
`ALTITUDE_GEODETIC` have no such reference, so commands using them are refused whenever either
elevation limit is configured.

**If you publish `altitudeASF`, say so** with `platform_capabilities.underwater.reports_altitude_asf`.
Without it, `ALTITUDE_ASF` commands are rejected at validation (nothing could track or bound them)
and `constraints.min_altitude_asf_m` is a startup error.

Losing bottom lock mid-command degrades rather than misbehaves: the frame-native bottom clearance
still applies, an ASF setpoint that only a depth bound could check is withheld instead of sent
unchecked, and elevation reports as not achieved. A **vector** command is not failed for it — the
hard-tolerance delay is suspended while the elevation cannot be judged. A **waypoint** route still
requires elevation to capture, because completing a depth-required waypoint on a depth that was
never measured would be a false success; a short dropout is absorbed by
`planner.max_misses_per_waypoint`, and a permanent one fails the route visibly.

## Navigation publication (read this one)

`SimVehicleControl` publishes `GlobalPoseReport`, `SpeedReport` and `VelocityReport` itself, every
cycle, under `identity.nav_source_id`. It does that because it *is* the vehicle and there is no
navigation suite behind it.

**On a real platform your strategy must not publish those topics.** The vehicle's own navigation
suite already does. Two publishers on the same topic is a silent failure mode: the pose observer
fires on each sample, so guidance recomputes against alternating truths at double rate, and the
symptom looks like a control instability rather than a wiring mistake.

What must be true instead:

- The three SA topics reach the autopilot on the same domain with reader-compatible QoS.
- They arrive at least as fast as you want guidance to run. **Guidance recomputes per pose sample,
  not per `loop.control_period_ms`** — the control period paces the housekeeping tick, not the
  control law.
- They arrive inside `loop.nav_staleness_timeout_ms`, or the brain latches a zero-speed hold.

Fields actually read: pose → geodetic latitude/longitude, yaw, depth, altitude ASF; speed → speed
over ground; velocity → NED velocity and yaw rate. The tracking law does not need yaw rate, but
`analyze_tracking.py` degrades to differencing yaw without it and says so loudly.

`identity.nav_source_id` is meaningful only for a strategy that publishes navigation. On a real
hull it is unused — and the factory still hands you a `NavReportSenders` whose DDS writers exist
but stay silent. Ignore them; an idle writer emits no samples.

If the platform has no UMAA navigation publisher at all, that is a bridge, and it belongs in its
own component rather than inside the vehicle strategy — unless the strategy already owns the
sensor link.

## Preconditions the platform config must satisfy

| Requirement | What it prevents |
|---|---|
| `platform_capabilities.surface` has a cruising or max-forward speed **and** a positive `max_turn_rate_rps` | startup fails without them; they derive the planned turn radius, and every tracking threshold scales off it |
| all `identity.*` values are well-formed UUIDs | a garbage GUID parses to something, and commands addressed to the configured ID would never match |
| navigation arrives inside `loop.nav_staleness_timeout_ms` | a permanent zero-speed hold |
| `constraints.max_depth_m` above the deepest mission depth | admission rejections that look like planner failures |
| `constraints.min_altitude_asf_m` below the shallowest water the mission crosses | a depth setpoint clamped toward the surface wherever the bottom comes up |
| `platform_capabilities.underwater.reports_altitude_asf` set only if the pose really carries `altitudeASF` | ASF commands admitted that nothing can track or bound |
| `platform_specs` dimensions | published once as `UVPlatformSpecsReport`; consumers size their own margins from it |

Then go to [tuning.md](tuning.md).

## File-touch checklist

Register the name **first**. The tools share `YamlConfigLoader::load`, so an unregistered
`vehicle_control.type` makes `mission_console`, `mission_runner` and `heading_probe` refuse to
start with a config error that looks unrelated to your vehicle work.

1. `src/vehicle/VehicleControlTypes.cpp` — add the name to `vehicleControlTypes()`
2. `include/autopilot/vehicle/<Name>VehicleControl.hpp` — guard
   `AUTOPILOT_VEHICLE_<NAME>VEHICLECONTROL_HPP_`
3. `src/vehicle/<Name>VehicleControl.cpp`
4. `include/autopilot/config/AutopilotConfig.hpp` — a `<Name>VehicleConfig` struct plus a member
   on `AutopilotConfig`, beside `SimVehicleConfig`
5. `src/config/YamlConfigLoader.cpp` — read the `vehicle_control.<name>` block, mirroring the `sim`
   one
6. `include/autopilot/config/ConfigKeyRegistry.hpp` / `.cpp` — add every new dotted key to
   `knownKeys()`, or the loader will warn that your own keys are unrecognised
7. `src/config/ConfigValidation.cpp` — a finiteness screen for every float, plus range rules.
   Enforce *required* fields only when `vehicle_control.type` selects your strategy, otherwise a
   sim-only config that omits your block will fail
8. `src/vehicle/VehicleControlFactory.cpp` — a `static` builder plus one dispatch branch
9. `CMakeLists.txt` — the new `.cpp` in `autopilot-app`, the test in `autopilot_test`. Sources are
   listed explicitly; never `file(GLOB ...)`
10. `config/autopilot.yaml` — your block, with a comment on each knob saying *why* it exists
11. `test/<Name>VehicleControlTest.cpp`

## Registering with the factory

```cpp
// src/vehicle/VehicleControlFactory.cpp
static std::unique_ptr<IVehicleControl> makeAcmeVehicleControl(const AutopilotConfig& config,
                                                               const NavReportSenders& navSenders) {
  return std::make_unique<AcmeVehicleControl>(config.platformCapabilities, config.acmeVehicle);
}

std::unique_ptr<IVehicleControl> makeVehicleControl(const AutopilotConfig& config,
                                                    const NavReportSenders& navSenders) {
  if (config.vehicleControlType == "sim") {
    return makeSimVehicleControl(config, navSenders);
  }
  if (config.vehicleControlType == "acme") {
    return makeAcmeVehicleControl(config, navSenders);
  }
  ...
}
```

The factory does **not** fall back. An unknown type returns `nullptr` and startup fails. This is a
deliberate asymmetry with `makeSafeModeStrategy`, which does fall back: there, a degraded safe
behaviour beats none, whereas silently substituting the simulator for a real hull means the
autopilot computes control vectors nobody actuates while every report says it is driving.

Config validation rejects unregistered types first, so a `nullptr` from the factory means a
registered name without a branch, or missing IO — which is what
`VehicleControlFactoryTest.EveryRegisteredTypeIsBuildableAndValid` exists to catch.

The `vehicle_control` block, once yours is registered:

```yaml
vehicle_control:
  type: acme                     # unregistered names fail startup, they do not fall back to sim
  sim:                           # ignored unless type: sim
    ...
  acme:                          # ignored unless type: acme
    can_interface: "can0"        # every knob gets a comment saying WHY it exists
    arm_timeout_ms: 2000
```

## Threading and shutdown

`shutdown()` runs on the control-loop thread at the end of `run()`, deliberately **not** in
`stop()`. `stop()` executes in signal-handler context on an arbitrary thread, and a repeated
SIGTERM delivered on your own worker thread would make it try to join itself (`EDEADLK`).

Therefore:

- **Join threads in `shutdown()` and nowhere else.**
- Call `shutdown()` from your destructor too, and make it idempotent — `SimVehicleControl` does
  both, and `SimVehicleControlTest` asserts it.
- `sendControlVector` arrives on the pose-observer path; `isManualEngaged` is polled from both the
  control loop and the actuation path. Guard shared state with `std::scoped_lock`.
- Do not block in `sendControlVector`; it runs inside the guidance recompute.

## Testing

`test/SimVehicleControlTest.cpp` is the template. Copy:

- The `SimFixture` shape: three `LocalReaderSender`s from `umaa-cpp::test-utils`, the capability
  and config structs, and a `make()`. This is what keeps the strategy transport-agnostic and
  testable without DDS.
- A deterministic step entry point (`stepOnce(dtS)`) so tests never depend on your background
  thread.

Baseline cases: accepts and stores a setpoint; respects the `platform_capabilities` limits;
`initialize()` fails cleanly on bad config; `shutdown()` is idempotent and re-`initialize()` works;
MANUAL suppresses actuation.

When you need a canned-return fake rather than a kinematic model, use GMock —
`MockGatingVehicle` in `test/OperationalModeGatingTest.cpp` is the three-line shape. Hand-rolled
kinematic simulators are allowed where a mock cannot express vehicle motion; canned-return fakes
are not.

## Build, test, format

```sh
cmake --preset amd64-debug && cmake --build --preset amd64-debug
ctest --preset amd64-debug
./build/autopilot_test --gtest_filter='<Name>VehicleControlTest.*'
git ls-files '*.cpp' '*.hpp' | grep -vE '^(third-party|umaa-cpp)/' | xargs clang-format -i
```

Then a real run: `docker compose up`, drive a mission from the console, and confirm the vehicle
does what the console says it is doing.

## Standards that bite here

From `CLAUDE.md`, the ones this work touches most:

- `flt32_t` / `flt64_t`, never bare `float`/`double`; fixed-width ints from `<cstdint>`, never bare
  `int`
- `std::scoped_lock`, never `std::lock_guard<std::mutex>`; `while`, never `do/while`
- Smart pointers only — no `new`/`delete`. `std::unique_ptr` by default
- `static` functions in the `.cpp` for single-use helpers; **never anonymous namespaces**
- One class per file, filename matching the class; pure interfaces as `I<Name>.hpp`
- Headers under `include/autopilot/vehicle/`, sources mirroring under `src/vehicle/`
- Write C++17 idioms even though the SDK forces C++20 on the build
- No paragraph comments, no banner comments; convey intent in names
- Deferred work as `// TODO(<project>-#<issue>): ...` referencing a real issue

## See also

- [tuning.md](tuning.md) — bring-up order once the strategy runs
- `include/autopilot/vehicle/SimVehicleControl.hpp` — the worked example
- `include/autopilot/vehicle/NavReportSenders.hpp` — the navigation-publication distinction
