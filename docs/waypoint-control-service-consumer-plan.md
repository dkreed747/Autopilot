# Implementation plan: `WaypointControlServiceConsumer`

A consumer-side class that lets autonomy behaviors drive the vehicle with a vector of UMAA
waypoints, regardless of whether the target platform implements the Global Waypoint control
service. The consumer prefers the native **waypoint mode** (publish the route to
`WaypointControlServiceProvider`); when no waypoint provider answers — or the caller forces it —
it falls back to **vector mode**, running the waypoint route through the in-repo guidance stack
locally and streaming the result as Global Vector commands. Callers submit a route, then poll
for command status and a unified execution-status/progress view that looks the same in both
modes.

This plan is written to be executed by an agent working in the RTI Connext build of the
`umaa-cpp` SDK. Everything below that names a UMAA type, topic constant, or SDK abstraction
(`ReaderBase<T>`, `SenderBase<T>`, `LargeListReader/Writer`, `LocalReaderSender<T>`) is
transport-agnostic — the types are generated from the same UMAA IDL and the topic-name
constants travel with them. Only the concrete transport construction differs; see §2.

---

## 1. Existing architecture (what to build on)

| Piece | Where | Relevance |
|---|---|---|
| `WaypointControlServiceProvider` | `include/autopilot/umaa/WaypointControlServiceProvider.hpp`, `src/umaa/WaypointControlServiceProvider.cpp` | The provider the consumer commands in waypoint mode. Defines the failure taxonomy the fallback logic keys on (§5). |
| `VectorControlServiceProvider` | `include/autopilot/umaa/VectorControlServiceProvider.hpp`, `src/umaa/VectorControlServiceProvider.cpp` | The provider the consumer commands in vector mode. `endTime` on the command is the deadman; no `endTime` means run forever. |
| `WaypointMissionClient` | `tools/clients/WaypointMissionClient.{hpp,cpp}` | Existing waypoint consumer session logic: session minting, large-list publication, ack/status filtering by `sessionID`, dispose-as-cancel, dispose-on-terminal, cancel-echo timeout. Reuse via facade, do not reimplement. |
| `VectorCommandClient` | `tools/clients/VectorCommandClient.{hpp,cpp}` | Existing vector consumer session logic incl. `update()` (same-session re-send re-anchoring `endTime`) and `pollExec()`. Reuse via facade. |
| `ClientIdentity` | `tools/clients/ClientIdentity.{hpp,cpp}` | `sourceId`/`platformId` stamping; LOCAL vs REMOTE classification is `source().parentID() == identity.platform_id`. |
| Guidance kit | `include/autopilot/guidance/` (built into `autopilot::core`, which `mission-tools` already links) | `DubinsPathPlanner` (`plan()` / `ControlVector update(pose, groundSpeedMps, dtS)` / `progress()` / `routeComplete()` / `failed()`), `derivePlannerParams()` in `PlannerParamsFactory.hpp`, `MissionRoute.hpp` (`MissionWaypoint`, `makeWaypoint()`, `statusName()`), `ControlVector.hpp`, `ProgressTypes.hpp` (`WaypointProgress`), `ToleranceUtils.hpp`, `ElevationUtils.hpp`. This is the entire waypoint→vector translation engine — the provider itself flies routes with exactly this planner (`src/core/AutopilotBrain.cpp`, `updateWaypointControl()`). |
| Mode/arbitration | `include/autopilot/modes/` | Determines when commands are admitted (§7). |
| Reference tests | `test/OperationalModeControlProviderTest.cpp`, `test/OperationalModeGatingTest.cpp`, `test/PathTrackingRegressionTest.cpp`, `test/ServoSimVehicle.{hpp,cpp}` | The loopback-IO fixture pattern and the closed-loop sim pattern to copy (§10). |

Precedent note on naming: existing consumer-side classes here are `*Client`, and the SDK
reserves `*Consumer` for report consumers (`GlobalPoseReportConsumer`, ...). The name
`WaypointControlServiceConsumer` is nevertheless the requested and UMAA-symmetric choice
(mirrors `WaypointControlServiceProvider`); keep it.

## 2. Transport seam (Cyclone here, Connext there)

This repo constructs transport as `arlcore::io::CycloneReader<T>` / `CycloneSender<T>`
(`(participant, TopicConstant, qos)`), with QoS from `arlcore::io::CycloneQosProviderWrapper`
and the participant from `arlcore::io::getDomainParticipant(domainId)`. All of it flows through
the abstract `arlcore::io::ReaderBase<T>` / `SenderBase<T>` (`read`, `readLatest`, `readUpToN`,
`send`, `dispose`, `ReadStatus`, `SendStatus`, `SampleEnvelope<T>`).

**Rule for the new code: the class logic must depend only on `ReaderBase`/`SenderBase`
(injected via an Io struct, §4), never on a concrete transport.** A single factory function
builds the concrete Io from a participant + QoS, mirroring whatever the local Connext tree's
equivalents of `CycloneReader`/`CycloneSender` and the QoS provider wrapper are (in the RTI
build, mirror the construction used by the local `WaypointMissionClient`/provider wiring — same
pattern, RTI QoS profiles `UMAA_QoS_Library::UMAA_Base_Profile` and
`UMAA_QoS_Library::UMAA_LargeCollections_Profile`, both reliable + transient-local). Everything
else in this plan is unchanged between transports.

## 3. Files to create / modify

Create (all in `mission-tools`, namespace `arlcore::autopilot::tools`):

- `tools/clients/WaypointControlServiceConsumer.hpp` / `.cpp` — the class (guard
  `AUTOPILOT_TOOLS_CLIENTS_WAYPOINTCONTROLSERVICECONSUMER_HPP_`).
- `tools/clients/WaypointControlServiceConsumerIo.hpp` — Io bundle + factory (guard
  `AUTOPILOT_TOOLS_CLIENTS_WAYPOINTCONTROLSERVICECONSUMERIO_HPP_`; tightly-coupled config
  struct is allowed in the class header per the one-class-per-file carve-out).
- `test/WaypointControlServiceConsumerTest.cpp`.

Modify:

- `tools/clients/WaypointMissionClient.{hpp,cpp}` — add an Io-injection constructor taking
  `shared_ptr<SenderBase/ReaderBase>` for its four endpoints (existing participant ctor
  delegates to it), and add `pollExec()` + an execution-status reader on
  `UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportTypeTopic`, mirroring
  `VectorCommandClient::pollExec()` (filter by `sessionID`). Today only the bus-wide monitor
  reads exec status; the consumer needs it session-scoped.
- `tools/clients/VectorCommandClient.{hpp,cpp}` — add the same Io-injection constructor
  (four endpoints). No behavior change.
- `tools/CMakeLists.txt` — add the two new sources to `mission-tools`' explicit list.
- `CMakeLists.txt` — add the test source to `autopilot_test`'s list, link `mission-tools`
  into `autopilot_test`, and add `tools/` to its include dirs (tools are currently untested;
  this is the one-time plumbing).

No `file(GLOB)`. Filenames match class names. `.hpp`/`.cpp` only.

## 4. Public API sketch

```cpp
namespace arlcore { namespace autopilot { namespace tools {

struct WaypointControlServiceConsumerIo {
    // waypoint service (5 endpoints, matching the provider's Io)
    std::shared_ptr<arlcore::io::SenderBase<GlobalWaypointCommandType>> wpCmdSender;
    std::shared_ptr<arlcore::io::SenderBase<GlobalWaypointCommandTypeWaypointsListElement>> wpListSender;
    std::shared_ptr<arlcore::io::ReaderBase<GlobalWaypointCommandAckReportType>> wpAckReader;
    std::shared_ptr<arlcore::io::ReaderBase<GlobalWaypointCommandStatusType>> wpStatusReader;
    std::shared_ptr<arlcore::io::ReaderBase<GlobalWaypointExecutionStatusReportType>> wpExecReader;
    // vector service (4 endpoints)
    std::shared_ptr<arlcore::io::SenderBase<GlobalVectorCommandType>> vecCmdSender;
    std::shared_ptr<arlcore::io::ReaderBase<GlobalVectorCommandAckReportType>> vecAckReader;
    std::shared_ptr<arlcore::io::ReaderBase<GlobalVectorCommandStatusType>> vecStatusReader;
    std::shared_ptr<arlcore::io::ReaderBase<GlobalVectorExecutionStatusReportType>> vecExecReader;
    // navigation feedback for the local guidance loop
    std::shared_ptr<arlcore::io::ReaderBase<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>> poseReader;
    std::shared_ptr<arlcore::io::ReaderBase<UMAA::SA::SpeedStatus::SpeedReportType>> speedReader;
};
// factory building concrete transport (participant, base wqos/rqos, large-collections wqos)
WaypointControlServiceConsumerIo makeWaypointControlServiceConsumerIo(...);

enum class ControlModePreference { AUTO, WAYPOINT_ONLY, VECTOR_ONLY };
enum class ActiveControlMode { NONE, WAYPOINT, VECTOR };
enum class ConsumerState { IDLE, WAYPOINT_PENDING, WAYPOINT_EXECUTING,
                           VECTOR_EXECUTING, COMPLETED, FAILED, CANCELED };

struct WaypointControlServiceConsumerConfig {
    arlcore::NumericGuid waypointDestinationId;   // identity.waypoint_source_id
    arlcore::NumericGuid vectorDestinationId;     // identity.vector_source_id
    ControlModePreference modePreference = ControlModePreference::AUTO;
    flt64_t ackTimeoutS = 5.0;          // no waypoint ack within this => provider absent
    flt64_t vectorDeadmanS = 5.0;       // endTime horizon stamped on every vector update
    flt64_t vectorResendMaxAgeS = 1.0;  // re-send unchanged setpoint at least this often
    int64_t navStalenessTimeoutMs = 2000;
    PlannerParams plannerParams;        // from derivePlannerParams(AutopilotConfig) or hand-set
    std::vector<CommandStatusReasonEnumType> fallbackReasons;  // extra reasons that trigger
                                                               // fallback; default empty (§5)
};

class WaypointControlServiceConsumer {
  public:
    WaypointControlServiceConsumer(const WaypointControlServiceConsumerIo& io,
                                   const ClientIdentity& identity,
                                   const WaypointControlServiceConsumerConfig& config);

    // Submit a route; cancels any in-flight session first (mirrors CANCEL_EXISTING).
    arlcore::NumericGuid execute(const std::vector<GlobalWaypointType>& waypoints);
    arlcore::NumericGuid execute(const std::vector<MissionWaypoint>& waypoints);  // via makeWaypoint()
    bool cancel();

    // Drive everything; call at the control cadence (>= 10 Hz, 20 Hz recommended).
    void poll();

    // Status surface
    ConsumerState state() const;
    ActiveControlMode activeMode() const;
    bool ackReceived() const;                                    // ack of the active session
    const std::optional<arlcore::NumericGuid>& sessionId() const; // active underlying session
    std::vector<MissionStatusUpdate> drainStatusUpdates();       // reuse the existing struct
    std::optional<WaypointProgress> progress() const;            // unified, both modes (§6)
    std::optional<int32_t> currentWaypointIndex() const;         // waypointID -> index lookup
};

}}}  // namespace arlcore::autopilot::tools
```

Composition: the class owns a `WaypointMissionClient` and a `VectorCommandClient` (constructed
from the Io slices) plus a `DubinsPathPlanner`, a `LocalCartesian`-free pose/speed cache, and
the state machine. Do **not** reuse `NavState` — it lives in `autopilot-app`, which
`mission-tools` deliberately does not link; a small internal cache (last finite pose, last
speed, receipt time) is enough. Screen non-finite lat/lon/yaw exactly as
`GlobalPoseObserver` does.

Threading: poll-driven and **not** internally synchronized, matching every existing client
(`mission_console` guards all clients with one mutex on the caller side). Document that in the
header in one line. If a lock ever becomes necessary it must be `std::scoped_lock`.

## 5. State machine and fallback policy

```
IDLE --execute()--> [preference?]
  AUTO / WAYPOINT_ONLY -> WAYPOINT_PENDING   (WaypointMissionClient::start)
  VECTOR_ONLY          -> VECTOR_EXECUTING   (plan locally, start vector session)

WAYPOINT_PENDING:
  ack received, non-terminal status         -> WAYPOINT_EXECUTING
  no ack within ackTimeoutS:
      AUTO          -> fall back: cancel() the waypoint session, plan locally,
                       -> VECTOR_EXECUTING
      WAYPOINT_ONLY -> FAILED
  terminal FAILED status with reason in fallbackReasons (AUTO only) -> VECTOR_EXECUTING
  terminal FAILED/CANCELED otherwise        -> FAILED / CANCELED

WAYPOINT_EXECUTING:
  status COMPLETED -> COMPLETED; FAILED -> FAILED; CANCELED -> CANCELED
  (no mid-flight auto-fallback: a session that got as far as EXECUTING proves the
   provider exists; a later failure is a mission failure, not a capability gap)

VECTOR_EXECUTING:
  planner.routeComplete() -> send zero-speed setpoint, cancel vector session -> COMPLETED
  planner.failed()        -> zero-speed, cancel                              -> FAILED
  nav stale > navStalenessTimeoutMs -> zero-speed, cancel                    -> FAILED
  vector session dies (update() false / terminal status) -> restart session once per
      setpoint tick (heading_probe / console RC precedent); repeated restarts failing
      with no ack within ackTimeoutS -> FAILED

any active state --cancel()--> CANCELED (dispose semantics per mode, §6)
```

Fallback reasoning, keyed to the provider's actual failure taxonomy:

- **Ack timeout is the primary trigger** — it is precisely "no waypoint provider on the bus",
  the stated use case. Note both providers publish on reliable + transient-local QoS, so a
  slow-starting provider still acks late; 5 s default with the existing 2 s discovery-settle
  precedent (`mission_runner` sleeps 2 s before first send).
- **Do not fall back on `RESOURCE_REJECTED`**: it means a vector command already holds the
  driving resource — another commander is actively driving. A same-class fallback vector would
  be denied too (priorities are strictly-greater-wins). Report the failure.
- **Do not fall back on `VALIDATION_FAILED`** by default: the route itself is bad (shape,
  gates) and would fail locally as well, or the mode gate rejected the commander — which
  applies to vector commands equally.
- `SERVICE_FAILED` (covers large-list assembly timeout, zone violation, no nav fix) and
  `INTERRUPTED` are ambiguous — off by default, opt-in via `fallbackReasons` for platforms
  known to have a broken-but-present waypoint service.
- Sessions can also be **held at ISSUED indefinitely** (mode gate hold when
  `commands_out_of_mode_are_failed: false`). The ack still arrives, so the consumer stays in
  WAYPOINT_PENDING; surface the ISSUED status via `drainStatusUpdates()` and leave the policy
  to the caller. Do not treat a held session as provider-absent.

## 6. Behavior details

### Waypoint mode (native path)

Entirely delegated to `WaypointMissionClient`: `start()` publishes the large list
(`LargeListWriter` — remember its destructor disposes elements; the client already holds it in
`std::optional` and resets before re-emplacing), mints the session GUID, stamps
`source().id/parentID`, `destination().id`, `timeStamp`, `waypointsListMetadata`, and sends.
`cancel()` = dispose of the command instance (the UMAA cancellation request). `pollStatus()`
filters by `sessionID` and re-disposes on terminal (transient-local replay defense). The
consumer adds: `pollAck()` deadline tracking for the fallback trigger, and the new
`pollExec()` to feed the unified progress view — reconstruct `WaypointProgress` fields from
`GlobalWaypointExecutionStatusReportType` (`positionAchieved`, `crossTrackError`,
`distanceToWaypoint`, `distanceRemaining`, `cumulativeDistance`, `waypointsRemaining`,
`waypointID`, ETAs). There is no index on the wire — build
`std::map<NumericGuid, int32_t>` from `waypoints[i].waypointID()` at `execute()` time
(`mission_runner` precedent) to serve `currentWaypointIndex()`.

### Vector mode (translation path)

On entry: `planner_.plan(waypoints_, lastPose_, config_.plannerParams)`. Require a finite pose
before entering; if none has arrived yet, wait (bounded by `navStalenessTimeoutMs`) before
declaring FAILED.

Every `poll()` while VECTOR_EXECUTING:

1. Drain `poseReader`/`speedReader` (`readLatest`), screen non-finite, update the cache;
   compute `dtS` between accepted poses from `std::chrono::steady_clock`, clamp to [0, 1] s.
2. On a new pose: `const ControlVector cv = planner_.update(pose, groundSpeedMps, dtS);`
3. Map `cv` to `VectorSetpoint{cv.headingRad, cv.speedMps, cv.elevationM,
   elevation::frameName(cv.elevationFrame), config_.vectorDeadmanS}`. The planner only emits
   DEPTH/ALTITUDE_ASF frames, matching the two variants `VectorCommandClient` encodes
   (`DepthRequirementVariantVariant` / `AltitudeASFRequirementVariantVariant`).
4. First tick: `vectorClient_.start(setpoint)`; afterwards `vectorClient_.update(setpoint)` —
   same session, `endTime` re-anchored to now + deadman on every send (rolling deadman: a dead
   consumer can never leave the vehicle driving). If `update()` returns false, `start()` a
   fresh session (console RC precedent). Even with no new pose, re-send the last setpoint when
   it is older than `vectorResendMaxAgeS`.
5. Synthesize progress directly: `WaypointProgress prog = planner_.progress();` then overlay
   `groundSpeedMps` from the cache and, when a session-matched
   `GlobalVectorExecutionStatusReportType` is available, its `speedAchieved` /
   `directionAchieved` / `elevationAchieved`. Derive the two ETAs as the provider does:
   `distance / std::max(groundSpeed, 0.1)`.
6. Terminal handling per §5. Completion/cancel sequence: send one zero-speed setpoint with a
   short deadman, then `vectorClient_.cancel()` (dispose), then reset the planner.

Speed-field asymmetry to honor in any hand-built samples: a `GlobalWaypointType` speed is a
`VariableSpeedVariantType` wrapping `RequiredSpeedVariantVariant` →
`GroundSpeedRequirementVariantVariant`; a `GlobalVectorCommandType` speed is a
`SpeedRequirementVariantType` set directly. `tolerance::extractSpeed` has overloads for both;
`MissionRoute::makeWaypoint` and `VectorCommandClient::fillVectorCommand` are the two
canonical writers.

### Progress parity caveat

Vector-mode progress is computed against the **consumer's** plan; the platform's own notion
(zones, constraints, elevation admission) is not in the loop. The vector provider still
validates each command (finite direction, speed ≤ platform max, elevation admission), so a
locally-planned setpoint can be rejected — surface those FAILED statuses via
`drainStatusUpdates()` rather than swallowing them. Zone-aware local planning is available
(`DubinsPathPlanner::setZones`) but out of scope for the first cut; leave a
`// TODO(@user): wire ZoneMap into vector-mode planning` marker.

## 7. Operational-mode interaction (document, don't implement)

With the shipped `allow_implicit_mode_transitions: true`, no explicit mode command is needed:
a REMOTE-identity consumer flips STANDBY→REMOTE implicitly; a LOCAL one flips
STANDBY→AUTONOMOUS. The sharp edges to note in the class header docs:

- A LOCAL consumer can never take the resource back implicitly while the vehicle is in REMOTE
  (operator precedence) — its commands are rejected/held. Pair with `OperationalModeClient` if
  the deployment needs explicit mode management; that stays the caller's job.
- Out-of-mode commands are FAILED/`VALIDATION_FAILED` when `commands_out_of_mode_are_failed:
  true` (default) or parked at ISSUED otherwise — both already handled by §5.

## 8. Configuration & identity

Take `ClientIdentity` (from `makeClientIdentity`/`makeLocalAutonomyIdentity`) and the two
destination GUIDs (`identity.waypoint_source_id`, `identity.vector_source_id` in the YAML) via
the config struct. `plannerParams` comes from `derivePlannerParams(const AutopilotConfig&)` when
the caller has the vehicle YAML; the fields that matter most for fidelity (`turnRadiusM`,
`tracker.headingLoopTauS` — measurable with `heading_probe`) must be documented as
vehicle-specific in the header.

## 9. CMake

```cmake
# tools/CMakeLists.txt: append to mission-tools sources
clients/WaypointControlServiceConsumer.cpp

# CMakeLists.txt: autopilot_test additions
test/WaypointControlServiceConsumerTest.cpp   # in the alphabetical source list
target_link_libraries(autopilot_test PRIVATE mission-tools)   # alongside existing links
# plus tools/ on autopilot_test's include path for "clients/..." includes
```

`mission-tools` already links `autopilot::core`; no new dependencies.

## 10. Testing (`test/WaypointControlServiceConsumerTest.cpp`)

House rules: no namespaces in tests (fully-qualified names; file-scope `using` aliases allowed
only for UMAA generated types), every body laid out `// GIVEN / WHEN / THEN`, GMock for canned
fakes, hand-rolled sim only for vehicle motion. Fixture: build a
`WaypointControlServiceConsumerIo` where every endpoint is an
`arlcore::io::LocalReaderSender<T>` (from `umaa-cpp::test-utils`) — the same loopback trick as
`test/OperationalModeControlProviderTest.cpp`, roles reversed: the test reads what the consumer
sends and injects acks/statuses/poses.

Minimum test list:

1. **execute() publishes correctly** — command sample carries fresh `sessionID`, identity
   stamps, `destination().id() == waypointDestinationId`, list metadata; list elements land on
   the element endpoint; a second `execute()` retires the first (dispose observed).
2. **Status routing** — statuses for a foreign `sessionID` are ignored; own-session ISSUED →
   COMMANDED → EXECUTING → COMPLETED drives WAYPOINT_PENDING → WAYPOINT_EXECUTING → COMPLETED,
   with `drainStatusUpdates()` surfacing each transition and the terminal one flagged.
3. **cancel() in waypoint mode** disposes the command instance; CANCELED echo lands the state
   machine in CANCELED; missing echo trips the 5 s anti-wedge inactivity rule.
4. **Fallback on ack timeout (AUTO)** — no ack injected; after `ackTimeoutS` the consumer
   cancels the waypoint session and a `GlobalVectorCommandType` appears with `endTime` ≈ now +
   `vectorDeadmanS`; state VECTOR_EXECUTING. Same setup with WAYPOINT_ONLY → FAILED, no vector
   command ever sent.
5. **No mid-execution fallback** — FAILED/`OBJECTIVE_FAILED` after EXECUTING → FAILED, no
   vector command. FAILED/`RESOURCE_REJECTED` from COMMANDED → FAILED, no vector command.
   A reason listed in `fallbackReasons` → VECTOR_EXECUTING.
6. **Closed-loop vector translation** — the core test. GIVEN a `ServoSimVehicle`
   (`test/ServoSimVehicle.hpp`) and VECTOR_ONLY preference with `derivePlannerParams` from the
   shipped config; WHEN each tick feeds `vehicle.pose()` into the pose endpoint, calls
   `poll()`, decodes the latest vector command (`tolerance::extractDirection/extractSpeed`),
   and steps the vehicle with the decoded `ControlVector` at 0.05 s (copy the
   `flyRoute()` loop from `test/PathTrackingRegressionTest.cpp`); THEN the route completes
   within the tick budget, state is COMPLETED, a zero-speed setpoint then a dispose close the
   session, and `progress()` marched `waypointsRemaining` down to 0 with bounded
   cross-track error.
7. **Deadman freshness** — with no new poses, the setpoint is still re-sent within
   `vectorResendMaxAgeS`; every observed command's `endTime` is in the future by ≤
   `vectorDeadmanS`.
8. **Vector session death recovery** — inject a terminal status for the vector session
   mid-route; next `poll()` starts a fresh session (new `sessionID`) with the current setpoint.
9. **Stale-nav failsafe** — stop feeding poses; after `navStalenessTimeoutMs` a zero-speed
   setpoint and a dispose are observed and the state is FAILED.
10. **Exec-status synthesis parity** — in vector mode `progress()` mirrors
    `planner_.progress()` plus overlaid ground speed; in waypoint mode an injected
    `GlobalWaypointExecutionStatusReportType` round-trips through `progress()` and
    `currentWaypointIndex()` (waypointID → index).
11. **Non-finite pose screening** — NaN lat/lon poses are dropped, no setpoint change results.

## 11. Implementation order

1. `git submodule update --init --recursive` (the `umaa-cpp/` submodule is empty in a fresh
   clone; nothing builds without it), then verify the baseline:
   `cmake --preset amd64-debug && cmake --build --preset amd64-debug && ctest --preset amd64-debug`.
2. Verify the SDK signatures this plan reconstructs from call sites (`ReaderBase`/`SenderBase`
   method names, `LargeListWriter/Reader`, `LocalReaderSender`, `UuidFactory`,
   `arlcore::umaa::getTimestamp()`); adjust names to the Connext tree where they differ.
3. Refactor `WaypointMissionClient` + `VectorCommandClient` for Io injection (delegating
   ctors, no behavior change); add `pollExec()` to `WaypointMissionClient`. Build; existing
   tools must compile untouched.
4. Add `WaypointControlServiceConsumerIo.hpp` + factory.
5. Implement the consumer: state machine + waypoint path first (tests 1–5), then the vector
   translation path (tests 6–11). Write tests alongside each half, not after.
6. CMake wiring (§9); full build + ctest.
7. Format: `git ls-files '*.cpp' '*.hpp' | grep -vE '^(third-party|umaa-cpp)/' | xargs clang-format -i`.
8. Optional smoke: run the sim autopilot (`./build/autopilot autopilot.yaml`) and drive it
   with a small harness using the new class in WAYPOINT and VECTOR_ONLY preferences; compare
   against a `mission_runner` reference run.

## 12. Risks / open questions

- **SDK drift**: §2/§11-2 — every SDK signature here is reconstructed from call sites because
  the submodule was empty in the planning checkout; the Connext build may differ in
  construction details. The Io seam confines the blast radius.
- **`VECTOR_ONLY` progress trust**: consumer-side progress is a claim about the consumer's own
  plan, not the platform's view (§6 caveat). Acceptable for the stated goal; flagged in docs.
- **Fallback while an operator drives**: deliberately not handled (no fallback on
  `RESOURCE_REJECTED`); revisit only with a real multi-commander requirement.
- **Elevation frames**: planner emits DEPTH/ASF only; waypoint elevations in other frames
  (MSL/AGL/geodetic) are not translatable in vector mode — `execute()` should reject such
  routes up front in VECTOR_ONLY/AUTO with a clear status, rather than fail mid-route.
- **Test-target growth**: linking `mission-tools` into `autopilot_test` is new plumbing; keep
  it `PRIVATE` and watch for symbol clashes with `autopilot::app` (none expected — disjoint
  translation units).
