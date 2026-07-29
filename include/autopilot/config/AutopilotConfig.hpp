#ifndef AUTOPILOT_CONFIG_AUTOPILOTCONFIG_HPP_
#define AUTOPILOT_CONFIG_AUTOPILOTCONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief DDS transport configuration (mirrors arlcore::AppConfig fields).
struct DdsConfig {
  int32_t domainId = 0;
  std::string qosFile = "CYCLONE_QOS_PROFILES.xml";
  std::string domainQosProfile = "UMAA_QoS_Library::UMAA_Base_Profile";
  std::string largeCollectionsQosProfile = "UMAA_QoS_Library::UMAA_LargeCollections_Profile";
};

//! \brief UMAA source identifiers (UUID strings) this application publishes under.
struct IdentityConfig {
  std::string platformId;  // this platform's UMAA identity; command sources whose parentID
                           // matches are classified as local (onboard) autonomy
  std::string vectorSourceId;
  std::string waypointSourceId;
  std::string specsSourceId;
  std::string capabilitiesSourceId;
  std::string navSourceId;                     // source for the sim vehicle's SA navigation reports
  std::string constraintsSourceId;             // MM conditional/constraint services source
  std::string operationalModeControlSourceId;  // MM operational mode control provider (empty = disabled)
  std::string operationalModeStatusSourceId;
};

//! \brief Operational-mode FSM policy (MM OperationalModeControl/Status services).
struct OperationalModeConfig {
  bool allowImplicitModeTransitions = true;  // incoming driving commands may move
                                             // STANDBY->REMOTE/AUTONOMOUS and AUTONOMOUS->REMOTE
  bool commandsOutOfModeAreFailed = true;    // true = UMAA-strict fail; false = hold at ISSUED
                                             // until the mode becomes compatible
  flt64_t idleRevertS = 5.0;                 // implicitly-entered REMOTE/AUTONOMOUS revert to STANDBY after
                                             // this long with no active command of the mode's class
};

//! \brief Driving-resource arbitration priorities for one command class. Higher wins.
struct ClassArbitrationPriorities {
  int32_t vectorPriority = 100;
  int32_t waypointPriority = 10;
};

//! \brief Driving-resource arbitration priorities. Defaults keep safe-mode maneuvers above
//! everything and any remote (off-board operator) command above any local (onboard autonomy)
//! command.
struct ArbitrationConfig {
  ClassArbitrationPriorities local;
  ClassArbitrationPriorities remote{500, 400};
  int32_t safePriority = 1000;
};

//! \brief Consumer-side UMAA identity for the mission console / mission runner tools. Their
//! outgoing commands carry source id = source_id and source parentID = platform_id; a
//! platform_id equal to identity.platform_id makes the tool a local autonomy, anything else
//! (the expected console default) makes it a remote operator.
struct ConsoleConfig {
  std::string platformId;
  std::string sourceId;  // stable so restarts keep the same identity; empty = mint per process
};

struct LoopConfig {
  int32_t controlPeriodMs = 50;
  int32_t navStalenessTimeoutMs = 2000;
};

//! \brief Default tolerances applied to a vector command when it omits them.
struct VectorToleranceConfig {
  flt64_t directionRad = 0.0873;
  flt64_t speedMps = 0.25;
  flt64_t elevationM = 1.0;
  bool hard = false;            // if true, persistent violation fails the command
  flt64_t failureDelayS = 5.0;  // how long a violation must persist before failing (hard only)
};

//! \brief Default capture tolerances applied to a waypoint when it omits them.
struct WaypointToleranceConfig {
  flt64_t positionM = 2.5;
  flt64_t yawRad = 0.1745;
  flt64_t elevationM = 1.0;
};

//! \brief RRT* fallback planner settings (used only when the direct Dubins leg clips a zone).
struct RrtConfig {
  uint32_t seed = 12345;  // deterministic sampling; salted per waypoint index
  int32_t maxIterations = 2000;
  int32_t timeBudgetMs = 150;
  flt64_t goalBias = 0.10;
  int32_t nearK = 8;
  flt64_t edgeCheckStepM = 4.0;   // coarse in-tree edge sampling
  flt64_t finalCheckStepM = 1.0;  // fine recheck of the accepted path
};

//! \brief Cross-track tracking-law tuning. Defaults reproduce the legacy pure-P behavior;
//! ki > 0 nulls the standing offset a lateral current or trim leaves behind.
struct XteConfig {
  flt64_t kpScale = 1.0;              // P gain scale: correction = atan2(kp*xte, turn_radius)
  flt64_t ki = 0.0;                   // integral gain, rad per meter-second (0 = pure P)
  flt64_t integratorLimitRad = 0.35;  // |integral| clamp (~20 deg of crab)
  flt64_t integratorGateM = 5.0;      // integrate only while |xte| is inside the gate
  flt64_t correctionLimitRad = 1.2;   // total correction clamp
  flt64_t leadTimeS = 1.0;            // tangent phase-lead seconds
};

struct PlannerConfig {
  flt64_t leadDistanceM = 50.0;
  flt64_t turnRadiusMargin = 1.25;  // planned radius = margin * (speed / max turn rate)
  int32_t maxListWaitCycles = 200;
  int32_t maxMissesPerWaypoint = 3;
  bool elevationCountsAsMiss = true;
  int32_t maxReplans = 10;
  flt64_t sampleStepM = 2.0;  // path polyline sampling resolution
  XteConfig xte;
  RrtConfig rrt;
};

//! \brief Static clamp settings for the autopilot's own constraint limits (merged with dynamic
//! active constraints and platform capabilities; the most restrictive value wins).
struct ConstraintsConfig {
  std::optional<flt64_t> maxSpeedMps;
  std::optional<flt64_t> minSpeedMps;
  std::optional<flt64_t> maxDepthM;  // deepest commanded depth allowed
  std::optional<flt64_t> minDepthM;  // shallowest commanded depth allowed
};

//! \brief Water-zone geometry margins and conversion settings.
struct ZonesConfig {
  flt64_t safetyMarginM = 5.0;          // planning/steering standoff from zone boundaries
  flt64_t complianceHysteresisM = 2.0;  // clearance needed to count as recovered (anti-flap)
  flt64_t elevationMarginM = 2.0;       // pad on the vertical envelope used for band gating
  int32_t ellipseSegments = 32;         // vertices of the conservative ellipse polygon
};

//! \brief Tangent-bug style vector-mode avoidance tuning.
struct VectorAvoidanceConfig {
  flt64_t lookaheadRhoFactor = 1.5;  // lookahead >= factor * turn radius
  flt64_t lookaheadSpeedS = 2.0;     // plus this many seconds at current speed
  flt64_t exitClearFactor = 1.3;     // leave boundary-follow when clear to factor * lookahead
  int32_t exitClearTicks = 10;       // ... for this many consecutive ticks
  flt64_t minFollowS = 2.0;          // minimum boundary-follow dwell (hysteresis)
};

//! \brief Zone-violation recovery maneuver tuning.
struct RecoveryConfig {
  flt64_t speedMps = 0.0;       // recovery transit speed; 0 = platform cruising speed
  flt64_t completeHoldS = 1.0;  // how long COMPLIANT must hold before recovery completes
};

//! \brief Safe Return Path settings. The CSV is anchored at the explicit origin given here (a
//! safety artifact must not float with the first GPS fix), matching mission_runner's convention.
struct SrpConfig {
  std::string csvPath;
  std::optional<flt64_t> originLatDeg;  // required when csvPath is set
  std::optional<flt64_t> originLonDeg;
  bool acceptCommandsAfterSrp = true;
  flt64_t holdRadiusM = 10.0;             // hold circle around the last SRP waypoint
  flt64_t repositionSpeedMps = 1.5;       // speed for drift-out repositioning
  std::optional<flt64_t> safeElevationM;  // depth to hold during the SRP (nullopt = per-waypoint)
};

struct SafeModeConfig {
  std::string strategy = "srp";  // "srp" (falls back to zero_speed_hold without a CSV) | "zero_speed_hold"
  SrpConfig srp;
};

//! \brief Violation-response policy: grace timing, debounce, and the safe-mode strategy.
struct SafetyConfig {
  flt64_t gracePeriodS = 10.0;        // 0 = instant safe mode on a confirmed violation
  std::optional<flt64_t> graceZoneS;  // per-class overrides of grace_period_s
  std::optional<flt64_t> graceSpeedS;
  std::optional<flt64_t> graceElevationS;
  int32_t violationConfirmTicks = 2;   // consecutive violating ticks before a violation is confirmed
  flt64_t clearHoldS = 2.0;            // how long compliant must hold before a violation clears
  bool exitOnAllClear = true;          // leave safe mode when violations clear (else strategy decides)
  int32_t stateReportPeriodMs = 1000;  // ConditionalStateReport publish period
  SafeModeConfig safeMode;
};

//! \brief Optional performance limits for one operating regime (surface or underwater).
struct CapabilityLimits {
  std::optional<flt64_t> maxForwardSpeedMps;
  std::optional<flt64_t> maxReverseSpeedMps;
  std::optional<flt64_t> cruisingSpeedMps;
  std::optional<flt64_t> maxTurnRateRps;  // radians/second
  std::optional<flt64_t> minSpeedInMediumMps;
  std::optional<flt64_t> maxDepthChangeRateMps;  // underwater only
};

//! \brief Physical platform specs -> UVPlatformSpecsReportType.
struct PlatformSpecsConfig {
  std::string name = "vehicle";
  flt64_t lengthAtWaterlineM = 0.0;
  flt64_t beamAtWaterlineM = 0.0;
  flt64_t draftM = 0.0;
  flt64_t forwardDistanceM = 0.0;
  flt64_t aftDistanceM = 0.0;
  flt64_t portDistanceM = 0.0;
  flt64_t starboardDistanceM = 0.0;
  flt64_t topDistanceM = 0.0;
  flt64_t bottomDistanceM = 0.0;
  flt64_t displacementMetricTon = 0.0;
  flt64_t weightLightMetricTon = 0.0;
  flt64_t weightLoadedMetricTon = 0.0;
};

//! \brief Performance capabilities -> UVPlatformCapabilitiesReportType + planner params.
struct PlatformCapabilitiesConfig {
  flt64_t minWaterDepthM = 0.0;
  CapabilityLimits surface;
  bool underwaterEnabled = false;
  CapabilityLimits underwater;
};

//! \brief Simulated-vehicle strategy configuration (vehicle_control.sim in the YAML). The sim
//! integrates the platform kinematics from the capability limits at cycle_rate_hz and
//! publishes the three SA navigation reports.
struct SimVehicleConfig {
  flt64_t cycleRateHz = 20.0;
  flt64_t initialLatitudeDeg = 39.0;
  flt64_t initialLongitudeDeg = -76.5;
  flt64_t initialHeadingRad = 0.0;
  flt64_t accelMps2 = 1.0;       // surge acceleration/deceleration limit
  flt64_t floorDepthM = 60.0;    // sea-floor depth below the surface (for depth/ASF simulation)
  flt64_t currentEastMps = 0.0;  // uniform water current (drift) for exercising the XTE integral
  flt64_t currentNorthMps = 0.0;
};

//! \brief Top-level configuration produced by YamlConfigLoader and consumed by
//! AutopilotApp::initialize().
struct AutopilotConfig {
  DdsConfig dds;
  IdentityConfig identity;
  OperationalModeConfig operationalMode;
  ArbitrationConfig arbitration;
  LoopConfig loop;
  VectorToleranceConfig vectorTolerances;
  WaypointToleranceConfig waypointTolerances;
  PlannerConfig planner;
  ConstraintsConfig constraints;
  ZonesConfig zones;
  VectorAvoidanceConfig vectorAvoidance;
  RecoveryConfig recovery;
  SafetyConfig safety;
  std::string vehicleControlType = "sim";
  SimVehicleConfig simVehicle;
  PlatformSpecsConfig platformSpecs;
  PlatformCapabilitiesConfig platformCapabilities;
  ConsoleConfig console;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_CONFIG_AUTOPILOTCONFIG_HPP_
