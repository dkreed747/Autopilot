#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTCONFIG_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTCONFIG_H_

#include <cstdint>
#include <optional>
#include <string>

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
  std::string navSourceId;  // source for the sim vehicle's SA navigation reports
  std::string constraintsSourceId;  // MM conditional/constraint services source
  std::string operationalModeControlSourceId;  // MM operational mode control provider (empty = disabled)
  std::string operationalModeStatusSourceId;   // MM operational mode status report source
};

//! \brief Operational-mode FSM policy (MM OperationalModeControl/Status services).
struct OperationalModeConfig {
  bool allowImplicitModeTransitions = true;  // incoming driving commands may move
                                             // STANDBY->REMOTE/AUTONOMOUS and AUTONOMOUS->REMOTE
  bool commandsOutOfModeAreFailed = true;    // true = UMAA-strict fail; false = hold at ISSUED
                                             // until the mode becomes compatible
  double idleRevertS = 5.0;  // implicitly-entered REMOTE/AUTONOMOUS revert to STANDBY after
                             // this long with no active command of the mode's class
};

//! \brief Driving-resource arbitration priorities for one command class. Higher wins.
struct ClassArbitrationPriorities {
  int vectorPriority = 100;
  int waypointPriority = 10;
};

//! \brief Driving-resource arbitration priorities. Defaults keep safe-mode maneuvers above
//! everything and any remote (off-board operator) command above any local (onboard autonomy)
//! command.
struct ArbitrationConfig {
  ClassArbitrationPriorities local;
  ClassArbitrationPriorities remote{500, 400};
  int safePriority = 1000;
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
  int controlPeriodMs = 50;
  int navStalenessTimeoutMs = 2000;
};

//! \brief Default tolerances applied to a vector command when it omits them.
struct VectorToleranceConfig {
  double directionRad = 0.0873;
  double speedMps = 0.25;
  double elevationM = 1.0;
  bool hard = false;           // if true, persistent violation fails the command
  double failureDelayS = 5.0;  // how long a violation must persist before failing (hard only)
};

//! \brief Default capture tolerances applied to a waypoint when it omits them.
struct WaypointToleranceConfig {
  double positionM = 2.5;
  double yawRad = 0.1745;
  double elevationM = 1.0;
};

//! \brief RRT* fallback planner settings (used only when the direct Dubins leg clips a zone).
struct RrtConfig {
  uint32_t seed = 12345;       // deterministic sampling; salted per waypoint index
  int maxIterations = 2000;
  int timeBudgetMs = 150;
  double goalBias = 0.10;
  int nearK = 8;
  double edgeCheckStepM = 4.0;   // coarse in-tree edge sampling
  double finalCheckStepM = 1.0;  // fine recheck of the accepted path
};

struct PlannerConfig {
  double leadDistanceM = 50.0;
  double turnRadiusMargin = 1.25;  // planned radius = margin * (speed / max turn rate)
  int maxListWaitCycles = 200;
  int maxMissesPerWaypoint = 3;
  bool elevationCountsAsMiss = true;
  int maxReplans = 10;
  RrtConfig rrt;
};

//! \brief Static clamp settings for the autopilot's own constraint limits (merged with dynamic
//! active constraints and platform capabilities; the most restrictive value wins).
struct ConstraintsConfig {
  std::optional<double> maxSpeedMps;
  std::optional<double> minSpeedMps;
  std::optional<double> maxDepthM;  // deepest commanded depth allowed
  std::optional<double> minDepthM;  // shallowest commanded depth allowed
};

//! \brief Water-zone geometry margins and conversion settings.
struct ZonesConfig {
  double safetyMarginM = 5.0;          // planning/steering standoff from zone boundaries
  double complianceHysteresisM = 2.0;  // clearance needed to count as recovered (anti-flap)
  double elevationMarginM = 2.0;       // pad on the vertical envelope used for band gating
  int ellipseSegments = 32;            // vertices of the conservative ellipse polygon
};

//! \brief Tangent-bug style vector-mode avoidance tuning.
struct VectorAvoidanceConfig {
  double lookaheadRhoFactor = 1.5;  // lookahead >= factor * turn radius
  double lookaheadSpeedS = 2.0;     // plus this many seconds at current speed
  double exitClearFactor = 1.3;     // leave boundary-follow when clear to factor * lookahead
  int exitClearTicks = 10;          // ... for this many consecutive ticks
  double minFollowS = 2.0;          // minimum boundary-follow dwell (hysteresis)
};

//! \brief Zone-violation recovery maneuver tuning.
struct RecoveryConfig {
  double speedMps = 0.0;      // recovery transit speed; 0 = platform cruising speed
  double completeHoldS = 1.0;  // how long COMPLIANT must hold before recovery completes
};

//! \brief Safe Return Path settings. The CSV is anchored at the explicit origin given here (a
//! safety artifact must not float with the first GPS fix), matching mission_runner's convention.
struct SrpConfig {
  std::string csvPath;
  std::optional<double> originLatDeg;  // required when csvPath is set
  std::optional<double> originLonDeg;
  bool acceptCommandsAfterSrp = true;
  double holdRadiusM = 10.0;           // hold circle around the last SRP waypoint
  double repositionSpeedMps = 1.5;     // speed for drift-out repositioning
  std::optional<double> safeElevationM;  // depth to hold during the SRP (nullopt = per-waypoint)
};

struct SafeModeConfig {
  std::string strategy = "srp";  // "srp" (falls back to zero_speed_hold without a CSV) | "zero_speed_hold"
  SrpConfig srp;
};

//! \brief Violation-response policy: grace timing, debounce, and the safe-mode strategy.
struct SafetyConfig {
  double gracePeriodS = 10.0;   // 0 = instant safe mode on a confirmed violation
  std::optional<double> graceZoneS;       // per-class overrides of grace_period_s
  std::optional<double> graceSpeedS;
  std::optional<double> graceElevationS;
  int violationConfirmTicks = 2;   // consecutive violating ticks before a violation is confirmed
  double clearHoldS = 2.0;         // how long compliant must hold before a violation clears
  bool exitOnAllClear = true;      // leave safe mode when violations clear (else strategy decides)
  int stateReportPeriodMs = 1000;  // ConditionalStateReport publish period
  SafeModeConfig safeMode;
};

//! \brief Optional performance limits for one operating regime (surface or underwater).
struct CapabilityLimits {
  std::optional<double> maxForwardSpeedMps;
  std::optional<double> maxReverseSpeedMps;
  std::optional<double> cruisingSpeedMps;
  std::optional<double> maxTurnRateRps;        // radians/second
  std::optional<double> minSpeedInMediumMps;
  std::optional<double> maxDepthChangeRateMps;  // underwater only
};

//! \brief Physical platform specs -> UVPlatformSpecsReportType.
struct PlatformSpecsConfig {
  std::string name = "vehicle";
  double lengthAtWaterlineM = 0.0;
  double beamAtWaterlineM = 0.0;
  double draftM = 0.0;
  double forwardDistanceM = 0.0;
  double aftDistanceM = 0.0;
  double portDistanceM = 0.0;
  double starboardDistanceM = 0.0;
  double topDistanceM = 0.0;
  double bottomDistanceM = 0.0;
  double displacementMetricTon = 0.0;
  double weightLightMetricTon = 0.0;
  double weightLoadedMetricTon = 0.0;
};

//! \brief Performance capabilities -> UVPlatformCapabilitiesReportType + planner params.
struct PlatformCapabilitiesConfig {
  double minWaterDepthM = 0.0;
  CapabilityLimits surface;
  bool underwaterEnabled = false;
  CapabilityLimits underwater;
};

//! \brief Simulated-vehicle strategy configuration (vehicle_control.sim in the YAML). The sim
//! integrates the platform kinematics from the capability limits at cycle_rate_hz and
//! publishes the three SA navigation reports.
struct SimVehicleConfig {
  double cycleRateHz = 20.0;
  double initialLatitudeDeg = 39.0;
  double initialLongitudeDeg = -76.5;
  double initialHeadingRad = 0.0;
  double accelMps2 = 1.0;    // surge acceleration/deceleration limit
  double floorDepthM = 60.0;  // sea-floor depth below the surface (for depth/ASF simulation)
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
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTCONFIG_H_
