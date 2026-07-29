#include "autopilot/config/ConfigValidation.hpp"

#include <fstream>
#include <regex>
#include <sstream>

namespace arlcore::autopilot {

bool isValidUuid(const std::string& uuid) {
  static const std::regex kUuidPattern("^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$");
  return std::regex_match(uuid, kUuidPattern);
}

namespace detail {

class Collector {
 public:
  Collector(std::vector<std::string>* errors, std::vector<std::string>* warnings)
      : errors_(errors), warnings_(warnings) {}

  template <class T>
  void error(const char* key, const T& value, const char* rule) {
    std::ostringstream oss;
    oss << key << " " << rule << " (got " << value << ")";
    errors_->push_back(oss.str());
  }
  void error(const std::string& message) { errors_->push_back(message); }
  void warn(const std::string& message) { warnings_->push_back(message); }

 private:
  std::vector<std::string>* errors_;
  std::vector<std::string>* warnings_;
};

static void requireUuidIfSet(Collector* c, const char* key, const std::string& value) {
  if (!value.empty() && !isValidUuid(value)) {
    c->error(std::string(key) + " is not a valid UUID: '" + value + "'");
  }
}

static void requirePositiveIfSet(Collector* c, const char* key, const std::optional<flt64_t>& value) {
  if (value.has_value() && value.value() <= 0.0) {
    c->error(key, value.value(), "must be > 0 when set");
  }
}

}  // namespace detail

bool validateConfig(const AutopilotConfig& config, std::vector<std::string>* errors,
                    std::vector<std::string>* warnings) {
  detail::Collector c(errors, warnings);

  if (config.dds.domainId < 0 || config.dds.domainId > 232) {
    c.error("dds.domain_id", config.dds.domainId, "must be in [0, 232]");
  }
  if (config.dds.domainQosProfile.empty() || config.dds.largeCollectionsQosProfile.empty()) {
    c.warn("dds QoS profile names are empty; the SDK will fall back to default QoS");
  }
  if (!std::ifstream(config.dds.qosFile).good()) {
    c.warn("dds.qos_file '" + config.dds.qosFile +
           "' is not readable from the working directory; DDS falls back to default QoS "
           "(drops RELIABLE/TRANSIENT_LOCAL - large route/constraint delivery will misbehave)");
  }

  detail::requireUuidIfSet(&c, "identity.platform_id", config.identity.platformId);
  detail::requireUuidIfSet(&c, "identity.vector_source_id", config.identity.vectorSourceId);
  detail::requireUuidIfSet(&c, "identity.waypoint_source_id", config.identity.waypointSourceId);
  detail::requireUuidIfSet(&c, "identity.specs_source_id", config.identity.specsSourceId);
  detail::requireUuidIfSet(&c, "identity.capabilities_source_id", config.identity.capabilitiesSourceId);
  detail::requireUuidIfSet(&c, "identity.nav_source_id", config.identity.navSourceId);
  detail::requireUuidIfSet(&c, "identity.constraints_source_id", config.identity.constraintsSourceId);
  detail::requireUuidIfSet(&c, "identity.operational_mode_control_source_id",
                           config.identity.operationalModeControlSourceId);
  detail::requireUuidIfSet(&c, "identity.operational_mode_status_source_id",
                           config.identity.operationalModeStatusSourceId);
  detail::requireUuidIfSet(&c, "console.platform_id", config.console.platformId);
  detail::requireUuidIfSet(&c, "console.source_id", config.console.sourceId);
  if (!config.console.platformId.empty() && config.console.platformId == config.identity.platformId) {
    c.warn(
        "console.platform_id equals identity.platform_id: console commands will be "
        "classified LOCAL (onboard autonomy), not REMOTE operator");
  }

  if (config.operationalMode.idleRevertS < 0.0) {
    c.error("operational_mode.idle_revert_s", config.operationalMode.idleRevertS, "must be >= 0");
  }

  const ArbitrationConfig& arb = config.arbitration;
  if (arb.local.vectorPriority < 0 || arb.local.waypointPriority < 0 || arb.remote.vectorPriority < 0 ||
      arb.remote.waypointPriority < 0 || arb.safePriority < 0) {
    c.error("arbitration priorities must all be >= 0");
  }
  const int32_t maxCommandPriority = std::max(std::max(arb.local.vectorPriority, arb.local.waypointPriority),
                                              std::max(arb.remote.vectorPriority, arb.remote.waypointPriority));
  if (arb.safePriority <= maxCommandPriority) {
    c.error("arbitration.safe_priority", arb.safePriority,
            "must exceed every local/remote priority (safe-mode maneuvers must win)");
  }
  if (arb.remote.vectorPriority <= arb.local.vectorPriority ||
      arb.remote.waypointPriority <= arb.local.waypointPriority) {
    c.warn(
        "arbitration: remote priorities do not exceed local ones; a remote operator "
        "will not preempt onboard autonomy");
  }

  if (config.loop.controlPeriodMs <= 0) {
    c.error("loop.control_period_ms", config.loop.controlPeriodMs, "must be > 0");
  } else {
    if (config.loop.controlPeriodMs > 1000) {
      c.warn("loop.control_period_ms is over 1000 ms; guidance and safety react sluggishly");
    }
    if (config.loop.navStalenessTimeoutMs > 0 && config.loop.navStalenessTimeoutMs < 2 * config.loop.controlPeriodMs) {
      c.warn(
          "loop.nav_staleness_timeout_ms is less than two control periods; expect "
          "spurious zero-speed holds");
    }
  }
  if (config.loop.navStalenessTimeoutMs <= 0) {
    c.error("loop.nav_staleness_timeout_ms", config.loop.navStalenessTimeoutMs, "must be > 0");
  }

  const VectorToleranceConfig& vt = config.vectorTolerances;
  if (vt.directionRad < 0.0 || vt.speedMps < 0.0 || vt.elevationM < 0.0) {
    c.error("tolerances.vector values must be >= 0");
  }
  if (vt.failureDelayS < 0.0) {
    c.error("tolerances.vector.failure_delay_s", vt.failureDelayS, "must be >= 0");
  }
  const WaypointToleranceConfig& wt = config.waypointTolerances;
  if (wt.positionM < 0.0 || wt.yawRad < 0.0 || wt.elevationM < 0.0) {
    c.error("tolerances.waypoint_defaults values must be >= 0");
  } else if (wt.positionM == 0.0) {
    c.warn(
        "tolerances.waypoint_defaults.position_m is 0: waypoints without their own "
        "tolerance can never be captured");
  }

  const PlannerConfig& p = config.planner;
  if (p.leadDistanceM <= 0.0) {
    c.error("planner.lead_distance_m", p.leadDistanceM, "must be > 0");
  }
  if (p.turnRadiusMargin <= 0.0) {
    c.error("planner.turn_radius_margin", p.turnRadiusMargin, "must be > 0");
  } else if (p.turnRadiusMargin < 1.0) {
    c.warn(
        "planner.turn_radius_margin below 1.0 plans turns tighter than the platform's "
        "stated capability");
  }
  if (p.maxListWaitCycles <= 0) {
    c.error("planner.max_list_wait_cycles", p.maxListWaitCycles, "must be > 0");
  }
  if (p.maxMissesPerWaypoint < 0) {
    c.error("planner.max_misses_per_waypoint", p.maxMissesPerWaypoint, "must be >= 0");
  }
  if (p.maxReplans < 0) {
    c.error("planner.max_replans", p.maxReplans, "must be >= 0");
  }
  const RrtConfig& rrt = p.rrt;
  if (rrt.maxIterations <= 0 || rrt.nearK <= 0) {
    c.error("planner.rrt.max_iterations/near_k must be > 0");
  }
  if (rrt.timeBudgetMs <= 0) {
    c.error("planner.rrt.time_budget_ms", rrt.timeBudgetMs, "must be > 0");
  } else if (config.loop.controlPeriodMs > 0 && rrt.timeBudgetMs > 4 * config.loop.controlPeriodMs) {
    c.warn(
        "planner.rrt.time_budget_ms far exceeds the control period; a mid-route replan "
        "can stall control ticks");
  }
  if (rrt.goalBias < 0.0 || rrt.goalBias > 1.0) {
    c.error("planner.rrt.goal_bias", rrt.goalBias, "must be in [0, 1]");
  }
  if (rrt.edgeCheckStepM <= 0.0 || rrt.finalCheckStepM <= 0.0) {
    c.error("planner.rrt edge/final check steps must be > 0");
  }

  const ConstraintsConfig& lim = config.constraints;
  detail::requirePositiveIfSet(&c, "constraints.max_speed_mps", lim.maxSpeedMps);
  if (lim.minSpeedMps.has_value() && lim.minSpeedMps.value() < 0.0) {
    c.error("constraints.min_speed_mps", lim.minSpeedMps.value(), "must be >= 0");
  }
  if (lim.minSpeedMps.has_value() && lim.maxSpeedMps.has_value() && lim.minSpeedMps.value() > lim.maxSpeedMps.value()) {
    c.error("constraints.min_speed_mps exceeds constraints.max_speed_mps");
  }
  if (lim.minDepthM.has_value() && lim.maxDepthM.has_value() && lim.minDepthM.value() > lim.maxDepthM.value()) {
    c.error("constraints.min_depth_m exceeds constraints.max_depth_m");
  }

  const ZonesConfig& z = config.zones;
  if (z.ellipseSegments < 3) {
    c.error("zones.ellipse_segments", z.ellipseSegments, "must be >= 3");
  } else if (z.ellipseSegments > 256) {
    c.warn("zones.ellipse_segments above 256 slows every zone query for no accuracy gain");
  }
  if (z.safetyMarginM < 0.0 || z.complianceHysteresisM < 0.0 || z.elevationMarginM < 0.0) {
    c.error("zones margins/hysteresis must be >= 0");
  }

  const VectorAvoidanceConfig& va = config.vectorAvoidance;
  if (va.lookaheadRhoFactor <= 0.0 || va.exitClearFactor <= 0.0) {
    c.error("vector_avoidance factors must be > 0");
  }
  if (va.lookaheadSpeedS < 0.0 || va.minFollowS < 0.0) {
    c.error("vector_avoidance times must be >= 0");
  }
  if (va.exitClearTicks <= 0) {
    c.error("vector_avoidance.exit_clear_ticks", va.exitClearTicks, "must be > 0");
  }

  if (config.recovery.speedMps < 0.0) {
    c.error("recovery.speed_mps", config.recovery.speedMps, "must be >= 0 (0 = cruising speed)");
  }
  if (config.recovery.completeHoldS < 0.0) {
    c.error("recovery.complete_hold_s", config.recovery.completeHoldS, "must be >= 0");
  }

  const SafetyConfig& s = config.safety;
  if (s.gracePeriodS < 0.0 || (s.graceZoneS.has_value() && s.graceZoneS.value() < 0.0) ||
      (s.graceSpeedS.has_value() && s.graceSpeedS.value() < 0.0) ||
      (s.graceElevationS.has_value() && s.graceElevationS.value() < 0.0)) {
    c.error("safety grace periods must be >= 0");
  }
  if (s.violationConfirmTicks < 1) {
    c.error("safety.violation_confirm_ticks", s.violationConfirmTicks, "must be >= 1");
  }
  if (s.clearHoldS < 0.0) {
    c.error("safety.clear_hold_s", s.clearHoldS, "must be >= 0");
  }
  if (s.stateReportPeriodMs <= 0) {
    c.error("safety.state_report_period_ms", s.stateReportPeriodMs, "must be > 0");
  }
  const SafeModeConfig& sm = s.safeMode;
  if (sm.strategy != "srp" && sm.strategy != "zero_speed_hold") {
    c.error("safety.safe_mode.strategy must be 'srp' or 'zero_speed_hold' (got '" + sm.strategy + "')");
  }
  if (!sm.srp.csvPath.empty()) {
    if (!sm.srp.originLatDeg.has_value() || !sm.srp.originLonDeg.has_value()) {
      c.error(
          "safety.safe_mode.srp.origin_lat_deg/origin_lon_deg are required when csv_path is set "
          "(a safety artifact must not float with the first GPS fix)");
    } else if (sm.srp.originLatDeg.value() < -90.0 || sm.srp.originLatDeg.value() > 90.0 ||
               sm.srp.originLonDeg.value() < -180.0 || sm.srp.originLonDeg.value() > 180.0) {
      c.error("safety.safe_mode.srp origin is outside [-90,90]/[-180,180]");
    }
  } else if (sm.strategy == "srp") {
    c.warn("safety.safe_mode.strategy is 'srp' with no csv_path: falls back to zero-speed hold");
  }
  if (sm.srp.holdRadiusM <= 0.0) {
    c.error("safety.safe_mode.srp.hold_radius_m", sm.srp.holdRadiusM, "must be > 0");
  }
  if (sm.srp.repositionSpeedMps <= 0.0) {
    c.error("safety.safe_mode.srp.reposition_speed_mps", sm.srp.repositionSpeedMps, "must be > 0");
  }

  if (config.vehicleControlType != "sim") {
    c.warn("vehicle_control.type '" + config.vehicleControlType + "' is unknown; the app defaults to sim");
  }
  const SimVehicleConfig& sim = config.simVehicle;
  if (sim.cycleRateHz <= 0.0) {
    c.error("vehicle_control.sim.cycle_rate_hz", sim.cycleRateHz, "must be > 0");
  } else if (sim.cycleRateHz > 100.0) {
    c.warn("vehicle_control.sim.cycle_rate_hz above 100 Hz burns CPU without control benefit");
  }
  if (sim.accelMps2 <= 0.0) {
    c.error("vehicle_control.sim.accel_mps2", sim.accelMps2, "must be > 0");
  }
  if (sim.floorDepthM <= 0.0) {
    c.error("vehicle_control.sim.floor_depth_m", sim.floorDepthM, "must be > 0");
  }
  if (sim.initialLatitudeDeg < -90.0 || sim.initialLatitudeDeg > 90.0 || sim.initialLongitudeDeg < -180.0 ||
      sim.initialLongitudeDeg > 180.0) {
    c.error("vehicle_control.sim initial position is outside [-90,90]/[-180,180]");
  }

  const PlatformSpecsConfig& specs = config.platformSpecs;
  if (specs.lengthAtWaterlineM < 0.0 || specs.beamAtWaterlineM < 0.0 || specs.draftM < 0.0 ||
      specs.forwardDistanceM < 0.0 || specs.aftDistanceM < 0.0 || specs.portDistanceM < 0.0 ||
      specs.starboardDistanceM < 0.0 || specs.topDistanceM < 0.0 || specs.bottomDistanceM < 0.0 ||
      specs.displacementMetricTon < 0.0 || specs.weightLightMetricTon < 0.0 || specs.weightLoadedMetricTon < 0.0) {
    c.error("platform_specs dimensions/weights must be >= 0");
  }

  const PlatformCapabilitiesConfig& caps = config.platformCapabilities;
  if (caps.minWaterDepthM < 0.0) {
    c.error("platform_capabilities.min_water_depth_m", caps.minWaterDepthM, "must be >= 0");
  }
  for (const auto& [name, limits] : {std::pair<const char*, const CapabilityLimits*>{"surface", &caps.surface},
                                     std::pair<const char*, const CapabilityLimits*>{"underwater", &caps.underwater}}) {
    const std::string prefix = std::string("platform_capabilities.") + name;
    detail::requirePositiveIfSet(&c, (prefix + ".max_forward_speed_mps").c_str(), limits->maxForwardSpeedMps);
    detail::requirePositiveIfSet(&c, (prefix + ".max_reverse_speed_mps").c_str(), limits->maxReverseSpeedMps);
    detail::requirePositiveIfSet(&c, (prefix + ".cruising_speed_mps").c_str(), limits->cruisingSpeedMps);
    detail::requirePositiveIfSet(&c, (prefix + ".max_turn_rate_rps").c_str(), limits->maxTurnRateRps);
    detail::requirePositiveIfSet(&c, (prefix + ".max_depth_change_rate_mps").c_str(), limits->maxDepthChangeRateMps);
    if (limits->minSpeedInMediumMps.has_value() && limits->minSpeedInMediumMps.value() < 0.0) {
      c.error((prefix + ".min_speed_in_medium_mps").c_str(), limits->minSpeedInMediumMps.value(), "must be >= 0");
    }
  }
  if (caps.underwaterEnabled && !caps.underwater.maxDepthChangeRateMps.has_value()) {
    c.warn(
        "platform_capabilities.underwater enabled without max_depth_change_rate_mps; "
        "depth-rate budgeting uses its default");
  }

  return errors->empty();
}

}  // namespace arlcore::autopilot
