#include "autopilot/config/YamlConfigLoader.hpp"

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>

#include "Logger.h"

namespace arlcore::autopilot {

//! \brief Read a scalar from node[key] into *out if present and non-null.
template <class T>
static void readScalar(const YAML::Node& node, const char* key, T* out) {
  if (!node) {
    return;
  }
  const YAML::Node child = node[key];
  if (child.IsDefined() && !child.IsNull()) {
    *out = child.as<T>();
  }
}

//! \brief Read a scalar into an std::optional if present.
template <class T>
static void readOptional(const YAML::Node& node, const char* key, std::optional<T>* out) {
  if (!node) {
    return;
  }
  const YAML::Node child = node[key];
  if (child.IsDefined() && !child.IsNull()) {
    *out = child.as<T>();
  }
}

static void readCapabilityLimits(const YAML::Node& node, CapabilityLimits* out) {
  readOptional(node, "max_forward_speed_mps", &out->maxForwardSpeedMps);
  readOptional(node, "max_reverse_speed_mps", &out->maxReverseSpeedMps);
  readOptional(node, "cruising_speed_mps", &out->cruisingSpeedMps);
  readOptional(node, "max_turn_rate_rps", &out->maxTurnRateRps);
  readOptional(node, "min_speed_in_medium_mps", &out->minSpeedInMediumMps);
  readOptional(node, "max_depth_change_rate_mps", &out->maxDepthChangeRateMps);
}

bool YamlConfigLoader::load(const std::string& path, AutopilotConfig* out) {
  if (out == nullptr) {
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& ex) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Failed to load autopilot config '" << path << "': " << ex.what())
    return false;
  }

  // Field parsing throws YAML::TypedBadConversion on type-mismatched values; contain it so a
  // bad config reports an error instead of aborting the process.
  try {
    const YAML::Node dds = root["dds"];
    readScalar(dds, "domain_id", &out->dds.domainId);
    readScalar(dds, "qos_file", &out->dds.qosFile);
    readScalar(dds, "domain_qos_profile", &out->dds.domainQosProfile);
    readScalar(dds, "large_collections_qos_profile", &out->dds.largeCollectionsQosProfile);

    const YAML::Node identity = root["identity"];
    readScalar(identity, "platform_id", &out->identity.platformId);
    readScalar(identity, "vector_source_id", &out->identity.vectorSourceId);
    readScalar(identity, "waypoint_source_id", &out->identity.waypointSourceId);
    readScalar(identity, "specs_source_id", &out->identity.specsSourceId);
    readScalar(identity, "capabilities_source_id", &out->identity.capabilitiesSourceId);
    readScalar(identity, "nav_source_id", &out->identity.navSourceId);
    readScalar(identity, "constraints_source_id", &out->identity.constraintsSourceId);
    readScalar(identity, "operational_mode_control_source_id", &out->identity.operationalModeControlSourceId);
    readScalar(identity, "operational_mode_status_source_id", &out->identity.operationalModeStatusSourceId);

    const YAML::Node opMode = root["operational_mode"];
    readScalar(opMode, "allow_implicit_mode_transitions", &out->operationalMode.allowImplicitModeTransitions);
    readScalar(opMode, "commands_out_of_mode_are_failed", &out->operationalMode.commandsOutOfModeAreFailed);
    readScalar(opMode, "idle_revert_s", &out->operationalMode.idleRevertS);

    const YAML::Node arb = root["arbitration"];
    if (arb && (arb["vector_priority"] || arb["waypoint_priority"])) {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                    "arbitration.vector_priority/waypoint_priority are obsolete; "
                    "use arbitration.local / arbitration.remote blocks (defaults applied)")
    }
    if (arb) {
      const YAML::Node local = arb["local"];
      readScalar(local, "vector_priority", &out->arbitration.local.vectorPriority);
      readScalar(local, "waypoint_priority", &out->arbitration.local.waypointPriority);
      const YAML::Node remote = arb["remote"];
      readScalar(remote, "vector_priority", &out->arbitration.remote.vectorPriority);
      readScalar(remote, "waypoint_priority", &out->arbitration.remote.waypointPriority);
    }
    readScalar(arb, "safe_priority", &out->arbitration.safePriority);

    const YAML::Node loop = root["loop"];
    readScalar(loop, "control_period_ms", &out->loop.controlPeriodMs);
    readScalar(loop, "nav_staleness_timeout_ms", &out->loop.navStalenessTimeoutMs);

    const YAML::Node tol = root["tolerances"];
    if (tol) {
      const YAML::Node vec = tol["vector"];
      readScalar(vec, "direction_rad", &out->vectorTolerances.directionRad);
      readScalar(vec, "speed_mps", &out->vectorTolerances.speedMps);
      readScalar(vec, "elevation_m", &out->vectorTolerances.elevationM);
      readScalar(vec, "hard", &out->vectorTolerances.hard);
      readScalar(vec, "failure_delay_s", &out->vectorTolerances.failureDelayS);

      const YAML::Node wp = tol["waypoint_defaults"];
      readScalar(wp, "position_m", &out->waypointTolerances.positionM);
      readScalar(wp, "yaw_rad", &out->waypointTolerances.yawRad);
      readScalar(wp, "elevation_m", &out->waypointTolerances.elevationM);
    }

    const YAML::Node planner = root["planner"];
    readScalar(planner, "lead_distance_m", &out->planner.leadDistanceM);
    readScalar(planner, "turn_radius_margin", &out->planner.turnRadiusMargin);
    readScalar(planner, "max_list_wait_cycles", &out->planner.maxListWaitCycles);
    readScalar(planner, "max_misses_per_waypoint", &out->planner.maxMissesPerWaypoint);
    readScalar(planner, "elevation_counts_as_miss", &out->planner.elevationCountsAsMiss);
    readScalar(planner, "max_replans", &out->planner.maxReplans);
    if (planner) {
      const YAML::Node rrt = planner["rrt"];
      readScalar(rrt, "seed", &out->planner.rrt.seed);
      readScalar(rrt, "max_iterations", &out->planner.rrt.maxIterations);
      readScalar(rrt, "time_budget_ms", &out->planner.rrt.timeBudgetMs);
      readScalar(rrt, "goal_bias", &out->planner.rrt.goalBias);
      readScalar(rrt, "near_k", &out->planner.rrt.nearK);
      readScalar(rrt, "edge_check_step_m", &out->planner.rrt.edgeCheckStepM);
      readScalar(rrt, "final_check_step_m", &out->planner.rrt.finalCheckStepM);
    }

    const YAML::Node constraints = root["constraints"];
    readOptional(constraints, "max_speed_mps", &out->constraints.maxSpeedMps);
    readOptional(constraints, "min_speed_mps", &out->constraints.minSpeedMps);
    readOptional(constraints, "max_depth_m", &out->constraints.maxDepthM);
    readOptional(constraints, "min_depth_m", &out->constraints.minDepthM);

    const YAML::Node zones = root["zones"];
    readScalar(zones, "safety_margin_m", &out->zones.safetyMarginM);
    readScalar(zones, "compliance_hysteresis_m", &out->zones.complianceHysteresisM);
    readScalar(zones, "elevation_margin_m", &out->zones.elevationMarginM);
    readScalar(zones, "ellipse_segments", &out->zones.ellipseSegments);

    const YAML::Node avoidance = root["vector_avoidance"];
    readScalar(avoidance, "lookahead_rho_factor", &out->vectorAvoidance.lookaheadRhoFactor);
    readScalar(avoidance, "lookahead_speed_s", &out->vectorAvoidance.lookaheadSpeedS);
    readScalar(avoidance, "exit_clear_factor", &out->vectorAvoidance.exitClearFactor);
    readScalar(avoidance, "exit_clear_ticks", &out->vectorAvoidance.exitClearTicks);
    readScalar(avoidance, "min_follow_s", &out->vectorAvoidance.minFollowS);

    const YAML::Node recovery = root["recovery"];
    readScalar(recovery, "speed_mps", &out->recovery.speedMps);
    readScalar(recovery, "complete_hold_s", &out->recovery.completeHoldS);

    const YAML::Node safety = root["safety"];
    readScalar(safety, "grace_period_s", &out->safety.gracePeriodS);
    if (safety) {
      const YAML::Node overrides = safety["grace_overrides"];
      readOptional(overrides, "zone_s", &out->safety.graceZoneS);
      readOptional(overrides, "speed_s", &out->safety.graceSpeedS);
      readOptional(overrides, "elevation_s", &out->safety.graceElevationS);
    }
    readScalar(safety, "violation_confirm_ticks", &out->safety.violationConfirmTicks);
    readScalar(safety, "clear_hold_s", &out->safety.clearHoldS);
    readScalar(safety, "exit_on_all_clear", &out->safety.exitOnAllClear);
    readScalar(safety, "state_report_period_ms", &out->safety.stateReportPeriodMs);
    if (safety) {
      const YAML::Node safeMode = safety["safe_mode"];
      readScalar(safeMode, "strategy", &out->safety.safeMode.strategy);
      if (safeMode) {
        const YAML::Node srp = safeMode["srp"];
        readScalar(srp, "csv_path", &out->safety.safeMode.srp.csvPath);
        readOptional(srp, "origin_lat_deg", &out->safety.safeMode.srp.originLatDeg);
        readOptional(srp, "origin_lon_deg", &out->safety.safeMode.srp.originLonDeg);
        readScalar(srp, "accept_commands_after_srp", &out->safety.safeMode.srp.acceptCommandsAfterSrp);
        readScalar(srp, "hold_radius_m", &out->safety.safeMode.srp.holdRadiusM);
        readScalar(srp, "reposition_speed_mps", &out->safety.safeMode.srp.repositionSpeedMps);
        readOptional(srp, "safe_elevation_m", &out->safety.safeMode.srp.safeElevationM);
      }
    }

    const YAML::Node vc = root["vehicle_control"];
    readScalar(vc, "type", &out->vehicleControlType);
    if (vc) {
      const YAML::Node sim = vc["sim"];
      readScalar(sim, "cycle_rate_hz", &out->simVehicle.cycleRateHz);
      readScalar(sim, "initial_latitude_deg", &out->simVehicle.initialLatitudeDeg);
      readScalar(sim, "initial_longitude_deg", &out->simVehicle.initialLongitudeDeg);
      readScalar(sim, "initial_heading_rad", &out->simVehicle.initialHeadingRad);
      readScalar(sim, "accel_mps2", &out->simVehicle.accelMps2);
      readScalar(sim, "floor_depth_m", &out->simVehicle.floorDepthM);
    }

    const YAML::Node specs = root["platform_specs"];
    readScalar(specs, "name", &out->platformSpecs.name);
    readScalar(specs, "length_at_waterline_m", &out->platformSpecs.lengthAtWaterlineM);
    readScalar(specs, "beam_at_waterline_m", &out->platformSpecs.beamAtWaterlineM);
    readScalar(specs, "draft_m", &out->platformSpecs.draftM);
    readScalar(specs, "forward_distance_m", &out->platformSpecs.forwardDistanceM);
    readScalar(specs, "aft_distance_m", &out->platformSpecs.aftDistanceM);
    readScalar(specs, "port_distance_m", &out->platformSpecs.portDistanceM);
    readScalar(specs, "starboard_distance_m", &out->platformSpecs.starboardDistanceM);
    readScalar(specs, "top_distance_m", &out->platformSpecs.topDistanceM);
    readScalar(specs, "bottom_distance_m", &out->platformSpecs.bottomDistanceM);
    readScalar(specs, "displacement_metric_ton", &out->platformSpecs.displacementMetricTon);
    readScalar(specs, "weight_light_metric_ton", &out->platformSpecs.weightLightMetricTon);
    readScalar(specs, "weight_loaded_metric_ton", &out->platformSpecs.weightLoadedMetricTon);

    const YAML::Node caps = root["platform_capabilities"];
    if (caps) {
      readScalar(caps, "min_water_depth_m", &out->platformCapabilities.minWaterDepthM);
      readCapabilityLimits(caps["surface"], &out->platformCapabilities.surface);
      const YAML::Node uw = caps["underwater"];
      if (uw) {
        readScalar(uw, "enabled", &out->platformCapabilities.underwaterEnabled);
        readCapabilityLimits(uw, &out->platformCapabilities.underwater);
      }
    }

    const YAML::Node console = root["console"];
    readScalar(console, "platform_id", &out->console.platformId);
    readScalar(console, "source_id", &out->console.sourceId);
  } catch (const YAML::Exception& ex) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Invalid value in autopilot config '" << path << "': " << ex.what())
    return false;
  }

  return true;
}

}  // namespace arlcore::autopilot
