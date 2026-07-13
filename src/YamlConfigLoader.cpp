//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
//
// Applied Research Laboratory
// Pennsylvania State University
// P.O. Box 30
// State College, PA 16804-0030
//
// DISTRIBUTION STATEMENT A. Approved for public release.
// Distribution is unlimited.
// This software was developed by the Department of the Navy,
// NAVSEA Unmanned and Small Combatants. It is provided under the terms of
// use found in the LICENSE file at the source code root directory.
//
//---------------------------------------------------------------------------

#include "YamlConfigLoader.h"

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>

#include "Logger.h"

namespace arlcore::autopilot {

namespace {

//! \brief Read a scalar from node[key] into *out if present and non-null.
template <class T>
void readScalar(const YAML::Node& node, const char* key, T* out) {
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
void readOptional(const YAML::Node& node, const char* key, std::optional<T>* out) {
  if (!node) {
    return;
  }
  const YAML::Node child = node[key];
  if (child.IsDefined() && !child.IsNull()) {
    *out = child.as<T>();
  }
}

void readCapabilityLimits(const YAML::Node& node, CapabilityLimits* out) {
  readOptional(node, "max_forward_speed_mps", &out->maxForwardSpeedMps);
  readOptional(node, "max_reverse_speed_mps", &out->maxReverseSpeedMps);
  readOptional(node, "cruising_speed_mps", &out->cruisingSpeedMps);
  readOptional(node, "max_turn_rate_rps", &out->maxTurnRateRps);
  readOptional(node, "min_speed_in_medium_mps", &out->minSpeedInMediumMps);
  readOptional(node, "max_depth_change_rate_mps", &out->maxDepthChangeRateMps);
}

}  // namespace

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
  readScalar(identity, "vector_source_id", &out->identity.vectorSourceId);
  readScalar(identity, "waypoint_source_id", &out->identity.waypointSourceId);
  readScalar(identity, "specs_source_id", &out->identity.specsSourceId);
  readScalar(identity, "capabilities_source_id", &out->identity.capabilitiesSourceId);
  readScalar(identity, "nav_source_id", &out->identity.navSourceId);

  const YAML::Node arb = root["arbitration"];
  readScalar(arb, "vector_priority", &out->arbitration.vectorPriority);
  readScalar(arb, "waypoint_priority", &out->arbitration.waypointPriority);

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
  } catch (const YAML::Exception& ex) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Invalid value in autopilot config '" << path << "': " << ex.what())
    return false;
  }

  return true;
}

}  // namespace arlcore::autopilot
