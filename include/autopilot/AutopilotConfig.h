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
  std::string vectorSourceId;
  std::string waypointSourceId;
  std::string specsSourceId;
  std::string capabilitiesSourceId;
  std::string navSourceId;  // source for the sim vehicle's SA navigation reports
};

//! \brief Driving-resource arbitration priorities. Higher wins.
struct ArbitrationConfig {
  int vectorPriority = 100;
  int waypointPriority = 10;
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

struct PlannerConfig {
  double leadDistanceM = 50.0;
  double turnRadiusMargin = 1.25;  // planned radius = margin * (speed / max turn rate)
  int maxListWaitCycles = 200;
  int maxMissesPerWaypoint = 3;
  bool elevationCountsAsMiss = true;
  int maxReplans = 10;
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
  ArbitrationConfig arbitration;
  LoopConfig loop;
  VectorToleranceConfig vectorTolerances;
  WaypointToleranceConfig waypointTolerances;
  PlannerConfig planner;
  std::string vehicleControlType = "sim";
  SimVehicleConfig simVehicle;
  PlatformSpecsConfig platformSpecs;
  PlatformCapabilitiesConfig platformCapabilities;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTCONFIG_H_
