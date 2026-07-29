#ifndef AUTOPILOT_GUIDANCE_PLANNERPARAMSFACTORY_HPP_
#define AUTOPILOT_GUIDANCE_PLANNERPARAMSFACTORY_HPP_

#include <algorithm>
#include <optional>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/guidance/DubinsPathPlanner.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief Derive the planner parameters from configuration and the platform capabilities.
//! The planned turn radius is the kinematic minimum (representative speed / max turn rate)
//! inflated by planner.turn_radius_margin so the tracker retains turn authority mid-maneuver.
//! Callers must validate the capabilities first (see AutopilotApp::initialize); absent
//! capabilities fall back to a conservative 25 m radius.
inline PlannerParams derivePlannerParams(const AutopilotConfig& config) {
  PlannerParams p;
  p.leadDistanceM = config.planner.leadDistanceM;
  p.posCaptureM = config.waypointTolerances.positionM;
  p.yawCaptureRad = config.waypointTolerances.yawRad;
  p.elevCaptureM = config.waypointTolerances.elevationM;
  p.maxMissesPerWaypoint = config.planner.maxMissesPerWaypoint;
  p.elevationCountsAsMiss = config.planner.elevationCountsAsMiss;
  p.maxReplans = config.planner.maxReplans;

  const CapabilityLimits& surf = config.platformCapabilities.surface;
  const std::optional<flt64_t> speed = surf.cruisingSpeedMps.has_value() ? surf.cruisingSpeedMps
                                                                        : surf.maxForwardSpeedMps;
  if (speed.has_value() && surf.maxTurnRateRps.has_value() && surf.maxTurnRateRps.value() > 0.0) {
    p.turnRadiusM = std::max(1.0, config.planner.turnRadiusMargin) * speed.value() /
                    surf.maxTurnRateRps.value();
  } else {
    p.turnRadiusM = 25.0;
  }

  if (config.platformCapabilities.underwaterEnabled &&
      config.platformCapabilities.underwater.maxDepthChangeRateMps.has_value()) {
    p.maxDepthRateMps = config.platformCapabilities.underwater.maxDepthChangeRateMps.value();
  }

  p.zoneMarginM = config.zones.safetyMarginM;
  p.rrt.seed = config.planner.rrt.seed;
  p.rrt.maxIterations = config.planner.rrt.maxIterations;
  p.rrt.timeBudgetMs = config.planner.rrt.timeBudgetMs;
  p.rrt.goalBias = config.planner.rrt.goalBias;
  p.rrt.nearK = config.planner.rrt.nearK;
  p.rrt.edgeCheckStepM = config.planner.rrt.edgeCheckStepM;
  p.rrt.finalCheckStepM = config.planner.rrt.finalCheckStepM;
  return p;
}

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_PLANNERPARAMSFACTORY_HPP_
