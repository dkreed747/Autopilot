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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PLANNERPARAMSFACTORY_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PLANNERPARAMSFACTORY_H_

#include <algorithm>
#include <optional>

#include "AutopilotConfig.h"
#include "DubinsPathPlanner.h"

namespace arlcore::autopilot {

//! \brief Derive the planner parameters from configuration. The platform capabilities drive
//! the planner: the planned turn radius is the kinematic minimum (representative speed / max
//! turn rate) inflated by planner.turn_radius_margin so the tracker retains turn authority to
//! close tracking error mid-maneuver, and the underwater depth-rate limit feeds the
//! spiral-descent approach budget. Callers must validate the capabilities first (see
//! AutopilotApp::initialize); this falls back to a conservative 25 m radius if they are absent.
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
  const std::optional<double> speed = surf.cruisingSpeedMps.has_value() ? surf.cruisingSpeedMps
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
  return p;
}

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PLANNERPARAMSFACTORY_H_
