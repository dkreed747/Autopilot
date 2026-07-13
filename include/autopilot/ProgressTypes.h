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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PROGRESSTYPES_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PROGRESSTYPES_H_

#include <cstdint>
#include <optional>

#include "NumericGuid.h"

namespace arlcore::autopilot {

//! \brief Achieved-flag snapshot for an active vector command, mapped into
//! UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportType.
struct VectorProgress {
  bool valid = false;            // false until the brain has run at least one tick
  bool directionAchieved = false;
  bool elevationAchieved = true;  // true when no elevation was requested
  bool speedAchieved = false;
  bool hardViolation = false;    // hard tolerances: violated persistently after being achieved
};

//! \brief Per-tick capture evaluation for the current target waypoint.
struct CaptureResult {
  bool positionAchieved = false;
  std::optional<bool> attitudeAchieved;   // nullopt if waypoint has no attitude requirement
  bool elevationAchieved = true;          // true when no elevation requirement
  bool speedAchieved = false;
  bool captured = false;                  // all *required* criteria met
};

//! \brief Progress snapshot for an active waypoint route, mapped into
//! UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType.
struct WaypointProgress {
  bool valid = false;
  bool positionAchieved = false;
  std::optional<bool> attitudeAchieved;
  bool elevationAchieved = true;
  bool speedAchieved = false;
  bool trackLineAchieved = true;          // true when no trackTolerance requested
  std::optional<double> crossTrackErrorM;  // set only when trackTolerance is defined
  double groundSpeedMps = 0.0;            // current ground speed (for ETA estimation)
  double distanceToWaypointM = 0.0;
  double distanceRemainingM = 0.0;
  double cumulativeDistanceM = 0.0;
  int32_t waypointsRemaining = 0;
  arlcore::NumericGuid waypointId;
  bool routeComplete = false;
  bool failed = false;                    // exceeded max misses / replans
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PROGRESSTYPES_H_
