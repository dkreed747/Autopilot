#ifndef AUTOPILOT_GUIDANCE_PROGRESSTYPES_HPP_
#define AUTOPILOT_GUIDANCE_PROGRESSTYPES_HPP_

#include <cstdint>
#include <optional>

#include "InternalTypes.h"
#include "NumericGuid.h"

namespace arlcore::autopilot {

//! \brief Achieved-flag snapshot for an active vector command, mapped into
//! UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportType.
struct VectorProgress {
  bool valid = false;  // false until the brain has run at least one tick
  bool directionAchieved = false;
  bool elevationAchieved = true;  // true when no elevation was requested
  bool speedAchieved = false;
  bool hardViolation = false;  // hard tolerances: violated persistently after being achieved
};

//! \brief Per-tick capture evaluation for the current target waypoint.
struct CaptureResult {
  bool positionAchieved = false;
  std::optional<bool> attitudeAchieved;  // nullopt if waypoint has no attitude requirement
  bool elevationAchieved = true;         // true when no elevation requirement
  bool speedAchieved = false;
  bool captured = false;  // all *required* criteria met
};

//! \brief Progress snapshot for an active waypoint route, mapped into
//! UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType.
struct WaypointProgress {
  bool valid = false;
  bool positionAchieved = false;
  std::optional<bool> attitudeAchieved;
  bool elevationAchieved = true;
  bool speedAchieved = false;
  bool trackLineAchieved = true;            // true when no trackTolerance requested
  std::optional<flt64_t> crossTrackErrorM;  // signed offset from the planned path (+ = starboard); set every update
  flt64_t groundSpeedMps = 0.0;             // current ground speed (for ETA estimation)
  flt64_t distanceToWaypointM = 0.0;
  flt64_t distanceRemainingM = 0.0;
  flt64_t cumulativeDistanceM = 0.0;
  int32_t waypointsRemaining = 0;
  arlcore::NumericGuid waypointId;
  bool routeComplete = false;
  bool failed = false;  // exceeded max misses / replans
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_PROGRESSTYPES_HPP_
