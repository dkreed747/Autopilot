#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVESOURCE_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVESOURCE_H_

namespace arlcore::autopilot {

//! \brief Identifies which command type currently owns (or wants) the single driving resource.
enum class DriveSource {
  NONE,
  VECTOR,
  WAYPOINT,
  SAFE  // safe-mode maneuvers (SRP / hold); highest priority
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVESOURCE_H_
