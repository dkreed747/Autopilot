#ifndef AUTOPILOT_MODES_DRIVESOURCE_HPP_
#define AUTOPILOT_MODES_DRIVESOURCE_HPP_

namespace arlcore::autopilot {

//! \brief Identifies which command type currently owns (or wants) the single driving resource.
enum class DriveSource {
  NONE,
  VECTOR,
  WAYPOINT,
  SAFE  // safe-mode maneuvers (SRP / hold); highest priority
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_MODES_DRIVESOURCE_HPP_
