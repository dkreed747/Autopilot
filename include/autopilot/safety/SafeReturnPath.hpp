#ifndef AUTOPILOT_SAFETY_SAFERETURNPATH_HPP_
#define AUTOPILOT_SAFETY_SAFERETURNPATH_HPP_

#include <optional>
#include <string>
#include <vector>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/guidance/MissionRoute.hpp"

namespace arlcore::autopilot {

//! \brief The Safe Return Path: a waypoint mission loaded from a CSV at startup and executed by
//! the SRP safe-mode strategy when the safety supervisor escalates.
//! The CSV's east/north coordinates are anchored at the explicit yaml origin — never the first
//! GPS fix — so the safety artifact is reviewable before the vehicle has navigation (matching
//! mission_runner's configured-origin convention).
class SafeReturnPath {
 public:
  //! \brief Load and validate the SRP described by `config`. Returns nullopt when no CSV is
  //! configured (caller falls back to zero-speed hold) or when the configuration/CSV is invalid
  //! (caller must treat that as a startup error if the SRP strategy was requested).
  //! `error` distinguishes the two: it is set only for invalid configurations.
  static std::optional<SafeReturnPath> load(const SrpConfig& config, std::string* error);

  const std::vector<MissionWaypoint>& waypoints() const { return waypoints_; }
  const MissionWaypoint& lastWaypoint() const { return waypoints_.back(); }
  const SrpConfig& config() const { return config_; }

 private:
  SafeReturnPath(SrpConfig config, std::vector<MissionWaypoint> waypoints);

  SrpConfig config_;
  std::vector<MissionWaypoint> waypoints_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_SAFERETURNPATH_HPP_
