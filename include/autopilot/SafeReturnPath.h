//---------------------------------------------------------------------------
// Copyright 2026 Pennsylvania State University
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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SAFERETURNPATH_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SAFERETURNPATH_H_

#include <optional>
#include <string>
#include <vector>

#include "AutopilotConfig.h"
#include "MissionRoute.h"

namespace arlcore::autopilot {

//! \brief The Safe Return Path: a waypoint mission loaded from a CSV at startup and executed by
//! the SRP safe-mode strategy when the safety supervisor escalates.
//!
//! The CSV's east/north coordinates are anchored at the explicit origin from the yaml config —
//! never at the first GPS fix — so the safety artifact is fixed, reviewable, and validatable
//! before the vehicle has navigation. (This matches mission_runner's configured-origin
//! convention.)
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
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SAFERETURNPATH_H_
