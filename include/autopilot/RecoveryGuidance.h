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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_RECOVERYGUIDANCE_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_RECOVERYGUIDANCE_H_

#include <chrono>
#include <optional>

#include "AutopilotConfig.h"
#include "ControlVector.h"
#include "ZoneMap.h"

namespace arlcore::autopilot {

//! \brief The zone-violation recovery maneuver: drive straight back to the nearest compliant
//! point (back into a keep-in / out of a keep-out) at recovery speed, holding elevation.
//!
//! Steering is a direct carrot pursuit of the (re-validated every tick) target — a Dubins path
//! back would duplicate the tracker for a tens-of-meters maneuver, and turn-rate-limited
//! pursuit self-heals under disturbance. Recovery is complete once the position has classified
//! COMPLIANT for the configured hold time.
class RecoveryGuidance {
 public:
  RecoveryGuidance(const RecoveryConfig& config, double cruiseSpeedMps, double safetyMarginM);

  //! \brief Pick the recovery target for the current violation. Returns false when no
  //! compliant point could be found (the caller keeps the zero-speed hold while grace runs).
  bool begin(const GeoPoint& position, double depthM, const ZoneMap& map);

  //! \brief Whether a recovery target is currently held.
  bool active() const { return target_.has_value(); }

  //! \brief The control vector for this tick: heading at the carrot, recovery speed,
  //! elevation held. The target is re-validated (and re-picked when constraints changed);
  //! nullopt when there is no reachable target.
  std::optional<ControlVector> tick(const GeoPoint& position, double depthM, const ZoneMap& map);

  //! \brief Whether the position has classified COMPLIANT for the configured hold time.
  bool complete(const GeoPoint& position, double depthM, const ZoneMap& map);

  //! \brief Drop the recovery target and timers.
  void end();

 private:
  RecoveryConfig config_;
  double cruiseSpeedMps_;
  double safetyMarginM_;

  std::optional<GeoPoint> target_;
  std::optional<std::chrono::steady_clock::time_point> compliantSince_;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_RECOVERYGUIDANCE_H_
