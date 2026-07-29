#ifndef AUTOPILOT_SAFETY_RECOVERYGUIDANCE_HPP_
#define AUTOPILOT_SAFETY_RECOVERYGUIDANCE_HPP_

#include <chrono>
#include <optional>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/guidance/ControlVector.hpp"
#include "autopilot/safety/ZoneMap.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief The zone-violation recovery maneuver: drive straight back to the nearest compliant
//! point (back into a keep-in / out of a keep-out) at recovery speed, holding elevation.
//! Steering is direct carrot pursuit of the every-tick-re-validated target — a Dubins path
//! would duplicate the tracker for a tens-of-meters maneuver, and pursuit self-heals under
//! disturbance.
class RecoveryGuidance {
 public:
  RecoveryGuidance(const RecoveryConfig& config, flt64_t cruiseSpeedMps, flt64_t safetyMarginM);

  //! \brief Pick the recovery target for the current violation. Returns false when no
  //! compliant point could be found (the caller keeps the zero-speed hold while grace runs).
  bool begin(const GeoPoint& position, flt64_t depthM, std::optional<flt64_t> asfM,
             const ZoneMap& map);

  //! \brief Whether a recovery target is currently held.
  bool active() const { return target_.has_value(); }

  //! \brief The control vector for this tick: heading at the carrot, recovery speed,
  //! elevation held. The target is re-validated (and re-picked when constraints changed);
  //! nullopt when there is no reachable target.
  std::optional<ControlVector> tick(const GeoPoint& position, flt64_t depthM,
                                    std::optional<flt64_t> asfM, const ZoneMap& map);

  //! \brief Whether the position has classified COMPLIANT for the configured hold time.
  bool complete(const GeoPoint& position, flt64_t depthM, std::optional<flt64_t> asfM,
                const ZoneMap& map);

  //! \brief Drop the recovery target and timers.
  void end();

 private:
  RecoveryConfig config_;
  flt64_t cruiseSpeedMps_;
  flt64_t safetyMarginM_;

  std::optional<GeoPoint> target_;
  std::optional<std::chrono::steady_clock::time_point> compliantSince_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_RECOVERYGUIDANCE_HPP_
