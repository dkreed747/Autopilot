#ifndef AUTOPILOT_SAFETY_VECTORZONEGUIDANCE_HPP_
#define AUTOPILOT_SAFETY_VECTORZONEGUIDANCE_HPP_

#include <chrono>
#include <cstdint>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/safety/ZoneGeometry.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief Tangent-bug style zone avoidance for vector mode, where the goal is only a heading
//! (never a position): with perfect zone knowledge the "bug" sensors become geometry queries.
//!
//! MOTION_TO_HEADING casts a clearance ray along the commanded heading out to a speed- and
//! turn-radius-scaled lookahead; a hit switches to BOUNDARY_FOLLOW, which circulates the
//! blocking boundary at the safety-margin standoff using the same atan2 cross-track law shape
//! as the waypoint tracker (regulating clearance - margin against the turn radius). The follow
//! direction is chosen once per episode by minimal heading deviation and remembered (direction
//! memory prevents oscillation at concave features). Corners need no special code: the
//! closest-feature clearance gradient rounds convex vertices and miters reflex ones. The
//! episode ends when the commanded heading has been clear (to an extended lookahead) for a
//! consecutive-tick streak and a minimum dwell has elapsed.
//!
//! A keep-in acts as an inverted obstacle automatically: a commanded heading pointing out of
//! the area keeps hitting the boundary from the inside, so the vehicle circulates it.
class VectorZoneGuidance {
 public:
  VectorZoneGuidance(const VectorAvoidanceConfig& config, flt64_t turnRadiusM, flt64_t safetyMarginM);

  //! \brief Reset the avoidance episode (call when a new vector setpoint is installed).
  void reset();

  //! \brief Produce the heading to fly for this tick.
  //! \param commandedAz The commanded heading (azimuth, radians true north)
  //! \param vehicle The vehicle position in the same local frame as `zones`
  //! \param sogMps Current speed over ground
  //! \param zones The active zones (vehicle-depth gated)
  //! \return The commanded heading, or the boundary-follow heading while avoiding
  flt64_t steer(flt64_t commandedAz, const Vec2& vehicle, flt64_t sogMps, const ZoneSet& zones);

  //! \brief Whether an avoidance episode is in progress (callers suspend hard-tolerance
  //! failure timers while true).
  bool avoidanceActive() const { return state_ == State::BOUNDARY_FOLLOW; }

 private:
  enum class State { MOTION_TO_HEADING, BOUNDARY_FOLLOW };

  flt64_t lookaheadM(flt64_t sogMps) const;

  VectorAvoidanceConfig config_;
  flt64_t turnRadiusM_;
  flt64_t safetyMarginM_;

  State state_ = State::MOTION_TO_HEADING;
  bool followRight_ = false;  // wall kept to port (follow clockwise) when true
  int32_t clearTicks_ = 0;
  std::chrono::steady_clock::time_point followStart_{};
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_VECTORZONEGUIDANCE_HPP_
