#include "autopilot/safety/VectorZoneGuidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "InternalTypes.h"
#include "Logger.h"

namespace arlcore::autopilot {

VectorZoneGuidance::VectorZoneGuidance(const VectorAvoidanceConfig& config, flt64_t turnRadiusM, flt64_t safetyMarginM)
    : config_(config), turnRadiusM_(std::max(turnRadiusM, 1.0)), safetyMarginM_(safetyMarginM) {}

void VectorZoneGuidance::reset() {
  state_ = State::MOTION_TO_HEADING;
  clearTicks_ = 0;
}

flt64_t VectorZoneGuidance::lookaheadM(flt64_t sogMps) const {
  return std::max(config_.lookaheadRhoFactor * turnRadiusM_,
                  turnRadiusM_ + config_.lookaheadSpeedS * std::max(sogMps, 0.0));
}

flt64_t VectorZoneGuidance::steer(flt64_t commandedAz, const Vec2& vehicle, flt64_t sogMps, const ZoneSet& zones) {
  if (zones.empty()) {
    reset();
    return commandedAz;
  }

  const flt64_t lookahead = lookaheadM(sogMps);
  const Vec2 wantDir{std::sin(commandedAz), std::cos(commandedAz)};
  const bool commandedBlocked = zones.raycastFirstHit(vehicle, wantDir, safetyMarginM_, lookahead).has_value();

  if (state_ == State::MOTION_TO_HEADING) {
    if (!commandedBlocked) {
      return commandedAz;
    }
    // Follow direction: the boundary tangent (perpendicular to the clearance gradient) closer
    // to the commanded heading, remembered for the whole episode.
    const ClearanceInfo info = zones.clearanceInfo(vehicle);
    const Vec2 n = info.improveDir;
    const Vec2 tRight{n.y, -n.x};
    followRight_ = (tRight.x * wantDir.x + tRight.y * wantDir.y) >= 0.0;
    state_ = State::BOUNDARY_FOLLOW;
    followStart_ = std::chrono::steady_clock::now();
    clearTicks_ = 0;
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Vector avoidance: commanded heading blocked within "
                                           << lookahead << " m; following zone boundary "
                                           << (followRight_ ? "clockwise" : "counter-clockwise"))
  }

  const ClearanceInfo info = zones.clearanceInfo(vehicle);
  const Vec2 n = info.improveDir;
  const Vec2 t = followRight_ ? Vec2{n.y, -n.x} : Vec2{-n.y, n.x};

  // Wall-standoff regulation, same shape as the tracker's cross-track law: the error between
  // the clearance and the margin steers toward (error > 0) or away from (error < 0) the wall,
  // scaled against the turn radius so the correction never saturates turn authority.
  const flt64_t sideError = info.clearanceM - safetyMarginM_;
  // Corner anticipation: following the inside of a keep-in, the binding-edge normal flips
  // discretely at corners, so a second ray along the follow tangent detects the wall ahead —
  // the turn must begin roughly one turn radius before it.
  flt64_t aheadError = std::numeric_limits<flt64_t>::max();
  const std::optional<flt64_t> aheadHit = zones.raycastFirstHit(vehicle, t, safetyMarginM_, lookahead);
  if (aheadHit.has_value()) {
    aheadError = aheadHit.value() - turnRadiusM_;
  }
  const flt64_t error = std::min(sideError, aheadError);
  const flt64_t correction = std::clamp(std::atan2(error, turnRadiusM_), -1.2, 1.2);
  const Vec2 dir{t.x * std::cos(correction) - n.x * std::sin(correction),
                 t.y * std::cos(correction) - n.y * std::sin(correction)};
  const flt64_t followAz = std::atan2(dir.x, dir.y);

  // Leave the episode once the commanded heading has stayed clear (to an extended lookahead)
  // for a streak of ticks, a minimum dwell has passed, and the standoff is honored.
  const bool exitClear =
      !zones.raycastFirstHit(vehicle, wantDir, safetyMarginM_, lookahead * config_.exitClearFactor).has_value();
  if (exitClear && info.clearanceM >= safetyMarginM_) {
    ++clearTicks_;
  } else {
    clearTicks_ = 0;
  }
  const flt64_t dwellS = std::chrono::duration<flt64_t>(std::chrono::steady_clock::now() - followStart_).count();
  if (clearTicks_ >= config_.exitClearTicks && dwellS >= config_.minFollowS) {
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Vector avoidance: commanded heading clear; resuming")
    state_ = State::MOTION_TO_HEADING;
    clearTicks_ = 0;
    return commandedAz;
  }
  return followAz;
}

}  // namespace arlcore::autopilot
