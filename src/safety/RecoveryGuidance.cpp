#include "autopilot/safety/RecoveryGuidance.hpp"

#include <algorithm>
#include <cmath>

#include "InternalTypes.h"
#include "Logger.h"

namespace arlcore::autopilot {

RecoveryGuidance::RecoveryGuidance(const RecoveryConfig& config, flt64_t cruiseSpeedMps, flt64_t safetyMarginM)
    : config_(config), cruiseSpeedMps_(cruiseSpeedMps), safetyMarginM_(safetyMarginM) {}

bool RecoveryGuidance::begin(const GeoPoint& position, flt64_t depthM, std::optional<flt64_t> asfM,
                             const ZoneMap& map) {
  end();
  const auto& anchor = map.anchor();
  if (!anchor.has_value() || !map.hasZones()) {
    return false;
  }
  const ZoneSet zones = map.activeSet(anchor.value(), ElevationEnvelope::atPoint(depthM, asfM));
  flt64_t x = 0.0;
  flt64_t y = 0.0;
  flt64_t z = 0.0;
  anchor->Forward(position.latDeg, position.lonDeg, 0.0, x, y, z);
  const std::optional<Vec2> local = zones.nearestCompliantPoint(Vec2{x, y}, safetyMarginM_);
  if (!local.has_value()) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Recovery: no compliant point found near the vehicle")
    return false;
  }
  GeoPoint target;
  flt64_t h = 0.0;
  anchor->Reverse(local->x, local->y, 0.0, target.latDeg, target.lonDeg, h);
  target_ = target;
  UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                "Recovery: driving to compliant point " << std::hypot(local->x - x, local->y - y) << " m away")
  return true;
}

std::optional<ControlVector> RecoveryGuidance::tick(const GeoPoint& position, flt64_t depthM,
                                                    std::optional<flt64_t> asfM, const ZoneMap& map) {
  if (!target_.has_value()) {
    return std::nullopt;
  }
  // Re-validate the carrot: a constraint change can invalidate the picked point mid-recovery.
  if (!map.pointCompliant(target_.value(), depthM, safetyMarginM_ - 1e-6, asfM)) {
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Recovery: target no longer compliant; re-picking")
    if (!begin(position, depthM, asfM, map)) {
      return std::nullopt;
    }
  }
  const auto& anchor = map.anchor();
  if (!anchor.has_value()) {
    return std::nullopt;
  }
  flt64_t vx = 0.0;
  flt64_t vy = 0.0;
  flt64_t tz = 0.0;
  anchor->Forward(position.latDeg, position.lonDeg, 0.0, vx, vy, tz);
  flt64_t tx = 0.0;
  flt64_t ty = 0.0;
  anchor->Forward(target_->latDeg, target_->lonDeg, 0.0, tx, ty, tz);

  ControlVector cv;
  cv.headingRad = std::atan2(tx - vx, ty - vy);  // azimuth toward the carrot
  cv.speedMps = config_.speedMps > 0.0 ? config_.speedMps : cruiseSpeedMps_;
  return cv;
}

bool RecoveryGuidance::complete(const GeoPoint& position, flt64_t depthM, std::optional<flt64_t> asfM,
                                const ZoneMap& map) {
  if (map.classify(position, depthM, asfM) == ZoneCompliance::COMPLIANT) {
    const auto now = std::chrono::steady_clock::now();
    if (!compliantSince_.has_value()) {
      compliantSince_ = now;
    }
    return std::chrono::duration<flt64_t>(now - compliantSince_.value()).count() >= config_.completeHoldS;
  }
  compliantSince_.reset();
  return false;
}

void RecoveryGuidance::end() {
  target_.reset();
  compliantSince_.reset();
}

}  // namespace arlcore::autopilot
