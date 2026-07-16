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

#include "RecoveryGuidance.h"

#include <algorithm>
#include <cmath>

#include "Logger.h"

namespace arlcore::autopilot {

RecoveryGuidance::RecoveryGuidance(const RecoveryConfig& config, double cruiseSpeedMps,
                                   double safetyMarginM)
    : config_(config), cruiseSpeedMps_(cruiseSpeedMps), safetyMarginM_(safetyMarginM) {}

bool RecoveryGuidance::begin(const GeoPoint& position, double depthM, std::optional<double> asfM,
                             const ZoneMap& map) {
  end();
  const auto& anchor = map.anchor();
  if (!anchor.has_value() || !map.hasZones()) {
    return false;
  }
  const ZoneSet zones = map.activeSet(anchor.value(), ElevationEnvelope::atPoint(depthM, asfM));
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  anchor->Forward(position.latDeg, position.lonDeg, 0.0, x, y, z);
  const std::optional<Vec2> local = zones.nearestCompliantPoint(Vec2{x, y}, safetyMarginM_);
  if (!local.has_value()) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Recovery: no compliant point found near the vehicle")
    return false;
  }
  GeoPoint target;
  double h = 0.0;
  anchor->Reverse(local->x, local->y, 0.0, target.latDeg, target.lonDeg, h);
  target_ = target;
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Recovery: driving to compliant point "
    << std::hypot(local->x - x, local->y - y) << " m away")
  return true;
}

std::optional<ControlVector> RecoveryGuidance::tick(const GeoPoint& position, double depthM,
                                                    std::optional<double> asfM,
                                                    const ZoneMap& map) {
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
  double vx = 0.0;
  double vy = 0.0;
  double tz = 0.0;
  anchor->Forward(position.latDeg, position.lonDeg, 0.0, vx, vy, tz);
  double tx = 0.0;
  double ty = 0.0;
  anchor->Forward(target_->latDeg, target_->lonDeg, 0.0, tx, ty, tz);

  ControlVector cv;
  cv.headingRad = std::atan2(tx - vx, ty - vy);  // azimuth toward the carrot
  cv.speedMps = config_.speedMps > 0.0 ? config_.speedMps : cruiseSpeedMps_;
  return cv;
}

bool RecoveryGuidance::complete(const GeoPoint& position, double depthM,
                                std::optional<double> asfM, const ZoneMap& map) {
  if (map.classify(position, depthM, asfM) == ZoneCompliance::COMPLIANT) {
    const auto now = std::chrono::steady_clock::now();
    if (!compliantSince_.has_value()) {
      compliantSince_ = now;
    }
    return std::chrono::duration<double>(now - compliantSince_.value()).count() >=
           config_.completeHoldS;
  }
  compliantSince_.reset();
  return false;
}

void RecoveryGuidance::end() {
  target_.reset();
  compliantSince_.reset();
}

}  // namespace arlcore::autopilot
