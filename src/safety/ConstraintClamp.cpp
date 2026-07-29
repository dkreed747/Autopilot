#include "autopilot/safety/ConstraintClamp.hpp"

#include <algorithm>

namespace arlcore::autopilot {

namespace {

//! \brief The smaller of two optional upper bounds (nullopt = unbounded).
std::optional<double> mergeMax(const std::optional<double>& a, const std::optional<double>& b) {
  if (a.has_value() && b.has_value()) {
    return std::min(a.value(), b.value());
  }
  return a.has_value() ? a : b;
}

//! \brief The larger of two optional lower bounds (nullopt = unbounded).
std::optional<double> mergeMin(const std::optional<double>& a, const std::optional<double>& b) {
  if (a.has_value() && b.has_value()) {
    return std::max(a.value(), b.value());
  }
  return a.has_value() ? a : b;
}

}  // namespace

ClampResult applyConstraintClamps(const ControlVector& cv, const ConstraintSnapshot& snapshot,
                                  const ClampLimits& staticLimits) {
  ClampResult result;
  result.cv = cv;

  std::optional<double> maxSpeed = mergeMax(snapshot.maxSpeedMps, staticLimits.maxSpeedMps);
  std::optional<double> minSpeed = mergeMin(snapshot.minSpeedMps, staticLimits.minSpeedMps);
  if (minSpeed.has_value() && maxSpeed.has_value() && minSpeed.value() > maxSpeed.value()) {
    result.conflict = true;
    minSpeed = maxSpeed;
  }
  if (maxSpeed.has_value()) {
    if (result.cv.speedMps > maxSpeed.value()) {
      result.cv.speedMps = maxSpeed.value();
      result.speedClamped = true;
    } else if (result.cv.speedMps < -maxSpeed.value()) {
      result.cv.speedMps = -maxSpeed.value();
      result.speedClamped = true;
    }
  }
  // A minimum speed never turns a commanded stop into motion.
  if (minSpeed.has_value() && result.cv.speedMps > 0.0 && result.cv.speedMps < minSpeed.value()) {
    result.cv.speedMps = minSpeed.value();
    result.speedClamped = true;
  }

  if (result.cv.elevationM.has_value() && result.cv.elevationFrame == ElevationFrame::DEPTH) {
    std::optional<double> maxDepth = mergeMax(snapshot.maxDepthM, staticLimits.maxDepthM);
    std::optional<double> minDepth = mergeMin(snapshot.minDepthM, staticLimits.minDepthM);
    if (minDepth.has_value() && maxDepth.has_value() && minDepth.value() > maxDepth.value()) {
      result.conflict = true;
      minDepth = maxDepth;
    }
    if (maxDepth.has_value() && result.cv.elevationM.value() > maxDepth.value()) {
      result.cv.elevationM = maxDepth.value();
      result.elevationClamped = true;
    }
    if (minDepth.has_value() && result.cv.elevationM.value() < minDepth.value()) {
      result.cv.elevationM = minDepth.value();
      result.elevationClamped = true;
    }
  }

  return result;
}

}  // namespace arlcore::autopilot
