#include "autopilot/safety/ConstraintClamp.hpp"

#include <algorithm>

#include "InternalTypes.h"
#include "autopilot/guidance/ElevationUtils.hpp"

namespace arlcore::autopilot {

//! \brief The smaller of two optional upper bounds (nullopt = unbounded).
static std::optional<flt64_t> mergeMax(const std::optional<flt64_t>& a, const std::optional<flt64_t>& b) {
  if (a.has_value() && b.has_value()) {
    return std::min(a.value(), b.value());
  }
  return a.has_value() ? a : b;
}

//! \brief The larger of two optional lower bounds (nullopt = unbounded).
static std::optional<flt64_t> mergeMin(const std::optional<flt64_t>& a, const std::optional<flt64_t>& b) {
  if (a.has_value() && b.has_value()) {
    return std::max(a.value(), b.value());
  }
  return a.has_value() ? a : b;
}

//! \brief Apply a resolved [lower, upper] window to the setpoint already in `result`.
static void clampIntoWindow(const std::optional<flt64_t>& lower, const std::optional<flt64_t>& upper,
                            ClampResult* result) {
  if (upper.has_value() && result->cv.elevationM.value() > upper.value()) {
    result->cv.elevationM = upper.value();
    result->elevationClamped = true;
  }
  if (lower.has_value() && result->cv.elevationM.value() < lower.value()) {
    result->cv.elevationM = lower.value();
    result->elevationClamped = true;
  }
}

//! \brief Clamp a DEPTH setpoint. The bottom clearance becomes a depth ceiling once the floor is
//! known; a water column thinner than the clearance leaves nothing legal, so it resolves to the
//! surface rather than to a depth above it.
static void clampDepthSetpoint(const std::optional<flt64_t>& maxDepth, const std::optional<flt64_t>& minDepth,
                               const std::optional<flt64_t>& minAsf, const std::optional<flt64_t>& floorDepthM,
                               ClampResult* result) {
  std::optional<flt64_t> upper = maxDepth;
  if (minAsf.has_value() && floorDepthM.has_value()) {
    const flt64_t clearanceCeilingM = elevation::flipDepthAsf(minAsf.value(), floorDepthM.value());
    if (clearanceCeilingM < 0.0) {
      result->conflict = true;
    }
    upper = mergeMax(upper, std::optional<flt64_t>(std::max(0.0, clearanceCeilingM)));
  }
  std::optional<flt64_t> lower = minDepth;
  if (lower.has_value() && upper.has_value() && lower.value() > upper.value()) {
    // The deep-side bound wins, which in the DEPTH frame is always the shallower of the two.
    result->conflict = true;
    lower = upper;
  }
  clampIntoWindow(lower, upper, result);
}

//! \brief Clamp an ALTITUDE_ASF setpoint. The bottom clearance is frame-native and always applies;
//! the depth limits convert once the floor is known. The surface (an altitude equal to the floor
//! depth) is the only upper bound this frame has, so it always caps the setpoint.
static void clampAsfSetpoint(const std::optional<flt64_t>& maxDepth, const std::optional<flt64_t>& minDepth,
                             const std::optional<flt64_t>& minAsf, const std::optional<flt64_t>& floorDepthM,
                             ClampResult* result) {
  if (!floorDepthM.has_value()) {
    if (maxDepth.has_value() || minDepth.has_value()) {
      // Nothing here can bound the setpoint against the depth limit, and a platform converting it
      // against its own bathymetry would drive straight past that limit unchecked.
      result->cv.elevationM = std::nullopt;
      result->elevationUnbounded = true;
      return;
    }
    clampIntoWindow(minAsf, std::nullopt, result);
    return;
  }
  const flt64_t floorM = floorDepthM.value();
  std::optional<flt64_t> lower = minAsf;
  if (maxDepth.has_value()) {
    lower = mergeMin(lower, std::optional<flt64_t>(elevation::flipDepthAsf(maxDepth.value(), floorM)));
  }
  std::optional<flt64_t> upper;
  if (minDepth.has_value()) {
    upper = elevation::flipDepthAsf(minDepth.value(), floorM);
  }
  if (lower.has_value() && upper.has_value() && lower.value() > upper.value()) {
    // The deep-side bound wins, which in the ASF frame is the larger (higher off the bottom) one.
    result->conflict = true;
    upper = lower;
  }
  if (lower.has_value() && lower.value() > floorM) {
    result->conflict = true;  // the required clearance is more water than there is
  }
  upper = upper.has_value() ? std::min(upper.value(), floorM) : floorM;
  if (lower.has_value()) {
    lower = std::min(lower.value(), floorM);
  }
  clampIntoWindow(lower, upper, result);
}

ClampResult applyConstraintClamps(const ControlVector& cv, const ConstraintSnapshot& snapshot,
                                  const ClampLimits& staticLimits, std::optional<flt64_t> floorDepthM) {
  ClampResult result;
  result.cv = cv;

  std::optional<flt64_t> maxSpeed = mergeMax(snapshot.maxSpeedMps, staticLimits.maxSpeedMps);
  std::optional<flt64_t> minSpeed = mergeMin(snapshot.minSpeedMps, staticLimits.minSpeedMps);
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

  if (!result.cv.elevationM.has_value()) {
    return result;
  }
  const std::optional<flt64_t> maxDepth = mergeMax(snapshot.maxDepthM, staticLimits.maxDepthM);
  const std::optional<flt64_t> minDepth = mergeMin(snapshot.minDepthM, staticLimits.minDepthM);
  if (result.cv.elevationFrame == ElevationFrame::DEPTH) {
    clampDepthSetpoint(maxDepth, minDepth, staticLimits.minAltitudeAsfM, floorDepthM, &result);
  } else if (result.cv.elevationFrame == ElevationFrame::ALTITUDE_ASF) {
    clampAsfSetpoint(maxDepth, minDepth, staticLimits.minAltitudeAsfM, floorDepthM, &result);
  }
  // ALTITUDE_MSL/AGL/GEODETIC have no depth equivalent without a geoid reference the autopilot
  // does not have, so they pass through; admission refuses them whenever a limit is configured.
  // TODO(autopilot-#4): clamp the MSL/AGL/geodetic frames once a geoid reference is available.

  return result;
}

}  // namespace arlcore::autopilot
