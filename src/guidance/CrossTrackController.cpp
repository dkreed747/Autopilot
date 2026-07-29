#include "autopilot/guidance/CrossTrackController.hpp"

#include <algorithm>
#include <cmath>

namespace arlcore::autopilot {

flt64_t CrossTrackController::pCorrection(flt64_t errM, flt64_t refM, flt64_t limitRad) {
  return std::clamp(std::atan2(errM, std::max(refM, 1.0)), -limitRad, limitRad);
}

flt64_t CrossTrackController::correction(flt64_t errM, flt64_t refM, flt64_t dtS) {
  const flt64_t p = std::atan2(params_.kpScale * errM, std::max(refM, 1.0));
  if (params_.ki > 0.0) {
    const flt64_t dt = std::clamp(dtS, 0.0, 1.0);
    // Conditional integration: freeze while the summed output is saturated and the error
    // would push it deeper (classic clamping anti-windup); back-off is always allowed.
    const flt64_t unclamped = p + integralRad_;
    const bool saturatedDeeper = std::abs(unclamped) >= params_.correctionLimitRad && (unclamped > 0.0) == (errM > 0.0);
    if (dt > 0.0 && std::abs(errM) <= params_.integratorGateM && !saturatedDeeper) {
      integralRad_ =
          std::clamp(integralRad_ + params_.ki * errM * dt, -params_.integratorLimitRad, params_.integratorLimitRad);
    }
  }
  return std::clamp(p + integralRad_, -params_.correctionLimitRad, params_.correctionLimitRad);
}

}  // namespace arlcore::autopilot
