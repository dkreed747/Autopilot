#include "autopilot/guidance/CrossTrackController.hpp"

#include <algorithm>
#include <cmath>

namespace arlcore::autopilot {

flt64_t CrossTrackController::approachCorrection(flt64_t errM, flt64_t approachLimitRad, flt64_t gainPerM) {
  return approachLimitRad * (2.0 / M_PI) * std::atan(gainPerM * errM);
}

flt64_t CrossTrackController::integrate(flt64_t pRad, flt64_t errM, flt64_t dtS) {
  if (params_.ki > 0.0) {
    const flt64_t dt = std::clamp(dtS, 0.0, 1.0);
    // Conditional integration: freeze while the summed output is saturated and the error
    // would push it deeper (classic clamping anti-windup); back-off is always allowed.
    const flt64_t unclamped = pRad + integralRad_;
    const bool saturatedDeeper = std::abs(unclamped) >= params_.correctionLimitRad && (unclamped > 0.0) == (errM > 0.0);
    if (dt > 0.0 && std::abs(errM) <= params_.integratorGateM && !saturatedDeeper) {
      integralRad_ =
          std::clamp(integralRad_ + params_.ki * errM * dt, -params_.integratorLimitRad, params_.integratorLimitRad);
    }
  }
  return std::clamp(pRad + integralRad_, -params_.correctionLimitRad, params_.correctionLimitRad);
}

}  // namespace arlcore::autopilot
