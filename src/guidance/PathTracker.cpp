#include "autopilot/guidance/PathTracker.hpp"

#include <algorithm>
#include <cmath>

#include "autopilot/guidance/AngleMath.hpp"

namespace arlcore::autopilot {

PathTracker::Output PathTracker::update(const Inputs& in) {
  Output out;
  if (!std::isfinite(in.pathAzimuthRad)) {
    // Nothing can be commanded without a tangent; propagate so the caller's finiteness screen
    // rejects the cycle rather than silently steering due north.
    out.headingRad = in.pathAzimuthRad;
    return out;
  }

  const bool kinematicsUsable = std::isfinite(in.groundSpeedMps) && std::isfinite(in.pathCurvatureAzRadPerM);
  const flt64_t vMps = kinematicsUsable ? std::max(in.groundSpeedMps, 0.0) : 0.0;
  out.desiredYawRateRps = vMps * (kinematicsUsable ? in.pathCurvatureAzRadPerM : 0.0);

  // Params is a public aggregate, so the limit is floored: std::clamp with hi < lo is undefined,
  // and a negative limit must disable the term rather than invert it.
  const flt64_t feedforwardLimitRad = std::max(params_.feedforwardLimitRad, 0.0);
  out.feedforwardRad = std::clamp(out.desiredYawRateRps * std::max(params_.headingLoopTauS, 0.0), -feedforwardLimitRad,
                                  feedforwardLimitRad);

  if (std::isfinite(in.crossTrackErrorM)) {
    const flt64_t p = CrossTrackController::approachCorrection(in.crossTrackErrorM, params_.crossTrackApproachRad,
                                                               params_.xte.kpScale * params_.crossTrackGainPerM);
    out.crossTrackRad = xteCtl_.integrate(p, in.crossTrackErrorM, std::isfinite(in.dtS) ? in.dtS : 0.0);
  }

  out.headingRad = wrapPi(in.pathAzimuthRad + out.feedforwardRad - out.crossTrackRad);
  return out;
}

}  // namespace arlcore::autopilot
