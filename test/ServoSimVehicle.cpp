#include "ServoSimVehicle.hpp"

#include <algorithm>
#include <cmath>

#include "autopilot/guidance/AngleMath.hpp"

UMAA::SA::GlobalPoseStatus::GlobalPoseReportType ServoSimVehicle::pose() const {
  flt64_t lat = 0.0;
  flt64_t lon = 0.0;
  flt64_t h = 0.0;
  frame_.Reverse(xE, yN, 0.0, lat, lon, h);
  UMAA::SA::GlobalPoseStatus::GlobalPoseReportType out;
  out.position().geodeticLatitude(lat);
  out.position().geodeticLongitude(lon);
  out.attitude().yaw().yaw(yawRad);
  return out;
}

void ServoSimVehicle::step(const arlcore::autopilot::ControlVector& cv, flt64_t dtS) {
  if (dtS <= 0.0) {
    return;
  }
  const flt64_t headingErr = arlcore::autopilot::wrapPi(cv.headingRad - yawRad);
  const flt64_t demanded = kTrueRpsPerRad * headingErr;
  rateSaturated = std::fabs(demanded) >= maxTurnRateRps;
  const flt64_t limited = std::clamp(demanded, -maxTurnRateRps, maxTurnRateRps);
  if (lagTauS > 0.0) {
    yawRateRps += (dtS / (lagTauS + dtS)) * (limited - yawRateRps);
  } else {
    yawRateRps = limited;
  }
  yawRad = arlcore::autopilot::wrapPi(yawRad + yawRateRps * dtS);
  speedMps = cv.speedMps;
  xE += (speedMps * std::sin(yawRad) + driftEastMps) * dtS;
  yN += (speedMps * std::cos(yawRad) + driftNorthMps) * dtS;
}
