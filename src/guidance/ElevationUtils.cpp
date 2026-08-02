#include "autopilot/guidance/ElevationUtils.hpp"

#include <cmath>

namespace arlcore::autopilot::elevation {

using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

std::optional<flt64_t> poseElevation(const GlobalPoseReportType& pose, ElevationFrame frame) {
  switch (frame) {
    case ElevationFrame::DEPTH:
      return pose.depth().has_value() ? std::optional<flt64_t>(pose.depth().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_MSL:
      return pose.altitude().has_value() ? std::optional<flt64_t>(pose.altitude().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_AGL:
      return pose.altitudeAGL().has_value() ? std::optional<flt64_t>(pose.altitudeAGL().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_ASF:
      return pose.altitudeASF().has_value() ? std::optional<flt64_t>(pose.altitudeASF().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_GEODETIC:
      return pose.altitudeGeodetic().has_value() ? std::optional<flt64_t>(pose.altitudeGeodetic().value())
                                                 : std::nullopt;
    default:
      return std::nullopt;
  }
}

DepthAsf poseDepthAsf(const GlobalPoseReportType& pose) {
  DepthAsf out;
  out.depthM = pose.depth().has_value() ? pose.depth().value() : 0.0;
  out.asfM = pose.altitudeASF().has_value() ? std::optional<flt64_t>(pose.altitudeASF().value()) : std::nullopt;
  return out;
}

std::optional<flt64_t> floorDepthM(const GlobalPoseReportType& pose) {
  if (!pose.depth().has_value() || !pose.altitudeASF().has_value()) {
    return std::nullopt;
  }
  const flt64_t depthM = pose.depth().value();
  const flt64_t asfM = pose.altitudeASF().value();
  if (!std::isfinite(depthM) || !std::isfinite(asfM) || asfM <= 0.0) {
    return std::nullopt;
  }
  return depthM + asfM;
}

flt64_t flipDepthAsf(flt64_t valueM, flt64_t floorDepthM) { return floorDepthM - valueM; }

const char* frameName(ElevationFrame frame) {
  switch (frame) {
    case ElevationFrame::DEPTH:
      return "depth";
    case ElevationFrame::ALTITUDE_MSL:
      return "msl";
    case ElevationFrame::ALTITUDE_AGL:
      return "agl";
    case ElevationFrame::ALTITUDE_ASF:
      return "asf";
    case ElevationFrame::ALTITUDE_GEODETIC:
      return "geodetic";
    default:
      return "unknown";
  }
}

}  // namespace arlcore::autopilot::elevation
