#ifndef AUTOPILOT_GUIDANCE_ELEVATIONUTILS_HPP_
#define AUTOPILOT_GUIDANCE_ELEVATIONUTILS_HPP_

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <optional>

#include "InternalTypes.h"
#include "autopilot/guidance/ControlVector.hpp"

namespace arlcore::autopilot {

//! \brief The vertical pair every zone query needs: depth below the surface, plus the
//! above-sea-floor altitude when the platform reports one.
struct DepthAsf {
  flt64_t depthM = 0.0;
  std::optional<flt64_t> asfM;
};

}  // namespace arlcore::autopilot

//! \brief Elevation-frame helpers: read the pose in a commanded frame, derive the seafloor
//! reference from it, and convert between DEPTH and ALTITUDE_ASF.
namespace arlcore::autopilot::elevation {

//! \brief The vehicle's own elevation in `frame`, or nullopt when the pose carries no such field.
std::optional<flt64_t> poseElevation(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose,
                                     ElevationFrame frame);

//! \brief The pose's depth (0 when absent, as every zone query already assumes) and its ASF
//! altitude (nullopt when absent, which never exonerates an ASF-framed bound).
DepthAsf poseDepthAsf(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);

//! \brief Depth of the sea floor under the vehicle: depth + altitudeASF, the reference that makes
//! DEPTH and ALTITUDE_ASF interconvertible. Requires a strictly positive finite ASF - an altimeter
//! past its range reports 0, which would put the floor at the vehicle and derive a depth ceiling
//! shallower than the vehicle's own depth.
std::optional<flt64_t> floorDepthM(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);

//! \brief Convert a DEPTH value to ALTITUDE_ASF or back. The flip is its own inverse.
flt64_t flipDepthAsf(flt64_t valueM, flt64_t floorDepthM);

//! \brief Short frame name for logs and monitors.
const char* frameName(ElevationFrame frame);

}  // namespace arlcore::autopilot::elevation
#endif  // AUTOPILOT_GUIDANCE_ELEVATIONUTILS_HPP_
