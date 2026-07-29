#ifndef AUTOPILOT_GUIDANCE_CONTROLVECTOR_HPP_
#define AUTOPILOT_GUIDANCE_CONTROLVECTOR_HPP_

#include <optional>

namespace arlcore::autopilot {

//! \brief The reference frame an elevation/depth setpoint is expressed in.
enum class ElevationFrame {
  DEPTH,             // meters below the surface (positive down, 0 at the surface)
  ALTITUDE_MSL,      // meters above mean sea level
  ALTITUDE_AGL,      // meters above ground level
  ALTITUDE_ASF,      // meters above the sea floor (positive up, 0 at the floor)
  ALTITUDE_GEODETIC  // meters above the WGS84 ellipsoid
};

//! \brief The vector-like control command produced by the autopilot brain (IAutopilot)
//! and handed to the vehicle-control strategy (IVehicleControl). Mirrors the UMAA
//! Global Vector command layout: heading, speed, and an optional elevation/depth.
struct ControlVector {
  //! \brief Desired heading in radians, true north, in [-pi, pi].
  double headingRad = 0.0;

  //! \brief Desired speed in meters per second (over ground).
  double speedMps = 0.0;

  //! \brief Target elevation/depth. std::nullopt means "hold current / any acceptable".
  std::optional<double> elevationM = std::nullopt;

  //! \brief Reference frame for elevationM.
  ElevationFrame elevationFrame = ElevationFrame::DEPTH;

  //! \brief Optional desired pitch while changing depth/elevation (radians).
  std::optional<double> pitchRad = std::nullopt;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_CONTROLVECTOR_HPP_
