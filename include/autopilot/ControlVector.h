//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONTROLVECTOR_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONTROLVECTOR_H_

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
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONTROLVECTOR_H_
