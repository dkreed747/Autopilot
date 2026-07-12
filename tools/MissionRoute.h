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

#ifndef APPS_AUTOPILOT_TOOLS_MISSIONROUTE_H_
#define APPS_AUTOPILOT_TOOLS_MISSIONROUTE_H_

#include <optional>
#include <string>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include <UMAA/Common/MaritimeEnumeration/MaritimeEnumerationSets.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointType.hpp>

namespace arlcore::autopilot::tools {

//! \brief One mission waypoint. Position is geodetic; the CSV loader fills it from local
//! tangent-plane east/north, the console API supplies it directly.
struct MissionWaypoint {
  double latDeg = 0.0;
  double lonDeg = 0.0;
  double speedMps = 3.0;
  double captureRadiusM = 2.5;
  std::optional<double> arrivalYawRad;
  std::optional<double> elevValueM;
  std::string elevFrame;  // "depth" or "asf"
};

//! \brief Parse a mission CSV in the local tangent plane anchored at `frame`, one waypoint
//! per line: east_m,north_m,speed_mps,capture_radius_m[,arrival_yaw_rad][,elev_value_m,elev_frame]
//! (header/comment lines that start with a letter are skipped).
std::vector<MissionWaypoint> loadMissionCsv(const std::string& path,
                                            const GeographicLib::LocalCartesian& frame);

//! \brief Build the UMAA waypoint sample for one mission waypoint (position + capture
//! tolerance, required ground speed, optional arrival attitude and depth/ASF elevation).
UMAA::MO::GlobalWaypointControl::GlobalWaypointType makeWaypoint(const MissionWaypoint& wp);

//! \brief Human-readable name for a UMAA command status.
std::string statusName(
    UMAA::Common::MaritimeEnumeration::CommandStatusEnumModule::CommandStatusEnumType s);

//! \brief Human-readable name for a UMAA command status reason.
std::string statusReasonName(
    UMAA::Common::MaritimeEnumeration::CommandStatusReasonEnumModule::CommandStatusReasonEnumType r);

}  // namespace arlcore::autopilot::tools

#endif  // APPS_AUTOPILOT_TOOLS_MISSIONROUTE_H_
