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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVESOURCE_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVESOURCE_H_

namespace arlcore::autopilot {

//! \brief Identifies which command type currently owns (or wants) the single driving resource.
enum class DriveSource {
  NONE,
  VECTOR,
  WAYPOINT
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVESOURCE_H_
