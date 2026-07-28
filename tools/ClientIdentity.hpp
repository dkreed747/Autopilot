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

#ifndef APPS_AUTOPILOT_TOOLS_CLIENTIDENTITY_HPP_
#define APPS_AUTOPILOT_TOOLS_CLIENTIDENTITY_HPP_

#include "AutopilotConfig.h"
#include "NumericGuid.h"

namespace arlcore::autopilot::tools {

//! \brief The UMAA identity stamped on every command a consumer-side tool
//! publishes: source id = sourceId, source parentID = platformId. The autopilot
//! classifies commands whose parentID equals its identity.platform_id as LOCAL
//! autonomy; everything else is a REMOTE operator.
struct ClientIdentity {
  arlcore::NumericGuid sourceId;
  arlcore::NumericGuid platformId;
};

//! \brief Build the console's identity from the console: config block. An empty
//! source_id mints a per-process id (and warns: restarts then lose "own
//! command" recognition); an empty platform_id stays nil, which classifies as
//! REMOTE.
ClientIdentity makeClientIdentity(const ConsoleConfig& config);

//! \brief Identity for a tool acting as this platform's onboard autonomy
//! (mission_runner): a minted per-run source id under the autopilot's own
//! platform_id.
ClientIdentity makeLocalAutonomyIdentity(const IdentityConfig& identity);

}  // namespace arlcore::autopilot::tools
#endif  // APPS_AUTOPILOT_TOOLS_CLIENTIDENTITY_HPP_
