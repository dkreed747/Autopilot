#ifndef AUTOPILOT_TOOLS_CLIENTS_CLIENTIDENTITY_HPP_
#define AUTOPILOT_TOOLS_CLIENTS_CLIENTIDENTITY_HPP_

#include "autopilot/config/AutopilotConfig.hpp"
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
#endif  // AUTOPILOT_TOOLS_CLIENTS_CLIENTIDENTITY_HPP_
