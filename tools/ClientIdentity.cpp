#include "ClientIdentity.hpp"

#include "Logger.h"
#include "UuidFactory.h"

namespace arlcore::autopilot::tools {

ClientIdentity makeClientIdentity(const ConsoleConfig& config) {
  ClientIdentity identity;
  if (config.sourceId.empty()) {
    identity.sourceId = arlcore::UuidFactory::getInstance().generateGuid();
    UMAA_LOG_WARN(
        util::SYSTEM_LOGGER,
        "console.source_id not set; minted a per-process source "
        "id (restarts will not recognize earlier commands as their own)")
  } else {
    identity.sourceId = arlcore::UuidFactory::getInstance().parseGuidFromString(
        config.sourceId);
  }
  if (config.platformId.empty()) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                  "console.platform_id not set; commands will carry a nil "
                  "parentID and classify as a REMOTE operator")
  } else {
    identity.platformId =
        arlcore::UuidFactory::getInstance().parseGuidFromString(
            config.platformId);
  }
  return identity;
}

ClientIdentity makeLocalAutonomyIdentity(const IdentityConfig& identity) {
  ClientIdentity clientIdentity;
  clientIdentity.sourceId = arlcore::UuidFactory::getInstance().generateGuid();
  if (!identity.platformId.empty()) {
    clientIdentity.platformId =
        arlcore::UuidFactory::getInstance().parseGuidFromString(
            identity.platformId);
  }
  return clientIdentity;
}

}  // namespace arlcore::autopilot::tools
