#ifndef AUTOPILOT_CONFIG_CONFIGVALIDATION_HPP_
#define AUTOPILOT_CONFIG_CONFIGVALIDATION_HPP_

#include <string>
#include <vector>

#include "autopilot/config/AutopilotConfig.hpp"

namespace arlcore::autopilot {

//! \brief Whether a string is a well-formed 8-4-4-4-12 UUID. Malformed IDs would silently
//! become indeterminate GUIDs downstream (the SDK's uuid_parse result is unchecked).
bool isValidUuid(const std::string& uuid);

//! \brief Range/semantic validation of a loaded config. Errors mean the config must be
//! rejected; warnings are logged and the value kept. Returns true when there are no errors.
bool validateConfig(const AutopilotConfig& config, std::vector<std::string>* errors,
                    std::vector<std::string>* warnings);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_CONFIG_CONFIGVALIDATION_HPP_
