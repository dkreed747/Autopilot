#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_YAMLCONFIGLOADER_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_YAMLCONFIGLOADER_H_

#include <string>

#include "AutopilotConfig.h"

namespace arlcore::autopilot {

//! \brief Loads an AutopilotConfig from a YAML file. Missing fields keep their struct
//! defaults; only the file's presence is required.
class YamlConfigLoader {
 public:
  //! \brief Load configuration from a YAML file into out.
  //! \param path Path to the YAML file
  //! \param out Configuration object to populate
  //! \return true if the file was loaded successfully
  static bool load(const std::string& path, AutopilotConfig* out);
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_YAMLCONFIGLOADER_H_
