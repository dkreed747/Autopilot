#ifndef AUTOPILOT_CONFIG_YAMLCONFIGLOADER_HPP_
#define AUTOPILOT_CONFIG_YAMLCONFIGLOADER_HPP_

#include <string>

#include "autopilot/config/AutopilotConfig.hpp"

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
#endif  // AUTOPILOT_CONFIG_YAMLCONFIGLOADER_HPP_
