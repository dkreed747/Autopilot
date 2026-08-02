#ifndef AUTOPILOT_CONFIG_CONFIGKEYREGISTRY_HPP_
#define AUTOPILOT_CONFIG_CONFIGKEYREGISTRY_HPP_

#include <string>
#include <vector>

namespace arlcore::autopilot {

//! \brief The set of dotted keys the YAML loader reads, plus the keys it used to read.
//!
//! The loader ignores anything it does not recognise, which has already cost a tuning campaign:
//! planner.tracker.trim_tau_limit_s was renamed and five experiment configs went on silently
//! running the default instead of the value they documented. A typo'd key and a key that describes
//! a control law which no longer exists are different problems, so they are reported separately:
//! unknown keys warn, because a gap in this registry must never be able to ground the vehicle,
//! while removed keys are errors, because loading them would mean honouring a config whose
//! intent the software can no longer carry out. The pre-operational-mode flat arbitration keys are
//! deliberately left as unknown rather than removed: a priority number is still expressible, just
//! under a different path, so the long-standing warn-and-accept behaviour is preserved.
class ConfigKeyRegistry {
 public:
  //! \brief Every dotted key path the loader reads.
  static const std::vector<std::string>& knownKeys();

  //! \brief Why a key was removed and what replaced it, or an empty string if it is not a removed
  //! key. The message is the operator's only clue, so it names the replacement.
  static std::string removalReason(const std::string& dottedKey);

  //! \brief Walk a parsed config and collect the leaf keys that are neither known nor removed
  //! into unknown, and the removed ones into removed. Either output may be null.
  static void collect(const std::string& yamlPath, std::vector<std::string>* unknown,
                      std::vector<std::string>* removed);
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_CONFIG_CONFIGKEYREGISTRY_HPP_
