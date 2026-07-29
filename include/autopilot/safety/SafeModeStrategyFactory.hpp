#ifndef AUTOPILOT_SAFETY_SAFEMODESTRATEGYFACTORY_HPP_
#define AUTOPILOT_SAFETY_SAFEMODESTRATEGYFACTORY_HPP_

#include <memory>
#include <optional>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/safety/ISafeModeStrategy.hpp"
#include "autopilot/safety/SafeReturnPath.hpp"

namespace arlcore::autopilot {

//! \brief Build the configured strategy: "srp" runs the Safe Return Path (falling back to a
//! zero-speed hold when no SRP is loaded), "zero_speed_hold" holds position.
std::unique_ptr<ISafeModeStrategy> makeSafeModeStrategy(const SafetyConfig& config,
                                                        const std::optional<SafeReturnPath>& srp);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_SAFEMODESTRATEGYFACTORY_HPP_
