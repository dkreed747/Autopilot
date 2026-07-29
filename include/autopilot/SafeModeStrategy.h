#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SAFEMODESTRATEGY_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SAFEMODESTRATEGY_H_

#include <memory>
#include <optional>

#include "AutopilotConfig.h"
#include "NavState.h"
#include "SafeReturnPath.h"

namespace arlcore::autopilot {

class AutopilotBrain;

//! \brief What the vehicle does once the safety supervisor escalates to safe mode. Selected in
//! the yaml (safety.safe_mode.strategy) so different missions can carry different responses;
//! new strategies plug in through makeSafeModeStrategy.
class SafeModeStrategy {
 public:
  virtual ~SafeModeStrategy() = default;

  //! \brief Safe mode engaged: take the vehicle (via the brain's SAFE-mode entry points).
  virtual void onEnter(AutopilotBrain* brain, NavState* nav) = 0;

  //! \brief Called every supervisor tick while safe mode is active.
  virtual void onTick(AutopilotBrain* brain, NavState* nav) = 0;

  //! \brief Whether the strategy has finished its response and consents to the autopilot
  //! accepting commands again (e.g. SRP complete with accept_commands_after_srp).
  virtual bool readyToRelease() const = 0;

  //! \brief Safe mode is being exited (release or all-clear): give the vehicle back.
  virtual void onExit(AutopilotBrain* brain) = 0;

  virtual const char* name() const = 0;
};

//! \brief Build the configured strategy: "srp" runs the Safe Return Path (falling back to a
//! zero-speed hold when no SRP is loaded), "zero_speed_hold" holds position.
std::unique_ptr<SafeModeStrategy> makeSafeModeStrategy(const SafetyConfig& config,
                                                       const std::optional<SafeReturnPath>& srp);

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_SAFEMODESTRATEGY_H_
