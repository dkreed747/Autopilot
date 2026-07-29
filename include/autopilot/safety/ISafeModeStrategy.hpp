#ifndef AUTOPILOT_SAFETY_ISAFEMODESTRATEGY_HPP_
#define AUTOPILOT_SAFETY_ISAFEMODESTRATEGY_HPP_

#include "autopilot/core/NavState.hpp"

namespace arlcore::autopilot {

class AutopilotBrain;

//! \brief What the vehicle does once the safety supervisor escalates to safe mode. Selected in
//! the yaml (safety.safe_mode.strategy); new strategies plug in through makeSafeModeStrategy.
class ISafeModeStrategy {
 public:
  virtual ~ISafeModeStrategy() = default;

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

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_ISAFEMODESTRATEGY_HPP_
