#ifndef AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDER_HPP_
#define AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDER_HPP_

#include <memory>

#include "CommandProviderBase.h"
#include "autopilot/modes/OperationalModeControlProviderIo.hpp"
#include "autopilot/modes/OperationalModeManager.hpp"

namespace arlcore::autopilot {

//! \brief UMAA MM OperationalModeControl PROVIDER: applies mode commands to the
//! OperationalModeManager and runs each promptly to COMPLETED (no standing
//! session). Commands are honored from any source and during recovery/safe
//! mode; the single rejection is MANUAL, which the platform alone controls.
class OperationalModeControlProvider
    : public arlcore::umaa::services::CommandProviderBase<
          OperationalModeCommandType, OperationalModeCommandAckReportType, OperationalModeCommandStatusType> {
 public:
  OperationalModeControlProvider(const arlcore::NumericGuid& source,
                                 std::shared_ptr<OperationalModeControlProviderIo> io,
                                 OperationalModeManager* modeManager);

 protected:
  bool isCommandValid(const OperationalModeCommandType& cmd) override;
  arlcore::umaa::services::CommandStateResult onCommanded(const std::weak_ptr<CmdSession> session) override;
  arlcore::umaa::services::CommandStateResult onExecuting(const std::weak_ptr<CmdSession> session) override;

 private:
  OperationalModeManager* modeManager_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDER_HPP_
