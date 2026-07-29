#ifndef AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDER_HPP_
#define AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDER_HPP_

#include <memory>

#include "CommandProviderBase.h"
#include "autopilot/modes/OperationalModeControlProviderIo.hpp"
#include "autopilot/modes/OperationalModeManager.hpp"

namespace arlcore::autopilot {

//! \brief UMAA MM OperationalModeControl PROVIDER. Applies STANDBY / REMOTE /
//! AUTONOMOUS mode commands to the OperationalModeManager and runs each command
//! promptly to COMPLETED (no standing session). Commands are honored from any
//! source (classification does not apply to mode commands) and while safety
//! recovery/safe mode is engaged; the single rejection is MANUAL, which the
//! platform alone controls (the command enum cannot even express it).
class OperationalModeControlProvider
    : public arlcore::umaa::services::CommandProviderBase<
          OperationalModeCommandType, OperationalModeCommandAckReportType,
          OperationalModeCommandStatusType> {
 public:
  OperationalModeControlProvider(
      const arlcore::NumericGuid& source,
      std::shared_ptr<OperationalModeControlProviderIo> io,
      OperationalModeManager* modeManager);

 protected:
  bool isCommandValid(const OperationalModeCommandType& cmd) override;
  arlcore::umaa::services::CommandStateResult onCommanded(
      const std::weak_ptr<CmdSession> session) override;
  arlcore::umaa::services::CommandStateResult onExecuting(
      const std::weak_ptr<CmdSession> session) override;

 private:
  OperationalModeManager* modeManager_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDER_HPP_
