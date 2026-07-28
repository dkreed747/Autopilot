//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
//
// Applied Research Laboratory
// Pennsylvania State University
// P.O. Box 30
// State College, PA 16804-0030
//
// DISTRIBUTION STATEMENT A. Approved for public release.
// Distribution is unlimited.
// This software was developed by the Department of the Navy,
// NAVSEA Unmanned and Small Combatants. It is provided under the terms of
// use found in the LICENSE file at the source code root directory.
//
//---------------------------------------------------------------------------

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODECONTROLPROVIDER_HPP_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODECONTROLPROVIDER_HPP_

#include <memory>

#include "CommandProviderBase.h"
#include "OperationalModeControlProviderIo.hpp"
#include "OperationalModeManager.hpp"

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
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODECONTROLPROVIDER_HPP_
