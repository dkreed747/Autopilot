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

#include "OperationalModeControlProvider.hpp"

#include "Logger.h"

namespace arlcore::autopilot {

using arlcore::umaa::services::CommandStateResult;
using arlcore::umaa::services::IncomingCommandBehavior;
using OperationalModeControlEnumType = UMAA::Common::MaritimeEnumeration::
    OperationalModeControlEnumModule::OperationalModeControlEnumType;

static OperationalMode toOperationalMode(OperationalModeControlEnumType mode) {
  switch (mode) {
    case OperationalModeControlEnumType::AUTONOMOUS:
      return OperationalMode::AUTONOMOUS;
    case OperationalModeControlEnumType::REMOTE:
      return OperationalMode::REMOTE;
    case OperationalModeControlEnumType::STANDBY:
    default:
      return OperationalMode::STANDBY;
  }
}

OperationalModeControlProvider::OperationalModeControlProvider(
    const arlcore::NumericGuid& source,
    std::shared_ptr<OperationalModeControlProviderIo> io,
    OperationalModeManager* modeManager)
    : CommandProviderBase(source, io), modeManager_(modeManager) {
  setBehavior(IncomingCommandBehavior::CANCEL_EXISTING);
}

bool OperationalModeControlProvider::isCommandValid(
    const OperationalModeCommandType& cmd) {
  if (modeManager_->mode() == OperationalMode::MANUAL) {
    UMAA_LOG_WARN(
        util::SYSTEM_LOGGER,
        "Operational mode command rejected: the platform holds MANUAL control")
    return false;
  }
  return true;
}

CommandStateResult OperationalModeControlProvider::onCommanded(
    const std::weak_ptr<CmdSession> session) {
  auto cmdSession = session.lock();
  if (!cmdSession) {
    return CommandStateResult::ERROR;
  }
  const OperationalMode requested =
      toOperationalMode(cmdSession->getCommand().operationalMode());
  // Only reachable in MANUAL if it engaged between validation and here; the
  // base then fails the command SERVICE_FAILED (legal from COMMANDED).
  return modeManager_->commandMode(requested) ? CommandStateResult::ADVANCE
                                              : CommandStateResult::ERROR;
}

CommandStateResult OperationalModeControlProvider::onExecuting(
    const std::weak_ptr<CmdSession> session) {
  // The transition already applied in onCommanded; complete in the same cycle.
  return CommandStateResult::ADVANCE;
}

}  // namespace arlcore::autopilot
