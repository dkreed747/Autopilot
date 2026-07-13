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

#include "VectorControlServiceProvider.h"

#include <memory>

#include "Logger.h"
#include "ToleranceUtils.h"
#include "UmaaUtils.h"

namespace arlcore::autopilot {

using arlcore::umaa::services::CommandStateResult;
using arlcore::umaa::services::IncomingCommandBehavior;

VectorControlServiceProvider::VectorControlServiceProvider(
    const arlcore::NumericGuid& source, std::shared_ptr<VectorControlServiceProviderIo> io,
    IAutopilot* autopilot, double maxForwardSpeedMps) :
    CommandProviderBase(source, io),
    sourceId_(source),
    autopilot_(autopilot),
    maxForwardSpeedMps_(maxForwardSpeedMps) {
  // A new vector command replaces an in-flight vector command (same driving resource).
  setBehavior(IncomingCommandBehavior::CANCEL_EXISTING);
}

void VectorControlServiceProvider::relinquish() {
  autopilot_->clearSetpoint(DriveSource::VECTOR);
  autopilot_->arbiter().release(DriveSource::VECTOR);
}

bool VectorControlServiceProvider::isCommandValid(const GlobalVectorCommandType& cmd) {
  const std::optional<DirectionValue> dir = tolerance::extractDirection(cmd.direction());
  if (!dir.has_value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vector command direction variant is unsupported")
    return false;
  }
  const std::optional<SpeedValue> speed = tolerance::extractSpeed(cmd.speed());
  if (!speed.has_value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vector command speed variant is unsupported")
    return false;
  }
  if (maxForwardSpeedMps_ > 0.0 && speed->speedMps > maxForwardSpeedMps_) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vector command speed " << speed->speedMps
      << " exceeds platform max forward speed " << maxForwardSpeedMps_)
    return false;
  }
  if (cmd.endTime().has_value() && arlcore::umaa::getTimestamp() > cmd.endTime().value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vector command endTime is already in the past")
    return false;
  }
  return true;
}

CommandStateResult VectorControlServiceProvider::onCommanded(const std::weak_ptr<CmdSession> session) {
  auto cmdSession = session.lock();
  if (!cmdSession) {
    return CommandStateResult::ERROR;
  }
  // Vector is the high-priority source: this acquire preempts any active waypoint route.
  if (!autopilot_->arbiter().acquire(DriveSource::VECTOR)) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vector provider failed to acquire driving resource")
    return CommandStateResult::ERROR;
  }
  autopilot_->setVectorSetpoint(cmdSession->getCommand());
  return CommandStateResult::ADVANCE;
}

CommandStateResult VectorControlServiceProvider::onExecuting(const std::weak_ptr<CmdSession> session) {
  // Control is recomputed on each navigation packet by the brain; keep the command executing.
  return CommandStateResult::OK;
}

bool VectorControlServiceProvider::onUpdated(const std::weak_ptr<CmdSession> session,
    const GlobalVectorCommandType& previousCmd, const GlobalVectorCommandType& updatedCmd) {
  if (!isCommandValid(updatedCmd)) {
    return false;
  }
  autopilot_->setVectorSetpoint(updatedCmd);
  return true;
}

bool VectorControlServiceProvider::isCommandCompleted(const std::weak_ptr<CmdSession> session) {
  auto cmdSession = session.lock();
  if (!cmdSession) {
    return false;
  }
  const GlobalVectorCommandType cmd = cmdSession->getCommand();
  // Vector commands run indefinitely unless an end time is specified and has passed.
  if (cmd.endTime().has_value()) {
    return arlcore::umaa::getTimestamp() > cmd.endTime().value();
  }
  return false;
}

CommandStatusReasonEnumType VectorControlServiceProvider::isCommandFailed(
    const std::weak_ptr<CmdSession> session) {
  if (autopilot_->arbiter().wasRevoked(DriveSource::VECTOR)) {
    return CommandStatusReasonEnumType::INTERRUPTED;
  }
  if (autopilot_->vectorProgress().hardViolation) {
    return CommandStatusReasonEnumType::OBJECTIVE_FAILED;
  }
  return CommandStatusReasonEnumType::SUCCEEDED;
}

SendStatus VectorControlServiceProvider::sendExecutionStatus(const GlobalVectorCommandType& cmd) {
  if (!io_->cmdExeStatusSender.has_value()) {
    return SendStatus::SUCCESS;
  }
  const VectorProgress progress = autopilot_->vectorProgress();
  GlobalVectorExecutionStatusReportType report;
  report.sessionID() = cmd.sessionID();
  report.source().id() = sourceId_.getGuid();
  report.timeStamp() = arlcore::umaa::getTimestamp();
  report.directionAchieved() = progress.directionAchieved;
  report.elevationAchieved() = progress.elevationAchieved;
  report.speedAchieved() = progress.speedAchieved;
  return io_->cmdExeStatusSender.value()->send(report);
}

SendStatus VectorControlServiceProvider::disposeExecutionStatus(const GlobalVectorCommandType& cmd) {
  if (!io_->cmdExeStatusSender.has_value()) {
    return SendStatus::SUCCESS;
  }
  GlobalVectorExecutionStatusReportType report;
  report.sessionID() = cmd.sessionID();
  report.source().id() = sourceId_.getGuid();
  report.timeStamp() = arlcore::umaa::getTimestamp();
  return io_->cmdExeStatusSender.value()->dispose(report);
}

bool VectorControlServiceProvider::onCanceled(const std::weak_ptr<CmdSession> session) {
  relinquish();
  return true;
}

bool VectorControlServiceProvider::onFailed(const std::weak_ptr<CmdSession> session) {
  relinquish();
  return true;
}

bool VectorControlServiceProvider::onCompleted(const std::weak_ptr<CmdSession> session) {
  relinquish();
  return true;
}

}  // namespace arlcore::autopilot
