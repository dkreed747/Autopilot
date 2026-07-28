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
    IAutopilot* autopilot, double maxForwardSpeedMps, const ISafetyGate* safetyGate,
    ICommandModeGate* modeGate) :
    CommandProviderBase(source, io),
    sourceId_(source),
    autopilot_(autopilot),
    maxForwardSpeedMps_(maxForwardSpeedMps),
    safetyGate_(safetyGate),
    modeGate_(modeGate) {
  // A new vector command replaces an in-flight vector command (same driving resource).
  setBehavior(IncomingCommandBehavior::CANCEL_EXISTING);
}

void VectorControlServiceProvider::relinquish() {
  autopilot_->clearSetpoint(DriveSource::VECTOR);
  autopilot_->arbiter().release(DriveSource::VECTOR);
  held_ = false;
}

CommandClass VectorControlServiceProvider::classOf(const GlobalVectorCommandType& cmd) const {
  return modeGate_ != nullptr ? modeGate_->classify(cmd.source()) : CommandClass::LOCAL;
}

bool VectorControlServiceProvider::isCommandValid(const GlobalVectorCommandType& cmd) {
  if (safetyGate_ != nullptr && !safetyGate_->commandsAllowed()) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Vector command rejected: the safety supervisor holds "
      "the vehicle (recovery/safe mode)")
    return false;
  }
  if (modeGate_ != nullptr) {
    // Held sessions bypass the validation-stage mode check: an authoritative flush must fail
    // them with INTERRUPTED (in onIssued), never VALIDATION_FAILED.
    const bool heldSession = held_ && heldSessionId_ == arlcore::NumericGuid(cmd.sessionID());
    if (!heldSession && modeGate_->rejectedAtValidation(classOf(cmd))) {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                    "Vector command rejected: not permitted in the current operational mode")
      return false;
    }
  }
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

CommandStateResult VectorControlServiceProvider::onIssued(const std::weak_ptr<CmdSession> session) {
  auto cmdSession = session.lock();
  if (!cmdSession) {
    return CommandStateResult::ERROR;
  }
  if (modeGate_ == nullptr) {
    return CommandStateResult::ADVANCE;
  }
  const GlobalVectorCommandType cmd = cmdSession->getCommand();
  const CommandClass cls = classOf(cmd);
  if (held_ && heldSessionId_ == arlcore::NumericGuid(cmd.sessionID()) &&
      heldEpoch_ != modeGate_->authoritativeEpoch() && !modeGate_->classAllowed(cls)) {
    // An explicit mode command or manual engagement occurred while held: flush the hold.
    // INTERRUPTED is legal from ISSUED; the base reaps the session once it sees the state.
    if (!cmdSession->fail(CommandStatusReasonEnumType::INTERRUPTED)) {
      return CommandStateResult::ERROR;
    }
    relinquish();
    cmdSession->sendStatus("Interrupted: an authoritative mode change disallowed the held command");
    return CommandStateResult::OK;
  }
  if (modeGate_->requestAdmission(cls) == AdmissionDecision::HOLD) {
    if (!held_ || heldSessionId_ != arlcore::NumericGuid(cmd.sessionID())) {
      // The session may have been EXECUTING before an update re-issued it: release the
      // driving resource and clear the installed setpoint so nothing keeps driving
      // out-of-mode while it waits.
      relinquish();
      held_ = true;
      heldSessionId_ = arlcore::NumericGuid(cmd.sessionID());
      heldEpoch_ = modeGate_->authoritativeEpoch();
      UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                    "Vector command held at ISSUED until the operational mode permits it")
    }
    return CommandStateResult::OK;
  }
  held_ = false;
  return CommandStateResult::ADVANCE;
}

CommandStateResult VectorControlServiceProvider::onCommanded(const std::weak_ptr<CmdSession> session) {
  auto cmdSession = session.lock();
  if (!cmdSession) {
    return CommandStateResult::ERROR;
  }
  // Vector is the high-priority source: this acquire preempts any active waypoint route.
  if (!autopilot_->arbiter().acquire(DriveSource::VECTOR, classOf(cmdSession->getCommand()))) {
    // RESOURCE_REJECTED is only legal from COMMANDED, so fail directly here (the base
    // reaps the FAILED session) rather than returning ERROR (= SERVICE_FAILED).
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Vector provider failed to acquire driving resource")
    if (!cmdSession->fail(CommandStatusReasonEnumType::RESOURCE_REJECTED)) {
      return CommandStateResult::ERROR;
    }
    cmdSession->sendStatus("Driving resource is held by a higher-priority command");
    return CommandStateResult::OK;
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
  // The base re-runs handleIssued right after this, so validation/mode admission apply to the
  // updated command and the ISSUED->COMMANDED path reinstalls the setpoint. Returning false
  // here would fail EVERY active session with SERVICE_FAILED, so rejection must flow through
  // the re-validation instead.
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
  if (modeGate_ != nullptr) {
    auto cmdSession = session.lock();
    if (cmdSession && !modeGate_->classAllowed(classOf(cmdSession->getCommand()))) {
      return CommandStatusReasonEnumType::INTERRUPTED;
    }
  }
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
