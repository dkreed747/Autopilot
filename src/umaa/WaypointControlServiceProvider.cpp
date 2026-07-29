#include "autopilot/umaa/WaypointControlServiceProvider.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "LargeList.h"
#include "Logger.h"
#include "autopilot/guidance/ToleranceUtils.hpp"
#include "UmaaUtils.h"
#include "InternalTypes.h"

namespace arlcore::autopilot {

using arlcore::umaa::services::CommandStateResult;
using arlcore::umaa::services::IncomingCommandBehavior;
using arlcore::umaa::LargeListStatus;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;

//! \brief A DateTime `secondsAhead` seconds in the future (clamped to now for non-finite or
//! negative inputs).
static UMAA::Common::Measurement::DateTime timestampPlus(flt64_t secondsAhead) {
  UMAA::Common::Measurement::DateTime t = arlcore::umaa::getTimestamp();
  if (std::isfinite(secondsAhead) && secondsAhead > 0.0) {
    t.seconds() += static_cast<int64_t>(secondsAhead);
  }
  return t;
}

WaypointControlServiceProvider::WaypointControlServiceProvider(
    const arlcore::NumericGuid& source, std::shared_ptr<WaypointControlServiceProviderIo> io,
    IAutopilot* autopilot, flt64_t maxForwardSpeedMps, int32_t maxListWaitCycles,
    const ISafetyGate* safetyGate, const ZoneMap* zoneMap, ICommandModeGate* modeGate) :
    CommandProviderBase(source, io),
    sourceId_(source),
    autopilot_(autopilot),
    wpIo_(io),
    listReader_(io->listElementReader),
    maxForwardSpeedMps_(maxForwardSpeedMps),
    maxListWaitCycles_(maxListWaitCycles),
    safetyGate_(safetyGate),
    zoneMap_(zoneMap),
    modeGate_(modeGate) {
  // A new waypoint route replaces an in-flight route (same driving resource).
  setBehavior(IncomingCommandBehavior::CANCEL_EXISTING);
}

CommandClass WaypointControlServiceProvider::classOf(const GlobalWaypointCommandType& cmd) const {
  return modeGate_ != nullptr ? modeGate_->classify(cmd.source()) : CommandClass::LOCAL;
}

void WaypointControlServiceProvider::resetPlanningState() {
  acquired_ = false;
  planned_ = false;
  listWaitCycles_ = 0;
}

void WaypointControlServiceProvider::relinquish(const std::weak_ptr<CmdSession> session) {
  autopilot_->clearSetpoint(DriveSource::WAYPOINT);
  autopilot_->arbiter().release(DriveSource::WAYPOINT);
  if (auto s = session.lock()) {
    listReader_.removeListByMetadata(s->getCommand().waypointsListMetadata());
  }
  resetPlanningState();
  sessionActive_ = false;
  held_ = false;
}

CommandStateResult WaypointControlServiceProvider::failInCommanded(
    const std::weak_ptr<CmdSession> session, CommandStatusReasonEnumType reason,
    const std::string& logMessage) {
  auto s = session.lock();
  if (!s) {
    return CommandStateResult::ERROR;
  }
  // Fail directly from COMMANDED: reasons like RESOURCE_REJECTED are only legal from this
  // state (CommandStateMachine), so they cannot be routed through isCommandFailed() in
  // EXECUTING. The base reaps the session once it observes the FAILED state.
  if (!s->fail(reason)) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Unable to fail waypoint session " << s->getSessionId()
      << " with reason " << reason)
    return CommandStateResult::ERROR;
  }
  relinquish(session);
  s->sendStatus(logMessage);
  s->sendExecutionStatus();
  return CommandStateResult::OK;
}

bool WaypointControlServiceProvider::validateWaypoints(
    const std::vector<GlobalWaypointType>& waypoints) const {
  if (waypoints.empty()) {
    return false;
  }
  for (const GlobalWaypointType& wp : waypoints) {
    const std::optional<SpeedValue> sp = tolerance::extractSpeed(wp.speed());
    if (!sp.has_value()) {
      // RECOMMENDED / TIME-WITH-SPEED variants are unsupported: accepting them would drive
      // the route at 0 m/s and hang the command in EXECUTING.
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Waypoint speed variant is unsupported (require a "
        "REQUIRED ground/water speed)")
      return false;
    }
    if (maxForwardSpeedMps_ > 0.0 && sp->speedMps > maxForwardSpeedMps_) {
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Waypoint speed " << sp->speedMps
        << " exceeds platform max forward speed " << maxForwardSpeedMps_)
      return false;
    }
  }
  return true;
}

bool WaypointControlServiceProvider::isCommandValid(const GlobalWaypointCommandType& cmd) {
  if (safetyGate_ != nullptr && !safetyGate_->commandsAllowed()) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Waypoint command rejected: the safety supervisor holds "
      "the vehicle (recovery/safe mode)")
    return false;
  }
  if (modeGate_ != nullptr) {
    // Held sessions bypass the validation-stage mode check: an authoritative flush must fail
    // them with INTERRUPTED (in onIssued), never VALIDATION_FAILED.
    const bool heldSession = held_ && heldSessionId_ == arlcore::NumericGuid(cmd.sessionID());
    if (!heldSession && modeGate_->rejectedAtValidation(classOf(cmd))) {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                    "Waypoint command rejected: not permitted in the current operational mode")
      return false;
    }
  }
  return true;
}

CommandStateResult WaypointControlServiceProvider::onIssued(const std::weak_ptr<CmdSession> session) {
  auto s = session.lock();
  if (!s) {
    return CommandStateResult::ERROR;
  }
  if (modeGate_ == nullptr) {
    return CommandStateResult::ADVANCE;
  }
  const GlobalWaypointCommandType cmd = s->getCommand();
  const CommandClass cls = classOf(cmd);
  if (held_ && heldSessionId_ == arlcore::NumericGuid(cmd.sessionID()) &&
      heldEpoch_ != modeGate_->authoritativeEpoch() && !modeGate_->classAllowed(cls)) {
    // An explicit mode command or manual engagement occurred while held: flush the hold.
    // INTERRUPTED is legal from ISSUED; the base reaps the session once it sees the state.
    if (!s->fail(CommandStatusReasonEnumType::INTERRUPTED)) {
      return CommandStateResult::ERROR;
    }
    relinquish(session);
    s->sendStatus("Interrupted: an authoritative mode change disallowed the held command");
    return CommandStateResult::OK;
  }
  if (modeGate_->requestAdmission(cls) == AdmissionDecision::HOLD) {
    if (!held_ || heldSessionId_ != arlcore::NumericGuid(cmd.sessionID())) {
      // The session may have been COMMANDED/EXECUTING before an update re-issued it:
      // release the driving resource and clear the installed route so nothing keeps
      // driving out-of-mode while it waits. The large list is deliberately kept (its
      // elements were already consumed from the shared reader and could never be
      // re-derived after a removal) so a later release can replan from it.
      autopilot_->clearSetpoint(DriveSource::WAYPOINT);
      autopilot_->arbiter().release(DriveSource::WAYPOINT);
      resetPlanningState();
      sessionActive_ = false;
      held_ = true;
      heldSessionId_ = arlcore::NumericGuid(cmd.sessionID());
      heldEpoch_ = modeGate_->authoritativeEpoch();
      UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                    "Waypoint command held at ISSUED until the operational mode permits it")
    }
    return CommandStateResult::OK;
  }
  held_ = false;
  return CommandStateResult::ADVANCE;
}

bool WaypointControlServiceProvider::waypointsZoneCompliant(
    const std::vector<GlobalWaypointType>& waypoints, std::string* message) const {
  if (zoneMap_ == nullptr || !zoneMap_->hasZones()) {
    return true;
  }
  const flt64_t marginM = zoneMap_->config().safetyMarginM;
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const GlobalWaypointType& wp = waypoints[i];
    const GeoPoint at{wp.position().value().geodeticLatitude(),
                      wp.position().value().geodeticLongitude()};
    // Gate at the waypoint's commanded vertical position. A depth elevation gives an exact
    // depth; an above-sea-floor elevation gives an exact ASF but an unknown depth (no
    // bathymetry), so the depth interval widens to everything — conservative. No elevation
    // means the surface.
    ElevationEnvelope envelope = ElevationEnvelope::atPoint(0.0);
    if (wp.elevation().has_value()) {
      const std::optional<ElevationValue> el = tolerance::extractElevation(wp.elevation().value());
      if (el.has_value() && el->frame == ElevationFrame::DEPTH) {
        envelope = ElevationEnvelope::atPoint(el->valueM);
      } else if (el.has_value() && el->frame == ElevationFrame::ALTITUDE_ASF) {
        envelope.minDepthM = -1.0e9;
        envelope.maxDepthM = 1.0e9;
        envelope.minAsfM = el->valueM;
        envelope.maxAsfM = el->valueM;
      }
    }
    if (zoneMap_->clearanceM(at, envelope) < marginM) {
      if (message != nullptr) {
        *message = "Waypoint " + std::to_string(i + 1) + " violates an active water zone";
      }
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Waypoint " << (i + 1) << " of " << waypoints.size()
        << " violates an active water zone (or its safety margin)")
      return false;
    }
  }
  return true;
}

bool WaypointControlServiceProvider::onCycle() {
  // Drain large-list element samples each cycle so the route can be assembled even when the
  // element samples arrive before (or after) the command metadata.
  listReader_.updateListElements();
  return true;
}

CommandStateResult WaypointControlServiceProvider::onCommanded(const std::weak_ptr<CmdSession> session) {
  auto s = session.lock();
  if (!s) {
    return CommandStateResult::ERROR;
  }

  const arlcore::NumericGuid sid = s->getSessionId();
  if (!sessionActive_ || sid != activeSession_) {
    resetPlanningState();
    activeSession_ = sid;
    sessionActive_ = true;
  }

  // The operational mode moved on while the route was being set up (waypoint commands can
  // dwell here for many cycles assembling their large list).
  if (modeGate_ != nullptr && !modeGate_->classAllowed(classOf(s->getCommand()))) {
    return failInCommanded(session, CommandStatusReasonEnumType::INTERRUPTED,
                           "Operational mode changed; the command class is no longer permitted");
  }

  // Lost the resource to a higher-priority (vector) command while we were setting up.
  if (acquired_ && autopilot_->arbiter().wasRevoked(DriveSource::WAYPOINT)) {
    return failInCommanded(session, CommandStatusReasonEnumType::INTERRUPTED,
                           "Preempted by a higher-priority driving command");
  }

  // Acquire the (low-priority) driving resource. Denied if a vector command holds it.
  if (!acquired_) {
    if (autopilot_->arbiter().acquire(DriveSource::WAYPOINT, classOf(s->getCommand()))) {
      acquired_ = true;
    } else {
      return failInCommanded(session, CommandStatusReasonEnumType::RESOURCE_REJECTED,
                             "Driving resource is held by a higher-priority command");
    }
  }

  // Assemble the waypoint route from the large list, waiting until it is complete.
  if (!planned_) {
    const arlcore::umaa::LargeListResult<GlobalWaypointType> result =
        listReader_.getListFromMetadata(s->getCommand().waypointsListMetadata());
    if (result.status == LargeListStatus::VALID_LIST) {
      std::vector<GlobalWaypointType> waypoints;
      if (auto locked = result.list.lock()) {
        waypoints.assign(locked->begin(), locked->end());
      }
      if (!validateWaypoints(waypoints)) {
        // VALIDATION_FAILED is only legal from ISSUED, but the route content is not known
        // until the large list arrives here in COMMANDED; SERVICE_FAILED is the legal reason.
        return failInCommanded(session, CommandStatusReasonEnumType::SERVICE_FAILED,
                               "Waypoint route failed validation");
      }
      std::string zoneMessage;
      if (!waypointsZoneCompliant(waypoints, &zoneMessage)) {
        return failInCommanded(session, CommandStatusReasonEnumType::SERVICE_FAILED, zoneMessage);
      }
      if (!autopilot_->setWaypointSetpoint(waypoints)) {
        return failInCommanded(session, CommandStatusReasonEnumType::SERVICE_FAILED,
                               "No navigation fix available to plan the route");
      }
      planned_ = true;
      return CommandStateResult::ADVANCE;
    }

    // List not yet complete: stay in COMMANDED and retry, up to the wait budget.
    if (++listWaitCycles_ > maxListWaitCycles_) {
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Timed out waiting for waypoint list to complete")
      return failInCommanded(session, CommandStatusReasonEnumType::SERVICE_FAILED,
                             "Timed out waiting for the waypoint list to complete");
    }
    return CommandStateResult::OK;
  }

  return CommandStateResult::ADVANCE;
}

CommandStateResult WaypointControlServiceProvider::onExecuting(const std::weak_ptr<CmdSession> session) {
  // The brain drives off each navigation packet; keep the command executing. Failure (revoked
  // resource / objective failure) is surfaced via isCommandFailed.
  return CommandStateResult::OK;
}

bool WaypointControlServiceProvider::onUpdated(const std::weak_ptr<CmdSession> session,
    const GlobalWaypointCommandType& previousCmd, const GlobalWaypointCommandType& updatedCmd) {
  // Treat an update as a new route: drop the previous large list and replan from the
  // (possibly updated) list next cycle. The driving resource is kept. When the update
  // reuses the SAME list id its elements were already consumed from the shared reader,
  // so removing the list would destroy the updated route too — only drop a switched-away
  // list.
  if (arlcore::NumericGuid(previousCmd.waypointsListMetadata().listID()) !=
      arlcore::NumericGuid(updatedCmd.waypointsListMetadata().listID())) {
    listReader_.removeListByMetadata(previousCmd.waypointsListMetadata());
  }
  planned_ = false;
  listWaitCycles_ = 0;
  return true;
}

bool WaypointControlServiceProvider::isCommandCompleted(const std::weak_ptr<CmdSession> session) {
  return planned_ && autopilot_->waypointProgress().routeComplete;
}

CommandStatusReasonEnumType WaypointControlServiceProvider::isCommandFailed(
    const std::weak_ptr<CmdSession> session) {
  if (modeGate_ != nullptr) {
    auto s = session.lock();
    if (s && !modeGate_->classAllowed(classOf(s->getCommand()))) {
      return CommandStatusReasonEnumType::INTERRUPTED;
    }
  }
  if (autopilot_->arbiter().wasRevoked(DriveSource::WAYPOINT)) {
    return CommandStatusReasonEnumType::INTERRUPTED;
  }
  if (planned_ && autopilot_->waypointProgress().failed) {
    return CommandStatusReasonEnumType::OBJECTIVE_FAILED;
  }
  return CommandStatusReasonEnumType::SUCCEEDED;
}

SendStatus WaypointControlServiceProvider::sendExecutionStatus(const GlobalWaypointCommandType& cmd) {
  if (!io_->cmdExeStatusSender.has_value()) {
    return SendStatus::SUCCESS;
  }
  const WaypointProgress prog = autopilot_->waypointProgress();
  GlobalWaypointExecutionStatusReportType report;
  report.sessionID() = cmd.sessionID();
  report.source().id() = sourceId_.getGuid();
  report.timeStamp() = arlcore::umaa::getTimestamp();
  report.positionAchieved() = prog.positionAchieved;
  if (prog.attitudeAchieved.has_value()) {
    report.attitudeAchieved() = prog.attitudeAchieved.value();
  }
  report.elevationAchieved() = prog.elevationAchieved;
  report.speedAchieved() = prog.speedAchieved;
  report.trackLineAchieved() = prog.trackLineAchieved;
  if (prog.crossTrackErrorM.has_value()) {
    report.crossTrackError() = prog.crossTrackErrorM.value();
  }
  report.distanceToWaypoint() = prog.distanceToWaypointM;
  report.distanceRemaining() = prog.distanceRemainingM;
  report.cumulativeDistance() = prog.cumulativeDistanceM;
  report.waypointsRemaining() = prog.waypointsRemaining;
  report.waypointID() = prog.waypointId.getGuid();
  // ETA estimates from the current ground speed (fall back to "now" when not moving).
  const flt64_t speed = std::max(prog.groundSpeedMps, 0.1);
  report.timeToWaypoint() = timestampPlus(prog.distanceToWaypointM / speed);
  report.arrivalTime() = timestampPlus(prog.distanceRemainingM / speed);
  return io_->cmdExeStatusSender.value()->send(report);
}

SendStatus WaypointControlServiceProvider::disposeExecutionStatus(const GlobalWaypointCommandType& cmd) {
  if (!io_->cmdExeStatusSender.has_value()) {
    return SendStatus::SUCCESS;
  }
  GlobalWaypointExecutionStatusReportType report;
  report.sessionID() = cmd.sessionID();
  report.source().id() = sourceId_.getGuid();
  report.timeStamp() = arlcore::umaa::getTimestamp();
  return io_->cmdExeStatusSender.value()->dispose(report);
}

bool WaypointControlServiceProvider::onCanceled(const std::weak_ptr<CmdSession> session) {
  relinquish(session);
  return true;
}

bool WaypointControlServiceProvider::onFailed(const std::weak_ptr<CmdSession> session) {
  relinquish(session);
  return true;
}

bool WaypointControlServiceProvider::onCompleted(const std::weak_ptr<CmdSession> session) {
  relinquish(session);
  return true;
}

}  // namespace arlcore::autopilot
