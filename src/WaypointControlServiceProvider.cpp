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

#include "WaypointControlServiceProvider.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "LargeList.h"
#include "Logger.h"
#include "ToleranceUtils.h"
#include "UmaaUtils.h"

namespace arlcore::autopilot {

using arlcore::umaa::services::CommandStateResult;
using arlcore::umaa::services::IncomingCommandBehavior;
using arlcore::umaa::LargeListStatus;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;

namespace {
//! \brief A DateTime `secondsAhead` seconds in the future (clamped to now for non-finite or
//! negative inputs).
UMAA::Common::Measurement::DateTime timestampPlus(double secondsAhead) {
  UMAA::Common::Measurement::DateTime t = arlcore::umaa::getTimestamp();
  if (std::isfinite(secondsAhead) && secondsAhead > 0.0) {
    t.seconds() += static_cast<int64_t>(secondsAhead);
  }
  return t;
}
}  // namespace

WaypointControlServiceProvider::WaypointControlServiceProvider(
    const arlcore::NumericGuid& source, std::shared_ptr<WaypointControlServiceProviderIo> io,
    IAutopilot* autopilot, double maxForwardSpeedMps, int maxListWaitCycles) :
    CommandProviderBase(source, io),
    sourceId_(source),
    autopilot_(autopilot),
    wpIo_(io),
    listReader_(io->listElementReader),
    maxForwardSpeedMps_(maxForwardSpeedMps),
    maxListWaitCycles_(maxListWaitCycles) {
  // A new waypoint route replaces an in-flight route (same driving resource).
  setBehavior(IncomingCommandBehavior::CANCEL_EXISTING);
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

  // Lost the resource to a higher-priority (vector) command while we were setting up.
  if (acquired_ && autopilot_->arbiter().wasRevoked(DriveSource::WAYPOINT)) {
    return failInCommanded(session, CommandStatusReasonEnumType::INTERRUPTED,
                           "Preempted by a higher-priority driving command");
  }

  // Acquire the (low-priority) driving resource. Denied if a vector command holds it.
  if (!acquired_) {
    if (autopilot_->arbiter().acquire(DriveSource::WAYPOINT)) {
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
  // (possibly updated) list next cycle. The driving resource is kept.
  listReader_.removeListByMetadata(previousCmd.waypointsListMetadata());
  planned_ = false;
  listWaitCycles_ = 0;
  return true;
}

bool WaypointControlServiceProvider::isCommandCompleted(const std::weak_ptr<CmdSession> session) {
  return planned_ && autopilot_->waypointProgress().routeComplete;
}

CommandStatusReasonEnumType WaypointControlServiceProvider::isCommandFailed(
    const std::weak_ptr<CmdSession> session) {
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
  const double speed = std::max(prog.groundSpeedMps, 0.1);
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
