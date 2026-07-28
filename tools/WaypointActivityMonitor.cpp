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

#include "WaypointActivityMonitor.hpp"

#include <algorithm>
#include <utility>

#include "MissionRoute.h"

namespace arlcore::autopilot::tools {

using arlcore::io::CycloneReader;
using arlcore::io::ReadStatus;
using arlcore::io::SampleEnvelope;
using arlcore::umaa::LargeListStatus;
using StatusEnum = UMAA::Common::MaritimeEnumeration::CommandStatusEnumModule::
    CommandStatusEnumType;

static constexpr uint32_t kMaxCommandDrain = 16;
static constexpr size_t kMaxTrackedMissions = 16;
static constexpr double kTerminalEvictS = 120.0;

WaypointActivityMonitor::WaypointActivityMonitor(
    const dds::domain::DomainParticipant& participant,
    const dds::sub::qos::DataReaderQos& rqos,
    const dds::sub::qos::DataReaderQos& largeListRqos)
    : cmdReader_(std::make_shared<CycloneReader<CommandType>>(
          participant,
          UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandTypeTopic,
          rqos)),
      statusReader_(std::make_shared<CycloneReader<StatusType>>(
          participant,
          UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandStatusTypeTopic,
          rqos)),
      execReader_(std::make_shared<CycloneReader<ExecType>>(
          participant,
          UMAA::MO::GlobalWaypointControl::
              GlobalWaypointExecutionStatusReportTypeTopic,
          rqos)),
      listReader_(std::make_shared<CycloneReader<ListElement>>(
          participant,
          UMAA::MO::GlobalWaypointControl::
              GlobalWaypointCommandTypeWaypointsListElementTopic,
          largeListRqos)) {}

void WaypointActivityMonitor::poll() {
  const auto now = std::chrono::steady_clock::now();

  // Drain element samples every tick so routes assemble regardless of metadata
  // timing and the shared reader's buffer never overflows.
  listReader_.updateListElements();

  // Command samples: disposed = canceled; valid = new mission or metadata
  // refresh.
  SampleEnvelope<CommandType> envelopes[kMaxCommandDrain];
  const size_t drained = cmdReader_->readUpToN(envelopes, kMaxCommandDrain);
  for (size_t i = 0; i < drained; ++i) {
    const CommandType& cmd = envelopes[i].data;
    const arlcore::NumericGuid session(cmd.sessionID());
    if (envelopes[i].status == ReadStatus::DISPOSED) {
      auto it = missions_.find(session);
      if (it != missions_.end()) {
        it->second.terminal = true;
        it->second.lastSeen = now;
      }
      // The list is kept until eviction: its elements were already consumed from the
      // shared reader, so dropping it here would strand any session sharing the listID.
      continue;
    }
    if (envelopes[i].status != ReadStatus::SUCCESS) {
      continue;
    }
    ObservedMission& mission = missions_[session];
    mission.sessionId = session;
    mission.sourceId = arlcore::NumericGuid(cmd.source().id());
    mission.sourceParentId = arlcore::NumericGuid(cmd.source().parentID());
    mission.lastSeen = now;
    if (mission.terminal) {
      // A late-drained command sample for a finished mission must not wipe its route.
      continue;
    }
    metadataBySession_[session] = cmd.waypointsListMetadata();
    mission.listComplete = false;
    mission.waypoints.clear();
    // Register the (newest) metadata with the list reader exactly once per
    // sample. Retries below must NOT re-submit retained metadata —
    // LargeList::receive rejects metadata older than the newest it has seen —
    // so they re-derive by list id instead.
    listReader_.getListFromMetadata(cmd.waypointsListMetadata());
  }

  // Retry pending routes from the retained elements.
  for (auto& [session, mission] : missions_) {
    if (mission.terminal || mission.listComplete) {
      continue;
    }
    const auto metadataIt = metadataBySession_.find(session);
    if (metadataIt == metadataBySession_.end()) {
      continue;
    }
    const arlcore::umaa::LargeListResult<GlobalWaypointType> result =
        listReader_.getListById(
            arlcore::NumericGuid(metadataIt->second.listID()));
    if (result.status == LargeListStatus::VALID_LIST) {
      if (auto locked = result.list.lock()) {
        mission.waypoints.assign(locked->begin(), locked->end());
        mission.listComplete = true;
      }
    }
  }

  // Status samples update whatever session they belong to. An unmatched session (status
  // drained before its command, e.g. a restart backlog) creates a stub entry rather than
  // silently dropping the consumed sample — the command sample fills in identity/route when
  // (if) it arrives.
  StatusType status;
  while (statusReader_->read(&status) == ReadStatus::SUCCESS) {
    const arlcore::NumericGuid session(status.sessionID());
    ObservedMission& mission = missions_[session];
    mission.sessionId = session;
    mission.lastStatus = statusName(status.commandStatus());
    mission.lastReason = statusReasonName(status.commandStatusReason());
    mission.lastSeen = now;
    if (status.commandStatus() == StatusEnum::COMPLETED ||
        status.commandStatus() == StatusEnum::FAILED ||
        status.commandStatus() == StatusEnum::CANCELED) {
      mission.terminal = true;
    }
  }

  // Execution status routed per session (this replaces the console's old
  // unfiltered reader).
  ExecType exec;
  while (execReader_->read(&exec) == ReadStatus::SUCCESS) {
    auto it = missions_.find(arlcore::NumericGuid(exec.sessionID()));
    if (it == missions_.end()) {
      continue;
    }
    it->second.execStatus = exec;
    it->second.execSeen = now;
  }

  // Eviction: terminal missions age out; a hard cap bounds the snapshot (oldest
  // first).
  for (auto it = missions_.begin(); it != missions_.end();) {
    const double idleS =
        std::chrono::duration<double>(now - it->second.lastSeen).count();
    if (it->second.terminal && idleS > kTerminalEvictS) {
      const auto metadataIt = metadataBySession_.find(it->first);
      if (metadataIt != metadataBySession_.end()) {
        releaseListIfUnshared(it->first, metadataIt->second);
        metadataBySession_.erase(metadataIt);
      }
      it = missions_.erase(it);
    } else {
      ++it;
    }
  }
  while (missions_.size() > kMaxTrackedMissions) {
    auto victim = missions_.begin();
    for (auto it = missions_.begin(); it != missions_.end(); ++it) {
      const bool betterVictim =
          (it->second.terminal && !victim->second.terminal) ||
          (it->second.terminal == victim->second.terminal &&
           it->second.lastSeen < victim->second.lastSeen);
      if (betterVictim) {
        victim = it;
      }
    }
    const auto metadataIt = metadataBySession_.find(victim->first);
    if (metadataIt != metadataBySession_.end()) {
      releaseListIfUnshared(victim->first, metadataIt->second);
      metadataBySession_.erase(metadataIt);
    }
    missions_.erase(victim);
  }
}

void WaypointActivityMonitor::releaseListIfUnshared(
    const arlcore::NumericGuid& session, const UMAA::Common::LargeListMetadata& metadata) {
  const arlcore::NumericGuid listId(metadata.listID());
  for (const auto& [otherSession, otherMetadata] : metadataBySession_) {
    if (otherSession != session && arlcore::NumericGuid(otherMetadata.listID()) == listId) {
      return;  // another tracked session still needs the retained elements
    }
  }
  listReader_.removeListByMetadata(metadata);
}

std::optional<WaypointActivityMonitor::ExecType>
WaypointActivityMonitor::execFor(const arlcore::NumericGuid& sessionId) const {
  const auto it = missions_.find(sessionId);
  if (it == missions_.end()) {
    return std::nullopt;
  }
  return it->second.execStatus;
}

}  // namespace arlcore::autopilot::tools
