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

#include "WaypointMissionClient.h"

#include <string>
#include <vector>

#include "MissionRoute.h"
#include "UmaaUtils.h"
#include "UuidFactory.h"

namespace arlcore::autopilot::tools {

using arlcore::io::CycloneReader;
using arlcore::io::CycloneSender;
using arlcore::io::ReadStatus;
using arlcore::io::SendStatus;
using CommandStatusEnumType =
    UMAA::Common::MaritimeEnumeration::CommandStatusEnumModule::CommandStatusEnumType;

WaypointMissionClient::WaypointMissionClient(const dds::domain::DomainParticipant& participant,
                                             const dds::pub::qos::DataWriterQos& wqos,
                                             const dds::sub::qos::DataReaderQos& rqos,
                                             const arlcore::NumericGuid& destinationId)
    : cmdSender_(std::make_shared<CycloneSender<CommandType>>(
          participant, UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandTypeTopic, wqos)),
      elementSender_(std::make_shared<CycloneSender<ListElement>>(
          participant,
          UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandTypeWaypointsListElementTopic, wqos)),
      statusReader_(std::make_shared<CycloneReader<CommandStatusType>>(
          participant, UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandStatusTypeTopic, rqos)),
      ackReader_(std::make_shared<CycloneReader<CommandAckType>>(
          participant, UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandAckReportTypeTopic, rqos)),
      sourceId_(arlcore::UuidFactory::getInstance().generateGuid()),
      destinationId_(destinationId) {}

arlcore::NumericGuid WaypointMissionClient::start(const std::vector<GlobalWaypointType>& waypoints) {
  // Retire the previous route first: the writer's destructor disposes its elements.
  listWriter_.reset();
  listWriter_.emplace(elementSender_, waypoints);
  waypoints_ = waypoints;

  const arlcore::NumericGuid sessionId = arlcore::UuidFactory::getInstance().generateGuid();
  cmd_ = CommandType();
  cmd_.sessionID() = sessionId.getGuid();
  cmd_.source().id() = sourceId_.getGuid();
  cmd_.destination().id() = destinationId_.getGuid();
  cmd_.timeStamp() = arlcore::umaa::getTimestamp();
  cmd_.waypointsListMetadata() = listWriter_->getMetadata();

  sessionId_ = sessionId;
  terminal_ = false;
  ackReceived_ = false;
  lastStatus_.clear();
  lastReason_.clear();
  if (cmdSender_->send(cmd_) != SendStatus::SUCCESS) {
    terminal_ = true;
    lastStatus_ = "SEND_FAILED";
  }
  return sessionId;
}

bool WaypointMissionClient::cancel() {
  if (!sessionId_.has_value() || terminal_) {
    return false;
  }
  return cmdSender_->dispose(cmd_) == SendStatus::SUCCESS;
}

std::vector<MissionStatusUpdate> WaypointMissionClient::pollStatus() {
  std::vector<MissionStatusUpdate> updates;
  if (!sessionId_.has_value()) {
    return updates;
  }
  CommandStatusType status;
  while (statusReader_->read(&status) == ReadStatus::SUCCESS) {
    if (arlcore::NumericGuid(status.sessionID()) != sessionId_.value()) {
      continue;
    }
    MissionStatusUpdate update;
    update.status = statusName(status.commandStatus());
    update.reason = statusReasonName(status.commandStatusReason());
    update.logMessage = status.logMessage();
    update.terminal = status.commandStatus() == CommandStatusEnumType::COMPLETED ||
                      status.commandStatus() == CommandStatusEnumType::FAILED ||
                      status.commandStatus() == CommandStatusEnumType::CANCELED;
    lastStatus_ = update.status;
    lastReason_ = update.reason;
    if (update.terminal) {
      terminal_ = true;
    }
    updates.push_back(update);
  }
  return updates;
}

bool WaypointMissionClient::pollAck() {
  if (!sessionId_.has_value() || ackReceived_) {
    return ackReceived_;
  }
  CommandAckType ack;
  while (ackReader_->read(&ack) == ReadStatus::SUCCESS) {
    if (arlcore::NumericGuid(ack.sessionID()) == sessionId_.value()) {
      ackReceived_ = true;
    }
  }
  return ackReceived_;
}

}  // namespace arlcore::autopilot::tools
