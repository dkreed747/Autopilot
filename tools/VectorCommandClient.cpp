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

#include "VectorCommandClient.hpp"

#include <cmath>

#include "MissionRoute.h"
#include "UmaaUtils.h"
#include "UuidFactory.h"

namespace arlcore::autopilot::tools {

using arlcore::io::CycloneReader;
using arlcore::io::CycloneSender;
using arlcore::io::ReadStatus;
using arlcore::io::SendStatus;
using StatusEnum = UMAA::Common::MaritimeEnumeration::CommandStatusEnumModule::
    CommandStatusEnumType;

//! \brief Build the UMAA command for a setpoint: a true-north HEADING direction
//! requirement, a required ground speed, an optional depth/ASF elevation, and
//! an optional end time.
static void fillVectorCommand(
    const VectorSetpoint& setpoint,
    UMAA::MO::GlobalVectorControl::GlobalVectorCommandType* cmd) {
  UMAA::Common::Orientation::DirectionTrueNorthRequirementVariantType dir;
  dir.direction().direction(setpoint.headingRad);
  cmd->direction()
      .DirectionRequirementVariantTypeSubtypes()
      .DirectionTrueNorthRequirementVariantVariant(dir);
  cmd->directionMode() = UMAA::Common::MaritimeEnumeration::
      DirectionModeEnumModule::DirectionModeEnumType::HEADING;

  UMAA::Common::Speed::GroundSpeedRequirementVariantType speed;
  speed.speed().speed(setpoint.speedMps);
  cmd->speed()
      .SpeedRequirementVariantTypeSubtypes()
      .GroundSpeedRequirementVariantVariant(speed);

  if (setpoint.elevValueM.has_value()) {
    UMAA::Common::Measurement::ElevationRequirementVariantType elev;
    if (setpoint.elevFrame == "asf") {
      elev.ElevationRequirementVariantTypeSubtypes()
          .AltitudeASFRequirementVariantVariant(
              UMAA::Common::Measurement::AltitudeASFRequirementVariantType());
      elev.ElevationRequirementVariantTypeSubtypes()
          .AltitudeASFRequirementVariantVariant()
          .altitude()
          .altitude(setpoint.elevValueM.value());
    } else {
      elev.ElevationRequirementVariantTypeSubtypes()
          .DepthRequirementVariantVariant(
              UMAA::Common::Measurement::DepthRequirementVariantType());
      elev.ElevationRequirementVariantTypeSubtypes()
          .DepthRequirementVariantVariant()
          .depth()
          .depth(setpoint.elevValueM.value());
    }
    cmd->elevation() = elev;
  } else {
    cmd->elevation().reset();
  }

  if (setpoint.timeoutS.has_value() &&
      std::isfinite(setpoint.timeoutS.value()) &&
      setpoint.timeoutS.value() > 0.0) {
    UMAA::Common::Measurement::DateTime endTime = arlcore::umaa::getTimestamp();
    endTime.seconds() +=
        static_cast<int64_t>(std::ceil(setpoint.timeoutS.value()));
    cmd->endTime() = endTime;
  } else {
    cmd->endTime().reset();
  }
}

VectorCommandClient::VectorCommandClient(
    const dds::domain::DomainParticipant& participant,
    const dds::pub::qos::DataWriterQos& wqos,
    const dds::sub::qos::DataReaderQos& rqos,
    const arlcore::NumericGuid& destinationId, const ClientIdentity& identity)
    : cmdSender_(std::make_shared<CycloneSender<CommandType>>(
          participant,
          UMAA::MO::GlobalVectorControl::GlobalVectorCommandTypeTopic, wqos)),
      ackReader_(std::make_shared<CycloneReader<AckType>>(
          participant,
          UMAA::MO::GlobalVectorControl::GlobalVectorCommandAckReportTypeTopic,
          rqos)),
      statusReader_(std::make_shared<CycloneReader<StatusType>>(
          participant,
          UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusTypeTopic,
          rqos)),
      execReader_(std::make_shared<CycloneReader<ExecType>>(
          participant,
          UMAA::MO::GlobalVectorControl::
              GlobalVectorExecutionStatusReportTypeTopic,
          rqos)),
      identity_(identity),
      destinationId_(destinationId) {}

arlcore::NumericGuid VectorCommandClient::start(
    const VectorSetpoint& setpoint) {
  const arlcore::NumericGuid sessionId =
      arlcore::UuidFactory::getInstance().generateGuid();
  cmd_ = CommandType();
  fillVectorCommand(setpoint, &cmd_);
  cmd_.sessionID() = sessionId.getGuid();
  cmd_.source().id() = identity_.sourceId.getGuid();
  cmd_.source().parentID() = identity_.platformId.getGuid();
  cmd_.destination().id() = destinationId_.getGuid();
  cmd_.timeStamp() = arlcore::umaa::getTimestamp();

  sessionId_ = sessionId;
  lastSetpoint_ = setpoint;
  terminal_ = false;
  ackReceived_ = false;
  lastExec_.reset();
  if (cmdSender_->send(cmd_) != SendStatus::SUCCESS) {
    terminal_ = true;
  }
  return sessionId;
}

bool VectorCommandClient::update(const VectorSetpoint& setpoint) {
  if (!active()) {
    return false;
  }
  fillVectorCommand(setpoint, &cmd_);
  cmd_.timeStamp() = arlcore::umaa::getTimestamp();
  lastSetpoint_ = setpoint;
  return cmdSender_->send(cmd_) == SendStatus::SUCCESS;
}

bool VectorCommandClient::cancel() {
  if (!sessionId_.has_value() || terminal_) {
    return false;
  }
  return cmdSender_->dispose(cmd_) == SendStatus::SUCCESS;
}

std::vector<MissionStatusUpdate> VectorCommandClient::pollStatus() {
  std::vector<MissionStatusUpdate> updates;
  if (!sessionId_.has_value()) {
    return updates;
  }
  StatusType status;
  while (statusReader_->read(&status) == ReadStatus::SUCCESS) {
    if (arlcore::NumericGuid(status.sessionID()) != sessionId_.value()) {
      continue;
    }
    MissionStatusUpdate update;
    update.status = statusName(status.commandStatus());
    update.reason = statusReasonName(status.commandStatusReason());
    update.logMessage = status.logMessage();
    update.terminal = status.commandStatus() == StatusEnum::COMPLETED ||
                      status.commandStatus() == StatusEnum::FAILED ||
                      status.commandStatus() == StatusEnum::CANCELED;
    if (update.terminal) {
      terminal_ = true;
      // Dispose the (transient-local) command instance so a restarted autopilot can never
      // re-read and re-execute a finished vector from the retained sample.
      cmdSender_->dispose(cmd_);
    }
    updates.push_back(update);
  }
  return updates;
}

bool VectorCommandClient::pollAck() {
  if (!sessionId_.has_value()) {
    return false;
  }
  AckType ack;
  while (ackReader_->read(&ack) == ReadStatus::SUCCESS) {
    if (arlcore::NumericGuid(ack.sessionID()) == sessionId_.value()) {
      ackReceived_ = true;
    }
  }
  return ackReceived_;
}

std::optional<VectorCommandClient::ExecType> VectorCommandClient::pollExec() {
  if (!sessionId_.has_value()) {
    return lastExec_;
  }
  ExecType exec;
  while (execReader_->read(&exec) == ReadStatus::SUCCESS) {
    if (arlcore::NumericGuid(exec.sessionID()) == sessionId_.value()) {
      lastExec_ = exec;
      execAt_ = std::chrono::steady_clock::now();
    }
  }
  return lastExec_;
}

std::optional<double> VectorCommandClient::execAgeS() const {
  if (!lastExec_.has_value()) {
    return std::nullopt;
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       execAt_)
      .count();
}

}  // namespace arlcore::autopilot::tools
