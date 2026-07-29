#include "clients/OperationalModeClient.hpp"

#include "InternalTypes.h"
#include "UmaaUtils.h"
#include "UuidFactory.h"
#include "autopilot/guidance/MissionRoute.hpp"

namespace arlcore::autopilot::tools {

using arlcore::io::CycloneReader;
using arlcore::io::CycloneSender;
using arlcore::io::ReadStatus;
using arlcore::io::SendStatus;
using ModeControlEnum =
    UMAA::Common::MaritimeEnumeration::OperationalModeControlEnumModule::OperationalModeControlEnumType;
using ModeReportEnum = UMAA::Common::MaritimeEnumeration::OperationalModeEnumModule::OperationalModeEnumType;
using StatusEnum = UMAA::Common::MaritimeEnumeration::CommandStatusEnumModule::CommandStatusEnumType;

static std::optional<ModeControlEnum> parseModeName(const std::string& mode) {
  if (mode == "STANDBY") {
    return ModeControlEnum::STANDBY;
  }
  if (mode == "REMOTE") {
    return ModeControlEnum::REMOTE;
  }
  if (mode == "AUTONOMOUS") {
    return ModeControlEnum::AUTONOMOUS;
  }
  return std::nullopt;
}

static std::string reportModeName(ModeReportEnum mode) {
  switch (mode) {
    case ModeReportEnum::MANUAL:
      return "MANUAL";
    case ModeReportEnum::REMOTE:
      return "REMOTE";
    case ModeReportEnum::AUTONOMOUS:
      return "AUTONOMOUS";
    case ModeReportEnum::STANDBY:
    default:
      return "STANDBY";
  }
}

OperationalModeClient::OperationalModeClient(const dds::domain::DomainParticipant& participant,
                                             const dds::pub::qos::DataWriterQos& wqos,
                                             const dds::sub::qos::DataReaderQos& rqos,
                                             const arlcore::NumericGuid& destinationId, const ClientIdentity& identity)
    : cmdSender_(std::make_shared<CycloneSender<CommandType>>(
          participant, UMAA::MM::OperationalModeControl::OperationalModeCommandTypeTopic, wqos)),
      ackReader_(std::make_shared<CycloneReader<AckType>>(
          participant, UMAA::MM::OperationalModeControl::OperationalModeCommandAckReportTypeTopic, rqos)),
      statusReader_(std::make_shared<CycloneReader<StatusType>>(
          participant, UMAA::MM::OperationalModeControl::OperationalModeCommandStatusTypeTopic, rqos)),
      reportReader_(std::make_shared<CycloneReader<ReportType>>(
          participant, UMAA::MM::OperationalModeStatus::OperationalModeReportTypeTopic, rqos)),
      identity_(identity),
      destinationId_(destinationId) {}

std::optional<arlcore::NumericGuid> OperationalModeClient::command(const std::string& mode) {
  const std::optional<ModeControlEnum> requested = parseModeName(mode);
  if (!requested.has_value()) {
    return std::nullopt;
  }
  const arlcore::NumericGuid sessionId = arlcore::UuidFactory::getInstance().generateGuid();
  cmd_ = CommandType();
  cmd_.operationalMode() = requested.value();
  cmd_.sessionID() = sessionId.getGuid();
  cmd_.source().id() = identity_.sourceId.getGuid();
  cmd_.source().parentID() = identity_.platformId.getGuid();
  cmd_.destination().id() = destinationId_.getGuid();
  cmd_.timeStamp() = arlcore::umaa::getTimestamp();
  if (cmdSender_->send(cmd_) != SendStatus::SUCCESS) {
    return std::nullopt;
  }
  sessionId_ = sessionId;
  pendingMode_ = mode;
  ackReceived_ = false;
  lastStatus_.reset();
  return sessionId;
}

void OperationalModeClient::poll() {
  ReportType report;
  if (reportReader_->readLatest(&report) == ReadStatus::SUCCESS) {
    reportedMode_ = reportModeName(report.operationalMode());
    reportAt_ = std::chrono::steady_clock::now();
  }
  if (!sessionId_.has_value()) {
    return;
  }
  AckType ack;
  while (ackReader_->read(&ack) == ReadStatus::SUCCESS) {
    if (arlcore::NumericGuid(ack.sessionID()) == sessionId_.value()) {
      ackReceived_ = true;
    }
  }
  StatusType status;
  while (statusReader_->read(&status) == ReadStatus::SUCCESS) {
    if (arlcore::NumericGuid(status.sessionID()) != sessionId_.value()) {
      continue;
    }
    lastStatus_ = ModeStatusEntry{statusName(status.commandStatus()), statusReasonName(status.commandStatusReason()),
                                  std::string(status.logMessage())};
    const bool terminal = status.commandStatus() == StatusEnum::COMPLETED ||
                          status.commandStatus() == StatusEnum::FAILED ||
                          status.commandStatus() == StatusEnum::CANCELED;
    if (terminal) {
      pendingMode_.reset();
      // Dispose the (transient-local) command instance so a restarted autopilot can never
      // re-apply a stale mode command from the retained sample.
      cmdSender_->dispose(cmd_);
    }
  }
}

std::optional<flt64_t> OperationalModeClient::reportAgeS() const {
  if (!reportedMode_.has_value()) {
    return std::nullopt;
  }
  return std::chrono::duration<flt64_t>(std::chrono::steady_clock::now() - reportAt_).count();
}

}  // namespace arlcore::autopilot::tools
