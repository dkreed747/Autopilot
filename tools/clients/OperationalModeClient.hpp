#ifndef AUTOPILOT_TOOLS_CLIENTS_OPERATIONALMODECLIENT_HPP_
#define AUTOPILOT_TOOLS_CLIENTS_OPERATIONALMODECLIENT_HPP_

#include <UMAA/MM/OperationalModeControl/OperationalModeCommandAckReportType.hpp>
#include <UMAA/MM/OperationalModeControl/OperationalModeCommandStatusType.hpp>
#include <UMAA/MM/OperationalModeControl/OperationalModeCommandType.hpp>
#include <UMAA/MM/OperationalModeStatus/OperationalModeReportType.hpp>
#include <chrono>
#include <dds/dds.hpp>
#include <memory>
#include <optional>
#include <string>

#include "CycloneReader.h"
#include "CycloneSender.h"
#include "InternalTypes.h"
#include "NumericGuid.h"
#include "clients/ClientIdentity.hpp"

namespace arlcore::autopilot::tools {

//! \brief The latest terminal-or-not command activity of the mode client's
//! session.
struct ModeStatusEntry {
  std::string status;
  std::string reason;
  std::string message;
};

//! \brief Consumer side of the UMAA MM OperationalModeControl/Status services:
//! commands STANDBY / REMOTE / AUTONOMOUS and tracks the vehicle's reported
//! operational mode. One client drives at most one mode-command session at a
//! time.
class OperationalModeClient {
 public:
  using CommandType = UMAA::MM::OperationalModeControl::OperationalModeCommandType;
  using AckType = UMAA::MM::OperationalModeControl::OperationalModeCommandAckReportType;
  using StatusType = UMAA::MM::OperationalModeControl::OperationalModeCommandStatusType;
  using ReportType = UMAA::MM::OperationalModeStatus::OperationalModeReportType;

  //! \brief `destinationId` is identity.operational_mode_control_source_id.
  OperationalModeClient(const dds::domain::DomainParticipant& participant, const dds::pub::qos::DataWriterQos& wqos,
                        const dds::sub::qos::DataReaderQos& rqos, const arlcore::NumericGuid& destinationId,
                        const ClientIdentity& identity);

  //! \brief Command a mode ("STANDBY" | "REMOTE" | "AUTONOMOUS"); returns the
  //! session id, nullopt on an unknown mode string or send failure.
  std::optional<arlcore::NumericGuid> command(const std::string& mode);

  //! \brief Drain the report / ack / status readers (call once per poller
  //! tick).
  void poll();

  //! \brief The last reported mode name (MANUAL/STANDBY/REMOTE/AUTONOMOUS);
  //! nullopt before the first report.
  const std::optional<std::string>& reportedMode() const { return reportedMode_; }

  //! \brief Seconds since the last mode report; nullopt before the first
  //! report.
  std::optional<flt64_t> reportAgeS() const;

  //! \brief The mode most recently commanded, until its session reaches a
  //! terminal status.
  const std::optional<std::string>& pendingMode() const { return pendingMode_; }

  bool ackReceived() const { return ackReceived_; }
  const std::optional<ModeStatusEntry>& lastStatus() const { return lastStatus_; }

 private:
  std::shared_ptr<arlcore::io::CycloneSender<CommandType>> cmdSender_;
  std::shared_ptr<arlcore::io::CycloneReader<AckType>> ackReader_;
  std::shared_ptr<arlcore::io::CycloneReader<StatusType>> statusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<ReportType>> reportReader_;

  ClientIdentity identity_;
  arlcore::NumericGuid destinationId_;
  CommandType cmd_;
  std::optional<arlcore::NumericGuid> sessionId_;
  std::optional<std::string> pendingMode_;
  bool ackReceived_ = false;
  std::optional<ModeStatusEntry> lastStatus_;
  std::optional<std::string> reportedMode_;
  std::chrono::steady_clock::time_point reportAt_;
};

}  // namespace arlcore::autopilot::tools
#endif  // AUTOPILOT_TOOLS_CLIENTS_OPERATIONALMODECLIENT_HPP_
