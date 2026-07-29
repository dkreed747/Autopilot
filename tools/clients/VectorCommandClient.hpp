#ifndef AUTOPILOT_TOOLS_CLIENTS_VECTORCOMMANDCLIENT_HPP_
#define AUTOPILOT_TOOLS_CLIENTS_VECTORCOMMANDCLIENT_HPP_

#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandAckReportType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandStatusType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorExecutionStatusReportType.hpp>
#include <chrono>
#include <dds/dds.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "clients/ClientIdentity.hpp"
#include "CycloneReader.h"
#include "CycloneSender.h"
#include "NumericGuid.h"
#include "clients/WaypointMissionClient.hpp"

namespace arlcore::autopilot::tools {

//! \brief One vector setpoint as the GUI expresses it.
struct VectorSetpoint {
  double headingRad = 0.0;
  double speedMps = 0.0;
  std::optional<double> elevValueM;
  std::string elevFrame = "depth";  // "depth" | "asf"
  std::optional<double>
      timeoutS;  // maps to endTime = now + timeoutS (provider auto-completes)
};

//! \brief The UMAA consumer side of the Global Vector control service:
//! publishes a heading / speed / elevation setpoint, updates it in place (same
//! session), cancels by disposing the command instance, and tracks ack / status
//! / execution status. One client drives at most one session at a time.
class VectorCommandClient {
 public:
  using CommandType = UMAA::MO::GlobalVectorControl::GlobalVectorCommandType;
  using AckType =
      UMAA::MO::GlobalVectorControl::GlobalVectorCommandAckReportType;
  using StatusType =
      UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusType;
  using ExecType =
      UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportType;

  //! \brief `destinationId` is the vector provider's source ID
  //! (identity.vector_source_id).
  VectorCommandClient(const dds::domain::DomainParticipant& participant,
                      const dds::pub::qos::DataWriterQos& wqos,
                      const dds::sub::qos::DataReaderQos& rqos,
                      const arlcore::NumericGuid& destinationId,
                      const ClientIdentity& identity);

  //! \brief Publish a new command session for this setpoint (the provider
  //! replaces any in-flight vector command). Returns the new session ID.
  arlcore::NumericGuid start(const VectorSetpoint& setpoint);

  //! \brief Re-send the active session with a new setpoint (UMAA update
  //! semantics). The end time, when a timeout is set, is re-anchored at now.
  bool update(const VectorSetpoint& setpoint);

  //! \brief Cancel the active session by disposing the command instance.
  bool cancel();

  //! \brief Drain new command-status samples for the active session.
  std::vector<MissionStatusUpdate> pollStatus();

  //! \brief True once the session's command acknowledgement has been received.
  bool pollAck();

  //! \brief Latest execution-status for the active session, if any (age in
  //! seconds).
  std::optional<ExecType> pollExec();
  std::optional<double> execAgeS() const;

  bool active() const { return sessionId_.has_value() && !terminal_; }
  const std::optional<arlcore::NumericGuid>& sessionId() const {
    return sessionId_;
  }
  const std::optional<VectorSetpoint>& lastSetpoint() const {
    return lastSetpoint_;
  }
  bool ackReceived() const { return ackReceived_; }

 private:
  std::shared_ptr<arlcore::io::CycloneSender<CommandType>> cmdSender_;
  std::shared_ptr<arlcore::io::CycloneReader<AckType>> ackReader_;
  std::shared_ptr<arlcore::io::CycloneReader<StatusType>> statusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<ExecType>> execReader_;

  ClientIdentity identity_;
  arlcore::NumericGuid destinationId_;
  CommandType cmd_;
  std::optional<arlcore::NumericGuid> sessionId_;
  std::optional<VectorSetpoint> lastSetpoint_;
  bool terminal_ = true;
  bool ackReceived_ = false;
  std::optional<ExecType> lastExec_;
  std::chrono::steady_clock::time_point execAt_;
};

}  // namespace arlcore::autopilot::tools
#endif  // AUTOPILOT_TOOLS_CLIENTS_VECTORCOMMANDCLIENT_HPP_
