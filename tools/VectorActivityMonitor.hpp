#ifndef APPS_AUTOPILOT_TOOLS_VECTORACTIVITYMONITOR_HPP_
#define APPS_AUTOPILOT_TOOLS_VECTORACTIVITYMONITOR_HPP_

#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandStatusType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorExecutionStatusReportType.hpp>
#include <chrono>
#include <dds/dds.hpp>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "CycloneReader.h"
#include "NumericGuid.h"

namespace arlcore::autopilot::tools {

//! \brief One vector command observed on the bus, with its variants decoded to
//! plain values (unsupported variants leave the fields unset).
struct ObservedVector {
  arlcore::NumericGuid sessionId;
  arlcore::NumericGuid sourceId;
  arlcore::NumericGuid sourceParentId;
  std::optional<double> headingRad;
  std::optional<double> speedMps;
  std::optional<double> elevValueM;
  std::string elevFrame;  // "depth" | "asf" | "" when unset/unsupported
  std::optional<UMAA::Common::Measurement::DateTime> endTime;
  std::string lastStatus;
  std::string lastReason;
  bool terminal = false;
  std::optional<
      UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportType>
      execStatus;
  std::chrono::steady_clock::time_point lastSeen;
  std::chrono::steady_clock::time_point execSeen;
};

//! \brief Read-only bus-wide observer of Global Vector commands, whoever
//! commanded them. Vector sessions churn faster than missions (RC updates), so
//! eviction is more aggressive.
class VectorActivityMonitor {
 public:
  using CommandType = UMAA::MO::GlobalVectorControl::GlobalVectorCommandType;
  using StatusType =
      UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusType;
  using ExecType =
      UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportType;

  VectorActivityMonitor(const dds::domain::DomainParticipant& participant,
                        const dds::sub::qos::DataReaderQos& rqos);

  //! \brief Drain all readers and reconcile the observed-vector table (once per
  //! poller tick).
  void poll();

  const std::map<arlcore::NumericGuid, ObservedVector>& vectors() const {
    return vectors_;
  }

 private:
  std::shared_ptr<arlcore::io::CycloneReader<CommandType>> cmdReader_;
  std::shared_ptr<arlcore::io::CycloneReader<StatusType>> statusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<ExecType>> execReader_;

  std::map<arlcore::NumericGuid, ObservedVector> vectors_;
};

}  // namespace arlcore::autopilot::tools
#endif  // APPS_AUTOPILOT_TOOLS_VECTORACTIVITYMONITOR_HPP_
