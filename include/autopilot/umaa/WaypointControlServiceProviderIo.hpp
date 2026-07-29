#ifndef AUTOPILOT_UMAA_WAYPOINTCONTROLSERVICEPROVIDERIO_HPP_
#define AUTOPILOT_UMAA_WAYPOINTCONTROLSERVICEPROVIDERIO_HPP_

#include <memory>

#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandAckReportType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandStatusType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointExecutionStatusReportType.hpp>

#include "UmaaCommandProviderIo.h"

namespace arlcore::autopilot {

using UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandType;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandAckReportType;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandStatusType;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandTypeWaypointsListElement;

//! \brief IO bundle for the global waypoint control service provider. In addition to the base
//! command reader + ack/status/exe-status senders, it carries the reader for the large-list
//! waypoint element topic used to assemble the waypoint route.
class WaypointControlServiceProviderIo : public arlcore::umaa::domain::UmaaCommandProviderIo<
    GlobalWaypointCommandType,
    GlobalWaypointCommandAckReportType,
    GlobalWaypointCommandStatusType,
    GlobalWaypointExecutionStatusReportType> {
 public:
  WaypointControlServiceProviderIo(
      std::shared_ptr<arlcore::io::ReaderBase<GlobalWaypointCommandType>> commandReader,
      std::shared_ptr<arlcore::io::SenderBase<GlobalWaypointCommandAckReportType>> ackSender,
      std::shared_ptr<arlcore::io::SenderBase<GlobalWaypointCommandStatusType>> statusSender,
      std::shared_ptr<arlcore::io::SenderBase<GlobalWaypointExecutionStatusReportType>> exeStatusSender,
      std::shared_ptr<arlcore::io::ReaderBase<GlobalWaypointCommandTypeWaypointsListElement>> listElementReader) :
      UmaaCommandProviderIo(commandReader, ackSender, statusSender, exeStatusSender),
      listElementReader(listElementReader) {}

  const std::shared_ptr<arlcore::io::ReaderBase<GlobalWaypointCommandTypeWaypointsListElement>> listElementReader;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_WAYPOINTCONTROLSERVICEPROVIDERIO_HPP_
