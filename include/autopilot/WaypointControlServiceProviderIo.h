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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_WAYPOINTCONTROLSERVICEPROVIDERIO_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_WAYPOINTCONTROLSERVICEPROVIDERIO_H_

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
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_WAYPOINTCONTROLSERVICEPROVIDERIO_H_
