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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_VECTORCONTROLSERVICEPROVIDERIO_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_VECTORCONTROLSERVICEPROVIDERIO_H_

#include <memory>

#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandAckReportType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandStatusType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorExecutionStatusReportType.hpp>

#include "UmaaCommandProviderIo.h"

namespace arlcore::autopilot {

using UMAA::MO::GlobalVectorControl::GlobalVectorCommandType;
using UMAA::MO::GlobalVectorControl::GlobalVectorCommandAckReportType;
using UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusType;
using UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportType;

//! \brief IO bundle (command reader + ack/status/exe-status senders) for the global vector
//! control service provider.
class VectorControlServiceProviderIo : public arlcore::umaa::domain::UmaaCommandProviderIo<
    GlobalVectorCommandType,
    GlobalVectorCommandAckReportType,
    GlobalVectorCommandStatusType,
    GlobalVectorExecutionStatusReportType> {
 public:
  VectorControlServiceProviderIo(
      std::shared_ptr<arlcore::io::ReaderBase<GlobalVectorCommandType>> commandReader,
      std::shared_ptr<arlcore::io::SenderBase<GlobalVectorCommandAckReportType>> ackSender,
      std::shared_ptr<arlcore::io::SenderBase<GlobalVectorCommandStatusType>> statusSender,
      std::shared_ptr<arlcore::io::SenderBase<GlobalVectorExecutionStatusReportType>> exeStatusSender) :
      UmaaCommandProviderIo(commandReader, ackSender, statusSender, exeStatusSender) {}
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_VECTORCONTROLSERVICEPROVIDERIO_H_
