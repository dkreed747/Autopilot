#ifndef AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDERIO_HPP_
#define AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDERIO_HPP_

#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandAckReportType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandStatusType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandType.hpp>
#include <UMAA/MO/GlobalVectorControl/GlobalVectorExecutionStatusReportType.hpp>
#include <memory>

#include "UmaaCommandProviderIo.h"

namespace arlcore::autopilot {

using UMAA::MO::GlobalVectorControl::GlobalVectorCommandAckReportType;
using UMAA::MO::GlobalVectorControl::GlobalVectorCommandStatusType;
using UMAA::MO::GlobalVectorControl::GlobalVectorCommandType;
using UMAA::MO::GlobalVectorControl::GlobalVectorExecutionStatusReportType;

//! \brief IO bundle (command reader + ack/status/exe-status senders) for the global vector
//! control service provider.
class VectorControlServiceProviderIo
    : public arlcore::umaa::domain::UmaaCommandProviderIo<GlobalVectorCommandType, GlobalVectorCommandAckReportType,
                                                          GlobalVectorCommandStatusType,
                                                          GlobalVectorExecutionStatusReportType> {
 public:
  VectorControlServiceProviderIo(
      std::shared_ptr<arlcore::io::ReaderBase<GlobalVectorCommandType>> commandReader,
      std::shared_ptr<arlcore::io::SenderBase<GlobalVectorCommandAckReportType>> ackSender,
      std::shared_ptr<arlcore::io::SenderBase<GlobalVectorCommandStatusType>> statusSender,
      std::shared_ptr<arlcore::io::SenderBase<GlobalVectorExecutionStatusReportType>> exeStatusSender)
      : UmaaCommandProviderIo(commandReader, ackSender, statusSender, exeStatusSender) {}
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDERIO_HPP_
