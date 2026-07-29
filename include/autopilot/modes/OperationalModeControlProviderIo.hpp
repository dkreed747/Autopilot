#ifndef AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDERIO_HPP_
#define AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDERIO_HPP_

#include <UMAA/MM/OperationalModeControl/OperationalModeCommandAckReportType.hpp>
#include <UMAA/MM/OperationalModeControl/OperationalModeCommandStatusType.hpp>
#include <UMAA/MM/OperationalModeControl/OperationalModeCommandType.hpp>
#include <memory>

#include "UmaaCommandProviderIo.h"

namespace arlcore::autopilot {

using UMAA::MM::OperationalModeControl::OperationalModeCommandAckReportType;
using UMAA::MM::OperationalModeControl::OperationalModeCommandStatusType;
using UMAA::MM::OperationalModeControl::OperationalModeCommandType;

//! \brief IO bundle (command reader + ack/status senders) for the MM
//! operational mode control service provider. The service defines no
//! execution-status type.
class OperationalModeControlProviderIo
    : public arlcore::umaa::domain::UmaaCommandProviderIo<
          OperationalModeCommandType, OperationalModeCommandAckReportType,
          OperationalModeCommandStatusType> {
 public:
  OperationalModeControlProviderIo(
      std::shared_ptr<arlcore::io::ReaderBase<OperationalModeCommandType>>
          commandReader,
      std::shared_ptr<
          arlcore::io::SenderBase<OperationalModeCommandAckReportType>>
          ackSender,
      std::shared_ptr<arlcore::io::SenderBase<OperationalModeCommandStatusType>>
          statusSender)
      : UmaaCommandProviderIo(commandReader, ackSender, statusSender) {}
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_MODES_OPERATIONALMODECONTROLPROVIDERIO_HPP_
