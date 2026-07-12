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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_VECTORCONTROLSERVICEPROVIDER_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_VECTORCONTROLSERVICEPROVIDER_H_

#include <memory>

#include "CommandProviderBase.h"
#include "IAutopilot.h"
#include "VectorControlServiceProviderIo.h"

namespace arlcore::autopilot {

//! \brief UMAA Global Vector control PROVIDER. Validates incoming vector commands against the
//! platform's speed limit, acquires the (high-priority) driving resource — preempting any
//! active waypoint route — installs the vector setpoint on the autopilot brain, and reports
//! per-cycle execution status (direction/elevation/speed achieved).
class VectorControlServiceProvider : public arlcore::umaa::services::CommandProviderBase<
    GlobalVectorCommandType,
    GlobalVectorCommandAckReportType,
    GlobalVectorCommandStatusType,
    GlobalVectorExecutionStatusReportType> {
 public:
  VectorControlServiceProvider(const arlcore::NumericGuid& source,
                               std::shared_ptr<VectorControlServiceProviderIo> io,
                               IAutopilot* autopilot,
                               double maxForwardSpeedMps);

 protected:
  bool isCommandValid(const GlobalVectorCommandType& cmd) override;
  arlcore::umaa::services::CommandStateResult onCommanded(const std::weak_ptr<CmdSession> session) override;
  arlcore::umaa::services::CommandStateResult onExecuting(const std::weak_ptr<CmdSession> session) override;
  bool onUpdated(const std::weak_ptr<CmdSession> session, const GlobalVectorCommandType& previousCmd,
                 const GlobalVectorCommandType& updatedCmd) override;
  bool isCommandCompleted(const std::weak_ptr<CmdSession> session) override;
  CommandStatusReasonEnumType isCommandFailed(const std::weak_ptr<CmdSession> session) override;
  SendStatus sendExecutionStatus(const GlobalVectorCommandType& cmd) override;
  SendStatus disposeExecutionStatus(const GlobalVectorCommandType& cmd) override;
  bool onCanceled(const std::weak_ptr<CmdSession> session) override;
  bool onFailed(const std::weak_ptr<CmdSession> session) override;
  bool onCompleted(const std::weak_ptr<CmdSession> session) override;

 private:
  void relinquish();

  arlcore::NumericGuid sourceId_;
  IAutopilot* autopilot_;
  double maxForwardSpeedMps_;  // <= 0 means no limit
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_VECTORCONTROLSERVICEPROVIDER_H_
