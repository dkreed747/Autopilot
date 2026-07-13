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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_WAYPOINTCONTROLSERVICEPROVIDER_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_WAYPOINTCONTROLSERVICEPROVIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "CommandProviderBase.h"
#include "IAutopilot.h"
#include "LargeListReader.h"
#include "WaypointControlServiceProviderIo.h"

namespace arlcore::autopilot {

//! \brief UMAA Global Waypoint control PROVIDER. Reads the waypoint route from the large-list
//! element topic, acquires the (low-priority) driving resource, installs the route on the
//! autopilot brain (which plans a Dubins path), and reports per-cycle execution status. It is
//! preempted by a vector command (-> FAILED/INTERRUPTED) and is rejected if a vector already
//! holds the driving resource (-> FAILED/RESOURCE_REJECTED).
class WaypointControlServiceProvider : public arlcore::umaa::services::CommandProviderBase<
    GlobalWaypointCommandType,
    GlobalWaypointCommandAckReportType,
    GlobalWaypointCommandStatusType,
    GlobalWaypointExecutionStatusReportType> {
 public:
  WaypointControlServiceProvider(const arlcore::NumericGuid& source,
                                 std::shared_ptr<WaypointControlServiceProviderIo> io,
                                 IAutopilot* autopilot,
                                 double maxForwardSpeedMps,
                                 int maxListWaitCycles);

 protected:
  bool onCycle() override;
  arlcore::umaa::services::CommandStateResult onCommanded(const std::weak_ptr<CmdSession> session) override;
  arlcore::umaa::services::CommandStateResult onExecuting(const std::weak_ptr<CmdSession> session) override;
  bool onUpdated(const std::weak_ptr<CmdSession> session, const GlobalWaypointCommandType& previousCmd,
                 const GlobalWaypointCommandType& updatedCmd) override;
  bool isCommandCompleted(const std::weak_ptr<CmdSession> session) override;
  CommandStatusReasonEnumType isCommandFailed(const std::weak_ptr<CmdSession> session) override;
  SendStatus sendExecutionStatus(const GlobalWaypointCommandType& cmd) override;
  SendStatus disposeExecutionStatus(const GlobalWaypointCommandType& cmd) override;
  bool onCanceled(const std::weak_ptr<CmdSession> session) override;
  bool onFailed(const std::weak_ptr<CmdSession> session) override;
  bool onCompleted(const std::weak_ptr<CmdSession> session) override;

 private:
  void resetPlanningState();
  void relinquish(const std::weak_ptr<CmdSession> session);

  //! \brief Fail the session directly from the COMMANDED state (reasons like
  //! RESOURCE_REJECTED are only legal there), releasing everything this command holds.
  arlcore::umaa::services::CommandStateResult failInCommanded(const std::weak_ptr<CmdSession> session,
      CommandStatusReasonEnumType reason, const std::string& logMessage);

  bool validateWaypoints(
      const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints) const;

  arlcore::NumericGuid sourceId_;
  IAutopilot* autopilot_;
  std::shared_ptr<WaypointControlServiceProviderIo> wpIo_;
  arlcore::umaa::LargeListReader<UMAA::MO::GlobalWaypointControl::GlobalWaypointType,
      GlobalWaypointCommandTypeWaypointsListElement> listReader_;
  double maxForwardSpeedMps_;
  int maxListWaitCycles_;

  // Per-command planning state.
  arlcore::NumericGuid activeSession_;
  bool sessionActive_ = false;
  bool acquired_ = false;
  bool planned_ = false;
  int listWaitCycles_ = 0;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_WAYPOINTCONTROLSERVICEPROVIDER_H_
