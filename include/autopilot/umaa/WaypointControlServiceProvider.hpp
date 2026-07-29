#ifndef AUTOPILOT_UMAA_WAYPOINTCONTROLSERVICEPROVIDER_HPP_
#define AUTOPILOT_UMAA_WAYPOINTCONTROLSERVICEPROVIDER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "CommandProviderBase.h"
#include "autopilot/safety/ConstraintTypes.hpp"
#include "autopilot/safety/ISafetyGate.hpp"
#include "autopilot/core/IAutopilot.hpp"
#include "autopilot/modes/ICommandModeGate.hpp"
#include "autopilot/safety/ZoneMap.hpp"
#include "LargeListReader.h"
#include "autopilot/umaa/WaypointControlServiceProviderIo.hpp"

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
                                 int32_t maxListWaitCycles,
                                 const ISafetyGate* safetyGate = nullptr,
                                 const ZoneMap* zoneMap = nullptr,
                                 ICommandModeGate* modeGate = nullptr);

 protected:
  bool isCommandValid(const GlobalWaypointCommandType& cmd) override;
  bool onCycle() override;
  arlcore::umaa::services::CommandStateResult onIssued(const std::weak_ptr<CmdSession> session) override;
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
  CommandClass classOf(const GlobalWaypointCommandType& cmd) const;

  //! \brief Fail the session directly from the COMMANDED state (reasons like
  //! RESOURCE_REJECTED are only legal there), releasing everything this command holds.
  arlcore::umaa::services::CommandStateResult failInCommanded(const std::weak_ptr<CmdSession> session,
      CommandStatusReasonEnumType reason, const std::string& logMessage);

  bool validateWaypoints(
      const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints) const;

  //! \brief Whether every waypoint keeps the zone safety margin (command-time validation
  //! against the active water zones). Fills `message` with the offending waypoint.
  bool waypointsZoneCompliant(
      const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints,
      std::string* message) const;

  arlcore::NumericGuid sourceId_;
  IAutopilot* autopilot_;
  std::shared_ptr<WaypointControlServiceProviderIo> wpIo_;
  arlcore::umaa::LargeListReader<UMAA::MO::GlobalWaypointControl::GlobalWaypointType,
      GlobalWaypointCommandTypeWaypointsListElement> listReader_;
  double maxForwardSpeedMps_;
  int32_t maxListWaitCycles_;
  const ISafetyGate* safetyGate_;
  const ZoneMap* zoneMap_;
  ICommandModeGate* modeGate_;

  // Per-command planning state.
  arlcore::NumericGuid activeSession_;
  bool sessionActive_ = false;
  bool acquired_ = false;
  bool planned_ = false;
  int32_t listWaitCycles_ = 0;

  // Hold bookkeeping (one session at a time under CANCEL_EXISTING): the epoch recorded when
  // the hold began; an authoritative mode change bumps the gate's epoch and flushes the hold.
  bool held_ = false;
  uint64_t heldEpoch_ = 0;
  arlcore::NumericGuid heldSessionId_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_WAYPOINTCONTROLSERVICEPROVIDER_HPP_
