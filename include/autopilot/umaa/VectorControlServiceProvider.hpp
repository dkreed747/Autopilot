#ifndef AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDER_HPP_
#define AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDER_HPP_

#include <memory>

#include "CommandProviderBase.h"
#include "InternalTypes.h"
#include "autopilot/core/IAutopilot.hpp"
#include "autopilot/modes/ICommandModeGate.hpp"
#include "autopilot/safety/ConstraintTypes.hpp"
#include "autopilot/safety/ISafetyGate.hpp"
#include "autopilot/umaa/VectorControlServiceProviderIo.hpp"

namespace arlcore::autopilot {

//! \brief UMAA Global Vector control PROVIDER: validates incoming commands, acquires the
//! high-priority driving resource (preempting any waypoint route), and installs the vector
//! setpoint on the brain. Under the hold policy an out-of-mode command parks at ISSUED until
//! the mode becomes compatible, an authoritative mode change flushes it (INTERRUPTED), or it
//! is canceled/replaced.
class VectorControlServiceProvider
    : public arlcore::umaa::services::CommandProviderBase<GlobalVectorCommandType, GlobalVectorCommandAckReportType,
                                                          GlobalVectorCommandStatusType,
                                                          GlobalVectorExecutionStatusReportType> {
 public:
  VectorControlServiceProvider(const arlcore::NumericGuid& source, std::shared_ptr<VectorControlServiceProviderIo> io,
                               IAutopilot* autopilot, flt64_t maxForwardSpeedMps,
                               const ISafetyGate* safetyGate = nullptr, ICommandModeGate* modeGate = nullptr);

  //! \brief Static (config-time) depth ceiling: commands deeper than this are rejected at
  //! validation instead of driving against the output clamp forever.
  void setStaticDepthLimit(std::optional<flt64_t> maxDepthM) { staticMaxDepthM_ = maxDepthM; }

 protected:
  bool isCommandValid(const GlobalVectorCommandType& cmd) override;
  arlcore::umaa::services::CommandStateResult onIssued(const std::weak_ptr<CmdSession> session) override;
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
  CommandClass classOf(const GlobalVectorCommandType& cmd) const;

  arlcore::NumericGuid sourceId_;
  IAutopilot* autopilot_;
  flt64_t maxForwardSpeedMps_;  // <= 0 means no limit
  std::optional<flt64_t> staticMaxDepthM_;
  const ISafetyGate* safetyGate_;
  ICommandModeGate* modeGate_;

  // Hold bookkeeping (one session at a time under CANCEL_EXISTING): the epoch recorded when
  // the hold began; an authoritative mode change bumps the gate's epoch and flushes the hold.
  bool held_ = false;
  uint64_t heldEpoch_ = 0;
  arlcore::NumericGuid heldSessionId_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDER_HPP_
