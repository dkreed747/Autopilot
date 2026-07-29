#ifndef AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDER_HPP_
#define AUTOPILOT_UMAA_VECTORCONTROLSERVICEPROVIDER_HPP_

#include <memory>

#include "CommandProviderBase.h"
#include "autopilot/safety/ConstraintTypes.hpp"
#include "autopilot/safety/ISafetyGate.hpp"
#include "autopilot/core/IAutopilot.hpp"
#include "autopilot/modes/ICommandModeGate.hpp"
#include "autopilot/umaa/VectorControlServiceProviderIo.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief UMAA Global Vector control PROVIDER. Validates incoming vector commands against the
//! platform's speed limit and the operational-mode gate, acquires the (high-priority) driving
//! resource — preempting any active waypoint route — installs the vector setpoint on the
//! autopilot brain, and reports per-cycle execution status.
//!
//! Mode gating: under the fail policy an out-of-mode command dies at validation
//! (VALIDATION_FAILED); under the hold policy it parks silently at ISSUED (onIssued returns
//! OK) until the mode becomes compatible, an authoritative mode change flushes it
//! (INTERRUPTED), or it is canceled/replaced. Sessions past ISSUED whose class becomes
//! disallowed fail INTERRUPTED via isCommandFailed.
class VectorControlServiceProvider : public arlcore::umaa::services::CommandProviderBase<
    GlobalVectorCommandType,
    GlobalVectorCommandAckReportType,
    GlobalVectorCommandStatusType,
    GlobalVectorExecutionStatusReportType> {
 public:
  VectorControlServiceProvider(const arlcore::NumericGuid& source,
                               std::shared_ptr<VectorControlServiceProviderIo> io,
                               IAutopilot* autopilot,
                               flt64_t maxForwardSpeedMps,
                               const ISafetyGate* safetyGate = nullptr,
                               ICommandModeGate* modeGate = nullptr);

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
