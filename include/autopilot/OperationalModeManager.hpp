#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODEMANAGER_HPP_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODEMANAGER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

#include "AutopilotConfig.h"
#include "ICommandModeGate.hpp"
#include "NumericGuid.h"
#include "OperationalModeTypes.hpp"

namespace arlcore::autopilot {

//! \brief The MANUAL / STANDBY / REMOTE / AUTONOMOUS command-authority state
//! machine.
//!
//! MANUAL is platform-owned: it is entered and exited only through the per-tick
//! manual poll (beginStep) and always wins. STANDBY/REMOTE/AUTONOMOUS move via
//! explicit mode commands (commandMode), via implicit transitions fired by
//! incoming driving commands (requestAdmission, config-gated), and via idle
//! revert (endStep): a mode entered implicitly falls back to STANDBY once no
//! command of its class has been active for idle_revert_s, while an
//! explicitly-commanded mode sticks until the next explicit command or manual
//! engagement.
//!
//! The mode-changed callback is invoked without the internal lock held
//! (ConstraintSupervisor pattern);
//! beginStep/endStep/commandMode/requestAdmission all run on the single
//! control-loop thread, the gate reads may be called from provider hooks on
//! that same thread.
class OperationalModeManager : public ICommandModeGate {
 public:
  OperationalModeManager(const OperationalModeConfig& config,
                         const arlcore::NumericGuid& platformId);

  //! \brief Callback fired after every reported-mode change, including the
  //! initial mode determination on the first beginStep.
  void setModeChangedCallback(std::function<void(OperationalMode)> callback);

  //! \brief Per-tick pre-provider update: the first call fixes the initial mode
  //! from the manual poll (engaged => MANUAL, else STANDBY); later calls apply
  //! manual engage (authoritative, from any state) and disengage (=> STANDBY)
  //! edges.
  void beginStep(bool manualEngaged);

  //! \brief Per-tick post-provider update: idle revert of implicitly-entered
  //! REMOTE / AUTONOMOUS to STANDBY. The activity flags are per class: REMOTE
  //! watches only remoteCommandActive, AUTONOMOUS only localCommandActive, so
  //! held sessions (always the other class) never block the active mode's
  //! revert.
  void endStep(bool localCommandActive, bool remoteCommandActive);

  //! \brief Apply an explicit mode command. Returns false in MANUAL (and for
  //! MANUAL itself, which is never commandable). Re-commanding the current mode
  //! succeeds without a callback and upgrades the entry to explicit (disarming
  //! idle revert).
  bool commandMode(OperationalMode requested);

  OperationalMode mode() const;

  // ICommandModeGate
  CommandClass classify(
      const UMAA::Common::IdentifierType& source) const override;
  bool rejectedAtValidation(CommandClass cls) const override;
  bool wouldAdmit(CommandClass cls) const override;
  AdmissionDecision requestAdmission(CommandClass cls) override;
  bool classAllowed(CommandClass cls) const override;
  uint64_t authoritativeEpoch() const override;

 private:
  bool wouldAdmitLocked(CommandClass cls) const;

  //! \brief Set mode/entry-kind and reset the idle timer; returns whether the
  //! mode changed.
  bool transitionLocked(OperationalMode next, bool explicitEntry);

  OperationalModeConfig config_;
  arlcore::NumericGuid platformId_;

  mutable std::mutex mtx_;
  std::function<void(OperationalMode)> onModeChanged_;
  bool initialized_ = false;
  OperationalMode mode_ = OperationalMode::STANDBY;
  bool explicitEntry_ = false;
  uint64_t epoch_ = 0;
  std::optional<std::chrono::steady_clock::time_point> idleSince_;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_OPERATIONALMODEMANAGER_HPP_
