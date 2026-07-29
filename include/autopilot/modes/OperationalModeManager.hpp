#ifndef AUTOPILOT_MODES_OPERATIONALMODEMANAGER_HPP_
#define AUTOPILOT_MODES_OPERATIONALMODEMANAGER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

#include "NumericGuid.h"
#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/modes/ICommandModeGate.hpp"
#include "autopilot/modes/OperationalModeTypes.hpp"

namespace arlcore::autopilot {

//! \brief The MANUAL / STANDBY / REMOTE / AUTONOMOUS command-authority state
//! machine: MANUAL is platform-owned and always wins, and an implicitly-entered
//! mode idle-reverts to STANDBY while an explicitly-commanded one sticks. The
//! mode-changed callback is invoked without the internal lock held.
class OperationalModeManager : public ICommandModeGate {
 public:
  OperationalModeManager(const OperationalModeConfig& config, const arlcore::NumericGuid& platformId);

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
  CommandClass classify(const UMAA::Common::IdentifierType& source) const override;
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
#endif  // AUTOPILOT_MODES_OPERATIONALMODEMANAGER_HPP_
