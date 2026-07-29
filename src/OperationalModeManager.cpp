#include "OperationalModeManager.hpp"

#include <utility>

#include "Logger.h"

namespace arlcore::autopilot {

static const char* modeName(OperationalMode mode) {
  switch (mode) {
    case OperationalMode::MANUAL:
      return "MANUAL";
    case OperationalMode::STANDBY:
      return "STANDBY";
    case OperationalMode::REMOTE:
      return "REMOTE";
    case OperationalMode::AUTONOMOUS:
      return "AUTONOMOUS";
    default:
      return "UNKNOWN";
  }
}

OperationalModeManager::OperationalModeManager(
    const OperationalModeConfig& config, const arlcore::NumericGuid& platformId)
    : config_(config), platformId_(platformId) {}

void OperationalModeManager::setModeChangedCallback(
    std::function<void(OperationalMode)> callback) {
  std::scoped_lock lock(mtx_);
  onModeChanged_ = std::move(callback);
}

void OperationalModeManager::beginStep(bool manualEngaged) {
  std::function<void(OperationalMode)> callback;
  std::optional<OperationalMode> changedTo;
  {
    std::scoped_lock lock(mtx_);
    if (!initialized_) {
      initialized_ = true;
      mode_ =
          manualEngaged ? OperationalMode::MANUAL : OperationalMode::STANDBY;
      explicitEntry_ = false;
      changedTo = mode_;  // always announce the initial mode
    } else if (manualEngaged && mode_ != OperationalMode::MANUAL) {
      ++epoch_;  // authoritative: flushes held sessions
      UMAA_LOG_WARN(
          util::SYSTEM_LOGGER,
          "Platform engaged MANUAL control; preempting " << modeName(mode_))
      if (transitionLocked(OperationalMode::MANUAL, false)) {
        changedTo = mode_;
      }
    } else if (!manualEngaged && mode_ == OperationalMode::MANUAL) {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                    "Platform released MANUAL control; entering STANDBY")
      if (transitionLocked(OperationalMode::STANDBY, false)) {
        changedTo = mode_;
      }
    }
    callback = onModeChanged_;
  }
  if (changedTo.has_value() && callback) {
    callback(changedTo.value());
  }
}

void OperationalModeManager::endStep(bool localCommandActive,
                                     bool remoteCommandActive) {
  std::function<void(OperationalMode)> callback;
  std::optional<OperationalMode> changedTo;
  {
    std::scoped_lock lock(mtx_);
    const bool revertible = (mode_ == OperationalMode::REMOTE ||
                             mode_ == OperationalMode::AUTONOMOUS) &&
                            !explicitEntry_;
    if (!revertible) {
      idleSince_.reset();
    } else {
      const bool activeClassBusy = (mode_ == OperationalMode::REMOTE)
                                       ? remoteCommandActive
                                       : localCommandActive;
      if (activeClassBusy) {
        idleSince_.reset();
      } else {
        const auto now = std::chrono::steady_clock::now();
        if (!idleSince_.has_value()) {
          idleSince_ = now;
        }
        const double idleS =
            std::chrono::duration<double>(now - idleSince_.value()).count();
        if (idleS >= config_.idleRevertS) {
          UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                        "Implicit " << modeName(mode_) << " idle for " << idleS
                                    << " s; reverting to STANDBY")
          if (transitionLocked(OperationalMode::STANDBY, false)) {
            changedTo = mode_;
          }
        }
      }
    }
    callback = onModeChanged_;
  }
  if (changedTo.has_value() && callback) {
    callback(changedTo.value());
  }
}

bool OperationalModeManager::commandMode(OperationalMode requested) {
  std::function<void(OperationalMode)> callback;
  std::optional<OperationalMode> changedTo;
  {
    std::scoped_lock lock(mtx_);
    if (mode_ == OperationalMode::MANUAL ||
        requested == OperationalMode::MANUAL) {
      return false;
    }
    ++epoch_;  // authoritative even when re-asserting the current mode
    if (transitionLocked(requested, true)) {
      UMAA_LOG_INFO(
          util::SYSTEM_LOGGER,
          "Operational mode explicitly commanded to " << modeName(requested))
      changedTo = mode_;
    }
    callback = onModeChanged_;
  }
  if (changedTo.has_value() && callback) {
    callback(changedTo.value());
  }
  return true;
}

OperationalMode OperationalModeManager::mode() const {
  std::scoped_lock lock(mtx_);
  return mode_;
}

CommandClass OperationalModeManager::classify(
    const UMAA::Common::IdentifierType& source) const {
  return (arlcore::NumericGuid(source.parentID()) == platformId_)
             ? CommandClass::LOCAL
             : CommandClass::REMOTE;
}

bool OperationalModeManager::rejectedAtValidation(CommandClass cls) const {
  std::scoped_lock lock(mtx_);
  if (mode_ == OperationalMode::MANUAL) {
    return true;  // nothing executes or holds in MANUAL, regardless of policy
  }
  return config_.commandsOutOfModeAreFailed && !wouldAdmitLocked(cls);
}

bool OperationalModeManager::wouldAdmit(CommandClass cls) const {
  std::scoped_lock lock(mtx_);
  return wouldAdmitLocked(cls);
}

AdmissionDecision OperationalModeManager::requestAdmission(CommandClass cls) {
  std::function<void(OperationalMode)> callback;
  std::optional<OperationalMode> changedTo;
  AdmissionDecision decision = AdmissionDecision::HOLD;
  {
    std::scoped_lock lock(mtx_);
    switch (mode_) {
      case OperationalMode::STANDBY:
        if (config_.allowImplicitModeTransitions) {
          const OperationalMode target = (cls == CommandClass::LOCAL)
                                             ? OperationalMode::AUTONOMOUS
                                             : OperationalMode::REMOTE;
          UMAA_LOG_INFO(
              util::SYSTEM_LOGGER,
              "Implicit mode transition STANDBY -> " << modeName(target))
          if (transitionLocked(target, false)) {
            changedTo = mode_;
          }
          decision = AdmissionDecision::ADMIT;
        }
        break;
      case OperationalMode::REMOTE:
        // Never an implicit REMOTE -> AUTONOMOUS: the onboard autonomy cannot
        // take the resource back from an operator.
        if (cls == CommandClass::REMOTE) {
          decision = AdmissionDecision::ADMIT;
        }
        break;
      case OperationalMode::AUTONOMOUS:
        if (cls == CommandClass::LOCAL) {
          decision = AdmissionDecision::ADMIT;
        } else if (config_.allowImplicitModeTransitions) {
          // Operator precedence: a remote command preempts the onboard
          // autonomy.
          UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                        "Implicit mode transition AUTONOMOUS -> REMOTE")
          if (transitionLocked(OperationalMode::REMOTE, false)) {
            changedTo = mode_;
          }
          decision = AdmissionDecision::ADMIT;
        }
        break;
      case OperationalMode::MANUAL:
      default:
        break;  // unreachable in practice: validation fails first in MANUAL
    }
    callback = onModeChanged_;
  }
  if (changedTo.has_value() && callback) {
    callback(changedTo.value());
  }
  return decision;
}

bool OperationalModeManager::classAllowed(CommandClass cls) const {
  std::scoped_lock lock(mtx_);
  switch (mode_) {
    case OperationalMode::REMOTE:
      return cls == CommandClass::REMOTE;
    case OperationalMode::AUTONOMOUS:
      return cls == CommandClass::LOCAL;
    default:
      return false;
  }
}

uint64_t OperationalModeManager::authoritativeEpoch() const {
  std::scoped_lock lock(mtx_);
  return epoch_;
}

bool OperationalModeManager::wouldAdmitLocked(CommandClass cls) const {
  switch (mode_) {
    case OperationalMode::STANDBY:
      return config_.allowImplicitModeTransitions;
    case OperationalMode::REMOTE:
      return cls == CommandClass::REMOTE;
    case OperationalMode::AUTONOMOUS:
      return cls == CommandClass::LOCAL || config_.allowImplicitModeTransitions;
    case OperationalMode::MANUAL:
    default:
      return false;
  }
}

bool OperationalModeManager::transitionLocked(OperationalMode next,
                                              bool explicitEntry) {
  explicitEntry_ = explicitEntry;
  idleSince_.reset();
  if (mode_ == next) {
    return false;
  }
  mode_ = next;
  return true;
}

}  // namespace arlcore::autopilot
