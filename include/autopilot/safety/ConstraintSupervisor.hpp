#ifndef AUTOPILOT_SAFETY_CONSTRAINTSUPERVISOR_HPP_
#define AUTOPILOT_SAFETY_CONSTRAINTSUPERVISOR_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

#include <UMAA/MM/ConditionalStateReport/ConditionalStateReportType.hpp>

#include "ConditionalBase.h"
#include "SenderBase.h"

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/core/NavState.hpp"
#include "autopilot/safety/IConstraintSource.hpp"
#include "autopilot/safety/ISafeModeStrategy.hpp"
#include "autopilot/safety/ISafetyGate.hpp"
#include "autopilot/safety/ZoneMap.hpp"
#include "autopilot/umaa/CallbackObserver.hpp"

namespace arlcore::autopilot {

class AutopilotBrain;

using ConditionalList = std::vector<std::shared_ptr<arlcore::umaa::conditional::ConditionalBase>>;

//! \brief The autopilot's constraint authority. Consumes the conditional set (from the report
//! consumer) and the active subset (from the standing ActiveConstraints provider), converts the
//! supported specializations (water zone, speed, depth) into a ConstraintSnapshot, feeds the
//! ZoneMap, and publishes per-conditional ConditionalStateReports. Implements IConstraintSource
//! for the brain/planner and ISafetyGate for the command providers.
class ConstraintSupervisor : public IConstraintSource, public ISafetyGate {
 public:
  ConstraintSupervisor(const AutopilotConfig& config, NavState* nav, ZoneMap* zoneMap,
    std::shared_ptr<arlcore::io::SenderBase<UMAA::MM::ConditionalStateReport::ConditionalStateReportType>>
      stateReportSender,
    const arlcore::NumericGuid& sourceId);

  //! \brief The observers to register on the conditional report consumer and the standing
  //! ActiveConstraints provider (both subjects share the same payload type).
  std::shared_ptr<CallbackObserver<ConditionalList>> conditionalSetObserver() { return setObserver_; }
  std::shared_ptr<CallbackObserver<ConditionalList>> activeSetObserver() { return activeObserver_; }

  //! \brief The violation-response states. RECOVERING = a zone violation is being driven back
  //! to compliance while the grace timer runs; SAFE_MODE = latched escalation running the
  //! configured strategy.
  enum class SafetyState { MONITORING, RECOVERING, SAFE_MODE };

  //! \brief Arm the violation FSM: violations confirmed against the active conditionals drive
  //! the brain's recovery hooks and, past their grace deadline, the safe-mode strategy.
  //! Without this the supervisor only monitors and reports.
  void attachSafety(AutopilotBrain* brain, std::unique_ptr<ISafeModeStrategy> strategy);

  SafetyState safetyState() const;

  //! \brief Re-run the safe-mode strategy's entry actions if SAFE_MODE is engaged. Called
  //! when the platform releases MANUAL control: the plan made at safe-mode entry may be
  //! arbitrarily stale after a human drove the vehicle elsewhere.
  void refreshSafeModePlan();

  //! \brief Per-tick work: rebuild the snapshot if the active set changed, publish
  //! ConditionalStateReports at the configured period, and run the violation FSM.
  void update();

  // IConstraintSource
  ConstraintSnapshot snapshot() const override;
  uint64_t revision() const override;

  // ISafetyGate: commands are accepted only while MONITORING.
  bool commandsAllowed() const override;

 private:
  //! \brief The kind of constraint a violation belongs to (selects its grace override).
  enum class ConstraintClass { ZONE, SPEED, ELEVATION, OTHER };

  //! \brief Debounce/clear state for one active conditional.
  struct ViolationTracker {
    ConstraintClass cls = ConstraintClass::OTHER;
    int rawViolatingTicks = 0;
    bool confirmed = false;
    std::chrono::steady_clock::time_point confirmedAt{};
    std::optional<std::chrono::steady_clock::time_point> compliantSince;
  };

  void onConditionalSetChanged(const ConditionalList& all);
  void onActiveSetChanged(const ConditionalList& active);

  //! \brief Advance violation trackers and the MONITORING/RECOVERING/SAFE_MODE transitions.
  void updateSafety();

  //! \brief Grace period for a constraint class (per-class override or the global default).
  double gracePeriodS(ConstraintClass cls) const;

  void enterSafeMode(const char* why);

  //! \brief Rebuild the ConstraintSnapshot from the current active conditionals and push the
  //! zones into the ZoneMap.
  void rebuildSnapshot();

  //! \brief Evaluate every known conditional and publish/dispose its state report.
  void publishStateReports();

  AutopilotConfig config_;
  NavState* nav_;
  ZoneMap* zoneMap_;
  std::shared_ptr<arlcore::io::SenderBase<UMAA::MM::ConditionalStateReport::ConditionalStateReportType>>
      stateReportSender_;
  arlcore::NumericGuid sourceId_;

  std::shared_ptr<CallbackObserver<ConditionalList>> setObserver_;
  std::shared_ptr<CallbackObserver<ConditionalList>> activeObserver_;

  mutable std::mutex mtx_;
  ConditionalList allConditionals_;
  ConditionalList activeConditionals_;
  bool activeDirty_ = false;
  ConstraintSnapshot snapshot_;
  uint64_t revision_ = 0;

  std::set<arlcore::NumericGuid> publishedStateIds_;
  std::chrono::steady_clock::time_point lastStateReport_{};

  // Violation FSM (armed by attachSafety).
  AutopilotBrain* brain_ = nullptr;
  std::unique_ptr<ISafeModeStrategy> strategy_;
  SafetyState state_ = SafetyState::MONITORING;
  std::map<arlcore::NumericGuid, ViolationTracker> trackers_;
  std::optional<std::chrono::steady_clock::time_point> lastSafetyTick_;
  std::optional<std::chrono::steady_clock::time_point> allCompliantSince_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_CONSTRAINTSUPERVISOR_HPP_
