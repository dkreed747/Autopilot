//---------------------------------------------------------------------------
// Copyright 2026 Pennsylvania State University
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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONSTRAINTSUPERVISOR_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONSTRAINTSUPERVISOR_H_

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
#include "Observer.h"
#include "SenderBase.h"

#include "AutopilotConfig.h"
#include "ConstraintTypes.h"
#include "NavState.h"
#include "ZoneMap.h"

namespace arlcore::autopilot {

//! \brief Observer adapter forwarding a Subject's notifications to a std::function, so one class
//! can observe several subjects of the same payload type.
template <class T>
class CallbackObserver : public arlcore::Observer<T> {
 public:
  explicit CallbackObserver(std::function<void(const T&)> fn) : fn_(std::move(fn)) {}
  void update(const T& data) override { fn_(data); }

 private:
  std::function<void(const T&)> fn_;
};

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

  //! \brief Per-tick work: rebuild the snapshot if the active set changed and publish
  //! ConditionalStateReports at the configured period.
  void update();

  // IConstraintSource
  ConstraintSnapshot snapshot() const override;
  uint64_t revision() const override;

  // ISafetyGate (the violation FSM drives this; until it engages, commands are allowed)
  bool commandsAllowed() const override;

 private:
  void onConditionalSetChanged(const ConditionalList& all);
  void onActiveSetChanged(const ConditionalList& active);

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
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_CONSTRAINTSUPERVISOR_H_
