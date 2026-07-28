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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTBRAIN_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTBRAIN_H_

#include <chrono>
#include <mutex>
#include <optional>
#include <vector>

#include <memory>

#include "AutopilotConfig.h"
#include "ConstraintClamp.h"
#include "ConstraintTypes.h"
#include "DubinsPathPlanner.h"
#include "IAutopilot.h"
#include "IVehicleControl.h"
#include "NavState.h"
#include "RecoveryGuidance.h"
#include "VectorZoneGuidance.h"
#include "ZoneMap.h"

namespace arlcore::autopilot {

//! \brief Concrete autopilot brain. Single-threaded by design: onNavUpdate() and the provider
//! setpoint calls all run on the main loop thread (the nav observer fires inside the nav
//! consumer's cycle()). The mutex is defensive should a future strategy introduce threads.
class AutopilotBrain : public IAutopilot {
 public:
  AutopilotBrain(NavState* nav, IVehicleControl* vehicle, const AutopilotConfig& config);

  void setVectorSetpoint(const UMAA::MO::GlobalVectorControl::GlobalVectorCommandType& cmd) override;
  bool setWaypointSetpoint(
      const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints) override;
  void clearSetpoint(DriveSource src) override;
  void onNavUpdate() override;

  //! \brief Called every control-loop tick: if a drive mode is active but the newest pose is
  //! older than the configured staleness timeout, command a zero-speed hold so the vehicle
  //! does not keep driving blind on stale navigation.
  void enforceNavStaleness();
  VectorProgress vectorProgress() const override;
  WaypointProgress waypointProgress() const override;
  DrivingResourceArbiter& arbiter() override { return arbiter_; }

  //! \brief Current active driving mode (for diagnostics/tests).
  DriveSource mode() const;

  //! \brief Install the constraint source whose snapshot clamps every emitted control vector.
  //! Nullable; without one the brain emits unclamped.
  void setConstraintSource(const IConstraintSource* source);

  //! \brief Install the shared zone map: vector ticks route through the tangent-bug avoidance
  //! and waypoint legs plan around the active zones (replanning when the constraint revision
  //! changes mid-route). Nullable; without one guidance is zone-blind.
  void setZoneMap(const ZoneMap* zoneMap);

  // --- Safe mode (driven by the safety supervisor's strategy) -------------------------------

  //! \brief Enter SAFE mode running a waypoint route on the dedicated safe planner (keeps SRP
  //! progress out of the failing mission provider's teardown). Preempts the current driving
  //! command through the arbiter (the provider surfaces INTERRUPTED). Falls back to a
  //! zero-speed hold (returning false) when the route cannot be planned.
  bool activateSafeRoute(const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints);

  //! \brief Enter SAFE mode holding zero speed at the current heading.
  void activateSafeHold();

  //! \brief Progress of the safe route (valid while a safe route is active).
  WaypointProgress safeProgress() const;
  bool safeRouteComplete() const;
  bool safeRouteFailed() const;

  //! \brief Leave SAFE mode and release the driving resource (commands flow again).
  void clearSafeMode();

  // --- Zone-violation recovery ---------------------------------------------------------------

  //! \brief Begin the recovery maneuver: the active command stays installed (and EXECUTING)
  //! but ticks route through RecoveryGuidance toward the nearest compliant point. Returns
  //! false when no recovery target could be found (the brain holds zero speed instead).
  bool beginRecovery();

  //! \brief Whether the recovery has held COMPLIANT long enough to count as recovered.
  bool recoveryComplete();

  //! \brief End recovery and resume the interrupted command: a waypoint route replans its
  //! current leg from the live pose; a vector setpoint simply resumes.
  void endRecovery();

  //! \brief Drop recovery without resuming (the supervisor is escalating to safe mode).
  void abortRecovery();

  bool recovering() const;

 private:
  PlannerParams derivePlannerParams() const;
  void updateVectorControl(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);
  void updateWaypointControl(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);
  void updateSafeControl(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);
  void updateRecoveryControl(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);

  //! \brief Zero-speed hold at the current (or last known) heading.
  void emitHold(const std::optional<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>& pose);

  //! \brief The single control-output funnel: applies the constraint clamps (most restrictive
  //! of dynamic constraints, static settings, and platform capabilities) to every control
  //! vector before it reaches the vehicle. All send sites route through here.
  void emitControl(const ControlVector& cv);

  NavState* nav_;
  IVehicleControl* vehicle_;
  AutopilotConfig config_;
  DrivingResourceArbiter arbiter_;
  DubinsPathPlanner planner_;
  const IConstraintSource* constraintSource_ = nullptr;
  ClampLimits staticClampLimits_;
  bool lastSpeedClamped_ = false;
  bool lastElevationClamped_ = false;
  bool manualSuppressed_ = false;
  const ZoneMap* zoneMap_ = nullptr;
  std::unique_ptr<VectorZoneGuidance> vectorGuidance_;
  uint64_t plannedConstraintRevision_ = 0;  // constraint revision the current route was planned under

  // Safe mode: a dedicated planner so SRP progress survives the preempted mission provider's
  // teardown, or a zero-speed hold when no safe route is available.
  DubinsPathPlanner safePlanner_;
  bool safeHold_ = false;

  // Zone-violation recovery (owned here; the supervisor drives begin/complete/end).
  std::unique_ptr<RecoveryGuidance> recovery_;
  bool recovering_ = false;

  mutable std::mutex mtx_;
  DriveSource mode_ = DriveSource::NONE;
  UMAA::MO::GlobalVectorControl::GlobalVectorCommandType activeVector_;
  VectorProgress vectorProgress_;
  WaypointProgress waypointProgress_;

  // Hard-tolerance tracking for the active vector command: once all criteria have been
  // achieved, a persistent violation (longer than failureDelayS) fails the command.
  bool vectorEverAchieved_ = false;
  std::optional<std::chrono::steady_clock::time_point> vectorViolationSince_;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_AUTOPILOTBRAIN_H_
