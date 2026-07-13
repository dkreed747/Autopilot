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

#include "AutopilotConfig.h"
#include "DubinsPathPlanner.h"
#include "IAutopilot.h"
#include "IVehicleControl.h"
#include "NavState.h"

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

 private:
  PlannerParams derivePlannerParams() const;
  void updateVectorControl(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);
  void updateWaypointControl(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);

  NavState* nav_;
  IVehicleControl* vehicle_;
  AutopilotConfig config_;
  DrivingResourceArbiter arbiter_;
  DubinsPathPlanner planner_;

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
