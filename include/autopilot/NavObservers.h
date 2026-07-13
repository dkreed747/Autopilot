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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_NAVOBSERVERS_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_NAVOBSERVERS_H_

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>

#include "IAutopilot.h"
#include "NavState.h"
#include "Observer.h"

namespace arlcore::autopilot {

//! \brief Observes Global Pose reports. Pose is the primary trigger: on each new pose it
//! refreshes the shared nav state and drives an autopilot control recompute.
class GlobalPoseObserver : public arlcore::Observer<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType> {
 public:
  GlobalPoseObserver(NavState* nav, IAutopilot* autopilot) : nav_(nav), autopilot_(autopilot) {}

  void update(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& report) override {
    nav_->setPose(report);
    autopilot_->onNavUpdate();
  }

 private:
  NavState* nav_;
  IAutopilot* autopilot_;
};

//! \brief Observes Speed reports; refreshes nav state for the next pose-driven tick.
class SpeedObserver : public arlcore::Observer<UMAA::SA::SpeedStatus::SpeedReportType> {
 public:
  explicit SpeedObserver(NavState* nav) : nav_(nav) {}

  void update(const UMAA::SA::SpeedStatus::SpeedReportType& report) override {
    nav_->setSpeed(report);
  }

 private:
  NavState* nav_;
};

//! \brief Observes Velocity reports; refreshes nav state for the next pose-driven tick.
class VelocityObserver : public arlcore::Observer<UMAA::SA::VelocityStatus::VelocityReportType> {
 public:
  explicit VelocityObserver(NavState* nav) : nav_(nav) {}

  void update(const UMAA::SA::VelocityStatus::VelocityReportType& report) override {
    nav_->setVelocity(report);
  }

 private:
  NavState* nav_;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_NAVOBSERVERS_H_
