#ifndef AUTOPILOT_UMAA_GLOBALPOSEOBSERVER_HPP_
#define AUTOPILOT_UMAA_GLOBALPOSEOBSERVER_HPP_

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <cmath>

#include "Logger.h"
#include "Observer.h"
#include "autopilot/core/IAutopilot.hpp"
#include "autopilot/core/NavState.hpp"

namespace arlcore::autopilot {

//! \brief Observes Global Pose reports. Pose is the primary trigger: on each new pose it
//! refreshes the shared nav state and drives an autopilot control recompute.
class GlobalPoseObserver : public arlcore::Observer<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType> {
 public:
  GlobalPoseObserver(NavState* nav, IAutopilot* autopilot) : nav_(nav), autopilot_(autopilot) {}

  void update(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& report) override {
    // A non-finite sample would poison every downstream integrator (NaN never washes out);
    // treat it like a missing fix so the staleness guard takes over if the source stays bad.
    if (!std::isfinite(report.position().geodeticLatitude()) || !std::isfinite(report.position().geodeticLongitude()) ||
        !std::isfinite(report.attitude().yaw().yaw())) {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Dropping non-finite Global Pose sample")
      return;
    }
    nav_->setPose(report);
    autopilot_->onNavUpdate();
  }

 private:
  NavState* nav_;
  IAutopilot* autopilot_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_GLOBALPOSEOBSERVER_HPP_
