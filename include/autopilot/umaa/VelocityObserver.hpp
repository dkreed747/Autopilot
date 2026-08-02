#ifndef AUTOPILOT_UMAA_VELOCITYOBSERVER_HPP_
#define AUTOPILOT_UMAA_VELOCITYOBSERVER_HPP_

#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>
#include <cmath>

#include "Logger.h"
#include "Observer.h"
#include "autopilot/core/NavState.hpp"

namespace arlcore::autopilot {

//! \brief Observes Velocity reports; refreshes nav state for the next pose-driven tick.
class VelocityObserver : public arlcore::Observer<UMAA::SA::VelocityStatus::VelocityReportType> {
 public:
  explicit VelocityObserver(NavState* nav) : nav_(nav) {}

  //! Screens the yaw rate before storing: a non-finite sample would otherwise replace a good
  //! report and refresh its timestamp, making a bad source look fresh to the tracker.
  void update(const UMAA::SA::VelocityStatus::VelocityReportType& report) override {
    if (!std::isfinite(report.attitudeRate().yawRate())) {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Dropping Velocity report with a non-finite yaw rate")
      return;
    }
    nav_->setVelocity(report);
  }

 private:
  NavState* nav_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_VELOCITYOBSERVER_HPP_
