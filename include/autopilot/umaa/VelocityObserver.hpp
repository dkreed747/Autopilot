#ifndef AUTOPILOT_UMAA_VELOCITYOBSERVER_HPP_
#define AUTOPILOT_UMAA_VELOCITYOBSERVER_HPP_

#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>

#include "Observer.h"
#include "autopilot/core/NavState.hpp"

namespace arlcore::autopilot {

//! \brief Observes Velocity reports; refreshes nav state for the next pose-driven tick.
class VelocityObserver : public arlcore::Observer<UMAA::SA::VelocityStatus::VelocityReportType> {
 public:
  explicit VelocityObserver(NavState* nav) : nav_(nav) {}

  void update(const UMAA::SA::VelocityStatus::VelocityReportType& report) override { nav_->setVelocity(report); }

 private:
  NavState* nav_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_VELOCITYOBSERVER_HPP_
