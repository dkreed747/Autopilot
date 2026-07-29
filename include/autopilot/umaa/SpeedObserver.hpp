#ifndef AUTOPILOT_UMAA_SPEEDOBSERVER_HPP_
#define AUTOPILOT_UMAA_SPEEDOBSERVER_HPP_

#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>

#include "Observer.h"
#include "autopilot/core/NavState.hpp"

namespace arlcore::autopilot {

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

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_SPEEDOBSERVER_HPP_
