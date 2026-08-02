#ifndef AUTOPILOT_VEHICLE_NAVREPORTSENDERS_HPP_
#define AUTOPILOT_VEHICLE_NAVREPORTSENDERS_HPP_

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>
#include <memory>

#include "SenderBase.h"

namespace arlcore::autopilot {

//! \brief The three SA navigation report senders the app offers a vehicle-control strategy.
//!
//! Only a strategy that *is* the platform's navigation source uses these: SimVehicleControl
//! publishes them because there is no nav suite behind it. A strategy on a real hull leaves all
//! three unused, because the vehicle's own nav suite already publishes those topics and a second
//! publisher makes guidance recompute against alternating truths at double rate.
//! Bundled rather than passed positionally so a new strategy's signature does not grow with the
//! transport, and so tests can inject LocalReaderSender in one place.
struct NavReportSenders {
  std::shared_ptr<arlcore::io::SenderBase<UMAA::SA::GlobalPoseStatus::GlobalPoseReportType>> pose;
  std::shared_ptr<arlcore::io::SenderBase<UMAA::SA::SpeedStatus::SpeedReportType>> speed;
  std::shared_ptr<arlcore::io::SenderBase<UMAA::SA::VelocityStatus::VelocityReportType>> velocity;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_VEHICLE_NAVREPORTSENDERS_HPP_
