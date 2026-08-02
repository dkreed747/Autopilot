#ifndef AUTOPILOT_VEHICLE_VEHICLECONTROLFACTORY_HPP_
#define AUTOPILOT_VEHICLE_VEHICLECONTROLFACTORY_HPP_

#include <memory>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/vehicle/IVehicleControl.hpp"
#include "autopilot/vehicle/NavReportSenders.hpp"

namespace arlcore::autopilot {

//! \brief Build the strategy named by vehicle_control.type, or nullptr.
//!
//! Note the deliberate asymmetry with makeSafeModeStrategy, which falls back to a zero-speed hold
//! on an unknown strategy: there, a degraded safe behaviour beats none. There is no safe fallback
//! for a vehicle-control strategy. Silently substituting the simulator on a real hull means the
//! autopilot computes control vectors that nobody actuates, while every report says it is driving.
//! So an unknown type returns nullptr and startup fails. Config validation rejects unknown types
//! first, so a nullptr from here means a registered name without a factory branch, or missing IO.
//!
//! navSenders is only used by a strategy that is itself the platform's navigation source; see
//! NavReportSenders. Adding a strategy means a branch here plus a name in vehicleControlTypes().
std::unique_ptr<IVehicleControl> makeVehicleControl(const AutopilotConfig& config, const NavReportSenders& navSenders);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_VEHICLE_VEHICLECONTROLFACTORY_HPP_
