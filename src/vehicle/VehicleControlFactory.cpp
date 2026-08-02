#include "autopilot/vehicle/VehicleControlFactory.hpp"

#include "Logger.h"
#include "UuidFactory.h"
#include "autopilot/vehicle/SimVehicleControl.hpp"
#include "autopilot/vehicle/VehicleControlTypes.hpp"

namespace arlcore::autopilot {

static std::unique_ptr<IVehicleControl> makeSimVehicleControl(const AutopilotConfig& config,
                                                              const NavReportSenders& navSenders) {
  // The sim is its own navigation source, so all three senders are mandatory for it specifically.
  // A real-platform strategy would not check them at all.
  if (navSenders.pose == nullptr || navSenders.speed == nullptr || navSenders.velocity == nullptr) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "vehicle_control.type 'sim' publishes the SA navigation reports itself and needs "
                   "all three senders")
    return nullptr;
  }
  return std::make_unique<SimVehicleControl>(
      config.platformCapabilities, config.simVehicle,
      arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.navSourceId), navSenders.pose,
      navSenders.speed, navSenders.velocity);
}

std::unique_ptr<IVehicleControl> makeVehicleControl(const AutopilotConfig& config, const NavReportSenders& navSenders) {
  if (config.vehicleControlType == "sim") {
    return makeSimVehicleControl(config, navSenders);
  }
  UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "vehicle_control.type '" << config.vehicleControlType
                                                               << "' has no factory branch; expected one of "
                                                               << joinVehicleControlTypes())
  return nullptr;
}

}  // namespace arlcore::autopilot
