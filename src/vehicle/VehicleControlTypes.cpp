#include "autopilot/vehicle/VehicleControlTypes.hpp"

namespace arlcore::autopilot {

const std::vector<std::string>& vehicleControlTypes() {
  // Add a name here and a branch in makeVehicleControl together; VehicleControlFactoryTest asserts
  // every registered name is buildable, so a name without a branch fails the build's test step.
  static const std::vector<std::string> types = {"sim"};
  return types;
}

bool isKnownVehicleControlType(const std::string& type) {
  for (const std::string& known : vehicleControlTypes()) {
    if (known == type) {
      return true;
    }
  }
  return false;
}

std::string joinVehicleControlTypes() {
  std::string out;
  for (const std::string& known : vehicleControlTypes()) {
    if (!out.empty()) {
      out += ", ";
    }
    out += "'" + known + "'";
  }
  return out;
}

}  // namespace arlcore::autopilot
