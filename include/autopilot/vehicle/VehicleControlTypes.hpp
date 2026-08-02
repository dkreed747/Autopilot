#ifndef AUTOPILOT_VEHICLE_VEHICLECONTROLTYPES_HPP_
#define AUTOPILOT_VEHICLE_VEHICLECONTROLTYPES_HPP_

#include <string>
#include <vector>

namespace arlcore::autopilot {

//! \brief The vehicle_control.type names the factory can build.
//!
//! This lives in autopilot-core rather than beside the factory because config validation is in
//! core and the factory must be in autopilot-app, which constructs concrete strategies. Core
//! cannot call app, so a shared list is the only way for validation to reject an unbuildable type
//! instead of duplicating the names and drifting.
//! Deliberately free of UMAA and DDS includes, so ConfigValidation does not acquire them.

//! \brief Every registered strategy name, in the order a diagnostic should list them.
const std::vector<std::string>& vehicleControlTypes();

//! \brief Whether the factory has a branch for this name. Case sensitive.
bool isKnownVehicleControlType(const std::string& type);

//! \brief The registered names as "'a', 'b'", for error messages.
std::string joinVehicleControlTypes();

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_VEHICLE_VEHICLECONTROLTYPES_HPP_
