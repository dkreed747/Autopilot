#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PLATFORMREPORTFACTORY_HPP_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PLATFORMREPORTFACTORY_HPP_

#include <UMAA/EO/UVPlatformSpecs/UVPlatformCapabilitiesReportType.hpp>
#include <UMAA/EO/UVPlatformSpecs/UVPlatformSpecsReportType.hpp>

#include "AutopilotConfig.h"

namespace arlcore::autopilot {

//! \brief Translate the platform_specs YAML block into the UMAA specs report.
//! The data is pure configuration, so publication lives with the app rather
//! than the vehicle strategy.
UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType makePlatformSpecsReport(
    const PlatformSpecsConfig& specs);

//! \brief Translate the platform_capabilities YAML block into the UMAA
//! capabilities report.
UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType
makePlatformCapabilitiesReport(const PlatformCapabilitiesConfig& caps);

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_PLATFORMREPORTFACTORY_HPP_
