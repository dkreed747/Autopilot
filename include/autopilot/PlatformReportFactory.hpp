//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
//
// Applied Research Laboratory
// Pennsylvania State University
// P.O. Box 30
// State College, PA 16804-0030
//
// DISTRIBUTION STATEMENT A. Approved for public release.
// Distribution is unlimited.
// This software was developed by the Department of the Navy,
// NAVSEA Unmanned and Small Combatants. It is provided under the terms of
// use found in the LICENSE file at the source code root directory.
//
//---------------------------------------------------------------------------

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
