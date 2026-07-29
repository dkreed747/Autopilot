#include "autopilot/umaa/PlatformReportFactory.hpp"

#include <optional>
#include "InternalTypes.h"

namespace arlcore::autopilot {

using UMAA::EO::UVPlatformSpecs::SurfaceCapabilityLimitsType;
using UMAA::EO::UVPlatformSpecs::UnderwaterCapabilityLimitsType;
using UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType;
using UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType;

//! \brief Copy an optional config value into a generated @optional field.
static void setIf(
    std::optional<flt64_t>& field,
    const std::optional<flt64_t>& value) {  // NOLINT(runtime/references)
  if (value.has_value()) {
    field = value.value();
  }
}

UVPlatformSpecsReportType makePlatformSpecsReport(
    const PlatformSpecsConfig& specs) {
  UVPlatformSpecsReportType report;
  report.name() = specs.name;
  report.lengthAtWaterline() = specs.lengthAtWaterlineM;
  report.beamAtWaterline() = specs.beamAtWaterlineM;
  report.draft() = specs.draftM;
  report.forwardDistance() = specs.forwardDistanceM;
  report.aftDistance() = specs.aftDistanceM;
  report.portDistance() = specs.portDistanceM;
  report.starboardDistance() = specs.starboardDistanceM;
  report.topDistance() = specs.topDistanceM;
  report.bottomDistance() = specs.bottomDistanceM;
  report.displacement() = specs.displacementMetricTon;
  report.weightLight() = specs.weightLightMetricTon;
  report.weightLoaded() = specs.weightLoadedMetricTon;
  // centerOfBuoyancy, centerOfGravity, and referenceFrameOrigin keep generated
  // defaults.
  return report;
}

UVPlatformCapabilitiesReportType makePlatformCapabilitiesReport(
    const PlatformCapabilitiesConfig& caps) {
  UVPlatformCapabilitiesReportType report;
  report.minWaterDepth() = caps.minWaterDepthM;

  SurfaceCapabilityLimitsType surface;
  setIf(surface.maxForwardSpeed(), caps.surface.maxForwardSpeedMps);
  setIf(surface.maxReverseSpeed(), caps.surface.maxReverseSpeedMps);
  setIf(surface.cruisingSpeed(), caps.surface.cruisingSpeedMps);
  setIf(surface.maxTurnRate(), caps.surface.maxTurnRateRps);
  setIf(surface.minSpeedInMedium(), caps.surface.minSpeedInMediumMps);
  report.surfaceCapabilities() = surface;

  if (caps.underwaterEnabled) {
    UnderwaterCapabilityLimitsType underwater;
    setIf(underwater.maxForwardSpeed(), caps.underwater.maxForwardSpeedMps);
    setIf(underwater.maxReverseSpeed(), caps.underwater.maxReverseSpeedMps);
    setIf(underwater.cruisingSpeed(), caps.underwater.cruisingSpeedMps);
    setIf(underwater.maxTurnRate(), caps.underwater.maxTurnRateRps);
    setIf(underwater.minSpeedInMedium(), caps.underwater.minSpeedInMediumMps);
    setIf(underwater.maxDepthChangeRate(),
          caps.underwater.maxDepthChangeRateMps);
    report.underwaterCapabilities() = underwater;
  }

  return report;
}

}  // namespace arlcore::autopilot
