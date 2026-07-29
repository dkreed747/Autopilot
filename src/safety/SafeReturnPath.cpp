#include "autopilot/safety/SafeReturnPath.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>
#include <string>
#include <utility>

#include "InternalTypes.h"
#include "Logger.h"

namespace arlcore::autopilot {

SafeReturnPath::SafeReturnPath(SrpConfig config, std::vector<MissionWaypoint> waypoints)
    : config_(std::move(config)), waypoints_(std::move(waypoints)) {}

std::optional<SafeReturnPath> SafeReturnPath::load(const SrpConfig& config, std::string* error) {
  if (error != nullptr) {
    error->clear();
  }
  if (config.csvPath.empty()) {
    return std::nullopt;  // no SRP configured; not an error
  }

  auto fail = [&](const std::string& message) -> std::optional<SafeReturnPath> {
    if (error != nullptr) {
      *error = message;
    }
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Safe Return Path rejected: " << message)
    return std::nullopt;
  };

  if (!config.originLatDeg.has_value() || !config.originLonDeg.has_value()) {
    return fail(
        "srp.csv_path is set but origin_lat_deg/origin_lon_deg are missing (the SRP "
        "anchor must be explicit)");
  }
  const flt64_t lat = config.originLatDeg.value();
  const flt64_t lon = config.originLonDeg.value();
  if (!std::isfinite(lat) || !std::isfinite(lon) || std::abs(lat) > 90.0 || std::abs(lon) > 180.0) {
    return fail("srp origin is not a valid lat/lon");
  }

  const GeographicLib::LocalCartesian frame(lat, lon, 0.0);
  std::vector<MissionWaypoint> waypoints = loadMissionCsv(config.csvPath, frame);
  if (waypoints.empty()) {
    return fail("srp csv '" + config.csvPath + "' is missing, unreadable, or holds no waypoints");
  }

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const MissionWaypoint& wp = waypoints[i];
    const std::string at = " (waypoint " + std::to_string(i + 1) + ")";
    if (!std::isfinite(wp.latDeg) || !std::isfinite(wp.lonDeg)) {
      return fail("srp waypoint position is not finite" + at);
    }
    if (!(wp.speedMps > 0.0) || !std::isfinite(wp.speedMps)) {
      return fail("srp waypoint speed must be positive" + at);
    }
    if (!(wp.captureRadiusM > 0.0) || !std::isfinite(wp.captureRadiusM)) {
      return fail("srp waypoint capture radius must be positive" + at);
    }
  }

  if (config.holdRadiusM <= 0.0) {
    return fail("srp hold_radius_m must be positive");
  }
  if (config.repositionSpeedMps <= 0.0) {
    return fail("srp reposition_speed_mps must be positive");
  }

  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Safe Return Path loaded: " << waypoints.size() << " waypoints from '"
                                                                 << config.csvPath << "' anchored at " << lat << ", "
                                                                 << lon)
  return SafeReturnPath(config, std::move(waypoints));
}

}  // namespace arlcore::autopilot
