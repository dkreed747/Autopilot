#include "SafeModeStrategy.h"

#include <cmath>
#include <utility>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "AutopilotBrain.h"
#include "Logger.h"
#include "MissionRoute.h"

namespace arlcore::autopilot {

namespace {

using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;

//! \brief Hold position at zero speed until the supervisor releases safe mode (all-clear).
class ZeroSpeedHoldStrategy : public SafeModeStrategy {
 public:
  void onEnter(AutopilotBrain* brain, NavState* nav) override { brain->activateSafeHold(); }
  void onTick(AutopilotBrain* brain, NavState* nav) override {}
  bool readyToRelease() const override { return false; }
  void onExit(AutopilotBrain* brain) override { brain->clearSafeMode(); }
  const char* name() const override { return "zero_speed_hold"; }
};

//! \brief Run the Safe Return Path mission; afterwards either release (configurable) or hold
//! the last SRP waypoint within the hold radius, repositioning back to its center (arriving
//! headed opposite the drift direction) whenever the vehicle drifts out.
class SrpMissionStrategy : public SafeModeStrategy {
 public:
  explicit SrpMissionStrategy(SafeReturnPath srp) : srp_(std::move(srp)) {}

  void onEnter(AutopilotBrain* brain, NavState* nav) override {
    released_ = false;
    phase_ = Phase::RUNNING;
    std::vector<GlobalWaypointType> waypoints;
    waypoints.reserve(srp_.waypoints().size());
    for (MissionWaypoint wp : srp_.waypoints()) {
      if (srp_.config().safeElevationM.has_value() && !wp.elevValueM.has_value()) {
        wp.elevValueM = srp_.config().safeElevationM;
        wp.elevFrame = "depth";
      }
      waypoints.push_back(makeWaypoint(wp));
    }
    if (!brain->activateSafeRoute(waypoints)) {
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "SRP strategy: safe route rejected; holding in place")
      phase_ = Phase::HOLDING;
    }
  }

  void onTick(AutopilotBrain* brain, NavState* nav) override {
    switch (phase_) {
      case Phase::RUNNING:
        if (brain->safeRouteFailed()) {
          UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "SRP strategy: safe route failed mid-run; "
            "holding in place")
          brain->activateSafeHold();
          phase_ = Phase::HOLDING;
        } else if (brain->safeRouteComplete()) {
          if (srp_.config().acceptCommandsAfterSrp) {
            UMAA_LOG_INFO(util::SYSTEM_LOGGER, "SRP complete; releasing the autopilot to "
              "accept commands")
            released_ = true;
          } else {
            UMAA_LOG_INFO(util::SYSTEM_LOGGER, "SRP complete; holding the final waypoint within "
              << srp_.config().holdRadiusM << " m")
            brain->activateSafeHold();
            phase_ = Phase::HOLDING;
          }
        }
        break;
      case Phase::HOLDING: {
        const auto pose = nav->pose();
        if (!pose.has_value()) {
          break;
        }
        const GeographicLib::LocalCartesian center(srp_.lastWaypoint().latDeg,
                                                   srp_.lastWaypoint().lonDeg, 0.0);
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        center.Forward(pose->position().geodeticLatitude(), pose->position().geodeticLongitude(),
                       0.0, x, y, z);
        if (std::hypot(x, y) > srp_.config().holdRadiusM) {
          // Drive back to the hold center, arriving headed opposite the drift direction (the
          // bearing from the drift position back to the center).
          MissionWaypoint back = srp_.lastWaypoint();
          back.speedMps = srp_.config().repositionSpeedMps;
          back.captureRadiusM = std::max(1.0, srp_.config().holdRadiusM * 0.5);
          back.arrivalYawRad = std::atan2(-x, -y);
          if (srp_.config().safeElevationM.has_value() && !back.elevValueM.has_value()) {
            back.elevValueM = srp_.config().safeElevationM;
            back.elevFrame = "depth";
          }
          UMAA_LOG_INFO(util::SYSTEM_LOGGER, "SRP hold: drifted " << std::hypot(x, y)
            << " m from the hold point; repositioning")
          brain->activateSafeRoute({makeWaypoint(back)});
          phase_ = Phase::REPOSITIONING;
        }
        break;
      }
      case Phase::REPOSITIONING:
        if (brain->safeRouteComplete() || brain->safeRouteFailed()) {
          brain->activateSafeHold();
          phase_ = Phase::HOLDING;
        }
        break;
    }
  }

  bool readyToRelease() const override { return released_; }

  void onExit(AutopilotBrain* brain) override { brain->clearSafeMode(); }

  const char* name() const override { return "srp"; }

 private:
  enum class Phase { RUNNING, HOLDING, REPOSITIONING };

  SafeReturnPath srp_;
  Phase phase_ = Phase::RUNNING;
  bool released_ = false;
};

}  // namespace

std::unique_ptr<SafeModeStrategy> makeSafeModeStrategy(const SafetyConfig& config,
                                                       const std::optional<SafeReturnPath>& srp) {
  if (config.safeMode.strategy == "srp") {
    if (srp.has_value()) {
      return std::make_unique<SrpMissionStrategy>(srp.value());
    }
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "safe_mode.strategy is 'srp' but no SRP is loaded; "
      "using zero-speed hold")
    return std::make_unique<ZeroSpeedHoldStrategy>();
  }
  if (config.safeMode.strategy != "zero_speed_hold") {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Unknown safe_mode.strategy '" << config.safeMode.strategy
      << "'; using zero-speed hold")
  }
  return std::make_unique<ZeroSpeedHoldStrategy>();
}

}  // namespace arlcore::autopilot
