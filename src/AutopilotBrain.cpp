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

#include "AutopilotBrain.h"

#include <cmath>
#include <optional>
#include <vector>

#include "AngleMath.h"
#include "Logger.h"
#include "PlannerParamsFactory.h"
#include "ToleranceUtils.h"

namespace arlcore::autopilot {

using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

namespace {

std::optional<double> poseElevation(const GlobalPoseReportType& p, ElevationFrame frame) {
  switch (frame) {
    case ElevationFrame::DEPTH:
      return p.depth().has_value() ? std::optional<double>(p.depth().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_MSL:
      return p.altitude().has_value() ? std::optional<double>(p.altitude().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_AGL:
      return p.altitudeAGL().has_value() ? std::optional<double>(p.altitudeAGL().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_GEODETIC:
      return p.altitudeGeodetic().has_value() ? std::optional<double>(p.altitudeGeodetic().value()) : std::nullopt;
    default:
      return std::nullopt;
  }
}

}  // namespace

AutopilotBrain::AutopilotBrain(NavState* nav, IVehicleControl* vehicle, const AutopilotConfig& config) :
    nav_(nav),
    vehicle_(vehicle),
    config_(config),
    arbiter_(config.arbitration.vectorPriority, config.arbitration.waypointPriority) {}

PlannerParams AutopilotBrain::derivePlannerParams() const {
  return arlcore::autopilot::derivePlannerParams(config_);
}

void AutopilotBrain::setVectorSetpoint(
    const UMAA::MO::GlobalVectorControl::GlobalVectorCommandType& cmd) {
  std::lock_guard<std::mutex> lock(mtx_);
  activeVector_ = cmd;
  mode_ = DriveSource::VECTOR;
  vectorProgress_ = VectorProgress{};
  vectorEverAchieved_ = false;
  vectorViolationSince_.reset();
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: vector setpoint installed")
}

bool AutopilotBrain::setWaypointSetpoint(
    const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints) {
  std::lock_guard<std::mutex> lock(mtx_);
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Cannot plan waypoint route without a navigation fix")
    return false;
  }
  planner_.plan(waypoints, pose.value(), derivePlannerParams());
  waypointProgress_ = planner_.progress();
  mode_ = DriveSource::WAYPOINT;
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: waypoint route installed ("
    << waypoints.size() << " waypoints)")
  return true;
}

void AutopilotBrain::clearSetpoint(DriveSource src) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (mode_ == src) {
    mode_ = DriveSource::NONE;
    // Bring the vehicle to a stop: without this a canceled/failed/preempted command would
    // leave the platform driving on the last setpoint forever.
    const std::optional<GlobalPoseReportType> pose = nav_->pose();
    ControlVector hold;
    hold.headingRad = pose.has_value() ? pose->attitude().yaw().yaw() : 0.0;
    hold.speedMps = 0.0;
    vehicle_->sendControlVector(hold);
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: setpoint cleared; commanding zero-speed hold")
  }
}

void AutopilotBrain::enforceNavStaleness() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (mode_ == DriveSource::NONE) {
    return;
  }
  const std::optional<int64_t> ageMs = nav_->poseAgeMs();
  if (!ageMs.has_value() || ageMs.value() <= config_.loop.navStalenessTimeoutMs) {
    return;
  }
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  ControlVector hold;
  hold.headingRad = pose.has_value() ? pose->attitude().yaw().yaw() : 0.0;
  hold.speedMps = 0.0;
  vehicle_->sendControlVector(hold);
  UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Navigation stale (" << ageMs.value()
    << " ms > " << config_.loop.navStalenessTimeoutMs << " ms); commanding zero-speed hold")
}

void AutopilotBrain::onNavUpdate() {
  std::lock_guard<std::mutex> lock(mtx_);
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    return;
  }
  switch (mode_) {
    case DriveSource::VECTOR:
      updateVectorControl(pose.value());
      break;
    case DriveSource::WAYPOINT:
      updateWaypointControl(pose.value());
      break;
    default:
      break;
  }
}

void AutopilotBrain::updateVectorControl(const GlobalPoseReportType& pose) {
  ControlVector cv;
  const double poseYaw = pose.attitude().yaw().yaw();

  const std::optional<DirectionValue> dir = tolerance::extractDirection(activeVector_.direction());
  cv.headingRad = dir.has_value() ? dir->headingRad : poseYaw;

  const std::optional<SpeedValue> sp = tolerance::extractSpeed(activeVector_.speed());
  cv.speedMps = sp.has_value() ? sp->speedMps : 0.0;

  std::optional<ElevationValue> elev;
  if (activeVector_.elevation().has_value()) {
    elev = tolerance::extractElevation(activeVector_.elevation().value());
    if (elev.has_value()) {
      cv.elevationM = elev->valueM;
      cv.elevationFrame = elev->frame;
    }
  }

  vehicle_->sendControlVector(cv);

  // Achieved-flag evaluation against the commanded tolerances (or configured defaults).
  VectorProgress prog;
  prog.valid = true;
  prog.directionAchieved = dir.has_value() &&
      tolerance::directionAchieved(dir.value(), poseYaw, config_.vectorTolerances.directionRad);

  prog.speedAchieved = sp.has_value() &&
      tolerance::speedAchieved(sp.value(), nav_->groundSpeedMps(), config_.vectorTolerances.speedMps);

  if (elev.has_value()) {
    const std::optional<double> cur = poseElevation(pose, elev->frame);
    prog.elevationAchieved = cur.has_value() &&
        tolerance::elevationAchieved(elev.value(), cur.value(), config_.vectorTolerances.elevationM);
  } else {
    prog.elevationAchieved = true;
  }

  // Hard tolerances: after all criteria have been achieved once, a violation persisting
  // longer than the configured failure delay fails the command (UMAA failureDelay semantics).
  const bool allAchieved = prog.directionAchieved && prog.speedAchieved && prog.elevationAchieved;
  if (allAchieved) {
    vectorEverAchieved_ = true;
    vectorViolationSince_.reset();
  } else if (config_.vectorTolerances.hard && vectorEverAchieved_) {
    const auto now = std::chrono::steady_clock::now();
    if (!vectorViolationSince_.has_value()) {
      vectorViolationSince_ = now;
    } else if (std::chrono::duration<double>(now - vectorViolationSince_.value()).count() >
               config_.vectorTolerances.failureDelayS) {
      prog.hardViolation = true;
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Vector command hard tolerance violated for more than "
        << config_.vectorTolerances.failureDelayS << " s")
    }
  }

  vectorProgress_ = prog;
}

void AutopilotBrain::updateWaypointControl(const GlobalPoseReportType& pose) {
  const ControlVector cv = planner_.update(pose, nav_->groundSpeedMps());
  vehicle_->sendControlVector(cv);

  WaypointProgress prog = planner_.progress();
  // The planner does not see speed; evaluate speed achievement here from the nav fix.
  prog.groundSpeedMps = nav_->groundSpeedMps();
  prog.speedAchieved = std::fabs(prog.groundSpeedMps - cv.speedMps) <= config_.vectorTolerances.speedMps;
  waypointProgress_ = prog;
}

VectorProgress AutopilotBrain::vectorProgress() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return vectorProgress_;
}

WaypointProgress AutopilotBrain::waypointProgress() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return waypointProgress_;
}

DriveSource AutopilotBrain::mode() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return mode_;
}

}  // namespace arlcore::autopilot
