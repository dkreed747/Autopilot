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
    arbiter_(config.arbitration) {
  // Static side of the output clamp: the autopilot's own constraint settings merged with the
  // platform's speed capability. Dynamic active constraints merge in per emit.
  staticClampLimits_.minSpeedMps = config_.constraints.minSpeedMps;
  staticClampLimits_.maxSpeedMps = config_.constraints.maxSpeedMps;
  staticClampLimits_.minDepthM = config_.constraints.minDepthM;
  staticClampLimits_.maxDepthM = config_.constraints.maxDepthM;
  const std::optional<double>& capSpeed = config_.platformCapabilities.surface.maxForwardSpeedMps;
  if (capSpeed.has_value() && (!staticClampLimits_.maxSpeedMps.has_value() ||
                               capSpeed.value() < staticClampLimits_.maxSpeedMps.value())) {
    staticClampLimits_.maxSpeedMps = capSpeed;
  }
}

void AutopilotBrain::setConstraintSource(const IConstraintSource* source) {
  std::lock_guard<std::mutex> lock(mtx_);
  constraintSource_ = source;
}

void AutopilotBrain::setZoneMap(const ZoneMap* zoneMap) {
  std::lock_guard<std::mutex> lock(mtx_);
  zoneMap_ = zoneMap;
  planner_.setZones(zoneMap);
  safePlanner_.setZones(zoneMap);
  if (zoneMap != nullptr) {
    vectorGuidance_ = std::make_unique<VectorZoneGuidance>(
        config_.vectorAvoidance, derivePlannerParams().turnRadiusM, zoneMap->config().safetyMarginM);
    const CapabilityLimits& surf = config_.platformCapabilities.surface;
    const double cruise = surf.cruisingSpeedMps.value_or(surf.maxForwardSpeedMps.value_or(1.5));
    recovery_ = std::make_unique<RecoveryGuidance>(config_.recovery, cruise,
                                                   zoneMap->config().safetyMarginM);
  } else {
    vectorGuidance_.reset();
    recovery_.reset();
  }
}

void AutopilotBrain::emitHold(const std::optional<GlobalPoseReportType>& pose) {
  ControlVector hold;
  hold.headingRad = pose.has_value() ? pose->attitude().yaw().yaw() : 0.0;
  hold.speedMps = 0.0;
  emitControl(hold);
}

bool AutopilotBrain::activateSafeRoute(
    const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints) {
  std::lock_guard<std::mutex> lock(mtx_);
  arbiter_.acquire(DriveSource::SAFE);
  mode_ = DriveSource::SAFE;
  recovering_ = false;
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Safe route requested without a navigation fix; "
      "holding zero speed instead")
    safeHold_ = true;
    return false;
  }
  safePlanner_.setZones(zoneMap_);
  safePlanner_.plan(waypoints, pose.value(), derivePlannerParams());
  if (safePlanner_.failed()) {
    // Safe mode often engages while the vehicle is IN violation, where no zone-compliant path
    // out of the current position exists. Reaching the safe route trumps zone margins: retry
    // zone-blind rather than parking the vehicle in the violating region.
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Safe route is zone-blocked from the current position; "
      "replanning zone-blind (the SRP takes precedence over zone margins)")
    safePlanner_.setZones(nullptr);
    safePlanner_.plan(waypoints, pose.value(), derivePlannerParams());
  }
  if (safePlanner_.failed()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Safe route could not be planned; holding zero speed instead")
    safeHold_ = true;
    emitHold(pose);
    return false;
  }
  safeHold_ = false;
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: SAFE mode running a "
    << waypoints.size() << "-waypoint safe route")
  return true;
}

void AutopilotBrain::activateSafeHold() {
  std::lock_guard<std::mutex> lock(mtx_);
  arbiter_.acquire(DriveSource::SAFE);
  mode_ = DriveSource::SAFE;
  recovering_ = false;
  safeHold_ = true;
  emitHold(nav_->pose());
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: SAFE mode holding zero speed")
}

WaypointProgress AutopilotBrain::safeProgress() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return safePlanner_.progress();
}

bool AutopilotBrain::safeRouteComplete() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return !safeHold_ && safePlanner_.routeComplete();
}

bool AutopilotBrain::safeRouteFailed() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return !safeHold_ && safePlanner_.failed();
}

void AutopilotBrain::clearSafeMode() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (mode_ != DriveSource::SAFE) {
    return;
  }
  mode_ = DriveSource::NONE;
  safeHold_ = false;
  arbiter_.release(DriveSource::SAFE);
  emitHold(nav_->pose());
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: SAFE mode released; accepting commands")
}

bool AutopilotBrain::beginRecovery() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!recovery_ || zoneMap_ == nullptr) {
    return false;
  }
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  recovering_ = true;  // ticks route through recovery even without a target (zero-hold)
  if (!pose.has_value()) {
    return false;
  }
  const GeoPoint at{pose->position().geodeticLatitude(), pose->position().geodeticLongitude()};
  const double depth = pose->depth().has_value() ? pose->depth().value() : 0.0;
  const std::optional<double> asf = pose->altitudeASF().has_value()
      ? std::optional<double>(pose->altitudeASF().value()) : std::nullopt;
  const bool found = recovery_->begin(at, depth, asf, *zoneMap_);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: zone recovery engaged"
    << (found ? "" : " (no target found; holding zero speed while grace runs)"))
  return found;
}

bool AutopilotBrain::recoveryComplete() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!recovery_ || zoneMap_ == nullptr) {
    return false;
  }
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    return false;
  }
  const GeoPoint at{pose->position().geodeticLatitude(), pose->position().geodeticLongitude()};
  const double depth = pose->depth().has_value() ? pose->depth().value() : 0.0;
  const std::optional<double> asf = pose->altitudeASF().has_value()
      ? std::optional<double>(pose->altitudeASF().value()) : std::nullopt;
  return recovery_->complete(at, depth, asf, *zoneMap_);
}

void AutopilotBrain::endRecovery() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!recovering_) {
    return;
  }
  recovering_ = false;
  if (recovery_) {
    recovery_->end();
  }
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (mode_ == DriveSource::WAYPOINT && pose.has_value()) {
    planner_.replanCurrentLegFrom(pose.value());
  }
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: recovery complete; resuming the command")
}

void AutopilotBrain::abortRecovery() {
  std::lock_guard<std::mutex> lock(mtx_);
  recovering_ = false;
  if (recovery_) {
    recovery_->end();
  }
}

bool AutopilotBrain::recovering() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return recovering_;
}

void AutopilotBrain::emitControl(const ControlVector& cv) {
  // MANUAL is polled from the strategy directly (not the mode FSM) so actuation safety never
  // depends on whether the mode services are configured. While engaged nothing reaches the
  // platform -- not even safe-mode outputs or zero-speed holds, which would fight the human.
  if (vehicle_->isManualEngaged()) {
    if (!manualSuppressed_) {
      manualSuppressed_ = true;
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "MANUAL control engaged; suppressing all autopilot actuation")
    }
    return;
  }
  if (manualSuppressed_) {
    manualSuppressed_ = false;
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "MANUAL control released; autopilot actuation resumed")
  }
  if (constraintSource_ == nullptr) {
    vehicle_->sendControlVector(cv);
    return;
  }
  const ClampResult result = applyConstraintClamps(cv, constraintSource_->snapshot(), staticClampLimits_);
  if (result.speedClamped != lastSpeedClamped_ || result.elevationClamped != lastElevationClamped_) {
    lastSpeedClamped_ = result.speedClamped;
    lastElevationClamped_ = result.elevationClamped;
    if (result.speedClamped || result.elevationClamped) {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Constraint clamp engaged (speed=" << result.speedClamped
        << ", elevation=" << result.elevationClamped << ")")
    } else {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Constraint clamp released")
    }
  }
  if (result.conflict) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Contradictory speed/depth constraint bounds; the max bound won")
  }
  vehicle_->sendControlVector(result.cv);
}

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
  if (vectorGuidance_) {
    vectorGuidance_->reset();
  }
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
  plannedConstraintRevision_ = constraintSource_ != nullptr ? constraintSource_->revision() : 0;
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
    emitControl(hold);
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
  emitControl(hold);
  UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Navigation stale (" << ageMs.value()
    << " ms > " << config_.loop.navStalenessTimeoutMs << " ms); commanding zero-speed hold")
}

void AutopilotBrain::onNavUpdate() {
  std::lock_guard<std::mutex> lock(mtx_);
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    return;
  }
  // Recovery overrides whatever guidance is (or is not) active: an installed command stays
  // EXECUTING with its progress frozen, and an idle vehicle is still driven back to compliance.
  if (recovering_ && mode_ != DriveSource::SAFE) {
    updateRecoveryControl(pose.value());
    return;
  }
  switch (mode_) {
    case DriveSource::VECTOR:
      updateVectorControl(pose.value());
      break;
    case DriveSource::WAYPOINT:
      updateWaypointControl(pose.value());
      break;
    case DriveSource::SAFE:
      updateSafeControl(pose.value());
      break;
    default:
      break;
  }
}

void AutopilotBrain::updateRecoveryControl(const GlobalPoseReportType& pose) {
  if (recovery_ && zoneMap_ != nullptr) {
    const GeoPoint at{pose.position().geodeticLatitude(), pose.position().geodeticLongitude()};
    const double depth = pose.depth().has_value() ? pose.depth().value() : 0.0;
    const std::optional<double> asf = pose.altitudeASF().has_value()
        ? std::optional<double>(pose.altitudeASF().value()) : std::nullopt;
    const std::optional<ControlVector> cv = recovery_->tick(at, depth, asf, *zoneMap_);
    if (cv.has_value()) {
      emitControl(cv.value());
      return;
    }
  }
  emitHold(pose);
}

void AutopilotBrain::updateSafeControl(const GlobalPoseReportType& pose) {
  if (!safeHold_ && safePlanner_.hasRoute() && !safePlanner_.failed()) {
    emitControl(safePlanner_.update(pose, nav_->groundSpeedMps()));
    return;
  }
  emitHold(pose);
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

  // Tangent-bug zone avoidance: the commanded heading is overridden while it would carry the
  // vehicle into (or out of) an active zone within the lookahead.
  bool avoiding = false;
  if (vectorGuidance_ && zoneMap_ != nullptr && zoneMap_->hasZones() &&
      zoneMap_->anchor().has_value()) {
    const double depthM = pose.depth().has_value() ? pose.depth().value() : 0.0;
    const std::optional<double> asfM = pose.altitudeASF().has_value()
        ? std::optional<double>(pose.altitudeASF().value()) : std::nullopt;
    const ZoneSet zones = zoneMap_->activeSet(zoneMap_->anchor().value(),
                                              ElevationEnvelope::atPoint(depthM, asfM));
    if (!zones.empty()) {
      double xE = 0.0;
      double yN = 0.0;
      double z = 0.0;
      zoneMap_->anchor()->Forward(pose.position().geodeticLatitude(),
                                  pose.position().geodeticLongitude(), 0.0, xE, yN, z);
      cv.headingRad = vectorGuidance_->steer(cv.headingRad, Vec2{xE, yN},
                                             nav_->groundSpeedMps(), zones);
      avoiding = vectorGuidance_->avoidanceActive();
    }
  }

  emitControl(cv);

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
  // Suspended while zone avoidance overrides the heading: the deviation is deliberate.
  const bool allAchieved = prog.directionAchieved && prog.speedAchieved && prog.elevationAchieved;
  if (avoiding) {
    vectorViolationSince_.reset();
  } else if (allAchieved) {
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
  // A constraint-set change mid-route re-gates the zones and replans the current leg when it
  // is now blocked (or fails the route when the target waypoint became non-compliant).
  if (constraintSource_ != nullptr && constraintSource_->revision() != plannedConstraintRevision_) {
    plannedConstraintRevision_ = constraintSource_->revision();
    planner_.onConstraintsChanged(pose);
  }
  const ControlVector cv = planner_.update(pose, nav_->groundSpeedMps());
  emitControl(cv);

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
