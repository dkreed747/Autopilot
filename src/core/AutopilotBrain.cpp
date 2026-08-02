#include "autopilot/core/AutopilotBrain.hpp"

#include <cmath>
#include <optional>
#include <vector>

#include "InternalTypes.h"
#include "Logger.h"
#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/guidance/ElevationUtils.hpp"
#include "autopilot/guidance/PlannerParamsFactory.hpp"
#include "autopilot/guidance/ToleranceUtils.hpp"

namespace arlcore::autopilot {

using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

AutopilotBrain::AutopilotBrain(NavState* nav, IVehicleControl* vehicle, const AutopilotConfig& config)
    : nav_(nav), vehicle_(vehicle), config_(config), arbiter_(config.arbitration) {
  // Static side of the output clamp (own constraint settings merged with the platform's speed
  // capability); dynamic active constraints merge in per emit.
  staticClampLimits_.minSpeedMps = config_.constraints.minSpeedMps;
  staticClampLimits_.maxSpeedMps = config_.constraints.maxSpeedMps;
  staticClampLimits_.minDepthM = config_.constraints.minDepthM;
  staticClampLimits_.maxDepthM = config_.constraints.maxDepthM;
  staticClampLimits_.minAltitudeAsfM = config_.constraints.minAltitudeAsfM;
  const std::optional<flt64_t>& capSpeed = config_.platformCapabilities.surface.maxForwardSpeedMps;
  if (capSpeed.has_value() &&
      (!staticClampLimits_.maxSpeedMps.has_value() || capSpeed.value() < staticClampLimits_.maxSpeedMps.value())) {
    staticClampLimits_.maxSpeedMps = capSpeed;
  }
}

void AutopilotBrain::setConstraintSource(const IConstraintSource* source) {
  std::scoped_lock lock(mtx_);
  constraintSource_ = source;
}

void AutopilotBrain::setZoneMap(const ZoneMap* zoneMap) {
  std::scoped_lock lock(mtx_);
  zoneMap_ = zoneMap;
  planner_.setZones(zoneMap);
  safePlanner_.setZones(zoneMap);
  if (zoneMap != nullptr) {
    vectorGuidance_ = std::make_unique<VectorZoneGuidance>(config_.vectorAvoidance, derivePlannerParams().turnRadiusM,
                                                           zoneMap->config().safetyMarginM);
    const CapabilityLimits& surf = config_.platformCapabilities.surface;
    const flt64_t cruise = surf.cruisingSpeedMps.value_or(surf.maxForwardSpeedMps.value_or(1.5));
    recovery_ = std::make_unique<RecoveryGuidance>(config_.recovery, cruise, zoneMap->config().safetyMarginM);
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
  std::scoped_lock lock(mtx_);
  arbiter_.acquire(DriveSource::SAFE);
  mode_ = DriveSource::SAFE;
  recovering_ = false;
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "Safe route requested without a navigation fix; "
                   "holding zero speed instead")
    safeHold_ = true;
    return false;
  }
  safePlanner_.setZones(zoneMap_);
  safePlanner_.plan(waypoints, pose.value(), derivePlannerParams());
  if (safePlanner_.failed()) {
    // Safe mode often engages while the vehicle is IN violation (no zone-compliant path out
    // exists); reaching the safe route trumps zone margins, so retry zone-blind rather than
    // parking the vehicle in the violating region.
    UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                  "Safe route is zone-blocked from the current position; "
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
  UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                "Autopilot brain: SAFE mode running a " << waypoints.size() << "-waypoint safe route")
  return true;
}

void AutopilotBrain::activateSafeHold() {
  std::scoped_lock lock(mtx_);
  arbiter_.acquire(DriveSource::SAFE);
  mode_ = DriveSource::SAFE;
  recovering_ = false;
  safeHold_ = true;
  emitHold(nav_->pose());
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: SAFE mode holding zero speed")
}

WaypointProgress AutopilotBrain::safeProgress() const {
  std::scoped_lock lock(mtx_);
  return safePlanner_.progress();
}

bool AutopilotBrain::safeRouteComplete() const {
  std::scoped_lock lock(mtx_);
  return !safeHold_ && safePlanner_.routeComplete();
}

bool AutopilotBrain::safeRouteFailed() const {
  std::scoped_lock lock(mtx_);
  return !safeHold_ && safePlanner_.failed();
}

void AutopilotBrain::clearSafeMode() {
  std::scoped_lock lock(mtx_);
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
  std::scoped_lock lock(mtx_);
  if (!recovery_ || zoneMap_ == nullptr) {
    return false;
  }
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  recovering_ = true;  // ticks route through recovery even without a target (zero-hold)
  if (!pose.has_value()) {
    return false;
  }
  const GeoPoint at{pose->position().geodeticLatitude(), pose->position().geodeticLongitude()};
  const DepthAsf vertical = elevation::poseDepthAsf(pose.value());
  const bool found = recovery_->begin(at, vertical.depthM, vertical.asfM, *zoneMap_);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: zone recovery engaged"
                                         << (found ? "" : " (no target found; holding zero speed while grace runs)"))
  return found;
}

bool AutopilotBrain::recoveryComplete() {
  std::scoped_lock lock(mtx_);
  if (!recovery_ || zoneMap_ == nullptr) {
    return false;
  }
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    return false;
  }
  const GeoPoint at{pose->position().geodeticLatitude(), pose->position().geodeticLongitude()};
  const DepthAsf vertical = elevation::poseDepthAsf(pose.value());
  return recovery_->complete(at, vertical.depthM, vertical.asfM, *zoneMap_);
}

void AutopilotBrain::endRecovery() {
  std::scoped_lock lock(mtx_);
  if (!recovering_) {
    return;
  }
  recovering_ = false;
  vectorViolationSince_.reset();  // recovery time must not count toward the hard-tolerance delay
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
  std::scoped_lock lock(mtx_);
  recovering_ = false;
  vectorViolationSince_.reset();  // recovery time must not count toward the hard-tolerance delay
  if (recovery_) {
    recovery_->end();
  }
}

bool AutopilotBrain::recovering() const {
  std::scoped_lock lock(mtx_);
  return recovering_;
}

void AutopilotBrain::emitControl(const ControlVector& cv) {
  ControlVector out = cv;
  if (!std::isfinite(out.headingRad) || !std::isfinite(out.speedMps) ||
      (out.elevationM.has_value() && !std::isfinite(out.elevationM.value()))) {
    // Substituting a stop rather than returning: sending nothing leaves the platform driving
    // whatever setpoint it was last given, so a single poisoned cycle would turn a refusal into
    // "keep going indefinitely". Nothing else covers this - the nav-staleness hold only fires on a
    // stale pose, and here navigation is healthy and the command is the problem.
    if (!nonFiniteRefused_) {
      nonFiniteRefused_ = true;
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Non-finite control vector (heading "
                                              << out.headingRad << ", speed " << out.speedMps
                                              << "); commanding a zero-speed hold instead")
    }
    const std::optional<GlobalPoseReportType> pose = nav_->pose();
    const flt64_t heldHeading = pose.has_value() ? pose->attitude().yaw().yaw() : 0.0;
    out = ControlVector();
    out.headingRad = std::isfinite(heldHeading) ? heldHeading : 0.0;
    out.speedMps = 0.0;
  } else if (nonFiniteRefused_) {
    nonFiniteRefused_ = false;
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Control vectors are finite again; normal actuation resumed")
  }
  // MANUAL is polled from the strategy directly (not the mode FSM) so actuation safety never
  // depends on whether the mode services are configured; while engaged nothing reaches the
  // platform, not even safe-mode outputs, which would fight the human.
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
    vehicle_->sendControlVector(out);
    return;
  }
  const ClampResult result =
      applyConstraintClamps(out, constraintSource_->snapshot(), staticClampLimits_, floorDepthM_);
  if (result.speedClamped != lastSpeedClamped_ || result.elevationClamped != lastElevationClamped_) {
    lastSpeedClamped_ = result.speedClamped;
    lastElevationClamped_ = result.elevationClamped;
    if (result.speedClamped || result.elevationClamped) {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Constraint clamp engaged (speed=" << result.speedClamped << ", elevation="
                                                                            << result.elevationClamped << ")")
    } else {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Constraint clamp released")
    }
  }
  if (result.elevationUnbounded != lastElevationUnbounded_) {
    lastElevationUnbounded_ = result.elevationUnbounded;
    if (result.elevationUnbounded) {
      UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                    "No altitude above sea floor to check the commanded elevation against a depth "
                    "bound; dropping the elevation demand (the platform holds its current depth)")
    } else {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Seafloor reference recovered; the elevation demand is enforced again")
    }
  }
  if (result.conflict) {
    UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                  "Contradictory speed bounds or an unsatisfiable vertical window; the deep-side "
                  "bound won and the setpoint was clamped into the water column")
  }
  vehicle_->sendControlVector(result.cv);
}

std::optional<flt64_t> AutopilotBrain::floorDepthM() const {
  std::scoped_lock lock(mtx_);
  return floorDepthM_;
}

PlannerParams AutopilotBrain::derivePlannerParams() const { return arlcore::autopilot::derivePlannerParams(config_); }

void AutopilotBrain::setVectorSetpoint(const UMAA::MO::GlobalVectorControl::GlobalVectorCommandType& cmd) {
  std::scoped_lock lock(mtx_);
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
  std::scoped_lock lock(mtx_);
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "Cannot plan waypoint route without a navigation fix")
    return false;
  }
  planner_.plan(waypoints, pose.value(), derivePlannerParams());
  waypointProgress_ = planner_.progress();
  plannedConstraintRevision_ = constraintSource_ != nullptr ? constraintSource_->revision() : 0;
  mode_ = DriveSource::WAYPOINT;
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Autopilot brain: waypoint route installed (" << waypoints.size() << " waypoints)")
  return true;
}

void AutopilotBrain::clearSetpoint(DriveSource src) {
  std::scoped_lock lock(mtx_);
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
  std::scoped_lock lock(mtx_);
  // Recovery can drive with no command installed (mode NONE): the guard must still fire
  // or the vehicle keeps flying the last recovery setpoint blind when the pose stops.
  if (mode_ == DriveSource::NONE && !recovering_) {
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
  UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Navigation stale (" << ageMs.value() << " ms > "
                                                          << config_.loop.navStalenessTimeoutMs
                                                          << " ms); commanding zero-speed hold")
}

void AutopilotBrain::onNavUpdate() {
  std::scoped_lock lock(mtx_);
  const std::optional<GlobalPoseReportType> pose = nav_->pose();
  if (!pose.has_value()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  navTickDtS_ = lastNavTickAt_.has_value() ? std::chrono::duration<flt64_t>(now - lastNavTickAt_.value()).count() : 0.0;
  lastNavTickAt_ = now;
  floorDepthM_ = elevation::floorDepthM(pose.value());
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
    const DepthAsf vertical = elevation::poseDepthAsf(pose);
    const std::optional<ControlVector> cv = recovery_->tick(at, vertical.depthM, vertical.asfM, *zoneMap_);
    if (cv.has_value()) {
      emitControl(cv.value());
      return;
    }
  }
  emitHold(pose);
}

void AutopilotBrain::updateSafeControl(const GlobalPoseReportType& pose) {
  if (!safeHold_ && safePlanner_.hasRoute() && !safePlanner_.failed()) {
    emitControl(safePlanner_.update(pose, nav_->groundSpeedMps(), navTickDtS_));
    return;
  }
  emitHold(pose);
}

void AutopilotBrain::updateVectorControl(const GlobalPoseReportType& pose) {
  ControlVector cv;
  const flt64_t poseYaw = pose.attitude().yaw().yaw();

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
  if (vectorGuidance_ && zoneMap_ != nullptr && zoneMap_->hasZones() && zoneMap_->anchor().has_value()) {
    const DepthAsf vertical = elevation::poseDepthAsf(pose);
    const ZoneSet zones =
        zoneMap_->activeSet(zoneMap_->anchor().value(), ElevationEnvelope::atPoint(vertical.depthM, vertical.asfM));
    if (!zones.empty()) {
      flt64_t xE = 0.0;
      flt64_t yN = 0.0;
      flt64_t z = 0.0;
      zoneMap_->anchor()->Forward(pose.position().geodeticLatitude(), pose.position().geodeticLongitude(), 0.0, xE, yN,
                                  z);
      cv.headingRad = vectorGuidance_->steer(cv.headingRad, Vec2{xE, yN}, nav_->groundSpeedMps(), zones);
      avoiding = vectorGuidance_->avoidanceActive();
    }
  }

  emitControl(cv);

  // Achieved-flag evaluation against the commanded tolerances (or configured defaults).
  VectorProgress prog;
  prog.valid = true;
  prog.directionAchieved =
      dir.has_value() && tolerance::directionAchieved(dir.value(), poseYaw, config_.vectorTolerances.directionRad);

  prog.speedAchieved =
      sp.has_value() && tolerance::speedAchieved(sp.value(), nav_->groundSpeedMps(), config_.vectorTolerances.speedMps);

  bool elevationEvaluable = true;
  if (elev.has_value()) {
    const std::optional<flt64_t> cur = elevation::poseElevation(pose, elev->frame);
    elevationEvaluable = cur.has_value() && std::isfinite(cur.value());
    prog.elevationAchieved = elevationEvaluable && tolerance::elevationAchieved(elev.value(), cur.value(),
                                                                                config_.vectorTolerances.elevationM);
  } else {
    prog.elevationAchieved = true;
  }
  if (!elevationEvaluable && !elevUnevaluableLogged_) {
    elevUnevaluableLogged_ = true;
    UMAA_LOG_WARN(util::SYSTEM_LOGGER,
                  "Commanded elevation cannot be evaluated (the pose carries no value in the "
                  "commanded frame); reporting it as not achieved and suspending the failure delay")
  } else if (elevationEvaluable && elevUnevaluableLogged_) {
    elevUnevaluableLogged_ = false;
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Commanded elevation is evaluable again")
  }

  // Hard tolerances (UMAA failureDelay semantics): after all criteria have been achieved
  // once, a violation persisting past the failure delay fails the command; suspended while
  // zone avoidance overrides the heading, since that deviation is deliberate, and while the
  // elevation is unevaluable, since a lost bottom lock is not the command failing.
  const bool allAchieved = prog.directionAchieved && prog.speedAchieved && prog.elevationAchieved;
  if (avoiding || !elevationEvaluable) {
    vectorViolationSince_.reset();
  } else if (allAchieved) {
    vectorEverAchieved_ = true;
    vectorViolationSince_.reset();
  } else if (config_.vectorTolerances.hard && vectorEverAchieved_) {
    const auto now = std::chrono::steady_clock::now();
    if (!vectorViolationSince_.has_value()) {
      vectorViolationSince_ = now;
    } else if (std::chrono::duration<flt64_t>(now - vectorViolationSince_.value()).count() >
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
  const ControlVector cv = planner_.update(pose, nav_->groundSpeedMps(), navTickDtS_);
  emitControl(cv);

  WaypointProgress prog = planner_.progress();
  // The planner does not see speed; evaluate speed achievement here from the nav fix.
  prog.groundSpeedMps = nav_->groundSpeedMps();
  prog.speedAchieved = std::fabs(prog.groundSpeedMps - cv.speedMps) <= config_.vectorTolerances.speedMps;
  waypointProgress_ = prog;
}

VectorProgress AutopilotBrain::vectorProgress() const {
  std::scoped_lock lock(mtx_);
  return vectorProgress_;
}

WaypointProgress AutopilotBrain::waypointProgress() const {
  std::scoped_lock lock(mtx_);
  return waypointProgress_;
}

DriveSource AutopilotBrain::mode() const {
  std::scoped_lock lock(mtx_);
  return mode_;
}

}  // namespace arlcore::autopilot
