#include "autopilot/guidance/DubinsPathPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "InternalTypes.h"
#include "Logger.h"
#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/guidance/ElevationUtils.hpp"
#include "autopilot/guidance/ToleranceUtils.hpp"

namespace arlcore::autopilot {

using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

static flt64_t poseLat(const GlobalPoseReportType& p) { return p.position().geodeticLatitude(); }
static flt64_t poseLon(const GlobalPoseReportType& p) { return p.position().geodeticLongitude(); }
static flt64_t poseYaw(const GlobalPoseReportType& p) { return p.attitude().yaw().yaw(); }

static flt64_t wpLat(const GlobalWaypointType& w) { return w.position().value().geodeticLatitude(); }
static flt64_t wpLon(const GlobalWaypointType& w) { return w.position().value().geodeticLongitude(); }

//! \brief Azimuth (true north, clockwise) <-> math angle (+x east, counterclockwise).
//! The mapping is its own inverse.
static flt64_t azToMath(flt64_t azRad) { return wrapPi(M_PI_2 - azRad); }
static flt64_t mathToAz(flt64_t mathRad) { return wrapPi(M_PI_2 - mathRad); }

//! \brief How much heading change one curvature-preview window may span. It bounds both the
//! feedforward's preview horizon and the 2*pi branch ambiguity in meanPathCurvature, which unwraps
//! a theta difference and is therefore only sign-correct below a half turn.
static constexpr flt64_t kCurvaturePreviewTurnRad = 0.25;

//! \brief Capture-gate half-width for a waypoint (its position tolerance or the default).
static flt64_t gateHalfWidthM(const GlobalWaypointType& wp, const PlannerParams& params) {
  if (wp.position().tolerance().has_value()) {
    return wp.position().tolerance().value().limit();
  }
  return params.posCaptureM;
}

//! \brief The waypoint's commanded speed (0 when the variant is unsupported; validation in
//! the provider rejects such routes before they reach the planner).
static flt64_t waypointSpeedMps(const GlobalWaypointType& wp) {
  const std::optional<SpeedValue> sp = tolerance::extractSpeed(wp.speed());
  return sp.has_value() ? sp->speedMps : 0.0;
}

DubinsPathPlanner::Leg::Leg(std::vector<DubinsPath> chain, flt64_t runway, flt64_t endAz)
    : paths(std::move(chain)), runwayM(runway), endAzimuthRad(endAz) {
  pathStartM.reserve(paths.size());
  dubinsLengthM = 0.0;
  minTurnRadiusM = std::numeric_limits<flt64_t>::infinity();
  for (const DubinsPath& p : paths) {
    pathStartM.push_back(dubinsLengthM);
    dubinsLengthM += p.lengthM();
    if (p.rhoM() > 0.0) {
      minTurnRadiusM = std::min(minTurnRadiusM, p.rhoM());
    }
  }
  lengthM = dubinsLengthM + runwayM;
}

Dubins2DPose DubinsPathPlanner::Leg::sampleChain(flt64_t sM) const {
  std::size_t i = paths.size() - 1;
  while (i > 0 && sM < pathStartM[i]) {
    --i;
  }
  return paths[i].sample(sM - pathStartM[i]);
}

flt64_t DubinsPathPlanner::Leg::curvatureAtChain(flt64_t sM) const {
  // Everything from the virtual goal onward is straight, and an empty chain has no curvature.
  if (sM >= dubinsLengthM) {
    return 0.0;
  }
  const flt64_t s = std::max(0.0, sM);
  // The index walk mirrors sampleChain so the two agree on which path owns a given arc length.
  std::size_t i = paths.size() - 1;
  while (i > 0 && s < pathStartM[i]) {
    --i;
  }
  return paths[i].curvatureAt(s - pathStartM[i]);
}

std::string DubinsPathPlanner::Leg::word() const {
  std::string out;
  for (const DubinsPath& p : paths) {
    if (!out.empty()) {
      out += "+";
    }
    out += p.word();
  }
  return out;
}

void DubinsPathPlanner::toLocal(flt64_t latDeg, flt64_t lonDeg, flt64_t* xE, flt64_t* yN) const {
  flt64_t z = 0.0;
  localFrame_.Forward(latDeg, lonDeg, 0.0, *xE, *yN, z);
}

Dubins2DPose DubinsPathPlanner::sampleExtended(const Leg& leg, flt64_t sM) {
  if (sM <= leg.dubinsLengthM) {
    return leg.sampleChain(sM);
  }
  // Straight continuation covers both the final-approach runway (up to lengthM, ending at
  // the waypoint) and the fly-through extension beyond it.
  Dubins2DPose end = leg.sampleChain(leg.dubinsLengthM);
  const flt64_t over = sM - leg.dubinsLengthM;
  end.x += over * std::cos(end.theta);
  end.y += over * std::sin(end.theta);
  return end;
}

flt64_t DubinsPathPlanner::meanPathCurvature(const Leg& leg, flt64_t sM, flt64_t windowM) {
  // wrapPi below resolves the 2*pi branch differences that chained RRT* paths can carry in their
  // absolute theta, which is only sign-correct while the window spans well under a half turn. The
  // bound is enforced here rather than trusted from the caller, because the two live in different
  // functions and a window that violates it drives the feedforward the wrong way.
  const flt64_t safeWindowM = std::min(std::max(0.0, windowM), kCurvaturePreviewTurnRad * leg.minTurnRadiusM);
  if (safeWindowM <= 1e-6) {
    return -leg.curvatureAtChain(sM);  // math frame -> azimuth frame
  }
  // Derived from sampleExtended, so it agrees with the tracked geometry by construction and
  // needs no special case for the runway, the fly-through extension, or a degenerate leg.
  const flt64_t thetaStart = sampleExtended(leg, sM).theta;
  const flt64_t thetaEnd = sampleExtended(leg, sM + safeWindowM).theta;
  return -wrapPi(thetaEnd - thetaStart) / safeWindowM;
}

bool DubinsPathPlanner::legClear(const Leg& leg, flt64_t fromS) const {
  if (zoneSet_.empty()) {
    return true;
  }
  const flt64_t step = std::max(0.25, params_.rrt.finalCheckStepM);
  for (flt64_t s = std::max(0.0, fromS); s <= leg.lengthM; s += step) {
    const Dubins2DPose p = sampleExtended(leg, s);
    if (zoneSet_.clearanceM(Vec2{p.x, p.y}) < params_.zoneMarginM) {
      return false;
    }
  }
  return true;
}

std::optional<flt64_t> DubinsPathPlanner::compliantArrivalAzimuth(std::size_t wpIndex, flt64_t naturalAz,
                                                                  flt64_t runwayM) const {
  const auto runwayCompliant = [&](flt64_t az) {
    const flt64_t theta = azToMath(az);
    const Vec2 wp{wpX_[wpIndex], wpY_[wpIndex]};
    const Vec2 goal{wp.x - runwayM * std::cos(theta), wp.y - runwayM * std::sin(theta)};
    return zoneSet_.segmentClear(goal, wp, params_.zoneMarginM, std::max(0.25, params_.rrt.finalCheckStepM));
  };

  if (runwayCompliant(naturalAz)) {
    return naturalAz;
  }
  if (waypoints_[wpIndex].attitude().has_value()) {
    // The arrival attitude is commanded; there is no freedom to rotate the approach.
    return std::nullopt;
  }
  // Scan alternates outward from the natural azimuth in 22.5-degree steps.
  const flt64_t step = M_PI / 8.0;
  for (int32_t k = 1; k <= 8; ++k) {
    for (const flt64_t sign : {1.0, -1.0}) {
      const flt64_t az = wrapPi(naturalAz + sign * step * k);
      if (runwayCompliant(az)) {
        return az;
      }
      if (k == 8) {
        break;  // +180 and -180 coincide
      }
    }
  }
  return std::nullopt;
}

void DubinsPathPlanner::refreshZoneSet(const GlobalPoseReportType& pose) {
  zoneSet_ = ZoneSet{};
  if (zoneMap_ == nullptr || !zoneMap_->hasZones()) {
    return;
  }
  // Depth envelope of the whole route: the current depth plus every commanded DEPTH-frame
  // waypoint elevation (the spiral machinery can park the vehicle at any intermediate depth).
  // Elevation frames with no depth equivalent make the envelope unbounded — conservative.
  ElevationEnvelope envelope;
  const flt64_t startDepth = pose.depth().has_value() ? pose.depth().value() : 0.0;
  envelope.minDepthM = startDepth;
  envelope.maxDepthM = startDepth;
  bool unbounded = false;
  for (const GlobalWaypointType& wp : waypoints_) {
    if (!wp.elevation().has_value()) {
      continue;
    }
    const std::optional<ElevationValue> el = tolerance::extractElevation(wp.elevation().value());
    if (!el.has_value() || el->frame != ElevationFrame::DEPTH) {
      unbounded = true;
      break;
    }
    envelope.minDepthM = std::min(envelope.minDepthM, el->valueM);
    envelope.maxDepthM = std::max(envelope.maxDepthM, el->valueM);
  }
  if (unbounded) {
    envelope.minDepthM = -1.0e9;
    envelope.maxDepthM = 1.0e9;
  }
  zoneSet_ = zoneMap_->activeSet(localFrame_, envelope);
}

flt64_t DubinsPathPlanner::arrivalAzimuth(std::size_t wpIndex, flt64_t fromXE, flt64_t fromYN) const {
  const GlobalWaypointType& wp = waypoints_[wpIndex];
  if (wp.attitude().has_value()) {
    return tolerance::extractYaw(wp.attitude().value()).yawRad;
  }
  // Natural fly-through heading: toward the next waypoint, or along the final approach for
  // the last one.
  flt64_t dE = 0.0;
  flt64_t dN = 0.0;
  if (wpIndex + 1 < waypoints_.size()) {
    dE = wpX_[wpIndex + 1] - wpX_[wpIndex];
    dN = wpY_[wpIndex + 1] - wpY_[wpIndex];
  } else {
    dE = wpX_[wpIndex] - fromXE;
    dN = wpY_[wpIndex] - fromYN;
  }
  if (std::hypot(dE, dN) < 1e-9) {
    return 0.0;
  }
  return std::atan2(dE, dN);
}

std::optional<DubinsPathPlanner::Leg> DubinsPathPlanner::buildLeg(const Dubins2DPose& startPose,
                                                                  std::size_t wpIndex) const {
  const flt64_t runwayM = params_.turnRadiusM;
  flt64_t endAz = arrivalAzimuth(wpIndex, startPose.x, startPose.y);
  if (!zoneSet_.empty()) {
    const std::optional<flt64_t> compliantAz = compliantArrivalAzimuth(wpIndex, endAz, runwayM);
    if (!compliantAz.has_value()) {
      UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                     "DubinsPathPlanner: no zone-compliant final approach to "
                     "waypoint "
                         << wpIndex)
      return std::nullopt;
    }
    endAz = compliantAz.value();
  }
  const flt64_t endTheta = azToMath(endAz);
  // Solve the curved portion to a virtual goal one turn radius short of the waypoint along
  // the arrival bearing; the leg then finishes with a straight runway through the waypoint.
  const Dubins2DPose virtualGoal{wpX_[wpIndex] - runwayM * std::cos(endTheta),
                                 wpY_[wpIndex] - runwayM * std::sin(endTheta), endTheta};
  std::optional<DubinsPath> path = DubinsPath::solve(startPose, virtualGoal, params_.turnRadiusM);
  if (!path.has_value()) {
    // solve() only fails on non-finite input; fall back to a degenerate straight run from a
    // sanitized origin so guidance can still make progress.
    path = DubinsPath::solve({0.0, 0.0, 0.0}, virtualGoal, 0.0);
  }
  if (zoneSet_.empty() ||
      zoneSet_.pathClear(path.value(), params_.zoneMarginM, std::max(0.25, params_.rrt.finalCheckStepM))) {
    return Leg({path.value()}, runwayM, endAz);
  }

  // The direct solution clips a zone: fall back to Dubins-RRT*, salted by the waypoint index
  // so the whole route stays deterministic under a fixed seed.
  DubinsRrtParams rrt = params_.rrt;
  rrt.rhoM = params_.turnRadiusM;
  rrt.marginM = params_.zoneMarginM;
  std::optional<std::vector<DubinsPath>> chain =
      planDubinsRrtStar(startPose, virtualGoal, zoneSet_, rrt, static_cast<uint32_t>(wpIndex));
  if (!chain.has_value()) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "DubinsPathPlanner: no zone-compliant path to waypoint "
                                            << wpIndex << " (direct clipped, RRT* found no detour)")
    return std::nullopt;
  }
  return Leg(std::move(chain.value()), runwayM, endAz);
}

void DubinsPathPlanner::plan(const std::vector<GlobalWaypointType>& waypoints, const GlobalPoseReportType& start,
                             const PlannerParams& params) {
  waypoints_ = waypoints;
  params_ = params;
  params_.sampleStepM = std::max(0.5, params_.sampleStepM);
  tracker_.configure(params_.tracker);
  missCounts_.assign(waypoints_.size(), 0);
  targetIndex_ = 0;
  routeComplete_ = waypoints_.empty();
  failed_ = false;
  replanCount_ = 0;
  hasLastPos_ = false;
  legProgressS_ = 0.0;
  tracker_.resetLeg();
  previewCapLogged_ = false;
  lastGateAlongM_.reset();
  elevApproachBudget_.reset();
  elevApproachesUsed_ = 0;
  lastSpiralElevErrM_.reset();
  currentLeg_.reset();
  progress_ = WaypointProgress{};
  progress_.valid = true;
  progress_.routeComplete = routeComplete_;
  progress_.waypointsRemaining = static_cast<int32_t>(waypoints_.size());
  lastVector_ = ControlVector{};

  localFrame_.Reset(poseLat(start), poseLon(start), 0.0);
  wpX_.resize(waypoints_.size());
  wpY_.resize(waypoints_.size());
  for (std::size_t i = 0; i < waypoints_.size(); i++) {
    toLocal(wpLat(waypoints_[i]), wpLon(waypoints_[i]), &wpX_[i], &wpY_[i]);
  }

  if (waypoints_.empty()) {
    return;
  }

  refreshZoneSet(start);

  const Dubins2DPose startPose{0.0, 0.0, azToMath(poseYaw(start))};
  planStartPose_ = startPose;
  currentLeg_ = buildLeg(startPose, 0);
  if (!currentLeg_.has_value()) {
    failed_ = true;
    progress_.failed = true;
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "DubinsPathPlanner: first leg has no zone-compliant path;"
                   " failing route")
    return;
  }
  UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                "DubinsPathPlanner planned route: " << waypoints_.size() << " waypoints, first leg "
                                                    << currentLeg_->lengthM << " m (" << currentLeg_->word()
                                                    << "), turn radius " << params_.turnRadiusM << " m")
}

void DubinsPathPlanner::onConstraintsChanged(const GlobalPoseReportType& pose) {
  if (waypoints_.empty() || routeComplete_ || failed_) {
    return;
  }
  refreshZoneSet(pose);
  if (zoneSet_.empty()) {
    return;  // constraints relaxed away; the current leg stays valid
  }
  // A target waypoint that is no longer compliant cannot be reached legally: fail the route
  // (the provider reports OBJECTIVE_FAILED) rather than silently skipping it.
  if (targetIndex_ < waypoints_.size() && !zoneSet_.pointCompliant(Vec2{wpX_[targetIndex_], wpY_[targetIndex_]}, 0.0)) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "DubinsPathPlanner: constraint change put waypoint "
                                            << targetIndex_ << " inside a violating zone; failing route")
    failed_ = true;
    progress_.failed = true;
    return;
  }
  if (currentLeg_.has_value() && !legClear(currentLeg_.value(), legProgressS_)) {
    UMAA_LOG_INFO(util::SYSTEM_LOGGER,
                  "DubinsPathPlanner: constraint change blocked the current "
                  "leg; replanning from the live pose")
    replanCurrentLegFrom(pose);
  }
}

bool DubinsPathPlanner::replanCurrentLegFrom(const GlobalPoseReportType& pose) {
  if (waypoints_.empty() || routeComplete_ || failed_ || targetIndex_ >= waypoints_.size()) {
    return false;
  }
  flt64_t xE = 0.0;
  flt64_t yN = 0.0;
  toLocal(poseLat(pose), poseLon(pose), &xE, &yN);
  const Dubins2DPose current{xE, yN, azToMath(poseYaw(pose))};
  currentLeg_ = buildLeg(current, targetIndex_);
  if (!currentLeg_.has_value()) {
    failed_ = true;
    progress_.failed = true;
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "DubinsPathPlanner: replan from live pose found no "
                   "zone-compliant leg to waypoint "
                       << targetIndex_ << "; failing route")
    return false;
  }
  legProgressS_ = 0.0;
  tracker_.resetLeg();
  lastGateAlongM_.reset();
  elevApproachBudget_.reset();
  elevApproachesUsed_ = 0;
  lastSpiralElevErrM_.reset();
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner: current leg replanned from live pose: "
                                         << currentLeg_->lengthM << " m (" << currentLeg_->word() << ")")
  return true;
}

std::vector<std::pair<flt64_t, flt64_t>> DubinsPathPlanner::previewRoute(flt64_t stepM) const {
  std::vector<std::pair<flt64_t, flt64_t>> out;
  for (const PreviewSample& sample : previewRouteDetailed(stepM)) {
    out.emplace_back(sample.latDeg, sample.lonDeg);
  }
  return out;
}

std::vector<PreviewSample> DubinsPathPlanner::previewRouteDetailed(flt64_t stepM) const {
  std::vector<PreviewSample> out;
  if (waypoints_.empty()) {
    return out;
  }
  const flt64_t step = std::max(0.5, stepM);
  Dubins2DPose legStart = planStartPose_;
  flt64_t routeS = 0.0;
  for (std::size_t i = 0; i < waypoints_.size(); i++) {
    const std::optional<Leg> built = buildLeg(legStart, i);
    if (!built.has_value()) {
      break;  // preview what is plannable; the live planner fails the route at this leg
    }
    const Leg& leg = built.value();
    for (flt64_t s = 0.0; s <= leg.lengthM + step * 0.5; s += step) {
      const flt64_t legS = std::min(s, leg.lengthM);
      const Dubins2DPose p = sampleExtended(leg, legS);
      PreviewSample sample;
      flt64_t h = 0.0;
      localFrame_.Reverse(p.x, p.y, 0.0, sample.latDeg, sample.lonDeg, h);
      sample.arcLengthM = routeS + legS;
      sample.curvatureMathRadPerM = leg.curvatureAtChain(legS);
      sample.azimuthRad = mathToAz(p.theta);
      sample.legIndex = static_cast<int32_t>(i);
      out.push_back(sample);
    }
    routeS += leg.lengthM;
    legStart = Dubins2DPose{wpX_[i], wpY_[i], azToMath(leg.endAzimuthRad)};
  }
  return out;
}

CaptureResult DubinsPathPlanner::evaluateCapture(const GlobalPoseReportType& pose, flt64_t gateLateralM) const {
  CaptureResult result;
  const GlobalWaypointType& wp = waypoints_[targetIndex_];

  result.positionAchieved = std::fabs(gateLateralM) <= gateHalfWidthM(wp, params_);

  if (wp.attitude().has_value()) {
    const AttitudeValue att = tolerance::extractYaw(wp.attitude().value());
    result.attitudeAchieved = tolerance::attitudeAchieved(att, poseYaw(pose), params_.yawCaptureRad);
  }

  if (wp.elevation().has_value()) {
    const std::optional<ElevationValue> el = tolerance::extractElevation(wp.elevation().value());
    if (el.has_value()) {
      const std::optional<flt64_t> cur = elevation::poseElevation(pose, el->frame);
      result.elevationEvaluable = cur.has_value() && std::isfinite(cur.value());
      result.elevationAchieved =
          result.elevationEvaluable && tolerance::elevationAchieved(el.value(), cur.value(), params_.elevCaptureM);
    } else {
      result.elevationAchieved = false;
    }
  }

  const bool attitudeOk = !result.attitudeAchieved.has_value() || result.attitudeAchieved.value();
  // An unevaluable elevation deliberately still gates capture: completing a depth-required
  // waypoint whose depth was never measured is a false success reported over UMAA. A transient
  // dropout is absorbed by the miss budget instead - the route only fails once it is exhausted,
  // which is the same rule a vehicle that simply cannot make depth already meets.
  result.captured =
      result.positionAchieved && attitudeOk && (result.elevationAchieved || !params_.elevationCountsAsMiss);
  return result;
}

void DubinsPathPlanner::advanceToNextWaypoint() {
  const flt64_t arrivalAz = currentLeg_->endAzimuthRad;
  const flt64_t fromX = wpX_[targetIndex_];
  const flt64_t fromY = wpY_[targetIndex_];
  targetIndex_++;
  legProgressS_ = 0.0;
  tracker_.resetLeg();
  lastGateAlongM_.reset();
  elevApproachBudget_.reset();
  elevApproachesUsed_ = 0;
  lastSpiralElevErrM_.reset();
  if (targetIndex_ >= waypoints_.size()) {
    routeComplete_ = true;
    currentLeg_.reset();
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner route complete")
    return;
  }
  const Dubins2DPose startPose{fromX, fromY, azToMath(arrivalAz)};
  currentLeg_ = buildLeg(startPose, targetIndex_);
  if (!currentLeg_.has_value()) {
    failed_ = true;
    progress_.failed = true;
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "DubinsPathPlanner: no zone-compliant leg to waypoint " << targetIndex_ << "; failing route")
    return;
  }
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner advancing to waypoint " << targetIndex_ << ", leg "
                                                                                << currentLeg_->lengthM << " m ("
                                                                                << currentLeg_->word() << ")")
}

void DubinsPathPlanner::registerMiss(const Dubins2DPose& current) {
  missCounts_[targetIndex_]++;
  UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Missed waypoint " << targetIndex_ << " (miss " << missCounts_[targetIndex_] << "/"
                                                        << params_.maxMissesPerWaypoint << ")")
  if (missCounts_[targetIndex_] > params_.maxMissesPerWaypoint) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "DubinsPathPlanner exceeded miss budget for waypoint " << targetIndex_ << "; failing route")
    failed_ = true;
    return;
  }
  if (replanCount_ >= params_.maxReplans) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "DubinsPathPlanner exhausted replan budget; failing route")
    failed_ = true;
    return;
  }
  replanCount_++;
  currentLeg_ = buildLeg(current, targetIndex_);
  if (!currentLeg_.has_value()) {
    failed_ = true;
    progress_.failed = true;
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "DubinsPathPlanner: replan found no zone-compliant leg to "
                   "waypoint "
                       << targetIndex_ << "; failing route")
    return;
  }
  legProgressS_ = 0.0;
  tracker_.resetLeg();
  lastGateAlongM_.reset();
  elevApproachBudget_.reset();
  elevApproachesUsed_ = 0;
  lastSpiralElevErrM_.reset();
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner replanned waypoint "
                                         << targetIndex_ << " from live pose (replan " << replanCount_ << "/"
                                         << params_.maxReplans << "): leg " << currentLeg_->lengthM << " m ("
                                         << currentLeg_->word() << ")")
}

void DubinsPathPlanner::spiralReplan(const Dubins2DPose& current) {
  elevApproachesUsed_++;
  currentLeg_ = buildLeg(current, targetIndex_);
  if (!currentLeg_.has_value()) {
    failed_ = true;
    progress_.failed = true;
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER,
                   "DubinsPathPlanner: spiral replan found no zone-compliant "
                   "loop leg for waypoint "
                       << targetIndex_ << "; failing route")
    return;
  }
  legProgressS_ = 0.0;
  tracker_.resetLeg();
  lastGateAlongM_.reset();
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner spiral pass "
                                         << elevApproachesUsed_ << "/" << elevApproachBudget_.value_or(0)
                                         << " for waypoint " << targetIndex_
                                         << " (elevation still converging): loop leg " << currentLeg_->lengthM << " m ("
                                         << currentLeg_->word() << ")")
}

std::optional<flt64_t> DubinsPathPlanner::elevationErrorM(const GlobalPoseReportType& pose) const {
  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  if (!wp.elevation().has_value()) {
    return std::nullopt;
  }
  const std::optional<ElevationValue> el = tolerance::extractElevation(wp.elevation().value());
  if (!el.has_value()) {
    return std::nullopt;
  }
  const std::optional<flt64_t> cur = elevation::poseElevation(pose, el->frame);
  if (!cur.has_value() || !std::isfinite(cur.value())) {
    return std::nullopt;
  }
  return std::fabs(el->valueM - cur.value());
}

void DubinsPathPlanner::computeElevationApproachBudget(const GlobalPoseReportType& pose, flt64_t groundSpeedMps) {
  elevApproachBudget_ = 0;
  if (params_.maxDepthRateMps <= 0.0 || !currentLeg_.has_value()) {
    return;
  }
  const std::optional<flt64_t> errM = elevationErrorM(pose);
  if (!errM.has_value()) {
    return;
  }
  // How long the commanded elevation change needs at the platform's depth-rate limit vs how
  // long this pass of the 2D path provides: the shortfall, in whole passes, is the number of
  // planned spiral loops before gate failures start counting against the miss budget.
  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  const flt64_t speed = std::max({groundSpeedMps, waypointSpeedMps(wp), 0.5});
  const flt64_t neededS = errM.value() / params_.maxDepthRateMps;
  const flt64_t passS = currentLeg_->lengthM / speed;
  if (neededS > passS && passS > 1e-6) {
    elevApproachBudget_ = std::min(100, static_cast<int32_t>(std::ceil(neededS / passS)));
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner waypoint "
                                           << targetIndex_ << " elevation change of " << errM.value() << " m needs ~"
                                           << neededS << " s at the platform depth rate (pass is ~" << passS
                                           << " s): budgeting " << elevApproachBudget_.value() << " spiral approaches")
  }
}

bool DubinsPathPlanner::allowSpiralPass(const GlobalPoseReportType& pose, flt64_t groundSpeedMps) {
  if (!elevApproachBudget_.has_value() || elevApproachBudget_.value() <= 0 || params_.maxDepthRateMps <= 0.0 ||
      !currentLeg_.has_value()) {
    return false;
  }
  const std::optional<flt64_t> errM = elevationErrorM(pose);
  if (!errM.has_value()) {
    return false;
  }
  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  const flt64_t speed = std::max({groundSpeedMps, waypointSpeedMps(wp), 0.5});
  const flt64_t passS = currentLeg_->lengthM / speed;  // just-flown leg ~ the next loop
  const flt64_t expectedPerPassM = params_.maxDepthRateMps * passS;

  if (elevApproachesUsed_ < elevApproachBudget_.value()) {
    lastSpiralElevErrM_ = errM;
    return true;
  }
  // The up-front budget is estimated from the first (usually longest) pass and can undercount,
  // so extend it from the remaining error and the actual loop time — but only while the
  // elevation is genuinely converging; a vehicle that cannot make depth must consume the miss budget.
  if (lastSpiralElevErrM_.has_value() && expectedPerPassM > 1e-6 &&
      lastSpiralElevErrM_.value() - errM.value() >= 0.5 * expectedPerPassM) {
    const int32_t remaining = std::max(1, static_cast<int32_t>(std::ceil(errM.value() / expectedPerPassM)));
    const int32_t extended = std::min(100, elevApproachesUsed_ + remaining);
    if (extended > elevApproachBudget_.value()) {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner waypoint "
                                             << targetIndex_ << " elevation still converging with " << errM.value()
                                             << " m to go (~" << expectedPerPassM
                                             << " m per loop): extending spiral budget to " << extended)
      elevApproachBudget_ = extended;
    }
    lastSpiralElevErrM_ = errM;
    return elevApproachesUsed_ < elevApproachBudget_.value();
  }
  return false;
}

void DubinsPathPlanner::updateDistanceMetrics(flt64_t xE, flt64_t yN, flt64_t distToWaypointM) {
  flt64_t remaining = currentLeg_.has_value() ? std::max(0.0, currentLeg_->lengthM - legProgressS_) : distToWaypointM;
  for (std::size_t i = targetIndex_; i + 1 < waypoints_.size(); ++i) {
    remaining += std::hypot(wpX_[i + 1] - wpX_[i], wpY_[i + 1] - wpY_[i]);
  }
  progress_.distanceRemainingM = remaining;
  if (hasLastPos_) {
    progress_.cumulativeDistanceM += std::hypot(xE - lastXE_, yN - lastYN_);
  }
}

ControlVector DubinsPathPlanner::update(const GlobalPoseReportType& pose, flt64_t groundSpeedMps, flt64_t dtS) {
  progress_.valid = true;
  if (waypoints_.empty() || routeComplete_ || failed_) {
    progress_.routeComplete = routeComplete_;
    progress_.failed = failed_;
    // Hold heading but stop driving once the route is over.
    ControlVector hold = lastVector_;
    hold.speedMps = 0.0;
    return hold;
  }

  flt64_t xE = 0.0;
  flt64_t yN = 0.0;
  toLocal(poseLat(pose), poseLon(pose), &xE, &yN);
  const Dubins2DPose current{xE, yN, azToMath(poseYaw(pose))};

  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  const flt64_t dist = std::hypot(wpX_[targetIndex_] - xE, wpY_[targetIndex_] - yN);

  if (!currentLeg_.has_value()) {
    currentLeg_ = buildLeg(current, targetIndex_);
    if (!currentLeg_.has_value()) {
      failed_ = true;
      progress_.failed = true;
      ControlVector hold = lastVector_;
      hold.speedMps = 0.0;
      return hold;
    }
    legProgressS_ = 0.0;
    tracker_.resetLeg();
    lastGateAlongM_.reset();
    elevApproachBudget_.reset();
  }
  const Leg& leg = currentLeg_.value();
  if (!elevApproachBudget_.has_value()) {
    computeElevationApproachBudget(pose, groundSpeedMps);
  }

  const flt64_t step = params_.sampleStepM;
  const flt64_t searchLead = std::max(params_.leadDistanceM, 4.0 * step);
  const flt64_t gateHalfM = gateHalfWidthM(wp, params_);
  const flt64_t overshootBudget = std::max(searchLead, 4.0 * gateHalfM);

  // Advance the monotonic progress pointer by searching a bounded window around the previous
  // progress on a parametrization extended past the path end so progress keeps flowing through the gate.
  flt64_t pathXteM = 0.0;   // signed cross-track error from the planned path (+ = starboard)
  flt64_t pathAzRad = 0.0;  // planned-path tangent azimuth at the projection point
  {
    const flt64_t back = std::min(legProgressS_, 2.0 * step);
    // The forward window must stay small relative to the leg: a tight loop leg (a spiral
    // pass confined to a couple of turn radii) brings far-ahead samples spatially close to
    // the vehicle, and a wide window would let the progress pointer leap across the loop.
    const flt64_t windowAheadM = std::max(6.0 * step, 3.0 * std::max(groundSpeedMps, 1.0));
    const flt64_t sMax = leg.lengthM + overshootBudget + step;
    flt64_t bestS = legProgressS_;
    flt64_t bestD = std::numeric_limits<flt64_t>::max();
    for (flt64_t s = legProgressS_ - back; s <= std::min(legProgressS_ + windowAheadM, sMax); s += step) {
      const Dubins2DPose p = sampleExtended(leg, s);
      const flt64_t d = std::hypot(p.x - xE, p.y - yN);
      if (d < bestD) {
        bestD = d;
        bestS = s;
      }
    }
    legProgressS_ = bestS;
    const Dubins2DPose closest = sampleExtended(leg, bestS);
    pathAzRad = mathToAz(closest.theta);
    const flt64_t tx = std::cos(closest.theta);
    const flt64_t ty = std::sin(closest.theta);
    // Starboard-positive lateral offset from the path tangent.
    pathXteM = ty * (xE - closest.x) - tx * (yN - closest.y);
  }

  // Path-frame tracking law: the planned-path tangent at the projection point, plus a curvature
  // feedforward sized by the inner heading loop's time constant, plus a bounded cross-track
  // correction. The feedforward is what lets the vehicle hold the planned arc: the platform
  // accepts a heading rather than a turn rate, so an arc requires standing off the tangent by
  // exactly the heading error the loop needs to produce that arc's turn rate.
  const flt64_t vMps = std::max(groundSpeedMps, 0.5);
  // Preview curvature by the loop's own time constant so the feedforward arrives with the arc
  // rather than one time constant behind it. The cap is floored because PlannerParams is a public
  // aggregate and std::clamp with hi < lo is undefined.
  const flt64_t previewCapM = std::max(0.0, kCurvaturePreviewTurnRad * params_.turnRadiusM);
  const flt64_t previewWantedM = vMps * params_.tracker.headingLoopTauS;
  const flt64_t previewM = std::clamp(previewWantedM, 0.0, previewCapM);
  if (previewWantedM > previewCapM && !previewCapLogged_) {
    previewCapLogged_ = true;
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Curvature preview capped at "
                                           << previewCapM << " m but the loop time constant asks for " << previewWantedM
                                           << " m, so the feedforward is previewing less than one time "
                                              "constant ahead; raise planner.turn_radius_margin or lower "
                                              "planner.tracker.heading_loop_tau_s")
  }

  PathTracker::Inputs trackerIn;
  trackerIn.pathAzimuthRad = pathAzRad;
  trackerIn.pathCurvatureAzRadPerM = meanPathCurvature(leg, legProgressS_, previewM);
  trackerIn.crossTrackErrorM = pathXteM;
  trackerIn.groundSpeedMps = groundSpeedMps;
  trackerIn.dtS = dtS;
  const PathTracker::Output tracked = tracker_.update(trackerIn);

  ControlVector cv;
  cv.headingRad = tracked.headingRad;
  cv.speedMps = waypointSpeedMps(wp);
  if (wp.elevation().has_value()) {
    const std::optional<ElevationValue> el = tolerance::extractElevation(wp.elevation().value());
    if (el.has_value()) {
      cv.elevationM = el->valueM;
      cv.elevationFrame = el->frame;
    }
  }
  // Only cache a usable vector: this becomes the route-complete/failed hold, and a non-finite
  // heading there is refused by the brain's finiteness screen, which would leave the vehicle
  // driving on its previous setpoint instead of stopping.
  if (std::isfinite(cv.headingRad)) {
    lastVector_ = cv;
  }

  // Capture gate: a segment of half-width gateHalfM through the waypoint, perpendicular to
  // the arrival heading.
  const flt64_t dirE = std::sin(leg.endAzimuthRad);
  const flt64_t dirN = std::cos(leg.endAzimuthRad);
  const flt64_t relE = xE - wpX_[targetIndex_];
  const flt64_t relN = yN - wpY_[targetIndex_];
  const flt64_t gateAlongM = relE * dirE + relN * dirN;
  const flt64_t gateLateralM = relE * dirN - relN * dirE;  // starboard-positive

  // Progress reporting.
  const CaptureResult cap = evaluateCapture(pose, gateLateralM);
  if (!cap.elevationEvaluable && !elevUnevaluableLogged_) {
    elevUnevaluableLogged_ = true;
    UMAA_LOG_WARN(util::SYSTEM_LOGGER, "DubinsPathPlanner waypoint "
                                           << targetIndex_
                                           << " elevation cannot be evaluated (the pose carries no value in the "
                                              "commanded frame); gate crossings will spend the miss budget")
  } else if (cap.elevationEvaluable && elevUnevaluableLogged_) {
    elevUnevaluableLogged_ = false;
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner waypoint elevation is evaluable again")
  }
  progress_.distanceToWaypointM = dist;
  progress_.positionAchieved = dist <= gateHalfM;
  progress_.attitudeAchieved = cap.attitudeAchieved;
  progress_.elevationAchieved = cap.elevationAchieved;
  progress_.waypointId = arlcore::NumericGuid(wp.waypointID());
  progress_.waypointsRemaining = static_cast<int32_t>(waypoints_.size() - targetIndex_);
  // Track holding is judged against the planned Dubins path itself (not the straight lines
  // between waypoints): report the signed offset and evaluate the UMAA track tolerance on it.
  progress_.crossTrackErrorM = pathXteM;
  if (wp.trackTolerance().has_value()) {
    const std::optional<flt64_t> tol = tolerance::extractTrackToleranceM(wp.trackTolerance().value());
    progress_.trackLineAchieved = !tol.has_value() || std::fabs(pathXteM) <= tol.value();
  } else {
    progress_.trackLineAchieved = true;
  }
  updateDistanceMetrics(xE, yN, dist);
  lastXE_ = xE;
  lastYN_ = yN;
  hasLastPos_ = true;

  // Gate-crossing detection, evaluated only on final approach (the planned path of a dense
  // route may legitimately cross the gate plane mid-turn far from the waypoint).
  const bool onFinalApproach = legProgressS_ > leg.lengthM - (searchLead + 2.0 * gateHalfM);
  const bool crossedGate =
      onFinalApproach && lastGateAlongM_.has_value() && lastGateAlongM_.value() < 0.0 && gateAlongM >= 0.0;
  bool legStateChanged = false;
  if (crossedGate) {
    legStateChanged = true;
    if (cap.captured) {
      advanceToNextWaypoint();
      if (routeComplete_) {
        cv.speedMps = 0.0;
        lastVector_ = cv;
        progress_.routeComplete = true;
        progress_.waypointsRemaining = 0;
      }
    } else {
      const bool attitudeOk = !cap.attitudeAchieved.has_value() || cap.attitudeAchieved.value();
      const bool elevationOnlyFailure =
          cap.positionAchieved && attitudeOk && !cap.elevationAchieved && params_.elevationCountsAsMiss;
      if (elevationOnlyFailure && allowSpiralPass(pose, groundSpeedMps)) {
        // The commanded elevation change was known to need more passes than one: loop back
        // around without spending the miss budget — the spiral is the plan.
        spiralReplan(current);
      } else {
        registerMiss(current);
      }
    }
  } else if (legProgressS_ > leg.lengthM + overshootBudget - 1e-9) {
    // Overflew the end of the planned path without a usable gate crossing: loop back around.
    registerMiss(current);
    legStateChanged = true;
  }
  if (!legStateChanged) {
    lastGateAlongM_ = onFinalApproach ? std::optional<flt64_t>(gateAlongM) : std::nullopt;
  }
  progress_.failed = failed_;
  progress_.routeComplete = routeComplete_;

  return cv;
}

}  // namespace arlcore::autopilot
