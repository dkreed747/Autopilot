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

#include "DubinsPathPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "AngleMath.h"
#include "Logger.h"
#include "ToleranceUtils.h"

namespace arlcore::autopilot {

using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

namespace {

double poseLat(const GlobalPoseReportType& p) { return p.position().geodeticLatitude(); }
double poseLon(const GlobalPoseReportType& p) { return p.position().geodeticLongitude(); }
double poseYaw(const GlobalPoseReportType& p) { return p.attitude().yaw().yaw(); }

double wpLat(const GlobalWaypointType& w) { return w.position().value().geodeticLatitude(); }
double wpLon(const GlobalWaypointType& w) { return w.position().value().geodeticLongitude(); }

//! \brief Azimuth (true north, clockwise) <-> math angle (+x east, counterclockwise).
//! The mapping is its own inverse.
double azToMath(double azRad) { return wrapPi(M_PI_2 - azRad); }
double mathToAz(double mathRad) { return wrapPi(M_PI_2 - mathRad); }

//! \brief Current pose elevation in the requested frame, if available.
std::optional<double> poseElevation(const GlobalPoseReportType& p, ElevationFrame frame) {
  switch (frame) {
    case ElevationFrame::DEPTH:
      return p.depth().has_value() ? std::optional<double>(p.depth().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_MSL:
      return p.altitude().has_value() ? std::optional<double>(p.altitude().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_AGL:
      return p.altitudeAGL().has_value() ? std::optional<double>(p.altitudeAGL().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_ASF:
      return p.altitudeASF().has_value() ? std::optional<double>(p.altitudeASF().value()) : std::nullopt;
    case ElevationFrame::ALTITUDE_GEODETIC:
      return p.altitudeGeodetic().has_value() ? std::optional<double>(p.altitudeGeodetic().value()) : std::nullopt;
    default:
      return std::nullopt;
  }
}

//! \brief Capture-gate half-width for a waypoint (its position tolerance or the default).
double gateHalfWidthM(const GlobalWaypointType& wp, const PlannerParams& params) {
  if (wp.position().tolerance().has_value()) {
    return wp.position().tolerance().value().limit();
  }
  return params.posCaptureM;
}

//! \brief The waypoint's commanded speed (0 when the variant is unsupported; validation in
//! the provider rejects such routes before they reach the planner).
double waypointSpeedMps(const GlobalWaypointType& wp) {
  const std::optional<SpeedValue> sp = tolerance::extractSpeed(wp.speed());
  return sp.has_value() ? sp->speedMps : 0.0;
}

}  // namespace

void DubinsPathPlanner::toLocal(double latDeg, double lonDeg, double* xE, double* yN) const {
  double z = 0.0;
  localFrame_.Forward(latDeg, lonDeg, 0.0, *xE, *yN, z);
}

Dubins2DPose DubinsPathPlanner::sampleExtended(const Leg& leg, double sM) {
  if (sM <= leg.dubinsLengthM) {
    return leg.path.sample(sM);
  }
  // Straight continuation covers both the final-approach runway (up to lengthM, ending at
  // the waypoint) and the fly-through extension beyond it.
  Dubins2DPose end = leg.path.sample(leg.dubinsLengthM);
  const double over = sM - leg.dubinsLengthM;
  end.x += over * std::cos(end.theta);
  end.y += over * std::sin(end.theta);
  return end;
}

double DubinsPathPlanner::arrivalAzimuth(std::size_t wpIndex, double fromXE, double fromYN) const {
  const GlobalWaypointType& wp = waypoints_[wpIndex];
  if (wp.attitude().has_value()) {
    return tolerance::extractYaw(wp.attitude().value()).yawRad;
  }
  // Natural fly-through heading: toward the next waypoint, or along the final approach for
  // the last one.
  double dE = 0.0;
  double dN = 0.0;
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

DubinsPathPlanner::Leg DubinsPathPlanner::buildLeg(const Dubins2DPose& startPose, std::size_t wpIndex) const {
  const double endAz = arrivalAzimuth(wpIndex, startPose.x, startPose.y);
  const double endTheta = azToMath(endAz);
  // Solve the curved portion to a virtual goal one turn radius short of the waypoint along
  // the arrival bearing; the leg then finishes with a straight runway through the waypoint.
  const double runwayM = params_.turnRadiusM;
  const Dubins2DPose virtualGoal{wpX_[wpIndex] - runwayM * std::cos(endTheta),
                                 wpY_[wpIndex] - runwayM * std::sin(endTheta), endTheta};
  std::optional<DubinsPath> path = DubinsPath::solve(startPose, virtualGoal, params_.turnRadiusM);
  if (!path.has_value()) {
    // solve() only fails on non-finite input; fall back to a degenerate straight run from a
    // sanitized origin so guidance can still make progress.
    path = DubinsPath::solve({0.0, 0.0, 0.0}, virtualGoal, 0.0);
  }
  return Leg(path.value(), runwayM, endAz);
}

void DubinsPathPlanner::plan(const std::vector<GlobalWaypointType>& waypoints,
                             const GlobalPoseReportType& start, const PlannerParams& params) {
  waypoints_ = waypoints;
  params_ = params;
  params_.sampleStepM = std::max(0.5, params_.sampleStepM);
  missCounts_.assign(waypoints_.size(), 0);
  targetIndex_ = 0;
  routeComplete_ = waypoints_.empty();
  failed_ = false;
  replanCount_ = 0;
  hasLastPos_ = false;
  legProgressS_ = 0.0;
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

  const Dubins2DPose startPose{0.0, 0.0, azToMath(poseYaw(start))};
  planStartPose_ = startPose;
  currentLeg_ = buildLeg(startPose, 0);
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner planned route: " << waypoints_.size()
    << " waypoints, first leg " << currentLeg_->lengthM << " m (" << currentLeg_->path.word()
    << "), turn radius " << params_.turnRadiusM << " m")
}

std::vector<std::pair<double, double>> DubinsPathPlanner::previewRoute(double stepM) const {
  std::vector<std::pair<double, double>> out;
  if (waypoints_.empty()) {
    return out;
  }
  const double step = std::max(0.5, stepM);
  Dubins2DPose legStart = planStartPose_;
  for (std::size_t i = 0; i < waypoints_.size(); i++) {
    const Leg leg = buildLeg(legStart, i);
    for (double s = 0.0; s <= leg.lengthM + step * 0.5; s += step) {
      const Dubins2DPose p = sampleExtended(leg, std::min(s, leg.lengthM));
      double lat = 0.0;
      double lon = 0.0;
      double h = 0.0;
      localFrame_.Reverse(p.x, p.y, 0.0, lat, lon, h);
      out.emplace_back(lat, lon);
    }
    legStart = Dubins2DPose{wpX_[i], wpY_[i], azToMath(leg.endAzimuthRad)};
  }
  return out;
}

CaptureResult DubinsPathPlanner::evaluateCapture(const GlobalPoseReportType& pose,
                                                 double gateLateralM) const {
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
      const std::optional<double> cur = poseElevation(pose, el->frame);
      result.elevationAchieved = cur.has_value() &&
          tolerance::elevationAchieved(el.value(), cur.value(), params_.elevCaptureM);
    } else {
      result.elevationAchieved = false;
    }
  }

  const bool attitudeOk = !result.attitudeAchieved.has_value() || result.attitudeAchieved.value();
  result.captured = result.positionAchieved && attitudeOk &&
                    (result.elevationAchieved || !params_.elevationCountsAsMiss);
  return result;
}

void DubinsPathPlanner::advanceToNextWaypoint() {
  const double arrivalAz = currentLeg_->endAzimuthRad;
  const double fromX = wpX_[targetIndex_];
  const double fromY = wpY_[targetIndex_];
  targetIndex_++;
  legProgressS_ = 0.0;
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
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner advancing to waypoint " << targetIndex_
    << ", leg " << currentLeg_->lengthM << " m (" << currentLeg_->path.word() << ")")
}

void DubinsPathPlanner::registerMiss(const Dubins2DPose& current) {
  missCounts_[targetIndex_]++;
  UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Missed waypoint " << targetIndex_ << " (miss "
    << missCounts_[targetIndex_] << "/" << params_.maxMissesPerWaypoint << ")")
  if (missCounts_[targetIndex_] > params_.maxMissesPerWaypoint) {
    UMAA_LOG_ERROR(util::SYSTEM_LOGGER, "DubinsPathPlanner exceeded miss budget for waypoint "
      << targetIndex_ << "; failing route")
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
  legProgressS_ = 0.0;
  lastGateAlongM_.reset();
  elevApproachBudget_.reset();
  elevApproachesUsed_ = 0;
  lastSpiralElevErrM_.reset();
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner replanned waypoint " << targetIndex_
    << " from live pose (replan " << replanCount_ << "/" << params_.maxReplans << "): leg "
    << currentLeg_->lengthM << " m (" << currentLeg_->path.word() << ")")
}

void DubinsPathPlanner::spiralReplan(const Dubins2DPose& current) {
  elevApproachesUsed_++;
  currentLeg_ = buildLeg(current, targetIndex_);
  legProgressS_ = 0.0;
  lastGateAlongM_.reset();
  UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner spiral pass " << elevApproachesUsed_
    << "/" << elevApproachBudget_.value_or(0) << " for waypoint " << targetIndex_
    << " (elevation still converging): loop leg " << currentLeg_->lengthM << " m ("
    << currentLeg_->path.word() << ")")
}

std::optional<double> DubinsPathPlanner::elevationErrorM(const GlobalPoseReportType& pose) const {
  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  if (!wp.elevation().has_value()) {
    return std::nullopt;
  }
  const std::optional<ElevationValue> el = tolerance::extractElevation(wp.elevation().value());
  if (!el.has_value()) {
    return std::nullopt;
  }
  const std::optional<double> cur = poseElevation(pose, el->frame);
  if (!cur.has_value()) {
    return std::nullopt;
  }
  return std::fabs(el->valueM - cur.value());
}

void DubinsPathPlanner::computeElevationApproachBudget(const GlobalPoseReportType& pose,
                                                       double groundSpeedMps) {
  elevApproachBudget_ = 0;
  if (params_.maxDepthRateMps <= 0.0 || !currentLeg_.has_value()) {
    return;
  }
  const std::optional<double> errM = elevationErrorM(pose);
  if (!errM.has_value()) {
    return;
  }
  // How long the commanded elevation change needs at the platform's depth-rate limit vs how
  // long this pass of the 2D path provides: the shortfall, in whole passes, is the number of
  // planned spiral loops before gate failures start counting against the miss budget.
  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  const double speed = std::max({groundSpeedMps, waypointSpeedMps(wp), 0.5});
  const double neededS = errM.value() / params_.maxDepthRateMps;
  const double passS = currentLeg_->lengthM / speed;
  if (neededS > passS && passS > 1e-6) {
    elevApproachBudget_ = std::min(100, static_cast<int>(std::ceil(neededS / passS)));
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner waypoint " << targetIndex_
      << " elevation change of " << errM.value() << " m needs ~"
      << neededS << " s at the platform depth rate (pass is ~" << passS
      << " s): budgeting " << elevApproachBudget_.value() << " spiral approaches")
  }
}

bool DubinsPathPlanner::allowSpiralPass(const GlobalPoseReportType& pose, double groundSpeedMps) {
  if (!elevApproachBudget_.has_value() || elevApproachBudget_.value() <= 0 ||
      params_.maxDepthRateMps <= 0.0 || !currentLeg_.has_value()) {
    return false;
  }
  const std::optional<double> errM = elevationErrorM(pose);
  if (!errM.has_value()) {
    return false;
  }
  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  const double speed = std::max({groundSpeedMps, waypointSpeedMps(wp), 0.5});
  const double passS = currentLeg_->lengthM / speed;  // just-flown leg ~ the next loop
  const double expectedPerPassM = params_.maxDepthRateMps * passS;

  if (elevApproachesUsed_ < elevApproachBudget_.value()) {
    lastSpiralElevErrM_ = errM;
    return true;
  }
  // The up-front budget is estimated from the first pass of the leg, which is usually longer
  // than the loop-back passes, so it can undercount. Extend it from the remaining error and
  // the actual loop time — but only while the elevation is genuinely converging (at least half
  // the expected per-pass change since the previous pass); a vehicle that cannot make depth
  // must start consuming the miss budget.
  if (lastSpiralElevErrM_.has_value() && expectedPerPassM > 1e-6 &&
      lastSpiralElevErrM_.value() - errM.value() >= 0.5 * expectedPerPassM) {
    const int remaining = std::max(1, static_cast<int>(std::ceil(errM.value() / expectedPerPassM)));
    const int extended = std::min(100, elevApproachesUsed_ + remaining);
    if (extended > elevApproachBudget_.value()) {
      UMAA_LOG_INFO(util::SYSTEM_LOGGER, "DubinsPathPlanner waypoint " << targetIndex_
        << " elevation still converging with " << errM.value() << " m to go (~"
        << expectedPerPassM << " m per loop): extending spiral budget to " << extended)
      elevApproachBudget_ = extended;
    }
    lastSpiralElevErrM_ = errM;
    return elevApproachesUsed_ < elevApproachBudget_.value();
  }
  return false;
}

void DubinsPathPlanner::updateDistanceMetrics(double xE, double yN, double distToWaypointM) {
  double remaining = currentLeg_.has_value() ? std::max(0.0, currentLeg_->lengthM - legProgressS_)
                                             : distToWaypointM;
  for (std::size_t i = targetIndex_; i + 1 < waypoints_.size(); ++i) {
    remaining += std::hypot(wpX_[i + 1] - wpX_[i], wpY_[i + 1] - wpY_[i]);
  }
  progress_.distanceRemainingM = remaining;
  if (hasLastPos_) {
    progress_.cumulativeDistanceM += std::hypot(xE - lastXE_, yN - lastYN_);
  }
}

ControlVector DubinsPathPlanner::update(const GlobalPoseReportType& pose, double groundSpeedMps) {
  progress_.valid = true;
  if (waypoints_.empty() || routeComplete_ || failed_) {
    progress_.routeComplete = routeComplete_;
    progress_.failed = failed_;
    // Hold heading but stop driving once the route is over.
    ControlVector hold = lastVector_;
    hold.speedMps = 0.0;
    return hold;
  }

  double xE = 0.0;
  double yN = 0.0;
  toLocal(poseLat(pose), poseLon(pose), &xE, &yN);
  const Dubins2DPose current{xE, yN, azToMath(poseYaw(pose))};

  const GlobalWaypointType& wp = waypoints_[targetIndex_];
  const double dist = std::hypot(wpX_[targetIndex_] - xE, wpY_[targetIndex_] - yN);

  if (!currentLeg_.has_value()) {
    currentLeg_ = buildLeg(current, targetIndex_);
    legProgressS_ = 0.0;
    lastGateAlongM_.reset();
    elevApproachBudget_.reset();
  }
  const Leg& leg = currentLeg_.value();
  if (!elevApproachBudget_.has_value()) {
    computeElevationApproachBudget(pose, groundSpeedMps);
  }

  const double step = params_.sampleStepM;
  const double searchLead = std::max(params_.leadDistanceM, 4.0 * step);
  const double gateHalfM = gateHalfWidthM(wp, params_);
  const double overshootBudget = std::max(searchLead, 4.0 * gateHalfM);

  // Advance the monotonic arc-length progress pointer: search a bounded window ahead of the
  // previous progress (with a small allowance backward) for the closest path sample. The
  // parametrization extends past the path end so progress keeps flowing through the gate.
  double pathXteM = 0.0;  // signed cross-track error from the planned path (+ = starboard)
  {
    const double back = std::min(legProgressS_, 2.0 * step);
    // The forward window must stay small relative to the leg: a tight loop leg (a spiral
    // pass confined to a couple of turn radii) brings far-ahead samples spatially close to
    // the vehicle, and a wide window would let the progress pointer leap across the loop.
    const double windowAheadM = std::max(6.0 * step, 3.0 * std::max(groundSpeedMps, 1.0));
    const double sMax = leg.lengthM + overshootBudget + step;
    double bestS = legProgressS_;
    double bestD = std::numeric_limits<double>::max();
    for (double s = legProgressS_ - back; s <= std::min(legProgressS_ + windowAheadM, sMax);
         s += step) {
      const Dubins2DPose p = sampleExtended(leg, s);
      const double d = std::hypot(p.x - xE, p.y - yN);
      if (d < bestD) {
        bestD = d;
        bestS = s;
      }
    }
    legProgressS_ = bestS;
    const Dubins2DPose closest = sampleExtended(leg, bestS);
    const double tx = std::cos(closest.theta);
    const double ty = std::sin(closest.theta);
    // Starboard-positive lateral offset from the path tangent.
    pathXteM = ty * (xE - closest.x) - tx * (yN - closest.y);
  }

  // Path-frame tracking law: command the planned-path tangent (sampled ~1 s ahead of the
  // closest point as phase lead for the rate-limited heading loop) plus a cross-track
  // correction atan(xte / turnRadius) that converges back onto the path within roughly one
  // turn radius without saturating the vehicle's turn authority.
  const double vMps = std::max(groundSpeedMps, 0.5);
  const double tangentS = legProgressS_ + std::max(2.0 * step, 1.0 * vMps);
  const Dubins2DPose tangentPoint = sampleExtended(leg, tangentS);
  const double pathAz = mathToAz(tangentPoint.theta);
  const double correction =
      std::clamp(std::atan2(pathXteM, std::max(params_.turnRadiusM, 1.0)), -1.2, 1.2);

  ControlVector cv;
  cv.headingRad = wrapPi(pathAz - correction);
  cv.speedMps = waypointSpeedMps(wp);
  if (wp.elevation().has_value()) {
    const std::optional<ElevationValue> el = tolerance::extractElevation(wp.elevation().value());
    if (el.has_value()) {
      cv.elevationM = el->valueM;
      cv.elevationFrame = el->frame;
    }
  }
  lastVector_ = cv;

  // Capture gate: a segment of half-width gateHalfM through the waypoint, perpendicular to
  // the arrival heading. Signed along-track distance to the gate plane and lateral offset
  // along the gate.
  const double dirE = std::sin(leg.endAzimuthRad);
  const double dirN = std::cos(leg.endAzimuthRad);
  const double relE = xE - wpX_[targetIndex_];
  const double relN = yN - wpY_[targetIndex_];
  const double gateAlongM = relE * dirE + relN * dirN;
  const double gateLateralM = relE * dirN - relN * dirE;  // starboard-positive

  // Progress reporting.
  const CaptureResult cap = evaluateCapture(pose, gateLateralM);
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
    const std::optional<double> tol = tolerance::extractTrackToleranceM(wp.trackTolerance().value());
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
  const bool crossedGate = onFinalApproach && lastGateAlongM_.has_value() &&
                           lastGateAlongM_.value() < 0.0 && gateAlongM >= 0.0;
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
      const bool elevationOnlyFailure = cap.positionAchieved && attitudeOk &&
                                        !cap.elevationAchieved && params_.elevationCountsAsMiss;
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
    lastGateAlongM_ = onFinalApproach ? std::optional<double>(gateAlongM) : std::nullopt;
  }
  progress_.failed = failed_;
  progress_.routeComplete = routeComplete_;

  return cv;
}

}  // namespace arlcore::autopilot
