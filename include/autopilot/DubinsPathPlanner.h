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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DUBINSPATHPLANNER_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DUBINSPATHPLANNER_H_

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointType.hpp>
#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>

#include "ControlVector.h"
#include "DubinsPath.h"
#include "ProgressTypes.h"

namespace arlcore::autopilot {

//! \brief Parameters that tune the planner. Capture defaults are used when a waypoint omits
//! its own tolerances. Everything kinematic is derived from the platform capabilities (see
//! PlannerParamsFactory.h).
struct PlannerParams {
  double turnRadiusM = 25.0;        // margin * (speed / maxTurnRate) from the capabilities
  double leadDistanceM = 50.0;      // progress-search window scale along the planned path
  double posCaptureM = 2.5;         // default capture-gate half-width
  double yawCaptureRad = 0.1745;    // default arrival-attitude capture half-width
  double elevCaptureM = 1.0;        // default elevation capture tolerance
  int maxMissesPerWaypoint = 3;     // misses before the route fails
  bool elevationCountsAsMiss = true;
  int maxReplans = 10;              // guard against endless replanning (spirals excluded)
  double sampleStepM = 2.0;         // path polyline sampling resolution
  double maxDepthRateMps = 0.0;     // platform depth-change limit (0 = unknown/surface-only)
};

//! \brief A horizontal path planner that drives a vehicle through a series of 3D waypoints
//! along true Dubins paths. For every leg (previous waypoint or the plan/replan pose, to the
//! next waypoint) the shortest curvature-bounded Dubins path is solved (all six words, see
//! DubinsPath) using the platform-derived turn radius; each leg ends with a straight
//! final-approach runway through the waypoint so arrival happens with position and attitude
//! settled. Waypoints without an attitude requirement get a natural fly-through heading
//! toward the following waypoint. Elevation/depth and speed pass through per-waypoint.
//!
//! Tracking uses a path-frame guidance law: the commanded heading is the planned-path tangent
//! (sampled slightly ahead of the closest point for actuation phase lead) plus a cross-track
//! correction term atan(xte / turnRadius) that steers back onto the path — feedback comes
//! from the live navigation reports (pose + ground speed). Cross-track error is measured from
//! the planned Dubins path itself (not the straight lines between waypoints), which is also
//! the reference the UMAA track tolerance is evaluated against.
//!
//! Capture uses a gate, not a bubble: a segment of half-width posCapture (or the waypoint's
//! position tolerance) through the waypoint, perpendicular to the arrival heading. The
//! waypoint is captured the instant the vehicle crosses the gate plane inside the half-width
//! with attitude/elevation criteria met — so the vehicle always flies *through* the waypoint
//! instead of "popping a bubble" early and cutting the corner. Crossing the plane outside the
//! gate (or overflying the path without crossing) is a miss and replans the leg from the live
//! pose, bounded by maxMissesPerWaypoint and maxReplans. Depth-limited legs are special: when
//! the commanded elevation change needs more time than the 2D path provides (from the
//! platform's max depth rate), the planner budgets the expected number of loop-back passes up
//! front and elevation-only gate failures within that budget replan for free — the spiral is
//! the plan, not a failure. When the route completes the planner commands zero speed.
class DubinsPathPlanner {
 public:
  //! \brief Plan a route up front from the current pose through the given waypoints.
  void plan(const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints,
            const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& start,
            const PlannerParams& params);

  //! \brief Produce a control vector for the current pose and advance capture/miss/replan
  //! state. Call once per navigation packet with the latest ground speed (feeds the
  //! cross-track correction and the spiral approach budget).
  ControlVector update(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose,
                       double groundSpeedMps);

  //! \brief Latest progress snapshot (mapped into the UMAA waypoint execution status report).
  const WaypointProgress& progress() const { return progress_; }

  bool routeComplete() const { return routeComplete_; }
  bool failed() const { return failed_; }
  bool hasRoute() const { return !waypoints_.empty(); }

  //! \brief Sample the ideal planned route (all legs chained waypoint-to-waypoint from the
  //! plan start pose) as geodetic (lat, lon) points every `stepM`. For diagnostics/plots;
  //! call after plan().
  std::vector<std::pair<double, double>> previewRoute(double stepM = 2.0) const;

 private:
  //! \brief One planned leg: a Dubins path to a virtual goal short of the waypoint plus a
  //! straight final-approach runway through the waypoint. Arriving along a straight (instead
  //! of on the tail of an arc) lets the tracker settle position and attitude before the gate.
  struct Leg {
    Leg(const DubinsPath& p, double runway, double endAz)
        : path(p), dubinsLengthM(p.lengthM()), runwayM(runway),
          lengthM(p.lengthM() + runway), endAzimuthRad(endAz) {}
    DubinsPath path;       // in the local tangent plane (math convention)
    double dubinsLengthM;  // curved portion (ends at the virtual goal)
    double runwayM;        // straight final approach ending at the waypoint
    double lengthM;        // dubinsLengthM + runwayM
    double endAzimuthRad;  // arrival azimuth at the waypoint (true-north, [-pi, pi])
  };

  //! \brief Convert a geodetic position to the local tangent plane (x east, y north).
  void toLocal(double latDeg, double lonDeg, double* xE, double* yN) const;

  //! \brief Sample the leg's path at arc length s, extending past the end along the arrival
  //! heading so guidance keeps flowing through the waypoint.
  static Dubins2DPose sampleExtended(const Leg& leg, double sM);

  //! \brief Build the Dubins leg from a local start pose to waypoint `wpIndex`.
  Leg buildLeg(const Dubins2DPose& startPose, std::size_t wpIndex) const;

  //! \brief The commanded arrival azimuth for waypoint `wpIndex` (attitude requirement if
  //! present, otherwise a natural fly-through heading toward the next waypoint).
  double arrivalAzimuth(std::size_t wpIndex, double fromXE, double fromYN) const;

  //! \brief Evaluate capture criteria for the current target against the pose. The position
  //! criterion is the gate half-width (lateral offset from the arrival axis).
  CaptureResult evaluateCapture(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose,
                                double gateLateralM) const;

  //! \brief Advance to the next waypoint after a clean gate crossing.
  void advanceToNextWaypoint();

  //! \brief Register a miss on the current waypoint; replans the leg from the live pose.
  //! Marks the route failed when the miss/replan budget is exhausted.
  void registerMiss(const Dubins2DPose& current);

  //! \brief Replan the current leg from the live pose without consuming miss/replan budget:
  //! used for the planned spiral passes of a depth-rate-limited leg.
  void spiralReplan(const Dubins2DPose& current);

  //! \brief Compute the spiral approach budget for the current leg: how many loop-back
  //! passes the commanded elevation change is expected to need at the platform's max depth
  //! rate, given the leg's path time. 0 when the leg is achievable in one pass.
  void computeElevationApproachBudget(
      const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose, double groundSpeedMps);

  //! \brief Decide whether an elevation-only gate failure is a planned spiral pass (free) or
  //! a real miss. Within the up-front budget it is always a pass; past it the budget is
  //! recomputed from the remaining elevation error and the actual loop-leg time, but only
  //! while the elevation is still converging at the platform depth rate.
  bool allowSpiralPass(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose,
                       double groundSpeedMps);

  //! \brief |commanded - current| elevation for the current waypoint, in its frame.
  std::optional<double> elevationErrorM(
      const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose) const;

  //! \brief Update the distance metrics in progress_ for the current vehicle position.
  void updateDistanceMetrics(double xE, double yN, double distToWaypointM);

  std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType> waypoints_;
  std::vector<int> missCounts_;
  PlannerParams params_;

  GeographicLib::LocalCartesian localFrame_;  // origin at the plan start pose
  Dubins2DPose planStartPose_;                // local start pose recorded by plan()
  std::vector<double> wpX_;  // waypoint local coordinates (east)
  std::vector<double> wpY_;  // waypoint local coordinates (north)

  std::size_t targetIndex_ = 0;
  std::optional<Leg> currentLeg_;
  double legProgressS_ = 0.0;           // monotonic arc-length progress along the current leg
  std::optional<double> lastGateAlongM_;  // previous signed along-track distance to the gate
  bool routeComplete_ = false;
  bool failed_ = false;
  int replanCount_ = 0;
  int elevApproachesUsed_ = 0;          // spiral passes consumed on the current leg
  std::optional<int> elevApproachBudget_;  // planned spiral passes for the current leg
  std::optional<double> lastSpiralElevErrM_;  // elevation error at the previous spiral pass
  bool hasLastPos_ = false;
  double lastXE_ = 0.0;                 // previous update position (cumulative distance)
  double lastYN_ = 0.0;
  WaypointProgress progress_;
  ControlVector lastVector_;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DUBINSPATHPLANNER_H_
