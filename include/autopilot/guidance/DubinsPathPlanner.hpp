#ifndef AUTOPILOT_GUIDANCE_DUBINSPATHPLANNER_HPP_
#define AUTOPILOT_GUIDANCE_DUBINSPATHPLANNER_HPP_

#include <GeographicLib/LocalCartesian.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointType.hpp>
#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "InternalTypes.h"
#include "autopilot/guidance/ControlVector.hpp"
#include "autopilot/guidance/CrossTrackController.hpp"
#include "autopilot/guidance/DubinsPath.hpp"
#include "autopilot/guidance/DubinsRrtStar.hpp"
#include "autopilot/guidance/ProgressTypes.hpp"
#include "autopilot/safety/ZoneGeometry.hpp"
#include "autopilot/safety/ZoneMap.hpp"

namespace arlcore::autopilot {

//! \brief Parameters that tune the planner. Capture defaults are used when a waypoint omits
//! its own tolerances. Everything kinematic is derived from the platform capabilities (see
//! PlannerParamsFactory.h).
struct PlannerParams {
  flt64_t turnRadiusM = 25.0;        // margin * (speed / maxTurnRate) from the capabilities
  flt64_t leadDistanceM = 50.0;      // progress-search window scale along the planned path
  flt64_t posCaptureM = 2.5;         // default capture-gate half-width
  flt64_t yawCaptureRad = 0.1745;    // default arrival-attitude capture half-width
  flt64_t elevCaptureM = 1.0;        // default elevation capture tolerance
  int32_t maxMissesPerWaypoint = 3;  // misses before the route fails
  bool elevationCountsAsMiss = true;
  int32_t maxReplans = 10;           // guard against endless replanning (spirals excluded)
  flt64_t sampleStepM = 2.0;         // path polyline sampling resolution
  flt64_t maxDepthRateMps = 0.0;     // platform depth-change limit (0 = unknown/surface-only)
  flt64_t zoneMarginM = 5.0;         // required clearance from active zone boundaries
  flt64_t leadTimeS = 1.0;           // tangent phase-lead seconds ahead of the progress pointer
  CrossTrackController::Params xte;  // cross-track PI tuning (ki = 0 -> legacy pure P)
  DubinsRrtParams rrt;               // fallback planner tuning (rho/margin filled per leg)
};

//! \brief Drives a vehicle through a series of 3D waypoints along true Dubins paths, each leg
//! ending in a straight final-approach runway so arrival happens with position and attitude
//! settled.
//! Tracking commands the planned-path tangent (sampled slightly ahead for phase lead) plus a
//! cross-track correction atan(xte / turnRadius); cross-track error is measured from the
//! planned Dubins path itself, which is also the UMAA track-tolerance reference.
//! Capture is a gate, not a bubble: a segment of the position-tolerance half-width through the
//! waypoint, perpendicular to the arrival heading, so the vehicle always flies through the
//! waypoint; crossing outside the gate is a miss that replans the leg from the live pose.
class DubinsPathPlanner {
 public:
  //! \brief Install the shared zone map (nullable = no zone awareness); legs planned
  //! afterwards avoid the active zones with the configured margin.
  void setZones(const ZoneMap* zoneMap) { zoneMap_ = zoneMap; }

  //! \brief Re-project the active zones (the constraint set changed mid-route): the remaining
  //! portion of the current leg is rechecked and replanned budget-free when it is now blocked;
  //! a route whose current target waypoint became non-compliant fails.
  void onConstraintsChanged(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);

  //! \brief Replan the current leg from the live pose without consuming miss/replan budget
  //! (recovery handoff / constraint change). Returns false when the leg cannot be planned
  //! (the route is then failed).
  bool replanCurrentLegFrom(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);

  //! \brief Plan a route up front from the current pose through the given waypoints.
  void plan(const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints,
            const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& start, const PlannerParams& params);

  //! \brief Produce a control vector for the current pose and advance capture/miss/replan
  //! state. Call once per navigation packet with the latest ground speed and the elapsed
  //! seconds since the previous packet (feeds the cross-track PI and the spiral budget;
  //! pass 0 when unknown, which freezes the integral for that tick).
  ControlVector update(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose, flt64_t groundSpeedMps,
                       flt64_t dtS);

  //! \brief The cross-track integrator state, for tests and diagnostics.
  flt64_t xteIntegratorRad() const { return xteCtl_.integratorRad(); }

  //! \brief Latest progress snapshot (mapped into the UMAA waypoint execution status report).
  const WaypointProgress& progress() const { return progress_; }

  bool routeComplete() const { return routeComplete_; }
  bool failed() const { return failed_; }
  bool hasRoute() const { return !waypoints_.empty(); }

  //! \brief Sample the ideal planned route (all legs chained waypoint-to-waypoint from the
  //! plan start pose) as geodetic (lat, lon) points every `stepM`. For diagnostics/plots;
  //! call after plan().
  std::vector<std::pair<flt64_t, flt64_t>> previewRoute(flt64_t stepM = 2.0) const;

 private:
  //! \brief One planned leg: a chain of Dubins paths (a single direct solution, or the
  //! Dubins-RRT* detour around zones) to a virtual goal short of the waypoint plus a straight
  //! final-approach runway through the waypoint. Arriving along a straight (instead of on the
  //! tail of an arc) lets the tracker settle position and attitude before the gate.
  struct Leg {
    Leg(std::vector<DubinsPath> chain, flt64_t runway, flt64_t endAz);
    std::vector<DubinsPath> paths;    // in the local tangent plane (math convention)
    std::vector<flt64_t> pathStartM;  // arc length at the start of each chained path
    flt64_t dubinsLengthM;            // curved portion (ends at the virtual goal)
    flt64_t runwayM;                  // straight final approach ending at the waypoint
    flt64_t lengthM;                  // dubinsLengthM + runwayM
    flt64_t endAzimuthRad;            // arrival azimuth at the waypoint (true-north, [-pi, pi])

    //! \brief The pose at arc length s along the chained curved portion (clamped).
    Dubins2DPose sampleChain(flt64_t sM) const;

    //! \brief The chained word names, e.g. "LSL+RSR" (diagnostics/logs).
    std::string word() const;
  };

  //! \brief Convert a geodetic position to the local tangent plane (x east, y north).
  void toLocal(flt64_t latDeg, flt64_t lonDeg, flt64_t* xE, flt64_t* yN) const;

  //! \brief Sample the leg's path at arc length s, extending past the end along the arrival
  //! heading so guidance keeps flowing through the waypoint.
  static Dubins2DPose sampleExtended(const Leg& leg, flt64_t sM);

  //! \brief Build the leg from a local start pose to waypoint `wpIndex` through the pipeline:
  //! arrival-azimuth compliance (scanning alternates when the waypoint has no attitude
  //! requirement), direct Dubins solve + clearance check, Dubins-RRT* fallback. Returns
  //! nullopt when no compliant leg exists (the caller fails the route).
  std::optional<Leg> buildLeg(const Dubins2DPose& startPose, std::size_t wpIndex) const;

  //! \brief The arrival azimuth whose final-approach runway (virtual goal -> waypoint) keeps
  //! the zone margin: the natural/commanded azimuth when compliant, otherwise the nearest
  //! alternative in 22.5-degree steps (only when the waypoint has no attitude requirement).
  std::optional<flt64_t> compliantArrivalAzimuth(std::size_t wpIndex, flt64_t naturalAz, flt64_t runwayM) const;

  //! \brief Whether the remaining portion of a leg (from arc length `fromS`) keeps the margin.
  bool legClear(const Leg& leg, flt64_t fromS) const;

  //! \brief Re-project the active zones into the plan frame, gated by the route's depth
  //! envelope (current depth plus every DEPTH-frame waypoint elevation, padded; any
  //! unconvertible elevation frame conservatively activates every zone).
  void refreshZoneSet(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose);

  //! \brief The commanded arrival azimuth for waypoint `wpIndex` (attitude requirement if
  //! present, otherwise a natural fly-through heading toward the next waypoint).
  flt64_t arrivalAzimuth(std::size_t wpIndex, flt64_t fromXE, flt64_t fromYN) const;

  //! \brief Evaluate capture criteria for the current target against the pose. The position
  //! criterion is the gate half-width (lateral offset from the arrival axis).
  CaptureResult evaluateCapture(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose,
                                flt64_t gateLateralM) const;

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
  void computeElevationApproachBudget(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose,
                                      flt64_t groundSpeedMps);

  //! \brief Decide whether an elevation-only gate failure is a planned spiral pass (free) or
  //! a real miss. Within the up-front budget it is always a pass; past it the budget is
  //! recomputed from the remaining elevation error and the actual loop-leg time, but only
  //! while the elevation is still converging at the platform depth rate.
  bool allowSpiralPass(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose, flt64_t groundSpeedMps);

  //! \brief |commanded - current| elevation for the current waypoint, in its frame.
  std::optional<flt64_t> elevationErrorM(const UMAA::SA::GlobalPoseStatus::GlobalPoseReportType& pose) const;

  //! \brief Update the distance metrics in progress_ for the current vehicle position.
  void updateDistanceMetrics(flt64_t xE, flt64_t yN, flt64_t distToWaypointM);

  std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType> waypoints_;
  std::vector<int32_t> missCounts_;
  PlannerParams params_;

  const ZoneMap* zoneMap_ = nullptr;  // shared zone store (nullable)
  ZoneSet zoneSet_;                   // active zones projected into localFrame_

  GeographicLib::LocalCartesian localFrame_;  // origin at the plan start pose
  Dubins2DPose planStartPose_;                // local start pose recorded by plan()
  std::vector<flt64_t> wpX_;                  // waypoint local coordinates (east)
  std::vector<flt64_t> wpY_;                  // waypoint local coordinates (north)

  std::size_t targetIndex_ = 0;
  std::optional<Leg> currentLeg_;
  flt64_t legProgressS_ = 0.0;             // monotonic arc-length progress along the current leg
  CrossTrackController xteCtl_;            // reset at every leg boundary/replan
  std::optional<flt64_t> lastGateAlongM_;  // previous signed along-track distance to the gate
  bool routeComplete_ = false;
  bool failed_ = false;
  int32_t replanCount_ = 0;
  int32_t elevApproachesUsed_ = 0;             // spiral passes consumed on the current leg
  std::optional<int32_t> elevApproachBudget_;  // planned spiral passes for the current leg
  std::optional<flt64_t> lastSpiralElevErrM_;  // elevation error at the previous spiral pass
  bool hasLastPos_ = false;
  flt64_t lastXE_ = 0.0;  // previous update position (cumulative distance)
  flt64_t lastYN_ = 0.0;
  WaypointProgress progress_;
  ControlVector lastVector_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_DUBINSPATHPLANNER_HPP_
