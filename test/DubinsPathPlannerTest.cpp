#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "autopilot/guidance/DubinsPathPlanner.hpp"
#include "autopilot/guidance/AngleMath.hpp"
#include "InternalTypes.h"

using GlobalWaypointType = UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using GlobalPoseReportType = UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

constexpr flt64_t kOriginLat = 39.0;
constexpr flt64_t kOriginLon = -76.5;

//! \brief Test-local kinematic vehicle: instant speed response, rate-limited turning.
struct PlannerSimVehicle {
  GeographicLib::LocalCartesian frame{kOriginLat, kOriginLon, 0.0};
  flt64_t xE = 0.0;
  flt64_t yN = 0.0;
  flt64_t yawRad = 0.0;
  flt64_t speedMps = 0.0;
  flt64_t maxTurnRateRps = 0.25;
  flt64_t maxDepthRateMps = 0.5;
  flt64_t floorDepthM = 60.0;
  std::optional<flt64_t> depthM;

  GlobalPoseReportType pose() const {
    GlobalPoseReportType p;
    flt64_t lat = 0.0;
    flt64_t lon = 0.0;
    flt64_t h = 0.0;
    frame.Reverse(xE, yN, 0.0, lat, lon, h);
    p.position().geodeticLatitude(lat);
    p.position().geodeticLongitude(lon);
    p.attitude().yaw().yaw(yawRad);
    if (depthM.has_value()) {
      p.depth() = depthM.value();
      p.altitudeASF() = std::max(0.0, floorDepthM - depthM.value());
    }
    return p;
  }

  void step(const arlcore::autopilot::ControlVector& cv, flt64_t dtS) {
    const flt64_t err = arlcore::autopilot::wrapPi(cv.headingRad - yawRad);
    const flt64_t maxDelta = maxTurnRateRps * dtS;
    yawRad = arlcore::autopilot::wrapPi(yawRad + std::clamp(err, -maxDelta, maxDelta));
    speedMps = cv.speedMps;
    xE += speedMps * dtS * std::sin(yawRad);
    yN += speedMps * dtS * std::cos(yawRad);
    if (depthM.has_value() && cv.elevationM.has_value()) {
      std::optional<flt64_t> targetDepth;
      if (cv.elevationFrame == arlcore::autopilot::ElevationFrame::DEPTH) {
        targetDepth = cv.elevationM.value();
      } else if (cv.elevationFrame == arlcore::autopilot::ElevationFrame::ALTITUDE_ASF) {
        targetDepth = floorDepthM - cv.elevationM.value();
      }
      if (targetDepth.has_value()) {
        const flt64_t dErr = std::clamp(targetDepth.value(), 0.0, floorDepthM) - depthM.value();
        depthM = depthM.value() + std::clamp(dErr, -maxDepthRateMps * dtS, maxDepthRateMps * dtS);
      }
    }
  }
};

static GlobalWaypointType makeWaypoint(flt64_t xE, flt64_t yN, flt64_t speedMps,
                                std::optional<flt64_t> arrivalYawRad = std::nullopt,
                                std::optional<flt64_t> depthM = std::nullopt,
                                std::optional<flt64_t> altitudeAsfM = std::nullopt) {
  GeographicLib::LocalCartesian frame(kOriginLat, kOriginLon, 0.0);
  flt64_t lat = 0.0;
  flt64_t lon = 0.0;
  flt64_t h = 0.0;
  frame.Reverse(xE, yN, 0.0, lat, lon, h);

  GlobalWaypointType wp;
  wp.position().value().geodeticLatitude(lat);
  wp.position().value().geodeticLongitude(lon);
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant(
      UMAA::Common::Speed::RequiredSpeedVariantType());
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant().speed()
      .SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant(
          UMAA::Common::Speed::GroundSpeedRequirementVariantType());
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant().speed()
      .SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant().speed()
      .speed(speedMps);
  if (arrivalYawRad.has_value()) {
    UMAA::Common::Orientation::Orientation3DNEDRequirement att;
    att.yawZ().yaw().yaw(arrivalYawRad.value());
    wp.attitude() = att;
  }
  if (depthM.has_value()) {
    UMAA::Common::Measurement::ElevationRequirementVariantType elev;
    elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant(
        UMAA::Common::Measurement::DepthRequirementVariantType());
    elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant().depth()
        .depth(depthM.value());
    wp.elevation() = elev;
  } else if (altitudeAsfM.has_value()) {
    UMAA::Common::Measurement::ElevationRequirementVariantType elev;
    elev.ElevationRequirementVariantTypeSubtypes().AltitudeASFRequirementVariantVariant(
        UMAA::Common::Measurement::AltitudeASFRequirementVariantType());
    elev.ElevationRequirementVariantTypeSubtypes().AltitudeASFRequirementVariantVariant()
        .altitude().altitude(altitudeAsfM.value());
    wp.elevation() = elev;
  }
  return wp;
}

static arlcore::autopilot::PlannerParams testParams() {
  arlcore::autopilot::PlannerParams p;
  p.turnRadiusM = 20.0;
  p.leadDistanceM = 30.0;
  p.posCaptureM = 12.0;
  p.yawCaptureRad = 0.35;
  p.elevCaptureM = 1.0;
  p.maxMissesPerWaypoint = 3;
  p.maxReplans = 10;
  return p;
}

//! \brief Drive the vehicle under planner guidance until the route completes/fails.
static void runMission(arlcore::autopilot::DubinsPathPlanner* planner, PlannerSimVehicle* vehicle, int32_t maxSteps, flt64_t dtS = 0.5) {
  for (int32_t i = 0; i < maxSteps && !planner->routeComplete() && !planner->failed(); i++) {
    const arlcore::autopilot::ControlVector cv = planner->update(vehicle->pose(), vehicle->speedMps);
    vehicle->step(cv, dtS);
  }
}

TEST(DubinsPathPlannerTest, EmptyRouteIsImmediatelyComplete) {
  // GIVEN: a planner and a vehicle at the origin
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  // WHEN: an empty route is planned
  planner.plan({}, vehicle.pose(), testParams());
  // THEN: the route is immediately complete and the planner commands zero speed
  EXPECT_TRUE(planner.routeComplete());
  const arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  EXPECT_DOUBLE_EQ(cv.speedMps, 0.0);
}

TEST(DubinsPathPlannerTest, FollowsMultiWaypointRouteToCompletion) {
  // GIVEN: a planned three-waypoint route
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 300.0, 4.0),
      makeWaypoint(250.0, 500.0, 4.0),
      makeWaypoint(500.0, 300.0, 4.0),
  };
  planner.plan(route, vehicle.pose(), testParams());
  ASSERT_TRUE(planner.hasRoute());
  EXPECT_FALSE(planner.routeComplete());

  // WHEN: the vehicle flies the mission under planner guidance
  runMission(&planner, &vehicle, 3000);
  // THEN: the route completes with no failure, no waypoints remain, and speed drops to zero
  EXPECT_TRUE(planner.routeComplete()) << "distance to wp: " << planner.progress().distanceToWaypointM
      << " waypointsRemaining: " << planner.progress().waypointsRemaining;
  EXPECT_FALSE(planner.failed());
  EXPECT_EQ(planner.progress().waypointsRemaining, 0);
  // After completion the planner must command zero speed.
  const arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  EXPECT_DOUBLE_EQ(cv.speedMps, 0.0);
}

TEST(DubinsPathPlannerTest, HonorsArrivalAttitude) {
  // GIVEN: a planned single-waypoint route with a due-east arrival attitude
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  const flt64_t arrivalYaw = M_PI_2;  // arrive heading due east
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 400.0, 4.0, arrivalYaw)};
  planner.plan(route, vehicle.pose(), testParams());

  // WHEN: the vehicle flies the mission
  runMission(&planner, &vehicle, 4000);
  // THEN: the route completes and the vehicle yaw at capture was within tolerance
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
  // The capture criteria include attitude, so at capture the vehicle yaw was within tolerance.
  EXPECT_LE(std::fabs(arlcore::autopilot::wrapPi(vehicle.yawRad - arrivalYaw)), 0.35 + 0.1);
}

TEST(DubinsPathPlannerTest, WaypointBehindVehicleLoopsAround) {
  // GIVEN: a vehicle facing north with the only waypoint due south behind it
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  vehicle.yawRad = 0.0;  // facing north, waypoint due south behind the vehicle
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, -250.0, 4.0)};
  planner.plan(route, vehicle.pose(), testParams());

  // WHEN: the vehicle flies the mission
  runMission(&planner, &vehicle, 3000);
  // THEN: the planner loops the vehicle around and completes without failing
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, UnmeetableElevationFailsAfterBudget) {
  // GIVEN: a depth-required waypoint, a vehicle that reports no depth, and tight budgets
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;  // reports no depth, so a depth requirement can never be achieved
  arlcore::autopilot::PlannerParams params = testParams();
  params.maxMissesPerWaypoint = 2;
  params.maxReplans = 4;
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 250.0, 4.0, std::nullopt, 10.0)};
  planner.plan(route, vehicle.pose(), params);

  // WHEN: the vehicle flies until the miss/replan budget is exhausted
  runMission(&planner, &vehicle, 20000);
  // THEN: the planner fails the route rather than completing it
  EXPECT_TRUE(planner.failed());
  EXPECT_FALSE(planner.routeComplete());
  EXPECT_TRUE(planner.progress().failed);
}

TEST(DubinsPathPlannerTest, MeetableElevationCompletes) {
  // GIVEN: a depth-required waypoint and a vehicle already at that depth
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  vehicle.depthM = 10.0;  // already at the required depth
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 250.0, 4.0, std::nullopt, 10.0)};
  planner.plan(route, vehicle.pose(), testParams());

  // WHEN: the vehicle flies the mission
  runMission(&planner, &vehicle, 3000);
  // THEN: the route completes without failing
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, ProgressMetricsAreSane) {
  // GIVEN: a planned two-waypoint route heading straight north
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 300.0, 4.0),
      makeWaypoint(0.0, 600.0, 4.0),
  };
  planner.plan(route, vehicle.pose(), testParams());

  // WHEN: the first guidance update is taken
  arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  const arlcore::autopilot::WaypointProgress first = planner.progress();
  // THEN: the initial progress metrics are valid and consistent with the geometry
  EXPECT_TRUE(first.valid);
  EXPECT_EQ(first.waypointsRemaining, 2);
  EXPECT_NEAR(first.distanceToWaypointM, 300.0, 5.0);
  EXPECT_GE(first.distanceRemainingM, 590.0);

  // WHEN: the vehicle flies 200 steps toward the first waypoint
  for (int32_t i = 0; i < 200; i++) {
    cv = planner.update(vehicle.pose(), vehicle.speedMps);
    vehicle.step(cv, 0.5);
  }
  // THEN: distances shrink and cumulative distance grows
  const arlcore::autopilot::WaypointProgress later = planner.progress();
  EXPECT_LT(later.distanceToWaypointM, first.distanceToWaypointM);
  EXPECT_LT(later.distanceRemainingM, first.distanceRemainingM);
  EXPECT_GT(later.cumulativeDistanceM, 100.0);
}

TEST(DubinsPathPlannerTest, TightTurnRadiusWithLongLeadStillCaptures) {
  // Regression: a lead distance much longer than the turn radius used to orbit a pinned carrot
  // just past the waypoint forever; the lead is now capped relative to the turn radius.
  // GIVEN: a lead distance far longer than the turn radius and a five-waypoint route
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  vehicle.maxTurnRateRps = 0.2618;  // 3 m/s cruise -> 11.46 m turn radius
  arlcore::autopilot::PlannerParams params = testParams();
  params.turnRadiusM = 3.0 / 0.2618;
  params.leadDistanceM = 50.0;
  params.posCaptureM = 12.0;
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 350.0, 3.0),
      makeWaypoint(250.0, 600.0, 3.0),
      makeWaypoint(500.0, 350.0, 3.0),
      makeWaypoint(250.0, 100.0, 3.0),
      makeWaypoint(-50.0, 350.0, 3.0),
  };
  planner.plan(route, vehicle.pose(), params);

  // WHEN: the vehicle flies the mission
  runMission(&planner, &vehicle, 20000);
  // THEN: every waypoint is captured and the route completes
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, DenseLawnmowerWithArrivalAttitudes) {
  // Lawnmower lanes (10 m) tighter than the turning circle diameter (~23 m) force bulb turns
  // that cross neighboring capture zones mid-turn; those crossings must not burn the miss budget.
  // GIVEN: a dense lawnmower route with arrival attitudes and lanes tighter than the turn circle
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  vehicle.maxTurnRateRps = 0.2618;
  arlcore::autopilot::PlannerParams params = testParams();
  params.turnRadiusM = 3.0 / 0.2618;
  params.leadDistanceM = 50.0;
  params.posCaptureM = 5.0;
  const flt64_t north = 0.0;
  const flt64_t south = M_PI;
  std::vector<GlobalWaypointType> route;
  const flt64_t y0 = 100.0;
  const flt64_t y1 = 300.0;
  for (int32_t lane = 0; lane < 4; lane++) {
    const flt64_t x = 10.0 * lane;
    const bool up = (lane % 2 == 0);
    const flt64_t yaw = up ? north : south;
    route.push_back(makeWaypoint(x, up ? y0 : y1, 3.0, yaw));
    route.push_back(makeWaypoint(x, up ? y1 : y0, 3.0, yaw));
  }
  planner.plan(route, vehicle.pose(), params);

  // WHEN: the vehicle flies the mission
  runMission(&planner, &vehicle, 20000, 0.1);
  // THEN: the route completes without burning the miss budget
  EXPECT_TRUE(planner.routeComplete()) << "target " << planner.progress().waypointsRemaining
      << " remaining, dist " << planner.progress().distanceToWaypointM;
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, GateCaptureHappensAtTheWaypointPlane) {
  // With gate capture the vehicle flies THROUGH the waypoint instead of capturing at first
  // contact with a bubble: at capture the vehicle must be abeam the waypoint (crossing its
  // gate plane), not a capture-radius early.
  // GIVEN: two waypoints with arrival attitudes so each has a gate plane
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  arlcore::autopilot::PlannerParams params = testParams();
  params.posCaptureM = 2.5;
  const flt64_t arrivalYaw = 0.0;  // gate plane is the east-west line through the waypoint
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 3.0, arrivalYaw),
                                           makeWaypoint(0.0, 500.0, 3.0, arrivalYaw)};
  planner.plan(route, vehicle.pose(), params);

  // WHEN: the vehicle flies the mission while recording where the first capture happens
  bool sawFirstCapture = false;
  flt64_t captureNorth = 0.0;
  for (int32_t i = 0; i < 4000 && !planner.routeComplete() && !planner.failed(); i++) {
    const arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    if (!sawFirstCapture && planner.progress().waypointsRemaining == 1) {
      sawFirstCapture = true;
      captureNorth = vehicle.yN;
    }
    vehicle.step(cv, 0.1);
  }
  // THEN: the route completes and the first capture occurred at the gate plane
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
  ASSERT_TRUE(sawFirstCapture);
  // Captured when crossing the gate plane at north=300 m, not a capture radius before it.
  EXPECT_NEAR(captureNorth, 300.0, 0.5);
}

TEST(DubinsPathPlannerTest, DepthRateLimitedLegSpiralsWithoutFailing) {
  // Two depths where the platform's depth rate cannot complete the change within one pass of
  // the 2D path: the planner must budget spiral loop-backs and complete without consuming
  // the miss budget.
  // GIVEN: a depth change that cannot finish in one pass and a miss budget of one
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  vehicle.depthM = 5.0;
  vehicle.maxDepthRateMps = 0.15;
  arlcore::autopilot::PlannerParams params = testParams();
  params.posCaptureM = 3.0;
  params.maxDepthRateMps = 0.15;
  params.maxMissesPerWaypoint = 1;  // spirals must not consume misses
  params.maxReplans = 3;            // nor ordinary replans
  // 250 m leg at 3 m/s is ~83 s; 35 m depth change at 0.15 m/s needs ~233 s (~3 passes).
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 250.0, 3.0, std::nullopt, 40.0)};
  planner.plan(route, vehicle.pose(), params);

  // WHEN: the vehicle flies the mission
  runMission(&planner, &vehicle, 40000, 0.1);
  // THEN: the planner spirals until the depth is reached and completes without failing
  EXPECT_TRUE(planner.routeComplete()) << "depth err: " << (vehicle.depthM.value() - 40.0);
  EXPECT_FALSE(planner.failed());
  EXPECT_NEAR(vehicle.depthM.value(), 40.0, 1.5);
  // The spiral means the vehicle traveled well beyond the straight-line leg length.
  EXPECT_GT(planner.progress().cumulativeDistanceM, 400.0);
}

TEST(DubinsPathPlannerTest, AltitudeAboveSeaFloorWaypointCompletes) {
  // GIVEN: an altitude-above-sea-floor waypoint achievable within one pass
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  vehicle.depthM = 10.0;   // floor at 60 -> ASF 50
  vehicle.floorDepthM = 60.0;
  arlcore::autopilot::PlannerParams params = testParams();
  params.maxDepthRateMps = 0.5;
  // Command 15 m above the sea floor (= 45 m depth); 300 m at 3 m/s = 100 s; 35 m depth
  // change at 0.5 m/s = 70 s -> achievable in one pass.
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 300.0, 3.0, std::nullopt, std::nullopt, 15.0)};
  planner.plan(route, vehicle.pose(), params);

  // WHEN: the vehicle flies the mission
  runMission(&planner, &vehicle, 20000, 0.1);
  // THEN: the route completes at the commanded altitude above the floor
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
  EXPECT_NEAR(vehicle.depthM.value(), 45.0, 1.5);
}

TEST(DubinsPathPlannerTest, CrossTrackErrorIsJudgedAgainstThePlannedPath) {
  // GIVEN: a vehicle facing east so the planned path begins with a turn
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  vehicle.yawRad = M_PI_2;  // start facing east: the planned path begins with a turn
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 3.0)};
  planner.plan(route, vehicle.pose(), testParams());

  // On-path tracking: the reported cross-track error is measured from the planned Dubins
  // path, so it stays small even while the vehicle is mid-turn, far from any straight line
  // between the waypoints.
  // WHEN: the vehicle flies the mission while recording the maximum cross-track error
  flt64_t maxAbsXte = 0.0;
  for (int32_t i = 0; i < 4000 && !planner.routeComplete() && !planner.failed(); i++) {
    const arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    if (planner.progress().crossTrackErrorM.has_value()) {
      maxAbsXte = std::max(maxAbsXte, std::fabs(planner.progress().crossTrackErrorM.value()));
    }
    vehicle.step(cv, 0.1);
  }
  // THEN: the route completes and the error stayed small even mid-turn
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_LT(maxAbsXte, 5.0);
}

TEST(DubinsPathPlannerTest, CommandsWaypointSpeedAndElevation) {
  // GIVEN: a planned waypoint carrying a required speed and depth
  arlcore::autopilot::DubinsPathPlanner planner;
  PlannerSimVehicle vehicle;
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 3.5, std::nullopt, 25.0)};
  planner.plan(route, vehicle.pose(), testParams());

  // WHEN: the first guidance update is taken
  const arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  // THEN: the control vector commands the waypoint's speed and depth
  EXPECT_DOUBLE_EQ(cv.speedMps, 3.5);
  ASSERT_TRUE(cv.elevationM.has_value());
  EXPECT_DOUBLE_EQ(cv.elevationM.value(), 25.0);
  EXPECT_EQ(cv.elevationFrame, arlcore::autopilot::ElevationFrame::DEPTH);
}
