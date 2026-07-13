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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "DubinsPathPlanner.h"
#include "AngleMath.h"

namespace arlcore::autopilot {

namespace {

using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

constexpr double kOriginLat = 39.0;
constexpr double kOriginLon = -76.5;

//! \brief Test-local kinematic vehicle: instant speed response, rate-limited turning.
struct SimVehicle {
  GeographicLib::LocalCartesian frame{kOriginLat, kOriginLon, 0.0};
  double xE = 0.0;
  double yN = 0.0;
  double yawRad = 0.0;
  double speedMps = 0.0;
  double maxTurnRateRps = 0.25;
  double maxDepthRateMps = 0.5;
  double floorDepthM = 60.0;
  std::optional<double> depthM;

  GlobalPoseReportType pose() const {
    GlobalPoseReportType p;
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
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

  void step(const ControlVector& cv, double dtS) {
    const double err = wrapPi(cv.headingRad - yawRad);
    const double maxDelta = maxTurnRateRps * dtS;
    yawRad = wrapPi(yawRad + std::clamp(err, -maxDelta, maxDelta));
    speedMps = cv.speedMps;
    xE += speedMps * dtS * std::sin(yawRad);
    yN += speedMps * dtS * std::cos(yawRad);
    if (depthM.has_value() && cv.elevationM.has_value()) {
      std::optional<double> targetDepth;
      if (cv.elevationFrame == ElevationFrame::DEPTH) {
        targetDepth = cv.elevationM.value();
      } else if (cv.elevationFrame == ElevationFrame::ALTITUDE_ASF) {
        targetDepth = floorDepthM - cv.elevationM.value();
      }
      if (targetDepth.has_value()) {
        const double dErr = std::clamp(targetDepth.value(), 0.0, floorDepthM) - depthM.value();
        depthM = depthM.value() + std::clamp(dErr, -maxDepthRateMps * dtS, maxDepthRateMps * dtS);
      }
    }
  }
};

GlobalWaypointType makeWaypoint(double xE, double yN, double speedMps,
                                std::optional<double> arrivalYawRad = std::nullopt,
                                std::optional<double> depthM = std::nullopt,
                                std::optional<double> altitudeAsfM = std::nullopt) {
  GeographicLib::LocalCartesian frame(kOriginLat, kOriginLon, 0.0);
  double lat = 0.0;
  double lon = 0.0;
  double h = 0.0;
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

PlannerParams testParams() {
  PlannerParams p;
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
void runMission(DubinsPathPlanner* planner, SimVehicle* vehicle, int maxSteps, double dtS = 0.5) {
  for (int i = 0; i < maxSteps && !planner->routeComplete() && !planner->failed(); i++) {
    const ControlVector cv = planner->update(vehicle->pose(), vehicle->speedMps);
    vehicle->step(cv, dtS);
  }
}

}  // namespace

TEST(DubinsPathPlannerTest, EmptyRouteIsImmediatelyComplete) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  planner.plan({}, vehicle.pose(), testParams());
  EXPECT_TRUE(planner.routeComplete());
  const ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  EXPECT_DOUBLE_EQ(cv.speedMps, 0.0);
}

TEST(DubinsPathPlannerTest, FollowsMultiWaypointRouteToCompletion) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 300.0, 4.0),
      makeWaypoint(250.0, 500.0, 4.0),
      makeWaypoint(500.0, 300.0, 4.0),
  };
  planner.plan(route, vehicle.pose(), testParams());
  ASSERT_TRUE(planner.hasRoute());
  EXPECT_FALSE(planner.routeComplete());

  runMission(&planner, &vehicle, 3000);
  EXPECT_TRUE(planner.routeComplete()) << "distance to wp: " << planner.progress().distanceToWaypointM
      << " waypointsRemaining: " << planner.progress().waypointsRemaining;
  EXPECT_FALSE(planner.failed());
  EXPECT_EQ(planner.progress().waypointsRemaining, 0);
  // After completion the planner must command zero speed.
  const ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  EXPECT_DOUBLE_EQ(cv.speedMps, 0.0);
}

TEST(DubinsPathPlannerTest, HonorsArrivalAttitude) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  const double arrivalYaw = M_PI_2;  // arrive heading due east
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 400.0, 4.0, arrivalYaw)};
  planner.plan(route, vehicle.pose(), testParams());

  runMission(&planner, &vehicle, 4000);
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
  // The capture criteria include attitude, so at capture the vehicle yaw was within tolerance.
  EXPECT_LE(std::fabs(wrapPi(vehicle.yawRad - arrivalYaw)), 0.35 + 0.1);
}

TEST(DubinsPathPlannerTest, WaypointBehindVehicleLoopsAround) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  vehicle.yawRad = 0.0;  // facing north, waypoint due south behind the vehicle
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, -250.0, 4.0)};
  planner.plan(route, vehicle.pose(), testParams());

  runMission(&planner, &vehicle, 3000);
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, UnmeetableElevationFailsAfterBudget) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;  // reports no depth, so a depth requirement can never be achieved
  PlannerParams params = testParams();
  params.maxMissesPerWaypoint = 2;
  params.maxReplans = 4;
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 250.0, 4.0, std::nullopt, 10.0)};
  planner.plan(route, vehicle.pose(), params);

  runMission(&planner, &vehicle, 20000);
  EXPECT_TRUE(planner.failed());
  EXPECT_FALSE(planner.routeComplete());
  EXPECT_TRUE(planner.progress().failed);
}

TEST(DubinsPathPlannerTest, MeetableElevationCompletes) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  vehicle.depthM = 10.0;  // already at the required depth
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 250.0, 4.0, std::nullopt, 10.0)};
  planner.plan(route, vehicle.pose(), testParams());

  runMission(&planner, &vehicle, 3000);
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, ProgressMetricsAreSane) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 300.0, 4.0),
      makeWaypoint(0.0, 600.0, 4.0),
  };
  planner.plan(route, vehicle.pose(), testParams());

  ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  const WaypointProgress first = planner.progress();
  EXPECT_TRUE(first.valid);
  EXPECT_EQ(first.waypointsRemaining, 2);
  EXPECT_NEAR(first.distanceToWaypointM, 300.0, 5.0);
  EXPECT_GE(first.distanceRemainingM, 590.0);

  for (int i = 0; i < 200; i++) {
    cv = planner.update(vehicle.pose(), vehicle.speedMps);
    vehicle.step(cv, 0.5);
  }
  const WaypointProgress later = planner.progress();
  EXPECT_LT(later.distanceToWaypointM, first.distanceToWaypointM);
  EXPECT_LT(later.distanceRemainingM, first.distanceRemainingM);
  EXPECT_GT(later.cumulativeDistanceM, 100.0);
}

TEST(DubinsPathPlannerTest, TightTurnRadiusWithLongLeadStillCaptures) {
  // Regression: a lead distance much longer than the turn radius used to cut the final arc
  // so hard the vehicle orbited a pinned carrot just past the waypoint forever. The lead is
  // now capped relative to the turn radius and the carrot keeps receding past the path end.
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  vehicle.maxTurnRateRps = 0.2618;  // 3 m/s cruise -> 11.46 m turn radius
  PlannerParams params = testParams();
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

  runMission(&planner, &vehicle, 20000);
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, DenseLawnmowerWithArrivalAttitudes) {
  // Survey lawnmower: north/south lanes with required arrival attitudes, lane spacing (10 m)
  // tighter than the turning circle diameter (~23 m), forcing bulb turns whose planned path
  // crosses neighboring capture zones mid-turn. Those crossings must not burn the miss budget.
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  vehicle.maxTurnRateRps = 0.2618;
  PlannerParams params = testParams();
  params.turnRadiusM = 3.0 / 0.2618;
  params.leadDistanceM = 50.0;
  params.posCaptureM = 5.0;
  const double north = 0.0;
  const double south = M_PI;
  std::vector<GlobalWaypointType> route;
  const double y0 = 100.0;
  const double y1 = 300.0;
  for (int lane = 0; lane < 4; lane++) {
    const double x = 10.0 * lane;
    const bool up = (lane % 2 == 0);
    const double yaw = up ? north : south;
    route.push_back(makeWaypoint(x, up ? y0 : y1, 3.0, yaw));
    route.push_back(makeWaypoint(x, up ? y1 : y0, 3.0, yaw));
  }
  planner.plan(route, vehicle.pose(), params);

  runMission(&planner, &vehicle, 20000, 0.1);
  EXPECT_TRUE(planner.routeComplete()) << "target " << planner.progress().waypointsRemaining
      << " remaining, dist " << planner.progress().distanceToWaypointM;
  EXPECT_FALSE(planner.failed());
}

TEST(DubinsPathPlannerTest, GateCaptureHappensAtTheWaypointPlane) {
  // With gate capture the vehicle flies THROUGH the waypoint instead of capturing at first
  // contact with a bubble: at capture the vehicle must be abeam the waypoint (crossing its
  // gate plane), not a capture-radius early.
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  PlannerParams params = testParams();
  params.posCaptureM = 2.5;
  const double arrivalYaw = 0.0;  // gate plane is the east-west line through the waypoint
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 3.0, arrivalYaw),
                                           makeWaypoint(0.0, 500.0, 3.0, arrivalYaw)};
  planner.plan(route, vehicle.pose(), params);

  bool sawFirstCapture = false;
  double captureNorth = 0.0;
  for (int i = 0; i < 4000 && !planner.routeComplete() && !planner.failed(); i++) {
    const ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    if (!sawFirstCapture && planner.progress().waypointsRemaining == 1) {
      sawFirstCapture = true;
      captureNorth = vehicle.yN;
    }
    vehicle.step(cv, 0.1);
  }
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
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  vehicle.depthM = 5.0;
  vehicle.maxDepthRateMps = 0.15;
  PlannerParams params = testParams();
  params.posCaptureM = 3.0;
  params.maxDepthRateMps = 0.15;
  params.maxMissesPerWaypoint = 1;  // spirals must not consume misses
  params.maxReplans = 3;            // nor ordinary replans
  // 250 m leg at 3 m/s is ~83 s; 35 m depth change at 0.15 m/s needs ~233 s (~3 passes).
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 250.0, 3.0, std::nullopt, 40.0)};
  planner.plan(route, vehicle.pose(), params);

  runMission(&planner, &vehicle, 40000, 0.1);
  EXPECT_TRUE(planner.routeComplete()) << "depth err: " << (vehicle.depthM.value() - 40.0);
  EXPECT_FALSE(planner.failed());
  EXPECT_NEAR(vehicle.depthM.value(), 40.0, 1.5);
  // The spiral means the vehicle traveled well beyond the straight-line leg length.
  EXPECT_GT(planner.progress().cumulativeDistanceM, 400.0);
}

TEST(DubinsPathPlannerTest, AltitudeAboveSeaFloorWaypointCompletes) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  vehicle.depthM = 10.0;   // floor at 60 -> ASF 50
  vehicle.floorDepthM = 60.0;
  PlannerParams params = testParams();
  params.maxDepthRateMps = 0.5;
  // Command 15 m above the sea floor (= 45 m depth); 300 m at 3 m/s = 100 s; 35 m depth
  // change at 0.5 m/s = 70 s -> achievable in one pass.
  std::vector<GlobalWaypointType> route = {
      makeWaypoint(0.0, 300.0, 3.0, std::nullopt, std::nullopt, 15.0)};
  planner.plan(route, vehicle.pose(), params);

  runMission(&planner, &vehicle, 20000, 0.1);
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_FALSE(planner.failed());
  EXPECT_NEAR(vehicle.depthM.value(), 45.0, 1.5);
}

TEST(DubinsPathPlannerTest, CrossTrackErrorIsJudgedAgainstThePlannedPath) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  vehicle.yawRad = M_PI_2;  // start facing east: the planned path begins with a turn
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 3.0)};
  planner.plan(route, vehicle.pose(), testParams());

  // On-path tracking: the reported cross-track error is measured from the planned Dubins
  // path, so it stays small even while the vehicle is mid-turn, far from any straight line
  // between the waypoints.
  double maxAbsXte = 0.0;
  for (int i = 0; i < 4000 && !planner.routeComplete() && !planner.failed(); i++) {
    const ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    if (planner.progress().crossTrackErrorM.has_value()) {
      maxAbsXte = std::max(maxAbsXte, std::fabs(planner.progress().crossTrackErrorM.value()));
    }
    vehicle.step(cv, 0.1);
  }
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_LT(maxAbsXte, 5.0);
}

TEST(DubinsPathPlannerTest, CommandsWaypointSpeedAndElevation) {
  DubinsPathPlanner planner;
  SimVehicle vehicle;
  std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 3.5, std::nullopt, 25.0)};
  planner.plan(route, vehicle.pose(), testParams());

  const ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
  EXPECT_DOUBLE_EQ(cv.speedMps, 3.5);
  ASSERT_TRUE(cv.elevationM.has_value());
  EXPECT_DOUBLE_EQ(cv.elevationM.value(), 25.0);
  EXPECT_EQ(cv.elevationFrame, ElevationFrame::DEPTH);
}

}  // namespace arlcore::autopilot
