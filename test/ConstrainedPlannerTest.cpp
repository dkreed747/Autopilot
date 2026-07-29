#include <gtest/gtest.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "InternalTypes.h"
#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/guidance/DubinsPathPlanner.hpp"
#include "autopilot/safety/ZoneMap.hpp"

using GlobalWaypointType = UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using GlobalPoseReportType = UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

constexpr flt64_t kOriginLat = 39.0;
constexpr flt64_t kOriginLon = -76.5;

static const GeographicLib::LocalCartesian& testFrame() {
  static const GeographicLib::LocalCartesian frame(kOriginLat, kOriginLon, 0.0);
  return frame;
}

static arlcore::autopilot::GeoPoint at(flt64_t east, flt64_t north) {
  arlcore::autopilot::GeoPoint p;
  flt64_t h = 0.0;
  testFrame().Reverse(east, north, 0.0, p.latDeg, p.lonDeg, h);
  return p;
}

//! \brief Same kinematic vehicle as DubinsPathPlannerTest.
struct ConstrainedSimVehicle {
  flt64_t xE = 0.0;
  flt64_t yN = 0.0;
  flt64_t yawRad = 0.0;
  flt64_t speedMps = 0.0;
  flt64_t maxTurnRateRps = 0.25;

  GlobalPoseReportType pose() const {
    GlobalPoseReportType p;
    flt64_t lat = 0.0;
    flt64_t lon = 0.0;
    flt64_t h = 0.0;
    testFrame().Reverse(xE, yN, 0.0, lat, lon, h);
    p.position().geodeticLatitude(lat);
    p.position().geodeticLongitude(lon);
    p.attitude().yaw().yaw(yawRad);
    return p;
  }

  void step(const arlcore::autopilot::ControlVector& cv, flt64_t dtS) {
    const flt64_t err = arlcore::autopilot::wrapPi(cv.headingRad - yawRad);
    const flt64_t maxDelta = maxTurnRateRps * dtS;
    yawRad = arlcore::autopilot::wrapPi(yawRad + std::clamp(err, -maxDelta, maxDelta));
    speedMps = cv.speedMps;
    xE += speedMps * dtS * std::sin(yawRad);
    yN += speedMps * dtS * std::cos(yawRad);
  }
};

static GlobalWaypointType makeWaypoint(flt64_t xE, flt64_t yN, flt64_t speedMps) {
  const arlcore::autopilot::GeoPoint p = at(xE, yN);
  GlobalWaypointType wp;
  wp.position().value().geodeticLatitude(p.latDeg);
  wp.position().value().geodeticLongitude(p.lonDeg);
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant(
      UMAA::Common::Speed::RequiredSpeedVariantType());
  wp.speed()
      .VariableSpeedVariantTypeSubtypes()
      .RequiredSpeedVariantVariant()
      .speed()
      .SpeedRequirementVariantTypeSubtypes()
      .GroundSpeedRequirementVariantVariant(UMAA::Common::Speed::GroundSpeedRequirementVariantType());
  wp.speed()
      .VariableSpeedVariantTypeSubtypes()
      .RequiredSpeedVariantVariant()
      .speed()
      .SpeedRequirementVariantTypeSubtypes()
      .GroundSpeedRequirementVariantVariant()
      .speed()
      .speed(speedMps);
  return wp;
}

static arlcore::autopilot::PlannerParams testParams() {
  arlcore::autopilot::PlannerParams p;
  p.turnRadiusM = 20.0;
  p.leadDistanceM = 30.0;
  p.posCaptureM = 12.0;
  p.zoneMarginM = 5.0;
  p.rrt.seed = 42;
  p.rrt.maxIterations = 1500;
  p.rrt.timeBudgetMs = 2000.0;
  return p;
}

//! \brief A keep-out square in the test frame.
static arlcore::autopilot::ZoneRecord keepOut(flt64_t x0, flt64_t y0, flt64_t x1, flt64_t y1) {
  arlcore::autopilot::ZoneRecord zone;
  zone.kind = arlcore::autopilot::ZoneKind::KEEP_OUT;
  arlcore::autopilot::ZoneShape shape;
  shape.polygon = {at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)};
  zone.shapes.push_back(shape);
  return zone;
}

static arlcore::autopilot::ZoneMap makeMap(std::vector<arlcore::autopilot::ZoneRecord> zones, uint64_t revision = 1) {
  arlcore::autopilot::ZoneMap map(arlcore::autopilot::ZonesConfig{});
  arlcore::autopilot::ConstraintSnapshot snapshot;
  snapshot.revision = revision;
  snapshot.zones = std::move(zones);
  map.ingest(snapshot);
  return map;
}

//! \brief Minimum keep-out clearance over a geodetic preview polyline.
static flt64_t previewMinClearance(const std::vector<std::pair<flt64_t, flt64_t>>& preview,
                                   const arlcore::autopilot::ZoneMap& map) {
  flt64_t minClearance = 1e18;
  for (const auto& [lat, lon] : preview) {
    minClearance = std::min(minClearance, map.clearanceM(arlcore::autopilot::GeoPoint{lat, lon}, 0.0));
  }
  return minClearance;
}

static void runMission(arlcore::autopilot::DubinsPathPlanner* planner, ConstrainedSimVehicle* vehicle, int32_t maxSteps,
                       flt64_t dtS = 0.5) {
  for (int32_t i = 0; i < maxSteps && !planner->routeComplete() && !planner->failed(); i++) {
    const arlcore::autopilot::ControlVector cv = planner->update(vehicle->pose(), vehicle->speedMps);
    vehicle->step(cv, dtS);
  }
}

TEST(ConstrainedPlannerTest, NoZonesMatchesUnconstrainedBehavior) {
  // Regression: a planner with an empty zone map plans/flies exactly like a zone-blind one.
  // GIVEN: a zone-blind planner and one carrying an empty zone map
  arlcore::autopilot::DubinsPathPlanner blind;
  arlcore::autopilot::DubinsPathPlanner mapped;
  arlcore::autopilot::ZoneMap empty = makeMap({});
  mapped.setZones(&empty);

  ConstrainedSimVehicle vehicle;
  const std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 4.0), makeWaypoint(250.0, 500.0, 4.0)};
  // WHEN: both plan the same route from the same pose
  blind.plan(route, vehicle.pose(), testParams());
  mapped.plan(route, vehicle.pose(), testParams());
  // THEN: the mapped planner does not fail and both previews are identical
  EXPECT_FALSE(mapped.failed());

  const auto p1 = blind.previewRoute(5.0);
  const auto p2 = mapped.previewRoute(5.0);
  ASSERT_EQ(p1.size(), p2.size());
  for (std::size_t i = 0; i < p1.size(); ++i) {
    EXPECT_NEAR(p1[i].first, p2[i].first, 1e-12);
    EXPECT_NEAR(p1[i].second, p2[i].second, 1e-12);
  }
}

TEST(ConstrainedPlannerTest, DirectLegDetoursAroundKeepOut) {
  // GIVEN: a keep-out astride the straight line from the start to the waypoint
  arlcore::autopilot::ZoneMap map = makeMap({keepOut(-80.0, 150.0, 80.0, 250.0)});
  arlcore::autopilot::DubinsPathPlanner planner;
  planner.setZones(&map);

  ConstrainedSimVehicle vehicle;  // at origin facing north
  // WHEN: the route is planned
  planner.plan({makeWaypoint(0.0, 400.0, 4.0)}, vehicle.pose(), testParams());
  ASSERT_FALSE(planner.failed());

  // THEN: the planned route respects the margin end to end...
  EXPECT_GE(previewMinClearance(planner.previewRoute(2.0), map), testParams().zoneMarginM - 0.5);

  // WHEN: the vehicle flies the mission
  flt64_t minFlownClearance = 1e18;
  for (int32_t i = 0; i < 3000 && !planner.routeComplete() && !planner.failed(); ++i) {
    const arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    vehicle.step(cv, 0.5);
    minFlownClearance = std::min(minFlownClearance, map.clearanceM(at(vehicle.xE, vehicle.yN), 0.0));
  }
  // THEN: ... and the flown track never violates the zone
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_GT(minFlownClearance, 0.0);
}

TEST(ConstrainedPlannerTest, WaypointInsideKeepOutFailsRoute) {
  // GIVEN: a keep-out that contains the only waypoint
  arlcore::autopilot::ZoneMap map = makeMap({keepOut(-100.0, 300.0, 100.0, 500.0)});
  arlcore::autopilot::DubinsPathPlanner planner;
  planner.setZones(&map);

  ConstrainedSimVehicle vehicle;
  // WHEN: the route is planned
  planner.plan({makeWaypoint(0.0, 400.0, 4.0)}, vehicle.pose(), testParams());
  // THEN: the plan fails
  EXPECT_TRUE(planner.failed());
  EXPECT_TRUE(planner.progress().failed);
}

TEST(ConstrainedPlannerTest, MidRouteConstraintChangeReplansCurrentLeg) {
  // GIVEN: an unconstrained route partially flown
  arlcore::autopilot::ZoneMap map = makeMap({});
  arlcore::autopilot::DubinsPathPlanner planner;
  planner.setZones(&map);

  ConstrainedSimVehicle vehicle;
  planner.plan({makeWaypoint(0.0, 500.0, 4.0)}, vehicle.pose(), testParams());
  ASSERT_FALSE(planner.failed());

  // Fly a little, then drop a keep-out on the remaining straight.
  runMission(&planner, &vehicle, 100);
  ASSERT_FALSE(planner.routeComplete());
  // WHEN: a keep-out lands on the remaining straight and the planner is told constraints changed
  arlcore::autopilot::ConstraintSnapshot changed;
  changed.revision = 2;
  changed.zones.push_back(keepOut(-80.0, 250.0, 80.0, 350.0));
  map.ingest(changed);
  planner.onConstraintsChanged(vehicle.pose());
  ASSERT_FALSE(planner.failed());

  // THEN: the replanned remainder respects the new zone and the mission still completes
  flt64_t minFlownClearance = 1e18;
  for (int32_t i = 0; i < 4000 && !planner.routeComplete() && !planner.failed(); ++i) {
    const arlcore::autopilot::ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    vehicle.step(cv, 0.5);
    minFlownClearance = std::min(minFlownClearance, map.clearanceM(at(vehicle.xE, vehicle.yN), 0.0));
  }
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_GT(minFlownClearance, 0.0);
}

TEST(ConstrainedPlannerTest, MidRouteConstraintChangeFailsWhenTargetSwallowed) {
  // GIVEN: an unconstrained route partially flown
  arlcore::autopilot::ZoneMap map = makeMap({});
  arlcore::autopilot::DubinsPathPlanner planner;
  planner.setZones(&map);

  ConstrainedSimVehicle vehicle;
  planner.plan({makeWaypoint(0.0, 500.0, 4.0)}, vehicle.pose(), testParams());
  runMission(&planner, &vehicle, 50);
  ASSERT_FALSE(planner.failed());

  // WHEN: a new keep-out swallows the waypoint and the planner is told constraints changed
  arlcore::autopilot::ConstraintSnapshot changed;
  changed.revision = 2;
  changed.zones.push_back(keepOut(-100.0, 400.0, 100.0, 600.0));  // swallows the waypoint
  map.ingest(changed);
  planner.onConstraintsChanged(vehicle.pose());
  // THEN: the route fails
  EXPECT_TRUE(planner.failed());
}

TEST(ConstrainedPlannerTest, ReplanCurrentLegFromLivePose) {
  // GIVEN: a route planned around a keep-out and partially flown
  arlcore::autopilot::ZoneMap map = makeMap({keepOut(-80.0, 150.0, 80.0, 250.0)});
  arlcore::autopilot::DubinsPathPlanner planner;
  planner.setZones(&map);

  ConstrainedSimVehicle vehicle;
  planner.plan({makeWaypoint(0.0, 400.0, 4.0)}, vehicle.pose(), testParams());
  ASSERT_FALSE(planner.failed());
  runMission(&planner, &vehicle, 60);

  // WHEN: the vehicle is displaced and a budget-free replan is requested from the live pose
  // (the recovery handoff path)
  vehicle.xE += 30.0;
  EXPECT_TRUE(planner.replanCurrentLegFrom(vehicle.pose()));
  // THEN: the replan succeeds and the mission still completes
  EXPECT_FALSE(planner.failed());

  runMission(&planner, &vehicle, 4000);
  EXPECT_TRUE(planner.routeComplete());
}
