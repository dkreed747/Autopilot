//---------------------------------------------------------------------------
// Copyright 2026 Pennsylvania State University
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
#include <optional>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "AngleMath.h"
#include "DubinsPathPlanner.h"
#include "ZoneMap.h"

namespace arlcore::autopilot {

namespace {

using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

constexpr double kOriginLat = 39.0;
constexpr double kOriginLon = -76.5;

const GeographicLib::LocalCartesian& testFrame() {
  static const GeographicLib::LocalCartesian frame(kOriginLat, kOriginLon, 0.0);
  return frame;
}

GeoPoint at(double east, double north) {
  GeoPoint p;
  double h = 0.0;
  testFrame().Reverse(east, north, 0.0, p.latDeg, p.lonDeg, h);
  return p;
}

//! \brief Same kinematic vehicle as DubinsPathPlannerTest.
struct SimVehicle {
  double xE = 0.0;
  double yN = 0.0;
  double yawRad = 0.0;
  double speedMps = 0.0;
  double maxTurnRateRps = 0.25;

  GlobalPoseReportType pose() const {
    GlobalPoseReportType p;
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    testFrame().Reverse(xE, yN, 0.0, lat, lon, h);
    p.position().geodeticLatitude(lat);
    p.position().geodeticLongitude(lon);
    p.attitude().yaw().yaw(yawRad);
    return p;
  }

  void step(const ControlVector& cv, double dtS) {
    const double err = wrapPi(cv.headingRad - yawRad);
    const double maxDelta = maxTurnRateRps * dtS;
    yawRad = wrapPi(yawRad + std::clamp(err, -maxDelta, maxDelta));
    speedMps = cv.speedMps;
    xE += speedMps * dtS * std::sin(yawRad);
    yN += speedMps * dtS * std::cos(yawRad);
  }
};

GlobalWaypointType makeWaypoint(double xE, double yN, double speedMps) {
  const GeoPoint p = at(xE, yN);
  GlobalWaypointType wp;
  wp.position().value().geodeticLatitude(p.latDeg);
  wp.position().value().geodeticLongitude(p.lonDeg);
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant(
      UMAA::Common::Speed::RequiredSpeedVariantType());
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant().speed()
      .SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant(
          UMAA::Common::Speed::GroundSpeedRequirementVariantType());
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant().speed()
      .SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant().speed()
      .speed(speedMps);
  return wp;
}

PlannerParams testParams() {
  PlannerParams p;
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
ZoneRecord keepOut(double x0, double y0, double x1, double y1) {
  ZoneRecord zone;
  zone.kind = ZoneKind::KEEP_OUT;
  ZoneShape shape;
  shape.polygon = {at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)};
  zone.shapes.push_back(shape);
  return zone;
}

ZoneMap makeMap(std::vector<ZoneRecord> zones, uint64_t revision = 1) {
  ZoneMap map(ZonesConfig{});
  ConstraintSnapshot snapshot;
  snapshot.revision = revision;
  snapshot.zones = std::move(zones);
  map.ingest(snapshot);
  return map;
}

//! \brief Minimum keep-out clearance over a geodetic preview polyline.
double previewMinClearance(const std::vector<std::pair<double, double>>& preview, const ZoneMap& map) {
  double minClearance = 1e18;
  for (const auto& [lat, lon] : preview) {
    minClearance = std::min(minClearance, map.clearanceM(GeoPoint{lat, lon}, 0.0));
  }
  return minClearance;
}

void runMission(DubinsPathPlanner* planner, SimVehicle* vehicle, int maxSteps, double dtS = 0.5) {
  for (int i = 0; i < maxSteps && !planner->routeComplete() && !planner->failed(); i++) {
    const ControlVector cv = planner->update(vehicle->pose(), vehicle->speedMps);
    vehicle->step(cv, dtS);
  }
}

}  // namespace

TEST(ConstrainedPlannerTest, NoZonesMatchesUnconstrainedBehavior) {
  // Regression: a planner with an empty zone map plans/flies exactly like a zone-blind one.
  DubinsPathPlanner blind;
  DubinsPathPlanner mapped;
  ZoneMap empty = makeMap({});
  mapped.setZones(&empty);

  SimVehicle vehicle;
  const std::vector<GlobalWaypointType> route = {makeWaypoint(0.0, 300.0, 4.0),
                                                 makeWaypoint(250.0, 500.0, 4.0)};
  blind.plan(route, vehicle.pose(), testParams());
  mapped.plan(route, vehicle.pose(), testParams());
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
  // Keep-out astride the straight line from the start to the waypoint.
  ZoneMap map = makeMap({keepOut(-80.0, 150.0, 80.0, 250.0)});
  DubinsPathPlanner planner;
  planner.setZones(&map);

  SimVehicle vehicle;  // at origin facing north
  planner.plan({makeWaypoint(0.0, 400.0, 4.0)}, vehicle.pose(), testParams());
  ASSERT_FALSE(planner.failed());

  // The planned route respects the margin end to end...
  EXPECT_GE(previewMinClearance(planner.previewRoute(2.0), map), testParams().zoneMarginM - 0.5);

  // ... and the flown track never violates the zone.
  double minFlownClearance = 1e18;
  for (int i = 0; i < 3000 && !planner.routeComplete() && !planner.failed(); ++i) {
    const ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    vehicle.step(cv, 0.5);
    minFlownClearance = std::min(minFlownClearance,
                                 map.clearanceM(at(vehicle.xE, vehicle.yN), 0.0));
  }
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_GT(minFlownClearance, 0.0);
}

TEST(ConstrainedPlannerTest, WaypointInsideKeepOutFailsRoute) {
  ZoneMap map = makeMap({keepOut(-100.0, 300.0, 100.0, 500.0)});
  DubinsPathPlanner planner;
  planner.setZones(&map);

  SimVehicle vehicle;
  planner.plan({makeWaypoint(0.0, 400.0, 4.0)}, vehicle.pose(), testParams());
  EXPECT_TRUE(planner.failed());
  EXPECT_TRUE(planner.progress().failed);
}

TEST(ConstrainedPlannerTest, MidRouteConstraintChangeReplansCurrentLeg) {
  ZoneMap map = makeMap({});
  DubinsPathPlanner planner;
  planner.setZones(&map);

  SimVehicle vehicle;
  planner.plan({makeWaypoint(0.0, 500.0, 4.0)}, vehicle.pose(), testParams());
  ASSERT_FALSE(planner.failed());

  // Fly a little, then drop a keep-out on the remaining straight.
  runMission(&planner, &vehicle, 100);
  ASSERT_FALSE(planner.routeComplete());
  ConstraintSnapshot changed;
  changed.revision = 2;
  changed.zones.push_back(keepOut(-80.0, 250.0, 80.0, 350.0));
  map.ingest(changed);
  planner.onConstraintsChanged(vehicle.pose());
  ASSERT_FALSE(planner.failed());

  // The replanned remainder respects the new zone and the mission still completes.
  double minFlownClearance = 1e18;
  for (int i = 0; i < 4000 && !planner.routeComplete() && !planner.failed(); ++i) {
    const ControlVector cv = planner.update(vehicle.pose(), vehicle.speedMps);
    vehicle.step(cv, 0.5);
    minFlownClearance = std::min(minFlownClearance,
                                 map.clearanceM(at(vehicle.xE, vehicle.yN), 0.0));
  }
  EXPECT_TRUE(planner.routeComplete());
  EXPECT_GT(minFlownClearance, 0.0);
}

TEST(ConstrainedPlannerTest, MidRouteConstraintChangeFailsWhenTargetSwallowed) {
  ZoneMap map = makeMap({});
  DubinsPathPlanner planner;
  planner.setZones(&map);

  SimVehicle vehicle;
  planner.plan({makeWaypoint(0.0, 500.0, 4.0)}, vehicle.pose(), testParams());
  runMission(&planner, &vehicle, 50);
  ASSERT_FALSE(planner.failed());

  ConstraintSnapshot changed;
  changed.revision = 2;
  changed.zones.push_back(keepOut(-100.0, 400.0, 100.0, 600.0));  // swallows the waypoint
  map.ingest(changed);
  planner.onConstraintsChanged(vehicle.pose());
  EXPECT_TRUE(planner.failed());
}

TEST(ConstrainedPlannerTest, ReplanCurrentLegFromLivePose) {
  ZoneMap map = makeMap({keepOut(-80.0, 150.0, 80.0, 250.0)});
  DubinsPathPlanner planner;
  planner.setZones(&map);

  SimVehicle vehicle;
  planner.plan({makeWaypoint(0.0, 400.0, 4.0)}, vehicle.pose(), testParams());
  ASSERT_FALSE(planner.failed());
  runMission(&planner, &vehicle, 60);

  // A budget-free replan from wherever the vehicle is (the recovery handoff path).
  vehicle.xE += 30.0;
  EXPECT_TRUE(planner.replanCurrentLegFrom(vehicle.pose()));
  EXPECT_FALSE(planner.failed());

  runMission(&planner, &vehicle, 4000);
  EXPECT_TRUE(planner.routeComplete());
}

}  // namespace arlcore::autopilot
