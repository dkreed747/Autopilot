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

#include "AngleMath.h"
#include "VectorZoneGuidance.h"

namespace arlcore::autopilot {

namespace {

constexpr double kTurnRadiusM = 20.0;
constexpr double kMarginM = 5.0;

VectorAvoidanceConfig testConfig() {
  VectorAvoidanceConfig c;
  c.minFollowS = 0.0;   // let the episode end as soon as geometry allows (test speed)
  c.exitClearTicks = 3;
  return c;
}

//! \brief Kinematic loop: fly the guided heading with rate-limited turning; returns the
//! minimum zone clearance seen.
struct BugSim {
  Vec2 pos{0.0, 0.0};
  double yawAz = 0.0;
  double speed = 3.0;
  double maxTurnRateRps = 0.3;
  double dtS = 0.25;

  double run(VectorZoneGuidance* guidance, const ZoneSet& zones, double commandedAz, int steps) {
    double minClearance = 1e18;
    for (int i = 0; i < steps; ++i) {
      const double heading = guidance->steer(commandedAz, pos, speed, zones);
      const double err = wrapPi(heading - yawAz);
      yawAz = wrapPi(yawAz + std::clamp(err, -maxTurnRateRps * dtS, maxTurnRateRps * dtS));
      pos.x += speed * dtS * std::sin(yawAz);
      pos.y += speed * dtS * std::cos(yawAz);
      minClearance = std::min(minClearance, zones.clearanceM(pos));
    }
    return minClearance;
  }
};

}  // namespace

TEST(VectorZoneGuidanceTest, PassesThroughWhenClear) {
  VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{500.0, -50.0}, {600.0, -50.0},
                                                  {600.0, 50.0}, {500.0, 50.0}}));
  // Heading north, zone far east: untouched.
  EXPECT_DOUBLE_EQ(guidance.steer(0.0, {0.0, 0.0}, 3.0, zones), 0.0);
  EXPECT_FALSE(guidance.avoidanceActive());

  ZoneSet empty;
  EXPECT_DOUBLE_EQ(guidance.steer(1.0, {0.0, 0.0}, 3.0, empty), 1.0);
}

TEST(VectorZoneGuidanceTest, EntersBoundaryFollowWhenBlocked) {
  VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{-100.0, 30.0}, {100.0, 30.0},
                                                  {100.0, 130.0}, {-100.0, 130.0}}));
  // Heading due north straight at the wall 30 m ahead (< lookahead).
  const double heading = guidance.steer(0.0, {0.0, 0.0}, 3.0, zones);
  EXPECT_TRUE(guidance.avoidanceActive());
  EXPECT_GT(std::fabs(wrapPi(heading - 0.0)), 0.5);  // deflected well away from due north
}

TEST(VectorZoneGuidanceTest, NeverEntersKeepOutWhileCommandedInto) {
  VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{-150.0, 60.0}, {150.0, 60.0},
                                                  {150.0, 260.0}, {-150.0, 260.0}}));
  BugSim sim;
  // Commanded due north, straight into a 300 m wide box: the bug must deflect and circulate,
  // never crossing the boundary.
  const double minClearance = sim.run(&guidance, zones, 0.0, 1200);
  EXPECT_GT(minClearance, 0.0);
  // ... and it must actually have travelled somewhere (following the wall), not parked.
  EXPECT_GT(std::hypot(sim.pos.x, sim.pos.y), 100.0);
}

TEST(VectorZoneGuidanceTest, KeepInCirculation) {
  VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  ZoneSet zones;
  zones.addZone(ZoneKind::KEEP_IN, LocalPolygon({{-150.0, -150.0}, {150.0, -150.0},
                                                 {150.0, 150.0}, {-150.0, 150.0}}));
  BugSim sim;  // starts at the center
  // Commanded due east forever: the vehicle reaches the east wall and circulates the keep-in
  // boundary instead of exiting.
  const double minClearance = sim.run(&guidance, zones, M_PI / 2.0, 2000);
  EXPECT_GT(minClearance, 0.0);  // never left the keep-in
  EXPECT_LT(std::fabs(sim.pos.x), 150.0);
  EXPECT_LT(std::fabs(sim.pos.y), 150.0);
}

TEST(VectorZoneGuidanceTest, ResumesCommandedHeadingPastObstacle) {
  VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  ZoneSet zones;
  // A modest box offset to the east of the northbound track.
  zones.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{-40.0, 60.0}, {40.0, 60.0},
                                                  {40.0, 140.0}, {-40.0, 140.0}}));
  BugSim sim;
  const double minClearance = sim.run(&guidance, zones, 0.0, 1500);
  EXPECT_GT(minClearance, 0.0);
  // Far past the obstacle and back on the commanded (northbound) course.
  EXPECT_GT(sim.pos.y, 200.0);
  EXPECT_FALSE(guidance.avoidanceActive());
}

}  // namespace arlcore::autopilot
