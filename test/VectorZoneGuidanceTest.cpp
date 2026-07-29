#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/safety/VectorZoneGuidance.hpp"
#include "InternalTypes.h"

constexpr flt64_t kTurnRadiusM = 20.0;
constexpr flt64_t kMarginM = 5.0;

static arlcore::autopilot::VectorAvoidanceConfig testConfig() {
  arlcore::autopilot::VectorAvoidanceConfig c;
  c.minFollowS = 0.0;   // let the episode end as soon as geometry allows (test speed)
  c.exitClearTicks = 3;
  return c;
}

//! \brief Kinematic loop: fly the guided heading with rate-limited turning; returns the
//! minimum zone clearance seen.
struct BugSim {
  arlcore::autopilot::Vec2 pos{0.0, 0.0};
  flt64_t yawAz = 0.0;
  flt64_t speed = 3.0;
  flt64_t maxTurnRateRps = 0.3;
  flt64_t dtS = 0.25;

  flt64_t run(arlcore::autopilot::VectorZoneGuidance* guidance, const arlcore::autopilot::ZoneSet& zones,
              flt64_t commandedAz, int32_t steps) {
    flt64_t minClearance = 1e18;
    for (int32_t i = 0; i < steps; ++i) {
      const flt64_t heading = guidance->steer(commandedAz, pos, speed, zones);
      const flt64_t err = arlcore::autopilot::wrapPi(heading - yawAz);
      yawAz = arlcore::autopilot::wrapPi(yawAz + std::clamp(err, -maxTurnRateRps * dtS, maxTurnRateRps * dtS));
      pos.x += speed * dtS * std::sin(yawAz);
      pos.y += speed * dtS * std::cos(yawAz);
      minClearance = std::min(minClearance, zones.clearanceM(pos));
    }
    return minClearance;
  }
};

TEST(VectorZoneGuidanceTest, PassesThroughWhenClear) {
  // GIVEN: guidance and a keep-out far to the east of a northbound track
  arlcore::autopilot::VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{500.0, -50.0}, {600.0, -50.0},
                                                  {600.0, 50.0}, {500.0, 50.0}}));
  // WHEN: steering due north well clear of the zone
  // THEN: the commanded heading passes through untouched and avoidance stays off
  EXPECT_DOUBLE_EQ(guidance.steer(0.0, {0.0, 0.0}, 3.0, zones), 0.0);
  EXPECT_FALSE(guidance.avoidanceActive());

  // WHEN: steering against an empty zone set
  arlcore::autopilot::ZoneSet empty;
  // THEN: the commanded heading passes through untouched
  EXPECT_DOUBLE_EQ(guidance.steer(1.0, {0.0, 0.0}, 3.0, empty), 1.0);
}

TEST(VectorZoneGuidanceTest, EntersBoundaryFollowWhenBlocked) {
  // GIVEN: a keep-out wall 30 m ahead (< lookahead) of a northbound vehicle
  arlcore::autopilot::VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{-100.0, 30.0}, {100.0, 30.0},
                                                  {100.0, 130.0}, {-100.0, 130.0}}));
  // WHEN: steering due north straight at the wall
  const flt64_t heading = guidance.steer(0.0, {0.0, 0.0}, 3.0, zones);
  // THEN: avoidance engages and deflects well away from due north
  EXPECT_TRUE(guidance.avoidanceActive());
  EXPECT_GT(std::fabs(arlcore::autopilot::wrapPi(heading - 0.0)), 0.5);
}

TEST(VectorZoneGuidanceTest, NeverEntersKeepOutWhileCommandedInto) {
  // GIVEN: a 300 m wide keep-out box straight north of the bug simulator
  arlcore::autopilot::VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{-150.0, 60.0}, {150.0, 60.0},
                                                  {150.0, 260.0}, {-150.0, 260.0}}));
  BugSim sim;
  // WHEN: commanded due north straight into the box for 1200 steps
  const flt64_t minClearance = sim.run(&guidance, zones, 0.0, 1200);
  // THEN: the bug deflects and circulates, never crossing the boundary, and actually
  // travels along the wall rather than parking
  EXPECT_GT(minClearance, 0.0);
  EXPECT_GT(std::hypot(sim.pos.x, sim.pos.y), 100.0);
}

TEST(VectorZoneGuidanceTest, KeepInCirculation) {
  // GIVEN: a keep-in centered on the bug simulator's start
  arlcore::autopilot::VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_IN,
                arlcore::autopilot::LocalPolygon({{-150.0, -150.0}, {150.0, -150.0},
                                                  {150.0, 150.0}, {-150.0, 150.0}}));
  BugSim sim;  // starts at the center
  // WHEN: commanded due east forever (2000 steps)
  const flt64_t minClearance = sim.run(&guidance, zones, M_PI / 2.0, 2000);
  // THEN: the vehicle circulates the keep-in boundary instead of exiting
  EXPECT_GT(minClearance, 0.0);  // never left the keep-in
  EXPECT_LT(std::fabs(sim.pos.x), 150.0);
  EXPECT_LT(std::fabs(sim.pos.y), 150.0);
}

TEST(VectorZoneGuidanceTest, ResumesCommandedHeadingPastObstacle) {
  // GIVEN: a modest keep-out box offset to the east of the northbound track
  arlcore::autopilot::VectorZoneGuidance guidance(testConfig(), kTurnRadiusM, kMarginM);
  arlcore::autopilot::ZoneSet zones;
  zones.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
                arlcore::autopilot::LocalPolygon({{-40.0, 60.0}, {40.0, 60.0},
                                                  {40.0, 140.0}, {-40.0, 140.0}}));
  BugSim sim;
  // WHEN: commanded due north for 1500 steps
  const flt64_t minClearance = sim.run(&guidance, zones, 0.0, 1500);
  // THEN: it clears the obstacle, ends up far past it back on the commanded course,
  // and avoidance has disengaged
  EXPECT_GT(minClearance, 0.0);
  EXPECT_GT(sim.pos.y, 200.0);
  EXPECT_FALSE(guidance.avoidanceActive());
}
