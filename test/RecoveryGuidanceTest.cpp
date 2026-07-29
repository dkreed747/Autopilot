#include <gtest/gtest.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>

#include "InternalTypes.h"
#include "autopilot/safety/RecoveryGuidance.hpp"

constexpr flt64_t kLat = 39.0;
constexpr flt64_t kLon = -76.5;

static arlcore::autopilot::GeoPoint at(flt64_t east, flt64_t north) {
  static const GeographicLib::LocalCartesian frame(kLat, kLon, 0.0);
  arlcore::autopilot::GeoPoint p;
  flt64_t h = 0.0;
  frame.Reverse(east, north, 0.0, p.latDeg, p.lonDeg, h);
  return p;
}

//! A [0,200]x[0,200] zone in the test frame.
static arlcore::autopilot::ZoneRecord squareZone(arlcore::autopilot::ZoneKind kind) {
  arlcore::autopilot::ZoneRecord zone;
  zone.kind = kind;
  arlcore::autopilot::ZoneShape shape;
  shape.polygon = {at(0.0, 0.0), at(200.0, 0.0), at(200.0, 200.0), at(0.0, 200.0)};
  zone.shapes.push_back(shape);
  return zone;
}

static arlcore::autopilot::ZoneMap mapWith(arlcore::autopilot::ZoneKind kind) {
  arlcore::autopilot::ZoneMap map(arlcore::autopilot::ZonesConfig{});
  arlcore::autopilot::ConstraintSnapshot snapshot;
  snapshot.revision = 1;
  snapshot.zones.push_back(squareZone(kind));
  map.ingest(snapshot);
  return map;
}

static arlcore::autopilot::RecoveryConfig instantConfig() {
  arlcore::autopilot::RecoveryConfig c;
  c.speedMps = 1.5;
  c.completeHoldS = 0.0;  // complete as soon as COMPLIANT (test speed)
  return c;
}

TEST(RecoveryGuidanceTest, DrivesOutOfKeepOut) {
  // GIVEN: a keep-out map and a vehicle 20 m inside its west edge
  const arlcore::autopilot::ZoneMap map = mapWith(arlcore::autopilot::ZoneKind::KEEP_OUT);
  arlcore::autopilot::RecoveryGuidance recovery(instantConfig(), 3.0, 5.0);
  const arlcore::autopilot::GeoPoint inside = at(20.0, 100.0);

  // WHEN: recovery begins and produces its first tick
  ASSERT_TRUE(recovery.begin(inside, 0.0, std::nullopt, map));
  EXPECT_TRUE(recovery.active());
  const auto cv = recovery.tick(inside, 0.0, std::nullopt, map);

  // THEN: it commands the configured speed toward the nearest exit due west, elevation held
  ASSERT_TRUE(cv.has_value());
  EXPECT_DOUBLE_EQ(cv->speedMps, 1.5);
  EXPECT_NEAR(cv->headingRad, -M_PI / 2.0, 0.3);  // nearest exit is due west
  EXPECT_FALSE(cv->elevationM.has_value());       // elevation held

  // WHEN: completion is polled inside and then clear of the zone beyond the hysteresis
  // THEN: recovery completes only once clear
  EXPECT_FALSE(recovery.complete(inside, 0.0, std::nullopt, map));
  EXPECT_TRUE(recovery.complete(at(-10.0, 100.0), 0.0, std::nullopt, map));
}

TEST(RecoveryGuidanceTest, DrivesBackIntoKeepIn) {
  // GIVEN: a keep-in map and a vehicle 30 m west of it
  const arlcore::autopilot::ZoneMap map = mapWith(arlcore::autopilot::ZoneKind::KEEP_IN);
  arlcore::autopilot::RecoveryGuidance recovery(instantConfig(), 3.0, 5.0);
  const arlcore::autopilot::GeoPoint outside = at(-30.0, 100.0);

  // WHEN: recovery begins and produces its first tick
  ASSERT_TRUE(recovery.begin(outside, 0.0, std::nullopt, map));
  const auto cv = recovery.tick(outside, 0.0, std::nullopt, map);

  // THEN: it commands due east, back inside the keep-in
  ASSERT_TRUE(cv.has_value());
  EXPECT_NEAR(cv->headingRad, M_PI / 2.0, 0.3);

  // WHEN: completion is polled outside and then well inside the keep-in
  // THEN: recovery completes only once back in
  EXPECT_FALSE(recovery.complete(outside, 0.0, std::nullopt, map));
  EXPECT_TRUE(recovery.complete(at(50.0, 100.0), 0.0, std::nullopt, map));
}

TEST(RecoveryGuidanceTest, UsesCruiseSpeedWhenUnconfigured) {
  // GIVEN: a recovery speed of 0 (platform cruising speed) and a vehicle inside a keep-out
  const arlcore::autopilot::ZoneMap map = mapWith(arlcore::autopilot::ZoneKind::KEEP_OUT);
  arlcore::autopilot::RecoveryConfig config = instantConfig();
  config.speedMps = 0.0;  // 0 = platform cruising speed
  arlcore::autopilot::RecoveryGuidance recovery(config, 3.0, 5.0);
  const arlcore::autopilot::GeoPoint inside = at(20.0, 100.0);

  // WHEN: recovery begins and produces its first tick
  ASSERT_TRUE(recovery.begin(inside, 0.0, std::nullopt, map));
  const auto cv = recovery.tick(inside, 0.0, std::nullopt, map);

  // THEN: the commanded speed is the platform cruising speed
  ASSERT_TRUE(cv.has_value());
  EXPECT_DOUBLE_EQ(cv->speedMps, 3.0);
}

TEST(RecoveryGuidanceTest, NoZonesMeansNoRecoveryTarget) {
  // GIVEN: a zone map with no zones
  arlcore::autopilot::ZoneMap map(arlcore::autopilot::ZonesConfig{});
  arlcore::autopilot::RecoveryGuidance recovery(instantConfig(), 3.0, 5.0);
  // WHEN: recovery is asked to begin
  // THEN: it refuses, stays inactive, and produces no guidance
  EXPECT_FALSE(recovery.begin(at(0.0, 0.0), 0.0, std::nullopt, map));
  EXPECT_FALSE(recovery.active());
  EXPECT_FALSE(recovery.tick(at(0.0, 0.0), 0.0, std::nullopt, map).has_value());
}
