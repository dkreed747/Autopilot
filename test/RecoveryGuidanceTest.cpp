#include <gtest/gtest.h>

#include <cmath>

#include <GeographicLib/LocalCartesian.hpp>

#include "autopilot/safety/RecoveryGuidance.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

constexpr flt64_t kLat = 39.0;
constexpr flt64_t kLon = -76.5;

static GeoPoint at(flt64_t east, flt64_t north) {
  static const GeographicLib::LocalCartesian frame(kLat, kLon, 0.0);
  GeoPoint p;
  flt64_t h = 0.0;
  frame.Reverse(east, north, 0.0, p.latDeg, p.lonDeg, h);
  return p;
}

//! A [0,200]x[0,200] zone in the test frame.
static ZoneRecord squareZone(ZoneKind kind) {
  ZoneRecord zone;
  zone.kind = kind;
  ZoneShape shape;
  shape.polygon = {at(0.0, 0.0), at(200.0, 0.0), at(200.0, 200.0), at(0.0, 200.0)};
  zone.shapes.push_back(shape);
  return zone;
}

static ZoneMap mapWith(ZoneKind kind) {
  ZoneMap map(ZonesConfig{});
  ConstraintSnapshot snapshot;
  snapshot.revision = 1;
  snapshot.zones.push_back(squareZone(kind));
  map.ingest(snapshot);
  return map;
}

static RecoveryConfig instantConfig() {
  RecoveryConfig c;
  c.speedMps = 1.5;
  c.completeHoldS = 0.0;  // complete as soon as COMPLIANT (test speed)
  return c;
}

TEST(RecoveryGuidanceTest, DrivesOutOfKeepOut) {
  const ZoneMap map = mapWith(ZoneKind::KEEP_OUT);
  RecoveryGuidance recovery(instantConfig(), 3.0, 5.0);

  // Vehicle 20 m inside the west edge of the keep-out.
  const GeoPoint inside = at(20.0, 100.0);
  ASSERT_TRUE(recovery.begin(inside, 0.0, std::nullopt, map));
  EXPECT_TRUE(recovery.active());

  const auto cv = recovery.tick(inside, 0.0, std::nullopt, map);
  ASSERT_TRUE(cv.has_value());
  EXPECT_DOUBLE_EQ(cv->speedMps, 1.5);
  // Nearest exit is due west (azimuth -pi/2).
  EXPECT_NEAR(cv->headingRad, -M_PI / 2.0, 0.3);
  EXPECT_FALSE(cv->elevationM.has_value());  // elevation held

  EXPECT_FALSE(recovery.complete(inside, 0.0, std::nullopt, map));
  // Once clear of the zone by more than the hysteresis, recovery completes.
  EXPECT_TRUE(recovery.complete(at(-10.0, 100.0), 0.0, std::nullopt, map));
}

TEST(RecoveryGuidanceTest, DrivesBackIntoKeepIn) {
  const ZoneMap map = mapWith(ZoneKind::KEEP_IN);
  RecoveryGuidance recovery(instantConfig(), 3.0, 5.0);

  // Vehicle 30 m west of the keep-in.
  const GeoPoint outside = at(-30.0, 100.0);
  ASSERT_TRUE(recovery.begin(outside, 0.0, std::nullopt, map));
  const auto cv = recovery.tick(outside, 0.0, std::nullopt, map);
  ASSERT_TRUE(cv.has_value());
  // Back inside is due east.
  EXPECT_NEAR(cv->headingRad, M_PI / 2.0, 0.3);

  EXPECT_FALSE(recovery.complete(outside, 0.0, std::nullopt, map));
  EXPECT_TRUE(recovery.complete(at(50.0, 100.0), 0.0, std::nullopt, map));
}

TEST(RecoveryGuidanceTest, UsesCruiseSpeedWhenUnconfigured) {
  const ZoneMap map = mapWith(ZoneKind::KEEP_OUT);
  RecoveryConfig config = instantConfig();
  config.speedMps = 0.0;  // 0 = platform cruising speed
  RecoveryGuidance recovery(config, 3.0, 5.0);

  const GeoPoint inside = at(20.0, 100.0);
  ASSERT_TRUE(recovery.begin(inside, 0.0, std::nullopt, map));
  const auto cv = recovery.tick(inside, 0.0, std::nullopt, map);
  ASSERT_TRUE(cv.has_value());
  EXPECT_DOUBLE_EQ(cv->speedMps, 3.0);
}

TEST(RecoveryGuidanceTest, NoZonesMeansNoRecoveryTarget) {
  ZoneMap map(ZonesConfig{});
  RecoveryGuidance recovery(instantConfig(), 3.0, 5.0);
  EXPECT_FALSE(recovery.begin(at(0.0, 0.0), 0.0, std::nullopt, map));
  EXPECT_FALSE(recovery.active());
  EXPECT_FALSE(recovery.tick(at(0.0, 0.0), 0.0, std::nullopt, map).has_value());
}

}  // namespace arlcore::autopilot
