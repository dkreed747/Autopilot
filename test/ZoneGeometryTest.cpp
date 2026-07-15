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

#include <cmath>

#include "ZoneGeometry.h"

namespace arlcore::autopilot {

namespace {

//! An axis-aligned square [0,100]x[0,100], given clockwise to exercise re-winding.
LocalPolygon square100() {
  return LocalPolygon({{0.0, 0.0}, {0.0, 100.0}, {100.0, 100.0}, {100.0, 0.0}});
}

}  // namespace

TEST(LocalPolygonTest, ContainsAndSignedDistance) {
  const LocalPolygon poly = square100();

  EXPECT_TRUE(poly.contains({50.0, 50.0}));
  EXPECT_FALSE(poly.contains({150.0, 50.0}));
  EXPECT_FALSE(poly.contains({-1.0, 50.0}));
  EXPECT_TRUE(poly.contains({0.0, 50.0}));  // boundary counts as inside

  EXPECT_NEAR(poly.signedDistance({50.0, 50.0}), 50.0, 1e-9);
  EXPECT_NEAR(poly.signedDistance({10.0, 50.0}), 10.0, 1e-9);
  EXPECT_NEAR(poly.signedDistance({-20.0, 50.0}), -20.0, 1e-9);
  EXPECT_NEAR(poly.signedDistance({0.0, 50.0}), 0.0, 1e-9);
  // Outside a corner: distance to the vertex.
  EXPECT_NEAR(poly.signedDistance({103.0, 104.0}), -5.0, 1e-9);
}

TEST(LocalPolygonTest, ClosestBoundaryPoint) {
  const LocalPolygon poly = square100();
  const Vec2 c = poly.closestBoundaryPoint({50.0, 120.0});
  EXPECT_NEAR(c.x, 50.0, 1e-9);
  EXPECT_NEAR(c.y, 100.0, 1e-9);
}

TEST(LocalPolygonTest, NonConvexContains) {
  // L-shape: the notch [50,100]x[50,100] is outside.
  const LocalPolygon poly(
      {{0.0, 0.0}, {100.0, 0.0}, {100.0, 50.0}, {50.0, 50.0}, {50.0, 100.0}, {0.0, 100.0}});
  EXPECT_TRUE(poly.contains({25.0, 75.0}));
  EXPECT_TRUE(poly.contains({75.0, 25.0}));
  EXPECT_FALSE(poly.contains({75.0, 75.0}));
  EXPECT_NEAR(poly.signedDistance({75.0, 75.0}), -25.0, 1e-9);
}

TEST(ZoneSetTest, KeepOutClearance) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_OUT, square100());

  EXPECT_NEAR(set.clearanceM({-20.0, 50.0}), 20.0, 1e-9);   // outside = compliant
  EXPECT_NEAR(set.clearanceM({10.0, 50.0}), -10.0, 1e-9);   // inside = violating
  EXPECT_TRUE(set.pointCompliant({-20.0, 50.0}, 5.0));
  EXPECT_FALSE(set.pointCompliant({-3.0, 50.0}, 5.0));      // compliant but under margin
}

TEST(ZoneSetTest, KeepInClearance) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_IN, square100());

  EXPECT_NEAR(set.clearanceM({50.0, 50.0}), 50.0, 1e-9);    // inside = compliant
  EXPECT_NEAR(set.clearanceM({-20.0, 50.0}), -20.0, 1e-9);  // outside = violating
}

TEST(ZoneSetTest, ClearanceIsMinOverZones) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_IN, LocalPolygon({{-200.0, -200.0}, {200.0, -200.0},
                                               {200.0, 200.0}, {-200.0, 200.0}}));
  set.addZone(ZoneKind::KEEP_OUT, square100());

  // Inside the keep-in (clearance 150 to its boundary) but also inside the keep-out.
  EXPECT_NEAR(set.clearanceM({50.0, 50.0}), -50.0, 1e-9);
  // Compliant w.r.t. both; the keep-out is binding.
  EXPECT_NEAR(set.clearanceM({-30.0, 50.0}), 30.0, 1e-9);
}

TEST(ZoneSetTest, ClearanceInfoDirections) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_OUT, square100());

  // Compliant point west of the zone: clearance grows to the west.
  ClearanceInfo info = set.clearanceInfo({-10.0, 50.0});
  EXPECT_NEAR(info.clearanceM, 10.0, 1e-9);
  EXPECT_NEAR(info.improveDir.x, -1.0, 1e-9);
  EXPECT_NEAR(info.improveDir.y, 0.0, 1e-9);

  // Violating point just inside the west edge: improvement is west, through the boundary.
  info = set.clearanceInfo({10.0, 50.0});
  EXPECT_NEAR(info.clearanceM, -10.0, 1e-9);
  EXPECT_NEAR(info.improveDir.x, -1.0, 1e-9);
  EXPECT_NEAR(info.improveDir.y, 0.0, 1e-9);
}

TEST(ZoneSetTest, SegmentClear) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_OUT, square100());

  EXPECT_TRUE(set.segmentClear({-50.0, -50.0}, {-50.0, 150.0}, 5.0, 1.0));   // passes west of it
  EXPECT_FALSE(set.segmentClear({-50.0, 50.0}, {150.0, 50.0}, 5.0, 1.0));    // straight through
  EXPECT_FALSE(set.segmentClear({-4.0, -50.0}, {-4.0, 150.0}, 5.0, 1.0));    // clear but < margin
  EXPECT_TRUE(set.segmentClear({-50.0, -50.0}, {-50.0, 150.0}, 5.0, 1000.0));  // endpoint-only fallback
}

TEST(ZoneSetTest, PathClear) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_OUT, square100());

  // A straight Dubins "path" west of the zone is clear; one through it is not.
  auto clearPath = DubinsPath::solve({-50.0, -50.0, M_PI / 2.0}, {-50.0, 150.0, M_PI / 2.0}, 10.0);
  ASSERT_TRUE(clearPath.has_value());
  EXPECT_TRUE(set.pathClear(clearPath.value(), 5.0, 1.0));

  auto blockedPath = DubinsPath::solve({-50.0, 50.0, 0.0}, {150.0, 50.0, 0.0}, 10.0);
  ASSERT_TRUE(blockedPath.has_value());
  EXPECT_FALSE(set.pathClear(blockedPath.value(), 5.0, 1.0));
}

TEST(ZoneSetTest, RaycastFirstHit) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_OUT, square100());

  // Ray east from (-50, 50): margin-5 clearance is crossed 5 m before the west edge.
  auto hit = set.raycastFirstHit({-50.0, 50.0}, {1.0, 0.0}, 5.0, 200.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit.value(), 45.0, 0.2);

  // Ray north misses the zone entirely.
  EXPECT_FALSE(set.raycastFirstHit({-50.0, 50.0}, {0.0, 1.0}, 5.0, 200.0).has_value());

  // Starting below margin reports an immediate hit.
  hit = set.raycastFirstHit({-2.0, 50.0}, {0.0, 1.0}, 5.0, 200.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit.value(), 0.0, 1e-9);
}

TEST(ZoneSetTest, NearestCompliantPointFromInsideKeepOut) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_OUT, square100());

  const auto q = set.nearestCompliantPoint({10.0, 50.0}, 5.0);
  ASSERT_TRUE(q.has_value());
  EXPECT_GE(set.clearanceM(q.value()), 5.0 - 1e-6);
  // Nearest exit is through the west edge.
  EXPECT_LT(q->x, -4.9);
  EXPECT_NEAR(q->y, 50.0, 1.0);
}

TEST(ZoneSetTest, NearestCompliantPointOutsideKeepIn) {
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_IN, square100());

  const auto q = set.nearestCompliantPoint({-30.0, 50.0}, 5.0);
  ASSERT_TRUE(q.has_value());
  EXPECT_GE(set.clearanceM(q.value()), 5.0 - 1e-6);
  EXPECT_GT(q->x, 4.9);
}

TEST(ZoneSetTest, NearestCompliantPointMultiZoneCorner) {
  // Two overlapping keep-outs forming a wedge; projection may ping-pong, ring search must save it.
  ZoneSet set;
  set.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{0.0, 0.0}, {100.0, 0.0}, {100.0, 60.0}, {0.0, 60.0}}));
  set.addZone(ZoneKind::KEEP_OUT, LocalPolygon({{0.0, 40.0}, {100.0, 40.0}, {100.0, 100.0}, {0.0, 100.0}}));

  const auto q = set.nearestCompliantPoint({50.0, 50.0}, 5.0);
  ASSERT_TRUE(q.has_value());
  EXPECT_GE(set.clearanceM(q.value()), 5.0 - 1e-6);
}

TEST(ZoneSetTest, KeepInBounds) {
  ZoneSet set;
  EXPECT_FALSE(set.keepInBounds().has_value());

  set.addZone(ZoneKind::KEEP_OUT, square100());
  EXPECT_FALSE(set.keepInBounds().has_value());

  set.addZone(ZoneKind::KEEP_IN, LocalPolygon({{-200.0, -100.0}, {300.0, -100.0},
                                               {300.0, 400.0}, {-200.0, 400.0}}));
  set.addZone(ZoneKind::KEEP_IN, LocalPolygon({{-100.0, -200.0}, {250.0, -200.0},
                                               {250.0, 300.0}, {-100.0, 300.0}}));
  const auto bounds = set.keepInBounds();
  ASSERT_TRUE(bounds.has_value());
  EXPECT_NEAR(bounds->first.x, -100.0, 1e-9);
  EXPECT_NEAR(bounds->first.y, -100.0, 1e-9);
  EXPECT_NEAR(bounds->second.x, 250.0, 1e-9);
  EXPECT_NEAR(bounds->second.y, 300.0, 1e-9);
}

TEST(ZoneSetTest, EmptySetIsAlwaysCompliant) {
  ZoneSet set;
  EXPECT_TRUE(set.empty());
  EXPECT_TRUE(set.pointCompliant({0.0, 0.0}, 1000.0));
  EXPECT_TRUE(set.segmentClear({0.0, 0.0}, {1000.0, 1000.0}, 5.0, 1.0));
  EXPECT_FALSE(set.raycastFirstHit({0.0, 0.0}, {1.0, 0.0}, 5.0, 1000.0).has_value());
  const auto q = set.nearestCompliantPoint({7.0, 9.0}, 5.0);
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(q->x, 7.0, 1e-9);
  EXPECT_NEAR(q->y, 9.0, 1e-9);
}

}  // namespace arlcore::autopilot
