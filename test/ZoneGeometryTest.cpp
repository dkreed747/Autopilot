#include <gtest/gtest.h>

#include <cmath>

#include "autopilot/safety/ZoneGeometry.hpp"

//! An axis-aligned square [0,100]x[0,100], given clockwise to exercise re-winding.
static arlcore::autopilot::LocalPolygon square100() {
  return arlcore::autopilot::LocalPolygon({{0.0, 0.0}, {0.0, 100.0}, {100.0, 100.0}, {100.0, 0.0}});
}

TEST(LocalPolygonTest, ContainsAndSignedDistance) {
  // GIVEN: a 100 m axis-aligned square polygon
  const arlcore::autopilot::LocalPolygon poly = square100();

  // WHEN: querying containment for interior, exterior, and boundary points
  // THEN: interior and boundary count as inside, exterior does not
  EXPECT_TRUE(poly.contains({50.0, 50.0}));
  EXPECT_FALSE(poly.contains({150.0, 50.0}));
  EXPECT_FALSE(poly.contains({-1.0, 50.0}));
  EXPECT_TRUE(poly.contains({0.0, 50.0}));  // boundary counts as inside

  // WHEN: querying signed distance at the same kinds of points
  // THEN: inside is positive, outside negative, boundary zero
  EXPECT_NEAR(poly.signedDistance({50.0, 50.0}), 50.0, 1e-9);
  EXPECT_NEAR(poly.signedDistance({10.0, 50.0}), 10.0, 1e-9);
  EXPECT_NEAR(poly.signedDistance({-20.0, 50.0}), -20.0, 1e-9);
  EXPECT_NEAR(poly.signedDistance({0.0, 50.0}), 0.0, 1e-9);
  // Outside a corner: distance to the vertex.
  EXPECT_NEAR(poly.signedDistance({103.0, 104.0}), -5.0, 1e-9);
}

TEST(LocalPolygonTest, ClosestBoundaryPoint) {
  // GIVEN: a 100 m square polygon
  const arlcore::autopilot::LocalPolygon poly = square100();

  // WHEN: projecting a point north of the square onto the boundary
  const arlcore::autopilot::Vec2 c = poly.closestBoundaryPoint({50.0, 120.0});

  // THEN: the closest point lies on the north edge directly below it
  EXPECT_NEAR(c.x, 50.0, 1e-9);
  EXPECT_NEAR(c.y, 100.0, 1e-9);
}

TEST(LocalPolygonTest, NonConvexContains) {
  // GIVEN: an L-shaped polygon whose notch [50,100]x[50,100] is outside
  const arlcore::autopilot::LocalPolygon poly(
      {{0.0, 0.0}, {100.0, 0.0}, {100.0, 50.0}, {50.0, 50.0}, {50.0, 100.0}, {0.0, 100.0}});

  // WHEN: classifying points in the arms and in the notch
  // THEN: arm points are inside, the notch point is outside with negative distance
  EXPECT_TRUE(poly.contains({25.0, 75.0}));
  EXPECT_TRUE(poly.contains({75.0, 25.0}));
  EXPECT_FALSE(poly.contains({75.0, 75.0}));
  EXPECT_NEAR(poly.signedDistance({75.0, 75.0}), -25.0, 1e-9);
}

TEST(ZoneSetTest, KeepOutClearance) {
  // GIVEN: a zone set with one 100 m square keep-out
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());

  // WHEN: evaluating clearance and margin compliance around the zone
  // THEN: outside is positive and compliant, inside negative, under-margin fails
  EXPECT_NEAR(set.clearanceM({-20.0, 50.0}), 20.0, 1e-9);  // outside = compliant
  EXPECT_NEAR(set.clearanceM({10.0, 50.0}), -10.0, 1e-9);  // inside = violating
  EXPECT_TRUE(set.pointCompliant({-20.0, 50.0}, 5.0));
  EXPECT_FALSE(set.pointCompliant({-3.0, 50.0}, 5.0));  // compliant but under margin
}

TEST(ZoneSetTest, KeepInClearance) {
  // GIVEN: a zone set with one 100 m square keep-in
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_IN, square100());

  // WHEN: evaluating clearance inside and outside the zone
  // THEN: inside is compliant, outside violating
  EXPECT_NEAR(set.clearanceM({50.0, 50.0}), 50.0, 1e-9);    // inside = compliant
  EXPECT_NEAR(set.clearanceM({-20.0, 50.0}), -20.0, 1e-9);  // outside = violating
}

TEST(ZoneSetTest, ClearanceIsMinOverZones) {
  // GIVEN: a large keep-in containing a 100 m square keep-out
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_IN,
              arlcore::autopilot::LocalPolygon({{-200.0, -200.0}, {200.0, -200.0}, {200.0, 200.0}, {-200.0, 200.0}}));
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());

  // WHEN: evaluating clearance where different zones are binding
  // THEN: the minimum (most binding) zone clearance is reported
  // Inside the keep-in (clearance 150 to its boundary) but also inside the keep-out.
  EXPECT_NEAR(set.clearanceM({50.0, 50.0}), -50.0, 1e-9);
  // Compliant w.r.t. both; the keep-out is binding.
  EXPECT_NEAR(set.clearanceM({-30.0, 50.0}), 30.0, 1e-9);
}

TEST(ZoneSetTest, ClearanceInfoDirections) {
  // GIVEN: a zone set with one 100 m square keep-out
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());

  // WHEN: querying clearance info at a compliant point west of the zone
  arlcore::autopilot::ClearanceInfo info = set.clearanceInfo({-10.0, 50.0});
  // THEN: clearance grows to the west
  EXPECT_NEAR(info.clearanceM, 10.0, 1e-9);
  EXPECT_NEAR(info.improveDir.x, -1.0, 1e-9);
  EXPECT_NEAR(info.improveDir.y, 0.0, 1e-9);

  // WHEN: querying at a violating point just inside the west edge
  info = set.clearanceInfo({10.0, 50.0});
  // THEN: improvement is west, through the boundary
  EXPECT_NEAR(info.clearanceM, -10.0, 1e-9);
  EXPECT_NEAR(info.improveDir.x, -1.0, 1e-9);
  EXPECT_NEAR(info.improveDir.y, 0.0, 1e-9);
}

TEST(ZoneSetTest, SegmentClear) {
  // GIVEN: a zone set with one 100 m square keep-out
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());

  // WHEN: checking segments west of, through, and just beside the zone
  // THEN: only segments that keep the margin along their length are clear
  EXPECT_TRUE(set.segmentClear({-50.0, -50.0}, {-50.0, 150.0}, 5.0, 1.0));     // passes west of it
  EXPECT_FALSE(set.segmentClear({-50.0, 50.0}, {150.0, 50.0}, 5.0, 1.0));      // straight through
  EXPECT_FALSE(set.segmentClear({-4.0, -50.0}, {-4.0, 150.0}, 5.0, 1.0));      // clear but < margin
  EXPECT_TRUE(set.segmentClear({-50.0, -50.0}, {-50.0, 150.0}, 5.0, 1000.0));  // endpoint-only fallback
}

TEST(ZoneSetTest, PathClear) {
  // GIVEN: a zone set with one 100 m square keep-out
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());

  // WHEN: solving a straight Dubins "path" west of the zone
  auto clearPath = arlcore::autopilot::DubinsPath::solve({-50.0, -50.0, M_PI / 2.0}, {-50.0, 150.0, M_PI / 2.0}, 10.0);
  ASSERT_TRUE(clearPath.has_value());
  // THEN: the path is clear
  EXPECT_TRUE(set.pathClear(clearPath.value(), 5.0, 1.0));

  // WHEN: solving one straight through the zone
  auto blockedPath = arlcore::autopilot::DubinsPath::solve({-50.0, 50.0, 0.0}, {150.0, 50.0, 0.0}, 10.0);
  ASSERT_TRUE(blockedPath.has_value());
  // THEN: the path is not clear
  EXPECT_FALSE(set.pathClear(blockedPath.value(), 5.0, 1.0));
}

TEST(ZoneSetTest, RaycastFirstHit) {
  // GIVEN: a zone set with one 100 m square keep-out
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());

  // WHEN: casting a ray east from (-50, 50) toward the zone
  auto hit = set.raycastFirstHit({-50.0, 50.0}, {1.0, 0.0}, 5.0, 200.0);
  // THEN: the margin-5 clearance is crossed 5 m before the west edge
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit.value(), 45.0, 0.2);

  // WHEN: casting a ray north that misses the zone entirely
  // THEN: no hit is reported
  EXPECT_FALSE(set.raycastFirstHit({-50.0, 50.0}, {0.0, 1.0}, 5.0, 200.0).has_value());

  // WHEN: starting the ray below margin
  hit = set.raycastFirstHit({-2.0, 50.0}, {0.0, 1.0}, 5.0, 200.0);
  // THEN: an immediate hit is reported
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit.value(), 0.0, 1e-9);
}

TEST(ZoneSetTest, NearestCompliantPointFromInsideKeepOut) {
  // GIVEN: a 100 m square keep-out with the query point inside it
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());

  // WHEN: searching for the nearest compliant point with a 5 m margin
  const auto q = set.nearestCompliantPoint({10.0, 50.0}, 5.0);

  // THEN: a margin-compliant point past the nearest (west) edge is found
  ASSERT_TRUE(q.has_value());
  EXPECT_GE(set.clearanceM(q.value()), 5.0 - 1e-6);
  // Nearest exit is through the west edge.
  EXPECT_LT(q->x, -4.9);
  EXPECT_NEAR(q->y, 50.0, 1.0);
}

TEST(ZoneSetTest, NearestCompliantPointOutsideKeepIn) {
  // GIVEN: a 100 m square keep-in with the query point outside it
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_IN, square100());

  // WHEN: searching for the nearest compliant point with a 5 m margin
  const auto q = set.nearestCompliantPoint({-30.0, 50.0}, 5.0);

  // THEN: a margin-compliant point inside the keep-in is found
  ASSERT_TRUE(q.has_value());
  EXPECT_GE(set.clearanceM(q.value()), 5.0 - 1e-6);
  EXPECT_GT(q->x, 4.9);
}

TEST(ZoneSetTest, NearestCompliantPointMultiZoneCorner) {
  // GIVEN: two overlapping keep-outs forming a wedge; projection may ping-pong,
  // ring search must save it
  arlcore::autopilot::ZoneSet set;
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
              arlcore::autopilot::LocalPolygon({{0.0, 0.0}, {100.0, 0.0}, {100.0, 60.0}, {0.0, 60.0}}));
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT,
              arlcore::autopilot::LocalPolygon({{0.0, 40.0}, {100.0, 40.0}, {100.0, 100.0}, {0.0, 100.0}}));

  // WHEN: searching for the nearest compliant point from inside the overlap
  const auto q = set.nearestCompliantPoint({50.0, 50.0}, 5.0);

  // THEN: a margin-compliant point is still found
  ASSERT_TRUE(q.has_value());
  EXPECT_GE(set.clearanceM(q.value()), 5.0 - 1e-6);
}

TEST(ZoneSetTest, KeepInBounds) {
  // GIVEN: an empty zone set
  arlcore::autopilot::ZoneSet set;

  // WHEN: querying keep-in bounds with no zones, then with only a keep-out
  // THEN: no bounds are reported
  EXPECT_FALSE(set.keepInBounds().has_value());
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_OUT, square100());
  EXPECT_FALSE(set.keepInBounds().has_value());

  // WHEN: adding two overlapping keep-ins
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_IN,
              arlcore::autopilot::LocalPolygon({{-200.0, -100.0}, {300.0, -100.0}, {300.0, 400.0}, {-200.0, 400.0}}));
  set.addZone(arlcore::autopilot::ZoneKind::KEEP_IN,
              arlcore::autopilot::LocalPolygon({{-100.0, -200.0}, {250.0, -200.0}, {250.0, 300.0}, {-100.0, 300.0}}));
  const auto bounds = set.keepInBounds();

  // THEN: the bounds are the intersection of the keep-in boxes
  ASSERT_TRUE(bounds.has_value());
  EXPECT_NEAR(bounds->first.x, -100.0, 1e-9);
  EXPECT_NEAR(bounds->first.y, -100.0, 1e-9);
  EXPECT_NEAR(bounds->second.x, 250.0, 1e-9);
  EXPECT_NEAR(bounds->second.y, 300.0, 1e-9);
}

TEST(ZoneSetTest, EmptySetIsAlwaysCompliant) {
  // GIVEN: an empty zone set
  arlcore::autopilot::ZoneSet set;
  EXPECT_TRUE(set.empty());

  // WHEN: running compliance queries with no zones
  // THEN: everything is compliant and the nearest compliant point is the query itself
  EXPECT_TRUE(set.pointCompliant({0.0, 0.0}, 1000.0));
  EXPECT_TRUE(set.segmentClear({0.0, 0.0}, {1000.0, 1000.0}, 5.0, 1.0));
  EXPECT_FALSE(set.raycastFirstHit({0.0, 0.0}, {1.0, 0.0}, 5.0, 1000.0).has_value());
  const auto q = set.nearestCompliantPoint({7.0, 9.0}, 5.0);
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(q->x, 7.0, 1e-9);
  EXPECT_NEAR(q->y, 9.0, 1e-9);
}
