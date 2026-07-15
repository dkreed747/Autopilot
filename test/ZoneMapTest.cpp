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

#include <GeographicLib/LocalCartesian.hpp>

#include "ZoneMap.h"

namespace arlcore::autopilot {

namespace {

constexpr double kLat = 39.0;
constexpr double kLon = -76.5;

//! Geodetic point `east`/`north` meters from the test origin.
GeoPoint at(double east, double north) {
  static const GeographicLib::LocalCartesian frame(kLat, kLon, 0.0);
  GeoPoint p;
  double h = 0.0;
  frame.Reverse(east, north, 0.0, p.latDeg, p.lonDeg, h);
  return p;
}

//! A square keep-out/keep-in zone [0,size]x[0,size] in the test frame.
ZoneRecord squareZone(ZoneKind kind, double size, ElevationBand band = {}) {
  ZoneRecord zone;
  zone.kind = kind;
  zone.band = band;
  ZoneShape shape;
  shape.polygon = {at(0.0, 0.0), at(size, 0.0), at(size, size), at(0.0, size)};
  zone.shapes.push_back(shape);
  return zone;
}

ConstraintSnapshot snapshotWith(std::vector<ZoneRecord> zones, uint64_t revision = 1) {
  ConstraintSnapshot snapshot;
  snapshot.revision = revision;
  snapshot.zones = std::move(zones);
  return snapshot;
}

ZonesConfig defaultZonesConfig() {
  return ZonesConfig{};  // 5 m margin, 2 m hysteresis, 2 m elevation margin, 32 segments
}

}  // namespace

TEST(ZoneMapTest, ClassifyAgainstKeepOut) {
  ZoneMap map(defaultZonesConfig());
  EXPECT_FALSE(map.hasZones());
  map.ingest(snapshotWith({squareZone(ZoneKind::KEEP_OUT, 100.0)}));
  ASSERT_TRUE(map.hasZones());
  EXPECT_EQ(map.revision(), 1u);

  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(-1.0, 50.0), 0.0), ZoneCompliance::MARGINAL);   // < 2 m hysteresis
  EXPECT_EQ(map.classify(at(-50.0, 50.0), 0.0), ZoneCompliance::COMPLIANT);

  EXPECT_TRUE(map.pointCompliant(at(-50.0, 50.0), 0.0, 5.0));
  EXPECT_FALSE(map.pointCompliant(at(-3.0, 50.0), 0.0, 5.0));
}

TEST(ZoneMapTest, ClassifyAgainstKeepIn) {
  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(ZoneKind::KEEP_IN, 100.0)}));

  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), ZoneCompliance::COMPLIANT);
  EXPECT_EQ(map.classify(at(-10.0, 50.0), 0.0), ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, ElevationBandGatesZones) {
  // Zone covers depths 10..20 m only.
  ElevationBand band;
  band.ceilingDepthM = 10.0;
  band.floorDepthM = 20.0;
  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(ZoneKind::KEEP_OUT, 100.0, band)}));

  // At the surface (0 m, +/- 2 m margin) the zone is not an obstacle.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), ZoneCompliance::COMPLIANT);
  // At 15 m depth it is.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 15.0), ZoneCompliance::VIOLATION);
  // The elevation margin pads the vehicle envelope: 9 m depth + 2 m margin reaches the band.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 9.0), ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, UnconvertibleBandIsAlwaysActive) {
  ElevationBand band;
  band.convertible = false;
  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(ZoneKind::KEEP_OUT, 100.0, band)}));

  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(50.0, 50.0), 500.0), ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, ActiveSetProjectsIntoCallerFrame) {
  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(ZoneKind::KEEP_OUT, 100.0)}));

  // A planner frame anchored 1 km east of the zone still sees the same geometry.
  GeoPoint plannerOrigin = at(1000.0, 0.0);
  const GeographicLib::LocalCartesian plannerFrame(plannerOrigin.latDeg, plannerOrigin.lonDeg, 0.0);
  const ZoneSet set = map.activeSet(plannerFrame, ElevationEnvelope{0.0, 0.0});
  ASSERT_EQ(set.size(), 1u);
  // Zone center (50, 50) in the map frame is near (-950, 50) in the planner frame.
  EXPECT_LT(set.clearanceM({-950.0, 50.0}), 0.0);
  EXPECT_GT(set.clearanceM({0.0, 50.0}), 0.0);
}

TEST(ZoneMapTest, IngestReplacesZones) {
  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(ZoneKind::KEEP_OUT, 100.0)}, 1));
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), ZoneCompliance::VIOLATION);

  map.ingest(snapshotWith({}, 2));
  EXPECT_FALSE(map.hasZones());
  EXPECT_EQ(map.revision(), 2u);
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), ZoneCompliance::COMPLIANT);
}

TEST(ZoneMapTest, EllipseKeepOutIsCircumscribed) {
  // Circle of radius 100 m centered at (0, 0).
  ZoneRecord zone;
  zone.kind = ZoneKind::KEEP_OUT;
  ZoneShape shape;
  ZoneEllipse ellipse;
  ellipse.center = at(0.0, 0.0);
  ellipse.semiMajorM = 100.0;
  ellipse.semiMinorM = 100.0;
  shape.ellipse = ellipse;
  zone.shapes.push_back(shape);

  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({zone}));

  // Circumscribed: a point on the true ellipse boundary must still be (just) inside the polygon.
  EXPECT_EQ(map.classify(at(100.0, 0.0), 0.0), ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(0.0, 0.0), 0.0), ZoneCompliance::VIOLATION);
  // Well outside stays compliant.
  EXPECT_EQ(map.classify(at(150.0, 0.0), 0.0), ZoneCompliance::COMPLIANT);
}

TEST(ZoneMapTest, EllipseKeepInIsInscribed) {
  ZoneRecord zone;
  zone.kind = ZoneKind::KEEP_IN;
  ZoneShape shape;
  ZoneEllipse ellipse;
  ellipse.center = at(0.0, 0.0);
  ellipse.semiMajorM = 100.0;
  ellipse.semiMinorM = 100.0;
  shape.ellipse = ellipse;
  zone.shapes.push_back(shape);

  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({zone}));

  // Inscribed: a boundary point of the true ellipse midway between polygon vertices falls in the
  // chord sag, outside the conservative polygon. (An exact vertex direction would sit ON the
  // polygon boundary, so probe mid-edge: 84.375 degrees = 7.5 segments of 11.25 degrees.)
  EXPECT_EQ(map.classify(at(99.5185, 9.8017), 0.0), ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(0.0, 0.0), 0.0), ZoneCompliance::COMPLIANT);
}

TEST(ZoneMapTest, RotatedEllipseOrientation) {
  // Semi-major 200 m along a 90 degree bearing (east), semi-minor 50 m (north).
  ZoneRecord zone;
  zone.kind = ZoneKind::KEEP_OUT;
  ZoneShape shape;
  ZoneEllipse ellipse;
  ellipse.center = at(0.0, 0.0);
  ellipse.semiMajorM = 200.0;
  ellipse.semiMinorM = 50.0;
  ellipse.orientationRad = M_PI / 2.0;
  shape.ellipse = ellipse;
  zone.shapes.push_back(shape);

  ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({zone}));

  EXPECT_EQ(map.classify(at(150.0, 0.0), 0.0), ZoneCompliance::VIOLATION);   // along major (east)
  EXPECT_EQ(map.classify(at(0.0, 150.0), 0.0), ZoneCompliance::COMPLIANT);   // along minor (north)
}

}  // namespace arlcore::autopilot
