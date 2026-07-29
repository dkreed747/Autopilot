#include <gtest/gtest.h>

#include <cmath>

#include <GeographicLib/LocalCartesian.hpp>

#include "autopilot/safety/ZoneMap.hpp"
#include "InternalTypes.h"


constexpr flt64_t kLat = 39.0;
constexpr flt64_t kLon = -76.5;

//! Geodetic point `east`/`north` meters from the test origin.
static arlcore::autopilot::GeoPoint at(flt64_t east, flt64_t north) {
  static const GeographicLib::LocalCartesian frame(kLat, kLon, 0.0);
  arlcore::autopilot::GeoPoint p;
  flt64_t h = 0.0;
  frame.Reverse(east, north, 0.0, p.latDeg, p.lonDeg, h);
  return p;
}

//! A square keep-out/keep-in zone [0,size]x[0,size] in the test frame.
static arlcore::autopilot::ZoneRecord squareZone(
    arlcore::autopilot::ZoneKind kind, flt64_t size,
    arlcore::autopilot::ElevationBand band = {}) {
  arlcore::autopilot::ZoneRecord zone;
  zone.kind = kind;
  zone.band = band;
  arlcore::autopilot::ZoneShape shape;
  shape.polygon = {at(0.0, 0.0), at(size, 0.0), at(size, size), at(0.0, size)};
  zone.shapes.push_back(shape);
  return zone;
}

static arlcore::autopilot::ConstraintSnapshot snapshotWith(
    std::vector<arlcore::autopilot::ZoneRecord> zones, uint64_t revision = 1) {
  arlcore::autopilot::ConstraintSnapshot snapshot;
  snapshot.revision = revision;
  snapshot.zones = std::move(zones);
  return snapshot;
}

static arlcore::autopilot::ZonesConfig defaultZonesConfig() {
  return arlcore::autopilot::ZonesConfig{};  // 5 m margin, 2 m hysteresis, 2 m elevation margin, 32 segments
}


TEST(ZoneMapTest, ClassifyAgainstKeepOut) {
  // GIVEN: an empty zone map
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  EXPECT_FALSE(map.hasZones());

  // WHEN: ingesting a snapshot with one 100 m square keep-out
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_OUT, 100.0)}));

  // THEN: the map has zones and classifies points as violating, marginal, or compliant
  ASSERT_TRUE(map.hasZones());
  EXPECT_EQ(map.revision(), 1u);

  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(-1.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::MARGINAL);   // < 2 m hysteresis
  EXPECT_EQ(map.classify(at(-50.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);

  EXPECT_TRUE(map.pointCompliant(at(-50.0, 50.0), 0.0, 5.0));
  EXPECT_FALSE(map.pointCompliant(at(-3.0, 50.0), 0.0, 5.0));
}

TEST(ZoneMapTest, ClassifyAgainstKeepIn) {
  // GIVEN: a zone map with one 100 m square keep-in
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_IN, 100.0)}));

  // WHEN: classifying a point inside and a point outside the zone
  // THEN: inside is compliant, outside violating
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);
  EXPECT_EQ(map.classify(at(-10.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, ElevationBandGatesZones) {
  // GIVEN: a keep-out whose zone covers depths 10..20 m only
  arlcore::autopilot::ElevationBand band;
  band.ceiling = arlcore::autopilot::ElevationBound{
      arlcore::autopilot::ElevationBound::Frame::DEPTH, 10.0};
  band.floor = arlcore::autopilot::ElevationBound{
      arlcore::autopilot::ElevationBound::Frame::DEPTH, 20.0};
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_OUT, 100.0, band)}));

  // WHEN: classifying the same horizontal point at different depths
  // THEN: the zone applies only within the margin-padded band
  // At the surface (0 m, +/- 2 m margin) the zone is not an obstacle.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);
  // At 15 m depth it is.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 15.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
  // The elevation margin pads the vehicle envelope: 9 m depth + 2 m margin reaches the band.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 9.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, AsfFloorBoundGatesByAltitudeAboveSeaFloor) {
  // GIVEN: the common mixed-frame band: ceiling at depth 0 (the surface) with the floor 5 m above
  // the sea floor — the zone covers the whole water column except a near-bottom corridor
  arlcore::autopilot::ElevationBand band;
  band.ceiling = arlcore::autopilot::ElevationBound{
      arlcore::autopilot::ElevationBound::Frame::DEPTH, 0.0};
  band.floor = arlcore::autopilot::ElevationBound{
      arlcore::autopilot::ElevationBound::Frame::ASF, 5.0};
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_OUT, 100.0, band)}));

  // WHEN: classifying at mid-column, near-bottom, and unknown-ASF states
  // THEN: only the near-bottom corridor is gated out; unknown ASF stays conservative
  // Mid-column (well above the floor cutoff): the zone applies.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 10.0, 50.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
  // Hugging the bottom, below the floor cutoff (2 m ASF + 2 m margin < 5 m): gated out.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 55.0, 2.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);
  // Unknown altitude above the sea floor never exonerates an ASF bound (conservative).
  EXPECT_EQ(map.classify(at(50.0, 50.0), 55.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, AsfCeilingBoundGates) {
  // GIVEN: a near-bottom zone: from 10 m above the floor down to the floor itself
  arlcore::autopilot::ElevationBand band;
  band.ceiling = arlcore::autopilot::ElevationBound{
      arlcore::autopilot::ElevationBound::Frame::ASF, 10.0};
  band.floor = arlcore::autopilot::ElevationBound{
      arlcore::autopilot::ElevationBound::Frame::ASF, 0.0};
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_OUT, 100.0, band)}));

  // WHEN: classifying well above the layer and inside it
  // THEN: the zone is gated out above the ceiling and active within the layer
  // Sailing high above it (20 m ASF - 2 m margin > 10 m): gated out.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 5.0, 20.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);
  // Inside the near-bottom layer: active.
  EXPECT_EQ(map.classify(at(50.0, 50.0), 55.0, 4.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, UnconvertibleBandIsAlwaysActive) {
  // GIVEN: a keep-out whose elevation band cannot be converted
  arlcore::autopilot::ElevationBand band;
  band.convertible = false;
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_OUT, 100.0, band)}));

  // WHEN: classifying inside the zone at any depth
  // THEN: the zone is always active
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(50.0, 50.0), 500.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
}

TEST(ZoneMapTest, ActiveSetProjectsIntoCallerFrame) {
  // GIVEN: a map with one 100 m square keep-out and a planner frame anchored 1 km east
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_OUT, 100.0)}));
  arlcore::autopilot::GeoPoint plannerOrigin = at(1000.0, 0.0);
  const GeographicLib::LocalCartesian plannerFrame(plannerOrigin.latDeg, plannerOrigin.lonDeg, 0.0);

  // WHEN: projecting the active set into the planner frame
  const arlcore::autopilot::ZoneSet set =
      map.activeSet(plannerFrame, arlcore::autopilot::ElevationEnvelope{0.0, 0.0});

  // THEN: the same geometry appears, offset into the caller frame
  ASSERT_EQ(set.size(), 1u);
  // Zone center (50, 50) in the map frame is near (-950, 50) in the planner frame.
  EXPECT_LT(set.clearanceM({-950.0, 50.0}), 0.0);
  EXPECT_GT(set.clearanceM({0.0, 50.0}), 0.0);
}

TEST(ZoneMapTest, IngestReplacesZones) {
  // GIVEN: a map with one keep-out ingested at revision 1
  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({squareZone(arlcore::autopilot::ZoneKind::KEEP_OUT, 100.0)}, 1));
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);

  // WHEN: ingesting an empty snapshot at revision 2
  map.ingest(snapshotWith({}, 2));

  // THEN: the zones are replaced and the point becomes compliant
  EXPECT_FALSE(map.hasZones());
  EXPECT_EQ(map.revision(), 2u);
  EXPECT_EQ(map.classify(at(50.0, 50.0), 0.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);
}

TEST(ZoneMapTest, EllipseKeepOutIsCircumscribed) {
  // GIVEN: a keep-out circle of radius 100 m centered at (0, 0)
  arlcore::autopilot::ZoneRecord zone;
  zone.kind = arlcore::autopilot::ZoneKind::KEEP_OUT;
  arlcore::autopilot::ZoneShape shape;
  arlcore::autopilot::ZoneEllipse ellipse;
  ellipse.center = at(0.0, 0.0);
  ellipse.semiMajorM = 100.0;
  ellipse.semiMinorM = 100.0;
  shape.ellipse = ellipse;
  zone.shapes.push_back(shape);

  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({zone}));

  // WHEN: classifying points on the true boundary, at the center, and well outside
  // THEN: the circumscribed polygon keeps boundary points violating
  // Circumscribed: a point on the true ellipse boundary must still be (just) inside the polygon.
  EXPECT_EQ(map.classify(at(100.0, 0.0), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(0.0, 0.0), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
  // Well outside stays compliant.
  EXPECT_EQ(map.classify(at(150.0, 0.0), 0.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);
}

TEST(ZoneMapTest, EllipseKeepInIsInscribed) {
  // GIVEN: a keep-in circle of radius 100 m centered at (0, 0)
  arlcore::autopilot::ZoneRecord zone;
  zone.kind = arlcore::autopilot::ZoneKind::KEEP_IN;
  arlcore::autopilot::ZoneShape shape;
  arlcore::autopilot::ZoneEllipse ellipse;
  ellipse.center = at(0.0, 0.0);
  ellipse.semiMajorM = 100.0;
  ellipse.semiMinorM = 100.0;
  shape.ellipse = ellipse;
  zone.shapes.push_back(shape);

  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({zone}));

  // WHEN: classifying a mid-edge boundary point and the center
  // THEN: the inscribed polygon keeps the sagging boundary point violating
  // Inscribed: probe mid-edge (84.375 deg = 7.5 segments of 11.25 deg) where the true ellipse
  // boundary sags outside the conservative polygon — an exact vertex direction would sit ON it.
  EXPECT_EQ(map.classify(at(99.5185, 9.8017), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);
  EXPECT_EQ(map.classify(at(0.0, 0.0), 0.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);
}

TEST(ZoneMapTest, RotatedEllipseOrientation) {
  // GIVEN: a keep-out ellipse: semi-major 200 m along a 90 degree bearing (east),
  // semi-minor 50 m (north)
  arlcore::autopilot::ZoneRecord zone;
  zone.kind = arlcore::autopilot::ZoneKind::KEEP_OUT;
  arlcore::autopilot::ZoneShape shape;
  arlcore::autopilot::ZoneEllipse ellipse;
  ellipse.center = at(0.0, 0.0);
  ellipse.semiMajorM = 200.0;
  ellipse.semiMinorM = 50.0;
  ellipse.orientationRad = M_PI / 2.0;
  shape.ellipse = ellipse;
  zone.shapes.push_back(shape);

  arlcore::autopilot::ZoneMap map(defaultZonesConfig());
  map.ingest(snapshotWith({zone}));

  // WHEN: classifying 150 m east and 150 m north of the center
  // THEN: the major axis reaches east but the minor axis does not reach north
  EXPECT_EQ(map.classify(at(150.0, 0.0), 0.0), arlcore::autopilot::ZoneCompliance::VIOLATION);   // along major (east)
  EXPECT_EQ(map.classify(at(0.0, 150.0), 0.0), arlcore::autopilot::ZoneCompliance::COMPLIANT);   // along minor (north)
}
