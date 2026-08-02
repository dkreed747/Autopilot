#include <gtest/gtest.h>

#include "InternalTypes.h"
#include "autopilot/safety/ConstraintClamp.hpp"

static arlcore::autopilot::ControlVector cvWith(flt64_t speedMps, std::optional<flt64_t> depthM = std::nullopt) {
  arlcore::autopilot::ControlVector cv;
  cv.headingRad = 1.0;
  cv.speedMps = speedMps;
  cv.elevationM = depthM;
  cv.elevationFrame = arlcore::autopilot::ElevationFrame::DEPTH;
  return cv;
}

static arlcore::autopilot::ControlVector asfWith(flt64_t asfM) {
  arlcore::autopilot::ControlVector cv = cvWith(3.0, asfM);
  cv.elevationFrame = arlcore::autopilot::ElevationFrame::ALTITUDE_ASF;
  return cv;
}

TEST(ConstraintClampTest, NoLimitsPassThrough) {
  // GIVEN: a control vector and no dynamic or static limits
  // WHEN: applying constraint clamps
  const arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      cvWith(4.0, 12.0), arlcore::autopilot::ConstraintSnapshot{}, arlcore::autopilot::ClampLimits{}, std::nullopt);

  // THEN: speed and elevation pass through unclamped
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 4.0);
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 12.0);
  EXPECT_FALSE(r.speedClamped);
  EXPECT_FALSE(r.elevationClamped);
  EXPECT_FALSE(r.conflict);
}

TEST(ConstraintClampTest, MostRestrictiveMaxSpeedWins) {
  // GIVEN: a dynamic max speed tighter than the static one
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxSpeedMps = 2.0;  // dynamic constraint tighter than static
  arlcore::autopilot::ClampLimits statics;
  statics.maxSpeedMps = 3.0;

  // WHEN: clamping a 4 m/s command
  arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(4.0), dynamic, statics, std::nullopt);

  // THEN: the dynamic bound wins
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.0);
  EXPECT_TRUE(r.speedClamped);

  // WHEN: flipping the tightness so the static bound is tighter
  dynamic.maxSpeedMps = 3.0;
  statics.maxSpeedMps = 2.5;
  r = arlcore::autopilot::applyConstraintClamps(cvWith(4.0), dynamic, statics, std::nullopt);

  // THEN: the static bound wins
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.5);
}

TEST(ConstraintClampTest, MinSpeedRaisesOnlyNonzeroCommands) {
  // GIVEN: a dynamic minimum speed of 1.5 m/s
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.minSpeedMps = 1.5;

  // WHEN: clamping a slow but nonzero command
  arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(0.5), dynamic, arlcore::autopilot::ClampLimits{}, std::nullopt);

  // THEN: the command is raised to the minimum
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 1.5);
  EXPECT_TRUE(r.speedClamped);

  // WHEN: clamping a commanded stop/hold
  r = arlcore::autopilot::applyConstraintClamps(cvWith(0.0), dynamic, arlcore::autopilot::ClampLimits{}, std::nullopt);

  // THEN: it is never sped up
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 0.0);
  EXPECT_FALSE(r.speedClamped);
}

TEST(ConstraintClampTest, NegativeSpeedClampsMagnitude) {
  // GIVEN: a dynamic max speed of 2 m/s
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxSpeedMps = 2.0;

  // WHEN: clamping a -4 m/s command
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(-4.0), dynamic, arlcore::autopilot::ClampLimits{}, std::nullopt);

  // THEN: the magnitude is clamped and the sign preserved
  EXPECT_DOUBLE_EQ(r.cv.speedMps, -2.0);
  EXPECT_TRUE(r.speedClamped);
}

TEST(ConstraintClampTest, DepthClamps) {
  // GIVEN: dynamic depth bounds of 5..20 m
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;
  dynamic.minDepthM = 5.0;

  // WHEN: commanding a depth beyond the max
  arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      cvWith(3.0, 25.0), dynamic, arlcore::autopilot::ClampLimits{}, std::nullopt);

  // THEN: it clamps to the max
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 20.0);
  EXPECT_TRUE(r.elevationClamped);

  // WHEN: commanding a depth above the min
  r = arlcore::autopilot::applyConstraintClamps(cvWith(3.0, 2.0), dynamic, arlcore::autopilot::ClampLimits{},
                                                std::nullopt);

  // THEN: it clamps to the min
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 5.0);
  EXPECT_TRUE(r.elevationClamped);

  // WHEN: commanding a depth within the bounds
  r = arlcore::autopilot::applyConstraintClamps(cvWith(3.0, 10.0), dynamic, arlcore::autopilot::ClampLimits{},
                                                std::nullopt);

  // THEN: it passes through unclamped
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 10.0);
  EXPECT_FALSE(r.elevationClamped);
}

TEST(ConstraintClampTest, UnconvertibleFramesPassThrough) {
  // GIVEN: a depth bound and a command in the MSL altitude frame, which has no depth equivalent
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;
  arlcore::autopilot::ControlVector cv = cvWith(3.0, 50.0);
  cv.elevationFrame = arlcore::autopilot::ElevationFrame::ALTITUDE_MSL;

  // WHEN: applying constraint clamps with a live seafloor reference
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cv, dynamic, arlcore::autopilot::ClampLimits{}, 100.0);

  // THEN: the unconvertible elevation passes through untouched (admission refuses it instead)
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 50.0);
  EXPECT_FALSE(r.elevationClamped);
  EXPECT_FALSE(r.elevationUnbounded);
}

TEST(ConstraintClampTest, AsfSetpointClampsAgainstTheDepthCeiling) {
  // GIVEN: a 20 m depth ceiling and a sea floor 100 m down
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;

  // WHEN: commanding 5 m above the sea floor, which is 95 m down
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(asfWith(5.0), dynamic, arlcore::autopilot::ClampLimits{}, 100.0);

  // THEN: the altitude is raised to the one that sits exactly on the depth ceiling
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 80.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_FALSE(r.conflict);
}

TEST(ConstraintClampTest, AsfSetpointClampsAgainstTheBottomClearanceWithoutAnyReference) {
  // GIVEN: only a bottom-clearance limit, which is native to the ASF frame, and no floor reference
  arlcore::autopilot::ClampLimits statics;
  statics.minAltitudeAsfM = 5.0;

  // WHEN: commanding 3 m above the sea floor
  const arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      asfWith(3.0), arlcore::autopilot::ConstraintSnapshot{}, statics, std::nullopt);

  // THEN: it clamps to the clearance without needing to know where the floor is
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 5.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_FALSE(r.elevationUnbounded);
}

TEST(ConstraintClampTest, AsfSetpointIsDroppedWhenADepthBoundCannotBeConverted) {
  // GIVEN: a depth ceiling and no seafloor reference to convert an ASF setpoint against
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;

  // WHEN: commanding an altitude above the sea floor
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(asfWith(5.0), dynamic, arlcore::autopilot::ClampLimits{}, std::nullopt);

  // THEN: the elevation demand is withheld rather than forwarded unbounded, and it is reported
  EXPECT_FALSE(r.cv.elevationM.has_value());
  EXPECT_TRUE(r.elevationUnbounded);
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 3.0);  // heading and speed keep driving
}

TEST(ConstraintClampTest, AsfSetpointIsCappedAtTheSurface) {
  // GIVEN: no configured bounds at all and a sea floor 100 m down
  // WHEN: commanding an altitude above the water surface
  const arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      asfWith(500.0), arlcore::autopilot::ConstraintSnapshot{}, arlcore::autopilot::ClampLimits{}, 100.0);

  // THEN: the surface caps it, since the floor depth is the only upper bound this frame has
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 100.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_FALSE(r.conflict);
}

TEST(ConstraintClampTest, DepthSetpointIsCappedByTheBottomClearance) {
  // GIVEN: a bottom clearance of 5 m, a depth ceiling well below the floor, and a floor 100 m down
  arlcore::autopilot::ClampLimits statics;
  statics.minAltitudeAsfM = 5.0;
  statics.maxDepthM = 200.0;

  // WHEN: commanding 96 m of depth, 4 m off the bottom
  const arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      cvWith(3.0, 96.0), arlcore::autopilot::ConstraintSnapshot{}, statics, 100.0);

  // THEN: the depth is pulled up to the clearance
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 95.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_FALSE(r.conflict);
}

TEST(ConstraintClampTest, DepthSetpointKeepsItsOwnBoundsWithoutAFloorReference) {
  // GIVEN: a bottom clearance that needs a floor reference, plus a depth ceiling that does not
  arlcore::autopilot::ClampLimits statics;
  statics.minAltitudeAsfM = 5.0;
  statics.maxDepthM = 50.0;

  // WHEN: commanding a depth past the ceiling with no floor reference
  const arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      cvWith(3.0, 60.0), arlcore::autopilot::ConstraintSnapshot{}, statics, std::nullopt);

  // THEN: the frame-native ceiling still applies; only the bottom clearance is skipped
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 50.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_FALSE(r.elevationUnbounded);
}

TEST(ConstraintClampTest, WaterThinnerThanTheClearanceResolvesToTheSurface) {
  // GIVEN: a 5 m bottom clearance over a floor only 3 m down, and no minimum depth
  arlcore::autopilot::ClampLimits statics;
  statics.minAltitudeAsfM = 5.0;

  // WHEN: commanding a depth in that water
  arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      cvWith(3.0, 1.0), arlcore::autopilot::ConstraintSnapshot{}, statics, 3.0);

  // THEN: nothing in the column is legal, so it resolves to the surface rather than above it
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 0.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_TRUE(r.conflict);

  // WHEN: the same water is commanded in the ASF frame
  r = arlcore::autopilot::applyConstraintClamps(asfWith(1.0), arlcore::autopilot::ConstraintSnapshot{}, statics, 3.0);

  // THEN: it resolves to the surface too, not to an altitude above the water
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 3.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_TRUE(r.conflict);
}

TEST(ConstraintClampTest, TheBottomClearanceWinsOverTheMinimumDepth) {
  // GIVEN: a 20 m water column that must be flown 15 m off the bottom but no shallower than 10 m
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.minDepthM = 10.0;
  arlcore::autopilot::ClampLimits statics;
  statics.minAltitudeAsfM = 15.0;

  // WHEN: commanding an altitude between the two irreconcilable bounds
  arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(asfWith(12.0), dynamic, statics, 20.0);

  // THEN: the deep-side bound wins - a bottom strike is the worse outcome - and it is flagged
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 15.0);
  EXPECT_TRUE(r.elevationClamped);
  EXPECT_TRUE(r.conflict);

  // WHEN: the same window is commanded in the DEPTH frame
  r = arlcore::autopilot::applyConstraintClamps(cvWith(3.0, 12.0), dynamic, statics, 20.0);

  // THEN: it resolves to the same physical place, 5 m down / 15 m off the bottom
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 5.0);
  EXPECT_TRUE(r.conflict);
}

TEST(ConstraintClampTest, MissingElevationUntouched) {
  // GIVEN: a depth bound and a command with no elevation
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;

  // WHEN: applying constraint clamps
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(3.0), dynamic, arlcore::autopilot::ClampLimits{}, std::nullopt);

  // THEN: the elevation stays absent and unclamped
  EXPECT_FALSE(r.cv.elevationM.has_value());
  EXPECT_FALSE(r.elevationClamped);
  EXPECT_FALSE(r.elevationUnbounded);
}

TEST(ConstraintClampTest, ConflictingBoundsClampToMaxAndFlag) {
  // GIVEN: a dynamic min speed above the static max speed
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.minSpeedMps = 4.0;
  arlcore::autopilot::ClampLimits statics;
  statics.maxSpeedMps = 2.0;

  // WHEN: clamping a command between the conflicting bounds
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(3.0), dynamic, statics, std::nullopt);

  // THEN: the conflict is flagged and the max bound wins
  EXPECT_TRUE(r.conflict);
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.0);  // the max bound wins
}
