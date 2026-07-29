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

TEST(ConstraintClampTest, NoLimitsPassThrough) {
  // GIVEN: a control vector and no dynamic or static limits
  // WHEN: applying constraint clamps
  const arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(
      cvWith(4.0, 12.0), arlcore::autopilot::ConstraintSnapshot{}, arlcore::autopilot::ClampLimits{});

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
  arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(cvWith(4.0), dynamic, statics);

  // THEN: the dynamic bound wins
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.0);
  EXPECT_TRUE(r.speedClamped);

  // WHEN: flipping the tightness so the static bound is tighter
  dynamic.maxSpeedMps = 3.0;
  statics.maxSpeedMps = 2.5;
  r = arlcore::autopilot::applyConstraintClamps(cvWith(4.0), dynamic, statics);

  // THEN: the static bound wins
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.5);
}

TEST(ConstraintClampTest, MinSpeedRaisesOnlyNonzeroCommands) {
  // GIVEN: a dynamic minimum speed of 1.5 m/s
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.minSpeedMps = 1.5;

  // WHEN: clamping a slow but nonzero command
  arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(0.5), dynamic, arlcore::autopilot::ClampLimits{});

  // THEN: the command is raised to the minimum
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 1.5);
  EXPECT_TRUE(r.speedClamped);

  // WHEN: clamping a commanded stop/hold
  r = arlcore::autopilot::applyConstraintClamps(cvWith(0.0), dynamic, arlcore::autopilot::ClampLimits{});

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
      arlcore::autopilot::applyConstraintClamps(cvWith(-4.0), dynamic, arlcore::autopilot::ClampLimits{});

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
  arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(3.0, 25.0), dynamic, arlcore::autopilot::ClampLimits{});

  // THEN: it clamps to the max
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 20.0);
  EXPECT_TRUE(r.elevationClamped);

  // WHEN: commanding a depth above the min
  r = arlcore::autopilot::applyConstraintClamps(cvWith(3.0, 2.0), dynamic, arlcore::autopilot::ClampLimits{});

  // THEN: it clamps to the min
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 5.0);
  EXPECT_TRUE(r.elevationClamped);

  // WHEN: commanding a depth within the bounds
  r = arlcore::autopilot::applyConstraintClamps(cvWith(3.0, 10.0), dynamic, arlcore::autopilot::ClampLimits{});

  // THEN: it passes through unclamped
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 10.0);
  EXPECT_FALSE(r.elevationClamped);
}

TEST(ConstraintClampTest, NonDepthFramesPassThrough) {
  // GIVEN: a depth bound and a command in the ASF altitude frame
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;
  arlcore::autopilot::ControlVector cv = cvWith(3.0, 50.0);
  cv.elevationFrame = arlcore::autopilot::ElevationFrame::ALTITUDE_ASF;

  // WHEN: applying constraint clamps
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cv, dynamic, arlcore::autopilot::ClampLimits{});

  // THEN: the non-depth elevation passes through unclamped
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 50.0);
  EXPECT_FALSE(r.elevationClamped);
}

TEST(ConstraintClampTest, MissingElevationUntouched) {
  // GIVEN: a depth bound and a command with no elevation
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;

  // WHEN: applying constraint clamps
  const arlcore::autopilot::ClampResult r =
      arlcore::autopilot::applyConstraintClamps(cvWith(3.0), dynamic, arlcore::autopilot::ClampLimits{});

  // THEN: the elevation stays absent and unclamped
  EXPECT_FALSE(r.cv.elevationM.has_value());
  EXPECT_FALSE(r.elevationClamped);
}

TEST(ConstraintClampTest, ConflictingBoundsClampToMaxAndFlag) {
  // GIVEN: a dynamic min speed above the static max speed
  arlcore::autopilot::ConstraintSnapshot dynamic;
  dynamic.minSpeedMps = 4.0;
  arlcore::autopilot::ClampLimits statics;
  statics.maxSpeedMps = 2.0;

  // WHEN: clamping a command between the conflicting bounds
  const arlcore::autopilot::ClampResult r = arlcore::autopilot::applyConstraintClamps(cvWith(3.0), dynamic, statics);

  // THEN: the conflict is flagged and the max bound wins
  EXPECT_TRUE(r.conflict);
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.0);  // the max bound wins
}
