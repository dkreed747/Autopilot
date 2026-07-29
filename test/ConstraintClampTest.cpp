#include <gtest/gtest.h>

#include "autopilot/safety/ConstraintClamp.hpp"

namespace arlcore::autopilot {


static ControlVector cvWith(double speedMps, std::optional<double> depthM = std::nullopt) {
  ControlVector cv;
  cv.headingRad = 1.0;
  cv.speedMps = speedMps;
  cv.elevationM = depthM;
  cv.elevationFrame = ElevationFrame::DEPTH;
  return cv;
}


TEST(ConstraintClampTest, NoLimitsPassThrough) {
  const ClampResult r = applyConstraintClamps(cvWith(4.0, 12.0), ConstraintSnapshot{}, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 4.0);
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 12.0);
  EXPECT_FALSE(r.speedClamped);
  EXPECT_FALSE(r.elevationClamped);
  EXPECT_FALSE(r.conflict);
}

TEST(ConstraintClampTest, MostRestrictiveMaxSpeedWins) {
  ConstraintSnapshot dynamic;
  dynamic.maxSpeedMps = 2.0;   // dynamic constraint tighter than static
  ClampLimits statics;
  statics.maxSpeedMps = 3.0;

  ClampResult r = applyConstraintClamps(cvWith(4.0), dynamic, statics);
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.0);
  EXPECT_TRUE(r.speedClamped);

  // Flip the tightness: static wins.
  dynamic.maxSpeedMps = 3.0;
  statics.maxSpeedMps = 2.5;
  r = applyConstraintClamps(cvWith(4.0), dynamic, statics);
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.5);
}

TEST(ConstraintClampTest, MinSpeedRaisesOnlyNonzeroCommands) {
  ConstraintSnapshot dynamic;
  dynamic.minSpeedMps = 1.5;

  ClampResult r = applyConstraintClamps(cvWith(0.5), dynamic, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 1.5);
  EXPECT_TRUE(r.speedClamped);

  // A commanded stop/hold is never sped up.
  r = applyConstraintClamps(cvWith(0.0), dynamic, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 0.0);
  EXPECT_FALSE(r.speedClamped);
}

TEST(ConstraintClampTest, NegativeSpeedClampsMagnitude) {
  ConstraintSnapshot dynamic;
  dynamic.maxSpeedMps = 2.0;
  const ClampResult r = applyConstraintClamps(cvWith(-4.0), dynamic, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.speedMps, -2.0);
  EXPECT_TRUE(r.speedClamped);
}

TEST(ConstraintClampTest, DepthClamps) {
  ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;
  dynamic.minDepthM = 5.0;

  ClampResult r = applyConstraintClamps(cvWith(3.0, 25.0), dynamic, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 20.0);
  EXPECT_TRUE(r.elevationClamped);

  r = applyConstraintClamps(cvWith(3.0, 2.0), dynamic, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 5.0);
  EXPECT_TRUE(r.elevationClamped);

  r = applyConstraintClamps(cvWith(3.0, 10.0), dynamic, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 10.0);
  EXPECT_FALSE(r.elevationClamped);
}

TEST(ConstraintClampTest, NonDepthFramesPassThrough) {
  ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;
  ControlVector cv = cvWith(3.0, 50.0);
  cv.elevationFrame = ElevationFrame::ALTITUDE_ASF;

  const ClampResult r = applyConstraintClamps(cv, dynamic, ClampLimits{});
  EXPECT_DOUBLE_EQ(r.cv.elevationM.value(), 50.0);
  EXPECT_FALSE(r.elevationClamped);
}

TEST(ConstraintClampTest, MissingElevationUntouched) {
  ConstraintSnapshot dynamic;
  dynamic.maxDepthM = 20.0;
  const ClampResult r = applyConstraintClamps(cvWith(3.0), dynamic, ClampLimits{});
  EXPECT_FALSE(r.cv.elevationM.has_value());
  EXPECT_FALSE(r.elevationClamped);
}

TEST(ConstraintClampTest, ConflictingBoundsClampToMaxAndFlag) {
  ConstraintSnapshot dynamic;
  dynamic.minSpeedMps = 4.0;
  ClampLimits statics;
  statics.maxSpeedMps = 2.0;

  const ClampResult r = applyConstraintClamps(cvWith(3.0), dynamic, statics);
  EXPECT_TRUE(r.conflict);
  EXPECT_DOUBLE_EQ(r.cv.speedMps, 2.0);  // the max bound wins
}

}  // namespace arlcore::autopilot
