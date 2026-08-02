#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "InternalTypes.h"
#include "autopilot/guidance/ElevationUtils.hpp"

using ElevationTestPose = UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

//! \brief A pose carrying only the fields the test needs; every elevation field is optional.
static ElevationTestPose poseWith(std::optional<flt64_t> depthM, std::optional<flt64_t> asfM) {
  ElevationTestPose pose;
  pose.position().geodeticLatitude(39.0);
  pose.position().geodeticLongitude(-76.5);
  if (depthM.has_value()) {
    pose.depth() = depthM.value();
  }
  if (asfM.has_value()) {
    pose.altitudeASF() = asfM.value();
  }
  return pose;
}

TEST(ElevationUtilsTest, PoseElevationReadsTheFieldForEachFrame) {
  // GIVEN: a pose carrying every elevation field
  ElevationTestPose pose = poseWith(20.0, 30.0);
  pose.altitude() = -18.0;
  pose.altitudeAGL() = 31.0;
  pose.altitudeGeodetic() = -17.0;

  // WHEN: reading it in each frame
  // THEN: each frame reads its own field
  EXPECT_DOUBLE_EQ(
      arlcore::autopilot::elevation::poseElevation(pose, arlcore::autopilot::ElevationFrame::DEPTH).value(), 20.0);
  EXPECT_DOUBLE_EQ(
      arlcore::autopilot::elevation::poseElevation(pose, arlcore::autopilot::ElevationFrame::ALTITUDE_ASF).value(),
      30.0);
  EXPECT_DOUBLE_EQ(
      arlcore::autopilot::elevation::poseElevation(pose, arlcore::autopilot::ElevationFrame::ALTITUDE_MSL).value(),
      -18.0);
  EXPECT_DOUBLE_EQ(
      arlcore::autopilot::elevation::poseElevation(pose, arlcore::autopilot::ElevationFrame::ALTITUDE_AGL).value(),
      31.0);
  EXPECT_DOUBLE_EQ(
      arlcore::autopilot::elevation::poseElevation(pose, arlcore::autopilot::ElevationFrame::ALTITUDE_GEODETIC).value(),
      -17.0);
}

TEST(ElevationUtilsTest, PoseElevationIsAbsentWhenTheFieldIs) {
  // GIVEN: a pose carrying a depth but no altitude above the sea floor
  const ElevationTestPose pose = poseWith(20.0, std::nullopt);

  // WHEN: reading the frame the pose does not carry
  // THEN: it reads as absent rather than as zero
  EXPECT_FALSE(
      arlcore::autopilot::elevation::poseElevation(pose, arlcore::autopilot::ElevationFrame::ALTITUDE_ASF).has_value());
  EXPECT_TRUE(
      arlcore::autopilot::elevation::poseElevation(pose, arlcore::autopilot::ElevationFrame::DEPTH).has_value());
}

TEST(ElevationUtilsTest, PoseDepthAsfDefaultsDepthToTheSurface) {
  // GIVEN: poses with and without the vertical fields
  // WHEN: extracting the depth/ASF pair every zone query needs
  const arlcore::autopilot::DepthAsf both = arlcore::autopilot::elevation::poseDepthAsf(poseWith(20.0, 30.0));
  const arlcore::autopilot::DepthAsf neither =
      arlcore::autopilot::elevation::poseDepthAsf(poseWith(std::nullopt, std::nullopt));

  // THEN: a missing depth reads as the surface, and a missing ASF stays absent so it can never
  // exonerate an ASF-framed zone bound
  EXPECT_DOUBLE_EQ(both.depthM, 20.0);
  EXPECT_DOUBLE_EQ(both.asfM.value(), 30.0);
  EXPECT_DOUBLE_EQ(neither.depthM, 0.0);
  EXPECT_FALSE(neither.asfM.has_value());
}

TEST(ElevationUtilsTest, FloorDepthIsTheSumOfDepthAndAltitude) {
  // GIVEN: a pose 20 m down with 30 m of water beneath it
  // WHEN: deriving the seafloor reference
  const std::optional<flt64_t> floorM = arlcore::autopilot::elevation::floorDepthM(poseWith(20.0, 30.0));

  // THEN: the floor is 50 m down
  ASSERT_TRUE(floorM.has_value());
  EXPECT_DOUBLE_EQ(floorM.value(), 50.0);
}

TEST(ElevationUtilsTest, FloorDepthNeedsBothFieldsFiniteAndAPositiveAltitude) {
  // GIVEN: poses that cannot yield a usable reference
  const flt64_t nan = std::numeric_limits<flt64_t>::quiet_NaN();

  // WHEN: deriving the seafloor reference from each
  // THEN: a missing field, a non-finite field, or an altimeter reading exactly 0 all refuse.
  // The zero case matters most: an altimeter past its range reports 0, which would put the floor
  // at the vehicle and derive a depth ceiling shallower than the vehicle's own depth.
  EXPECT_FALSE(arlcore::autopilot::elevation::floorDepthM(poseWith(20.0, std::nullopt)).has_value());
  EXPECT_FALSE(arlcore::autopilot::elevation::floorDepthM(poseWith(std::nullopt, 30.0)).has_value());
  EXPECT_FALSE(arlcore::autopilot::elevation::floorDepthM(poseWith(nan, 30.0)).has_value());
  EXPECT_FALSE(arlcore::autopilot::elevation::floorDepthM(poseWith(20.0, nan)).has_value());
  EXPECT_FALSE(arlcore::autopilot::elevation::floorDepthM(poseWith(20.0, 0.0)).has_value());
  EXPECT_FALSE(arlcore::autopilot::elevation::floorDepthM(poseWith(20.0, -1.0)).has_value());
}

TEST(ElevationUtilsTest, TheDepthAsfFlipIsItsOwnInverse) {
  // GIVEN: a sea floor 50 m down and a setpoint 20 m down
  // WHEN: flipping into the other frame and back
  const flt64_t asfM = arlcore::autopilot::elevation::flipDepthAsf(20.0, 50.0);
  const flt64_t depthM = arlcore::autopilot::elevation::flipDepthAsf(asfM, 50.0);

  // THEN: 20 m of depth is 30 m off the bottom, and the round trip returns the original
  EXPECT_DOUBLE_EQ(asfM, 30.0);
  EXPECT_DOUBLE_EQ(depthM, 20.0);
}

TEST(ElevationUtilsTest, FrameNamesAreDistinct) {
  // GIVEN: the elevation frames
  // WHEN: naming them for logs and monitors
  // THEN: DEPTH and ASF do not collapse onto each other, which is what the tools used to do
  EXPECT_STREQ(arlcore::autopilot::elevation::frameName(arlcore::autopilot::ElevationFrame::DEPTH), "depth");
  EXPECT_STREQ(arlcore::autopilot::elevation::frameName(arlcore::autopilot::ElevationFrame::ALTITUDE_ASF), "asf");
  EXPECT_STREQ(arlcore::autopilot::elevation::frameName(arlcore::autopilot::ElevationFrame::ALTITUDE_MSL), "msl");
  EXPECT_STREQ(arlcore::autopilot::elevation::frameName(arlcore::autopilot::ElevationFrame::ALTITUDE_AGL), "agl");
  EXPECT_STREQ(arlcore::autopilot::elevation::frameName(arlcore::autopilot::ElevationFrame::ALTITUDE_GEODETIC),
               "geodetic");
}
