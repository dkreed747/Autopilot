#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "autopilot/safety/SafeReturnPath.hpp"

static const char* kCsvPath = "srp_test_mission.csv";

static void writeCsv(const std::string& contents) {
  std::ofstream out(kCsvPath);
  out << contents;
  out.close();
}

static arlcore::autopilot::SrpConfig configWithCsv() {
  arlcore::autopilot::SrpConfig config;
  config.csvPath = kCsvPath;
  config.originLatDeg = 39.0;
  config.originLonDeg = -76.5;
  return config;
}

class SafeReturnPathTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(kCsvPath); }
};

TEST_F(SafeReturnPathTest, NoCsvConfiguredIsNotAnError) {
  // GIVEN: a config with an empty csvPath
  arlcore::autopilot::SrpConfig config;  // empty csvPath
  std::string error;

  // WHEN: loading the safe return path
  // THEN: nothing loads and no error is reported
  EXPECT_FALSE(arlcore::autopilot::SafeReturnPath::load(config, &error).has_value());
  EXPECT_TRUE(error.empty());
}

TEST_F(SafeReturnPathTest, LoadsWaypointsAnchoredAtExplicitOrigin) {
  // GIVEN: a three-waypoint CSV and a config with an explicit origin
  writeCsv(
      "east_m,north_m,speed_mps,capture_radius_m\n"
      "0,0,2.0,3.0\n"
      "100,0,2.0,3.0\n"
      "100,200,1.5,5.0\n");
  std::string error;

  // WHEN: loading the safe return path
  const auto srp = arlcore::autopilot::SafeReturnPath::load(configWithCsv(), &error);

  // THEN: three waypoints load, anchored at the configured origin
  ASSERT_TRUE(srp.has_value()) << error;
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(srp->waypoints().size(), 3u);

  // The first waypoint (0, 0) sits exactly at the configured origin.
  EXPECT_NEAR(srp->waypoints().front().latDeg, 39.0, 1e-9);
  EXPECT_NEAR(srp->waypoints().front().lonDeg, -76.5, 1e-9);
  // 200 m north raises latitude by roughly 0.0018 degrees.
  EXPECT_NEAR(srp->lastWaypoint().latDeg, 39.0018, 3e-4);
  EXPECT_DOUBLE_EQ(srp->lastWaypoint().speedMps, 1.5);
  EXPECT_DOUBLE_EQ(srp->lastWaypoint().captureRadiusM, 5.0);
}

TEST_F(SafeReturnPathTest, MissingOriginIsAnError) {
  // GIVEN: a valid CSV but a config with no origin latitude
  writeCsv("0,0,2.0,3.0\n");
  arlcore::autopilot::SrpConfig config = configWithCsv();
  config.originLatDeg.reset();
  std::string error;

  // WHEN: loading the safe return path
  // THEN: the load fails with an error
  EXPECT_FALSE(arlcore::autopilot::SafeReturnPath::load(config, &error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST_F(SafeReturnPathTest, MissingCsvFileIsAnError) {
  // GIVEN: a config pointing at a CSV file that does not exist
  arlcore::autopilot::SrpConfig config = configWithCsv();
  config.csvPath = "does_not_exist_9876.csv";
  std::string error;

  // WHEN: loading the safe return path
  // THEN: the load fails with an error
  EXPECT_FALSE(arlcore::autopilot::SafeReturnPath::load(config, &error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST_F(SafeReturnPathTest, NonPositiveSpeedIsAnError) {
  // GIVEN: a CSV whose waypoint speed is zero
  writeCsv("0,0,0.0,3.0\n");
  std::string error;

  // WHEN: loading the safe return path
  // THEN: the load fails and the error names the speed
  EXPECT_FALSE(arlcore::autopilot::SafeReturnPath::load(configWithCsv(), &error).has_value());
  EXPECT_NE(error.find("speed"), std::string::npos);
}

TEST_F(SafeReturnPathTest, BadHoldRadiusIsAnError) {
  // GIVEN: a valid CSV but a config with a zero hold radius
  writeCsv("0,0,2.0,3.0\n");
  arlcore::autopilot::SrpConfig config = configWithCsv();
  config.holdRadiusM = 0.0;
  std::string error;

  // WHEN: loading the safe return path
  // THEN: the load fails and the error names the hold radius
  EXPECT_FALSE(arlcore::autopilot::SafeReturnPath::load(config, &error).has_value());
  EXPECT_NE(error.find("hold_radius"), std::string::npos);
}

TEST_F(SafeReturnPathTest, AnUnknownElevationFrameIsRejectedRatherThanReadAsDepth) {
  // GIVEN: an SRP row whose elevation frame is not one the loader knows
  //
  // This used to fall through to DEPTH, so a row authored to mean "3 m above the sea floor"
  // silently became "3 m below the surface" and the safe return path surfaced the vehicle during
  // the exact scenario it exists for. Refusing to load is the only safe reading of a safety
  // artifact whose units are ambiguous.
  writeCsv(
      "east_m,north_m,speed_mps,capture_radius_m,arrival_yaw_rad,elev_value_m,elev_frame\n"
      "0,0,2.0,5.0,,3.0,altitude\n");
  std::string error;

  // WHEN: loading
  // THEN: it is refused, with the offending frame named
  EXPECT_FALSE(arlcore::autopilot::SafeReturnPath::load(configWithCsv(), &error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST_F(SafeReturnPathTest, TheElevationFrameIsCaseAndWhitespaceInsensitive) {
  // GIVEN: the same route written with an uppercase frame and Windows line endings
  writeCsv(
      "east_m,north_m,speed_mps,capture_radius_m,arrival_yaw_rad,elev_value_m,elev_frame\r\n"
      "0,0,2.0,5.0,,3.0,ASF\r\n");
  std::string error;

  // WHEN: loading
  // THEN: it loads. A CSV saved on Windows, or with the frame capitalised, must not silently
  //       change what the safe return path means.
  const std::optional<arlcore::autopilot::SafeReturnPath> srp =
      arlcore::autopilot::SafeReturnPath::load(configWithCsv(), &error);
  ASSERT_TRUE(srp.has_value()) << error;
  ASSERT_EQ(srp->waypoints().size(), 1u);
  EXPECT_EQ(srp->waypoints().front().elevFrame, "asf");
}

TEST_F(SafeReturnPathTest, ANonFiniteElevationIsRejected) {
  // GIVEN: an SRP row whose elevation parses but is not finite. std::stod accepts "inf" and "nan"
  //        without throwing, so the malformed-row catch does not see them.
  writeCsv(
      "east_m,north_m,speed_mps,capture_radius_m,arrival_yaw_rad,elev_value_m,elev_frame\n"
      "0,0,2.0,5.0,,inf,depth\n");
  std::string error;

  // WHEN: loading
  // THEN: it is refused. An infinite elevation would reach the control vector and every safe-mode
  //       command would then be substituted with a hold, so safe mode would actuate nothing while
  //       the supervisor believed the strategy was running.
  EXPECT_FALSE(arlcore::autopilot::SafeReturnPath::load(configWithCsv(), &error).has_value());
  EXPECT_FALSE(error.empty());
}
