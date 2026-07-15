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

#include <cstdio>
#include <fstream>
#include <string>

#include "SafeReturnPath.h"

namespace arlcore::autopilot {

namespace {

const char* kCsvPath = "srp_test_mission.csv";

void writeCsv(const std::string& contents) {
  std::ofstream out(kCsvPath);
  out << contents;
  out.close();
}

SrpConfig configWithCsv() {
  SrpConfig config;
  config.csvPath = kCsvPath;
  config.originLatDeg = 39.0;
  config.originLonDeg = -76.5;
  return config;
}

}  // namespace

class SafeReturnPathTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(kCsvPath); }
};

TEST_F(SafeReturnPathTest, NoCsvConfiguredIsNotAnError) {
  SrpConfig config;  // empty csvPath
  std::string error;
  EXPECT_FALSE(SafeReturnPath::load(config, &error).has_value());
  EXPECT_TRUE(error.empty());
}

TEST_F(SafeReturnPathTest, LoadsWaypointsAnchoredAtExplicitOrigin) {
  writeCsv(
      "east_m,north_m,speed_mps,capture_radius_m\n"
      "0,0,2.0,3.0\n"
      "100,0,2.0,3.0\n"
      "100,200,1.5,5.0\n");
  std::string error;
  const auto srp = SafeReturnPath::load(configWithCsv(), &error);
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
  writeCsv("0,0,2.0,3.0\n");
  SrpConfig config = configWithCsv();
  config.originLatDeg.reset();
  std::string error;
  EXPECT_FALSE(SafeReturnPath::load(config, &error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST_F(SafeReturnPathTest, MissingCsvFileIsAnError) {
  SrpConfig config = configWithCsv();
  config.csvPath = "does_not_exist_9876.csv";
  std::string error;
  EXPECT_FALSE(SafeReturnPath::load(config, &error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST_F(SafeReturnPathTest, NonPositiveSpeedIsAnError) {
  writeCsv("0,0,0.0,3.0\n");
  std::string error;
  EXPECT_FALSE(SafeReturnPath::load(configWithCsv(), &error).has_value());
  EXPECT_NE(error.find("speed"), std::string::npos);
}

TEST_F(SafeReturnPathTest, BadHoldRadiusIsAnError) {
  writeCsv("0,0,2.0,3.0\n");
  SrpConfig config = configWithCsv();
  config.holdRadiusM = 0.0;
  std::string error;
  EXPECT_FALSE(SafeReturnPath::load(config, &error).has_value());
  EXPECT_NE(error.find("hold_radius"), std::string::npos);
}

}  // namespace arlcore::autopilot
