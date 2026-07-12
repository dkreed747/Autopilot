//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
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

#include "AutopilotConfig.h"
#include "YamlConfigLoader.h"

namespace arlcore::autopilot {

namespace {
const char* kTestConfigPath = "autopilot_test_config.yaml";

void writeConfig(const std::string& contents) {
  std::ofstream out(kTestConfigPath);
  out << contents;
  out.close();
}
}  // namespace

class YamlConfigLoaderTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(kTestConfigPath); }
};

TEST_F(YamlConfigLoaderTest, MissingFileReturnsFalse) {
  AutopilotConfig config;
  EXPECT_FALSE(YamlConfigLoader::load("does_not_exist_12345.yaml", &config));
}

TEST_F(YamlConfigLoaderTest, LoadsCoreFields) {
  writeConfig(
      "dds:\n"
      "  domain_id: 7\n"
      "identity:\n"
      "  vector_source_id: \"aaaa\"\n"
      "  waypoint_source_id: \"bbbb\"\n"
      "arbitration:\n"
      "  vector_priority: 50\n"
      "  waypoint_priority: 5\n"
      "planner:\n"
      "  max_misses_per_waypoint: 4\n"
      "  elevation_counts_as_miss: false\n"
      "platform_capabilities:\n"
      "  surface:\n"
      "    max_forward_speed_mps: 8.5\n"
      "    max_turn_rate_rps: 0.3\n");

  AutopilotConfig config;
  ASSERT_TRUE(YamlConfigLoader::load(kTestConfigPath, &config));

  EXPECT_EQ(config.dds.domainId, 7);
  EXPECT_EQ(config.identity.vectorSourceId, "aaaa");
  EXPECT_EQ(config.identity.waypointSourceId, "bbbb");
  EXPECT_EQ(config.arbitration.vectorPriority, 50);
  EXPECT_EQ(config.arbitration.waypointPriority, 5);
  EXPECT_EQ(config.planner.maxMissesPerWaypoint, 4);
  EXPECT_FALSE(config.planner.elevationCountsAsMiss);
  ASSERT_TRUE(config.platformCapabilities.surface.maxForwardSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(config.platformCapabilities.surface.maxForwardSpeedMps.value(), 8.5);
  ASSERT_TRUE(config.platformCapabilities.surface.maxTurnRateRps.has_value());
  EXPECT_DOUBLE_EQ(config.platformCapabilities.surface.maxTurnRateRps.value(), 0.3);
}

TEST_F(YamlConfigLoaderTest, UnsetFieldsKeepDefaults) {
  writeConfig("dds:\n  domain_id: 3\n");

  AutopilotConfig config;
  ASSERT_TRUE(YamlConfigLoader::load(kTestConfigPath, &config));

  EXPECT_EQ(config.dds.domainId, 3);
  // Untouched fields retain their struct defaults.
  EXPECT_EQ(config.arbitration.vectorPriority, 100);
  EXPECT_EQ(config.arbitration.waypointPriority, 10);
  EXPECT_EQ(config.planner.maxReplans, 10);
  EXPECT_FALSE(config.platformCapabilities.surface.maxForwardSpeedMps.has_value());
}

}  // namespace arlcore::autopilot
