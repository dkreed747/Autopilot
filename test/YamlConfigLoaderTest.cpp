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
  EXPECT_EQ(config.arbitration.safePriority, 1000);
  EXPECT_EQ(config.planner.maxReplans, 10);
  EXPECT_FALSE(config.platformCapabilities.surface.maxForwardSpeedMps.has_value());
  EXPECT_FALSE(config.constraints.maxSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(config.zones.safetyMarginM, 5.0);
  EXPECT_DOUBLE_EQ(config.safety.gracePeriodS, 10.0);
  EXPECT_EQ(config.safety.safeMode.strategy, "srp");
  EXPECT_TRUE(config.safety.safeMode.srp.csvPath.empty());
}

TEST_F(YamlConfigLoaderTest, LoadsConstraintAndSafetyFields) {
  writeConfig(
      "identity:\n"
      "  constraints_source_id: \"cccc\"\n"
      "arbitration:\n"
      "  safe_priority: 900\n"
      "planner:\n"
      "  rrt:\n"
      "    seed: 42\n"
      "    max_iterations: 500\n"
      "    time_budget_ms: 75\n"
      "    goal_bias: 0.2\n"
      "constraints:\n"
      "  max_speed_mps: 4.5\n"
      "  min_speed_mps:\n"       // explicit null stays unset
      "  max_depth_m: 25.0\n"
      "zones:\n"
      "  safety_margin_m: 8.0\n"
      "  compliance_hysteresis_m: 3.0\n"
      "  ellipse_segments: 16\n"
      "vector_avoidance:\n"
      "  lookahead_rho_factor: 2.0\n"
      "  exit_clear_ticks: 20\n"
      "recovery:\n"
      "  speed_mps: 1.2\n"
      "safety:\n"
      "  grace_period_s: 0.0\n"
      "  grace_overrides:\n"
      "    zone_s: 5.0\n"
      "  violation_confirm_ticks: 3\n"
      "  clear_hold_s: 4.0\n"
      "  exit_on_all_clear: false\n"
      "  state_report_period_ms: 500\n"
      "  safe_mode:\n"
      "    strategy: zero_speed_hold\n"
      "    srp:\n"
      "      csv_path: \"srp.csv\"\n"
      "      origin_lat_deg: 39.5\n"
      "      origin_lon_deg: -76.25\n"
      "      accept_commands_after_srp: false\n"
      "      hold_radius_m: 15.0\n"
      "      reposition_speed_mps: 2.0\n"
      "      safe_elevation_m: 3.0\n");

  AutopilotConfig config;
  ASSERT_TRUE(YamlConfigLoader::load(kTestConfigPath, &config));

  EXPECT_EQ(config.identity.constraintsSourceId, "cccc");
  EXPECT_EQ(config.arbitration.safePriority, 900);
  EXPECT_EQ(config.planner.rrt.seed, 42u);
  EXPECT_EQ(config.planner.rrt.maxIterations, 500);
  EXPECT_EQ(config.planner.rrt.timeBudgetMs, 75);
  EXPECT_DOUBLE_EQ(config.planner.rrt.goalBias, 0.2);
  ASSERT_TRUE(config.constraints.maxSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(config.constraints.maxSpeedMps.value(), 4.5);
  EXPECT_FALSE(config.constraints.minSpeedMps.has_value());
  ASSERT_TRUE(config.constraints.maxDepthM.has_value());
  EXPECT_DOUBLE_EQ(config.constraints.maxDepthM.value(), 25.0);
  EXPECT_DOUBLE_EQ(config.zones.safetyMarginM, 8.0);
  EXPECT_DOUBLE_EQ(config.zones.complianceHysteresisM, 3.0);
  EXPECT_EQ(config.zones.ellipseSegments, 16);
  EXPECT_DOUBLE_EQ(config.vectorAvoidance.lookaheadRhoFactor, 2.0);
  EXPECT_EQ(config.vectorAvoidance.exitClearTicks, 20);
  EXPECT_DOUBLE_EQ(config.recovery.speedMps, 1.2);
  EXPECT_DOUBLE_EQ(config.safety.gracePeriodS, 0.0);
  ASSERT_TRUE(config.safety.graceZoneS.has_value());
  EXPECT_DOUBLE_EQ(config.safety.graceZoneS.value(), 5.0);
  EXPECT_FALSE(config.safety.graceSpeedS.has_value());
  EXPECT_EQ(config.safety.violationConfirmTicks, 3);
  EXPECT_DOUBLE_EQ(config.safety.clearHoldS, 4.0);
  EXPECT_FALSE(config.safety.exitOnAllClear);
  EXPECT_EQ(config.safety.stateReportPeriodMs, 500);
  EXPECT_EQ(config.safety.safeMode.strategy, "zero_speed_hold");
  EXPECT_EQ(config.safety.safeMode.srp.csvPath, "srp.csv");
  ASSERT_TRUE(config.safety.safeMode.srp.originLatDeg.has_value());
  EXPECT_DOUBLE_EQ(config.safety.safeMode.srp.originLatDeg.value(), 39.5);
  EXPECT_FALSE(config.safety.safeMode.srp.acceptCommandsAfterSrp);
  EXPECT_DOUBLE_EQ(config.safety.safeMode.srp.holdRadiusM, 15.0);
  EXPECT_DOUBLE_EQ(config.safety.safeMode.srp.repositionSpeedMps, 2.0);
  ASSERT_TRUE(config.safety.safeMode.srp.safeElevationM.has_value());
  EXPECT_DOUBLE_EQ(config.safety.safeMode.srp.safeElevationM.value(), 3.0);
}

}  // namespace arlcore::autopilot
