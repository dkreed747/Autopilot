#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/config/YamlConfigLoader.hpp"

static const char* const kTestConfigPath = "autopilot_test_config.yaml";

static void writeConfig(const std::string& contents) {
  std::ofstream out(kTestConfigPath);
  out << contents;
  out.close();
}

class YamlConfigLoaderTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(kTestConfigPath); }
};

TEST_F(YamlConfigLoaderTest, MissingFileReturnsFalse) {
  // GIVEN: a path with no file on disk
  arlcore::autopilot::AutopilotConfig config;

  // WHEN: the config is loaded from that path
  // THEN: loading reports failure
  EXPECT_FALSE(arlcore::autopilot::YamlConfigLoader::load("does_not_exist_12345.yaml", &config));
}

TEST_F(YamlConfigLoaderTest, LoadsCoreFields) {
  // GIVEN: a config file setting dds, identity, arbitration, planner, and
  // platform capability fields
  writeConfig(
      "dds:\n"
      "  domain_id: 7\n"
      "identity:\n"
      "  vector_source_id: \"aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa\"\n"
      "  waypoint_source_id: \"bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb\"\n"
      "arbitration:\n"
      "  local:\n"
      "    vector_priority: 50\n"
      "    waypoint_priority: 5\n"
      "  remote:\n"
      "    vector_priority: 700\n"
      "    waypoint_priority: 600\n"
      "planner:\n"
      "  max_misses_per_waypoint: 4\n"
      "  elevation_counts_as_miss: false\n"
      "platform_capabilities:\n"
      "  surface:\n"
      "    max_forward_speed_mps: 8.5\n"
      "    max_turn_rate_rps: 0.3\n");

  // WHEN: the config is loaded
  arlcore::autopilot::AutopilotConfig config;
  ASSERT_TRUE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config));

  // THEN: every field round-trips
  EXPECT_EQ(config.dds.domainId, 7);
  EXPECT_EQ(config.identity.vectorSourceId, "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa");
  EXPECT_EQ(config.identity.waypointSourceId, "bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb");
  EXPECT_EQ(config.arbitration.local.vectorPriority, 50);
  EXPECT_EQ(config.arbitration.local.waypointPriority, 5);
  EXPECT_EQ(config.arbitration.remote.vectorPriority, 700);
  EXPECT_EQ(config.arbitration.remote.waypointPriority, 600);
  EXPECT_EQ(config.planner.maxMissesPerWaypoint, 4);
  EXPECT_FALSE(config.planner.elevationCountsAsMiss);
  ASSERT_TRUE(config.platformCapabilities.surface.maxForwardSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(config.platformCapabilities.surface.maxForwardSpeedMps.value(), 8.5);
  ASSERT_TRUE(config.platformCapabilities.surface.maxTurnRateRps.has_value());
  EXPECT_DOUBLE_EQ(config.platformCapabilities.surface.maxTurnRateRps.value(), 0.3);
}

TEST_F(YamlConfigLoaderTest, UnsetFieldsKeepDefaults) {
  // GIVEN: a config file that sets only the DDS domain
  writeConfig("dds:\n  domain_id: 3\n");

  // WHEN: the config is loaded
  arlcore::autopilot::AutopilotConfig config;
  ASSERT_TRUE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config));

  // THEN: the domain applies and untouched fields retain their struct defaults
  EXPECT_EQ(config.dds.domainId, 3);
  EXPECT_EQ(config.arbitration.local.vectorPriority, 100);
  EXPECT_EQ(config.arbitration.local.waypointPriority, 10);
  EXPECT_EQ(config.arbitration.remote.vectorPriority, 500);
  EXPECT_EQ(config.arbitration.remote.waypointPriority, 400);
  EXPECT_EQ(config.arbitration.safePriority, 1000);
  EXPECT_TRUE(config.operationalMode.allowImplicitModeTransitions);
  EXPECT_TRUE(config.operationalMode.commandsOutOfModeAreFailed);
  EXPECT_DOUBLE_EQ(config.operationalMode.idleRevertS, 5.0);
  EXPECT_TRUE(config.identity.platformId.empty());
  EXPECT_TRUE(config.identity.operationalModeControlSourceId.empty());
  EXPECT_TRUE(config.console.platformId.empty());
  EXPECT_EQ(config.planner.maxReplans, 10);
  EXPECT_FALSE(config.platformCapabilities.surface.maxForwardSpeedMps.has_value());
  EXPECT_FALSE(config.constraints.maxSpeedMps.has_value());
  EXPECT_DOUBLE_EQ(config.zones.safetyMarginM, 5.0);
  EXPECT_DOUBLE_EQ(config.safety.gracePeriodS, 10.0);
  EXPECT_EQ(config.safety.safeMode.strategy, "srp");
  EXPECT_TRUE(config.safety.safeMode.srp.csvPath.empty());
}

TEST_F(YamlConfigLoaderTest, LoadsConstraintAndSafetyFields) {
  // GIVEN: a config file with constraint, zone, avoidance, recovery, and
  // safety blocks, including an explicit null value
  writeConfig(
      "identity:\n"
      "  constraints_source_id: \"cccccccc-cccc-cccc-cccc-cccccccccccc\"\n"
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
      "  min_speed_mps:\n"  // explicit null stays unset
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

  // WHEN: the config is loaded
  arlcore::autopilot::AutopilotConfig config;
  ASSERT_TRUE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config));

  // THEN: every field round-trips and the explicit null stays unset
  EXPECT_EQ(config.identity.constraintsSourceId, "cccccccc-cccc-cccc-cccc-cccccccccccc");
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

TEST_F(YamlConfigLoaderTest, LoadsOperationalModeAndConsoleFields) {
  // GIVEN: a config with identity platform/mode IDs, an operational_mode block, and a
  // console identity block
  writeConfig(
      "identity:\n"
      "  platform_id: \"00000000-0000-0000-0000-00000000c0fe\"\n"
      "  operational_mode_control_source_id: \"77777777-7777-7777-7777-777777777777\"\n"
      "  operational_mode_status_source_id: \"99999999-9999-9999-9999-999999999999\"\n"
      "operational_mode:\n"
      "  allow_implicit_mode_transitions: false\n"
      "  commands_out_of_mode_are_failed: false\n"
      "  idle_revert_s: 12.5\n"
      "console:\n"
      "  platform_id: \"aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa\"\n"
      "  source_id: \"bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb\"\n");

  // WHEN: the config is loaded
  arlcore::autopilot::AutopilotConfig config;
  ASSERT_TRUE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config));

  // THEN: every new field round-trips
  EXPECT_EQ(config.identity.platformId, "00000000-0000-0000-0000-00000000c0fe");
  EXPECT_EQ(config.identity.operationalModeControlSourceId, "77777777-7777-7777-7777-777777777777");
  EXPECT_EQ(config.identity.operationalModeStatusSourceId, "99999999-9999-9999-9999-999999999999");
  EXPECT_FALSE(config.operationalMode.allowImplicitModeTransitions);
  EXPECT_FALSE(config.operationalMode.commandsOutOfModeAreFailed);
  EXPECT_DOUBLE_EQ(config.operationalMode.idleRevertS, 12.5);
  EXPECT_EQ(config.console.platformId, "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa");
  EXPECT_EQ(config.console.sourceId, "bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb");
}

TEST_F(YamlConfigLoaderTest, LegacyFlatArbitrationKeysAreIgnored) {
  // GIVEN: a config still using the pre-operational-mode flat arbitration schema
  writeConfig(
      "arbitration:\n"
      "  vector_priority: 42\n"
      "  waypoint_priority: 7\n"
      "  safe_priority: 800\n");

  // WHEN: the config is loaded
  arlcore::autopilot::AutopilotConfig config;
  ASSERT_TRUE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config));

  // THEN: the obsolete flat keys are ignored (warned), safe_priority still applies
  EXPECT_EQ(config.arbitration.local.vectorPriority, 100);
  EXPECT_EQ(config.arbitration.local.waypointPriority, 10);
  EXPECT_EQ(config.arbitration.safePriority, 800);
}

TEST_F(YamlConfigLoaderTest, EmptyFileYieldsValidDefaults) {
  // GIVEN: an empty config file
  writeConfig("");
  arlcore::autopilot::AutopilotConfig config;

  // WHEN: the config is loaded
  // THEN: every struct default passes validation
  EXPECT_TRUE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config));
}

TEST_F(YamlConfigLoaderTest, RejectsOutOfRangeValues) {
  // GIVEN: a table of configs that each violate one validation rule
  const char* const kBadConfigs[] = {
      "loop:\n  control_period_ms: 0\n",
      "loop:\n  control_period_ms: -50\n",
      "loop:\n  nav_staleness_timeout_ms: 0\n",
      "identity:\n  platform_id: \"not-a-uuid\"\n",
      "console:\n  platform_id: \"garbage\"\n",
      "dds:\n  domain_id: -1\n",
      "dds:\n  domain_id: 500\n",
      "zones:\n  ellipse_segments: 0\n",
      "zones:\n  ellipse_segments: -4\n",
      "zones:\n  safety_margin_m: -1.0\n",
      "tolerances:\n  waypoint_defaults:\n    position_m: -2.5\n",
      "tolerances:\n  vector:\n    speed_mps: -0.5\n",
      "planner:\n  lead_distance_m: 0\n",
      "planner:\n  turn_radius_margin: -1.0\n",
      "planner:\n  rrt:\n    max_iterations: 0\n",
      "planner:\n  rrt:\n    time_budget_ms: 0\n",
      "planner:\n  rrt:\n    goal_bias: 1.5\n",
      "constraints:\n  min_speed_mps: 3.0\n  max_speed_mps: 1.0\n",
      "constraints:\n  min_depth_m: 30.0\n  max_depth_m: 5.0\n",
      "safety:\n  grace_period_s: -1\n",
      "safety:\n  violation_confirm_ticks: 0\n",
      "safety:\n  state_report_period_ms: 0\n",
      "safety:\n  safe_mode:\n    strategy: \"banana\"\n",
      "safety:\n  safe_mode:\n    srp:\n      csv_path: \"srp.csv\"\n",
      "safety:\n  safe_mode:\n    srp:\n      csv_path: \"srp.csv\"\n"
      "      origin_lat_deg: 95.0\n      origin_lon_deg: 0.0\n",
      "vehicle_control:\n  sim:\n    cycle_rate_hz: 0\n",
      "vehicle_control:\n  sim:\n    initial_latitude_deg: 123.0\n",
      "platform_capabilities:\n  surface:\n    max_turn_rate_rps: 0\n",
      "platform_capabilities:\n  surface:\n    cruising_speed_mps: -3.0\n",
      "arbitration:\n  safe_priority: 50\n",
      "operational_mode:\n  idle_revert_s: -2\n",
      "vector_avoidance:\n  exit_clear_ticks: 0\n",
      "recovery:\n  speed_mps: -1\n",
  };

  for (const char* bad : kBadConfigs) {
    writeConfig(bad);
    arlcore::autopilot::AutopilotConfig config;

    // WHEN: the config is loaded
    // THEN: loading fails with the offending snippet reported by the test
    EXPECT_FALSE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config))
        << "config accepted but should have been rejected:\n"
        << bad;
  }
}

TEST_F(YamlConfigLoaderTest, WarnsButAcceptsMarginalValues) {
  // GIVEN: a table of configs that are suspicious but legal (warn + keep)
  const struct {
    const char* yaml;
    bool expectLoaded;
  } kMarginal[] = {
      {"planner:\n  turn_radius_margin: 0.8\n", true},
      {"zones:\n  ellipse_segments: 512\n", true},
      {"loop:\n  control_period_ms: 2000\n", true},
      {"vehicle_control:\n  type: \"hovercraft\"\n", true},
      {"safety:\n  safe_mode:\n    strategy: \"zero_speed_hold\"\n", true},
  };

  for (const auto& entry : kMarginal) {
    writeConfig(entry.yaml);
    arlcore::autopilot::AutopilotConfig config;

    // WHEN: the config is loaded
    // THEN: loading succeeds and the configured value is kept
    EXPECT_EQ(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config), entry.expectLoaded) << entry.yaml;
  }
}

TEST_F(YamlConfigLoaderTest, TypeMismatchedValueFailsLoad) {
  // GIVEN: a config whose control period is not numeric
  writeConfig("loop:\n  control_period_ms: \"fast\"\n");
  arlcore::autopilot::AutopilotConfig config;

  // WHEN: the config is loaded
  // THEN: the contained conversion error fails the load instead of aborting
  EXPECT_FALSE(arlcore::autopilot::YamlConfigLoader::load(kTestConfigPath, &config));
}
