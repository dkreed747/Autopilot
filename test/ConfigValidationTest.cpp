#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "InternalTypes.h"
#include "autopilot/config/ConfigValidation.hpp"

//! \brief A config that passes validation cleanly, so a failure in these tests is about the rule
//! under test rather than an unrelated default.
static arlcore::autopilot::AutopilotConfig validConfig() {
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.2618;
  return config;
}

static bool validate(const arlcore::autopilot::AutopilotConfig& config, std::vector<std::string>* errors,
                     std::vector<std::string>* warnings) {
  errors->clear();
  warnings->clear();
  return arlcore::autopilot::validateConfig(config, errors, warnings);
}

static bool mentions(const std::vector<std::string>& messages, const std::string& needle) {
  for (const std::string& message : messages) {
    if (message.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

TEST(ConfigValidationTest, TheShippedDefaultsAreValid) {
  // GIVEN: a default-constructed config, which is what every absent YAML key falls back to
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  // WHEN: it is validated
  // THEN: it passes. Any struct default that cannot pass validation is unreachable in practice
  //       but would make a partially specified config fail for a reason the operator did not set.
  EXPECT_TRUE(validate(arlcore::autopilot::AutopilotConfig(), &errors, &warnings)) << errors.front();
  EXPECT_TRUE(errors.empty());
}

TEST(ConfigValidationTest, EveryFloatingPointFieldIsScreenedForFiniteness) {
  // GIVEN: a setter for every field the finiteness sweep guards, paired with the key it reports
  //
  // This table is the point of the test. NaN and infinity pass every range comparison below
  // silently, because neither orders against anything, so a field added without a finiteness
  // screen is invisible until it poisons something downstream. Keeping the table exhaustive makes
  // that omission a test failure instead.
  using Config = arlcore::autopilot::AutopilotConfig;
  const struct {
    const char* key;
    std::function<void(Config*, flt64_t)> set;
  } kFields[] = {
      {"operational_mode.idle_revert_s", [](Config* c, flt64_t v) { c->operationalMode.idleRevertS = v; }},
      {"tolerances.vector.direction_rad", [](Config* c, flt64_t v) { c->vectorTolerances.directionRad = v; }},
      {"tolerances.vector.speed_mps", [](Config* c, flt64_t v) { c->vectorTolerances.speedMps = v; }},
      {"tolerances.vector.elevation_m", [](Config* c, flt64_t v) { c->vectorTolerances.elevationM = v; }},
      {"tolerances.vector.failure_delay_s", [](Config* c, flt64_t v) { c->vectorTolerances.failureDelayS = v; }},
      {"tolerances.waypoint_defaults.position_m", [](Config* c, flt64_t v) { c->waypointTolerances.positionM = v; }},
      {"tolerances.waypoint_defaults.yaw_rad", [](Config* c, flt64_t v) { c->waypointTolerances.yawRad = v; }},
      {"tolerances.waypoint_defaults.elevation_m", [](Config* c, flt64_t v) { c->waypointTolerances.elevationM = v; }},
      {"planner.lead_distance_m", [](Config* c, flt64_t v) { c->planner.leadDistanceM = v; }},
      {"planner.turn_radius_margin", [](Config* c, flt64_t v) { c->planner.turnRadiusMargin = v; }},
      {"planner.sample_step_m", [](Config* c, flt64_t v) { c->planner.sampleStepM = v; }},
      {"planner.xte.kp_scale", [](Config* c, flt64_t v) { c->planner.xte.kpScale = v; }},
      {"planner.xte.ki", [](Config* c, flt64_t v) { c->planner.xte.ki = v; }},
      {"planner.xte.integrator_limit_rad", [](Config* c, flt64_t v) { c->planner.xte.integratorLimitRad = v; }},
      {"planner.xte.integrator_gate_m", [](Config* c, flt64_t v) { c->planner.xte.integratorGateM = v; }},
      {"planner.xte.correction_limit_rad", [](Config* c, flt64_t v) { c->planner.xte.correctionLimitRad = v; }},
      {"planner.tracker.heading_loop_tau_s", [](Config* c, flt64_t v) { c->planner.tracker.headingLoopTauS = v; }},
      {"planner.tracker.feedforward_limit_rad",
       [](Config* c, flt64_t v) { c->planner.tracker.feedforwardLimitRad = v; }},
      {"planner.tracker.cross_track_approach_rad",
       [](Config* c, flt64_t v) { c->planner.tracker.crossTrackApproachRad = v; }},
      {"planner.tracker.cross_track_gain_per_m",
       [](Config* c, flt64_t v) { c->planner.tracker.crossTrackGainPerM = v; }},
      {"planner.rrt.goal_bias", [](Config* c, flt64_t v) { c->planner.rrt.goalBias = v; }},
      {"planner.rrt.edge_check_step_m", [](Config* c, flt64_t v) { c->planner.rrt.edgeCheckStepM = v; }},
      {"planner.rrt.final_check_step_m", [](Config* c, flt64_t v) { c->planner.rrt.finalCheckStepM = v; }},
      {"constraints.max_speed_mps", [](Config* c, flt64_t v) { c->constraints.maxSpeedMps = v; }},
      {"constraints.min_speed_mps", [](Config* c, flt64_t v) { c->constraints.minSpeedMps = v; }},
      {"constraints.max_depth_m", [](Config* c, flt64_t v) { c->constraints.maxDepthM = v; }},
      {"constraints.min_depth_m", [](Config* c, flt64_t v) { c->constraints.minDepthM = v; }},
      {"constraints.min_altitude_asf_m",
       [](Config* c, flt64_t v) {
         c->platformCapabilities.reportsAltitudeAsf = true;  // else the enforceability rule fires too
         c->constraints.minAltitudeAsfM = v;
       }},
      {"zones.safety_margin_m", [](Config* c, flt64_t v) { c->zones.safetyMarginM = v; }},
      {"zones.compliance_hysteresis_m", [](Config* c, flt64_t v) { c->zones.complianceHysteresisM = v; }},
      {"zones.elevation_margin_m", [](Config* c, flt64_t v) { c->zones.elevationMarginM = v; }},
      {"vector_avoidance.lookahead_rho_factor",
       [](Config* c, flt64_t v) { c->vectorAvoidance.lookaheadRhoFactor = v; }},
      {"vector_avoidance.lookahead_speed_s", [](Config* c, flt64_t v) { c->vectorAvoidance.lookaheadSpeedS = v; }},
      {"vector_avoidance.exit_clear_factor", [](Config* c, flt64_t v) { c->vectorAvoidance.exitClearFactor = v; }},
      {"vector_avoidance.min_follow_s", [](Config* c, flt64_t v) { c->vectorAvoidance.minFollowS = v; }},
      {"recovery.speed_mps", [](Config* c, flt64_t v) { c->recovery.speedMps = v; }},
      {"recovery.complete_hold_s", [](Config* c, flt64_t v) { c->recovery.completeHoldS = v; }},
      {"safety.grace_period_s", [](Config* c, flt64_t v) { c->safety.gracePeriodS = v; }},
      {"safety.grace_overrides.zone_s", [](Config* c, flt64_t v) { c->safety.graceZoneS = v; }},
      {"safety.grace_overrides.speed_s", [](Config* c, flt64_t v) { c->safety.graceSpeedS = v; }},
      {"safety.grace_overrides.elevation_s", [](Config* c, flt64_t v) { c->safety.graceElevationS = v; }},
      {"safety.clear_hold_s", [](Config* c, flt64_t v) { c->safety.clearHoldS = v; }},
      {"safety.safe_mode.srp.origin_lat_deg", [](Config* c, flt64_t v) { c->safety.safeMode.srp.originLatDeg = v; }},
      {"safety.safe_mode.srp.origin_lon_deg", [](Config* c, flt64_t v) { c->safety.safeMode.srp.originLonDeg = v; }},
      {"safety.safe_mode.srp.hold_radius_m", [](Config* c, flt64_t v) { c->safety.safeMode.srp.holdRadiusM = v; }},
      {"safety.safe_mode.srp.reposition_speed_mps",
       [](Config* c, flt64_t v) { c->safety.safeMode.srp.repositionSpeedMps = v; }},
      {"safety.safe_mode.srp.safe_elevation_m",
       [](Config* c, flt64_t v) { c->safety.safeMode.srp.safeElevationM = v; }},
      {"vehicle_control.sim.cycle_rate_hz", [](Config* c, flt64_t v) { c->simVehicle.cycleRateHz = v; }},
      {"vehicle_control.sim.initial_latitude_deg", [](Config* c, flt64_t v) { c->simVehicle.initialLatitudeDeg = v; }},
      {"vehicle_control.sim.initial_longitude_deg",
       [](Config* c, flt64_t v) { c->simVehicle.initialLongitudeDeg = v; }},
      {"vehicle_control.sim.initial_heading_rad", [](Config* c, flt64_t v) { c->simVehicle.initialHeadingRad = v; }},
      {"vehicle_control.sim.accel_mps2", [](Config* c, flt64_t v) { c->simVehicle.accelMps2 = v; }},
      {"vehicle_control.sim.heading_gain_rps_per_rad",
       [](Config* c, flt64_t v) { c->simVehicle.headingGainRpsPerRad = v; }},
      {"vehicle_control.sim.heading_lag_s", [](Config* c, flt64_t v) { c->simVehicle.headingLagS = v; }},
      {"vehicle_control.sim.floor_depth_m", [](Config* c, flt64_t v) { c->simVehicle.floorDepthM = v; }},
      {"vehicle_control.sim.current_east_mps", [](Config* c, flt64_t v) { c->simVehicle.currentEastMps = v; }},
      {"vehicle_control.sim.current_north_mps", [](Config* c, flt64_t v) { c->simVehicle.currentNorthMps = v; }},
      {"platform_capabilities.min_water_depth_m",
       [](Config* c, flt64_t v) { c->platformCapabilities.minWaterDepthM = v; }},
  };

  // WHEN: each field in turn is set to NaN and then to infinity
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  for (const auto& field : kFields) {
    for (const flt64_t bad : {std::nan(""), std::numeric_limits<flt64_t>::infinity()}) {
      Config config = validConfig();
      field.set(&config, bad);
      // THEN: validation fails and names the offending key
      EXPECT_FALSE(validate(config, &errors, &warnings)) << field.key;
      EXPECT_TRUE(mentions(errors, field.key)) << field.key << " was not named in the errors";
    }
  }
}

TEST(ConfigValidationTest, TheBottomClearanceMustBeNonNegativeAndEnforceable) {
  // GIVEN: an ASF-capable platform with a negative bottom clearance
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig negative = validConfig();
  negative.platformCapabilities.reportsAltitudeAsf = true;
  negative.constraints.minAltitudeAsfM = -1.0;

  // WHEN: validated
  // THEN: it is rejected; a clearance below the sea floor is not a clearance
  EXPECT_FALSE(validate(negative, &errors, &warnings));
  EXPECT_TRUE(mentions(errors, "constraints.min_altitude_asf_m"));

  // GIVEN: a valid clearance on a platform that cannot report an altitude above the sea floor
  arlcore::autopilot::AutopilotConfig unenforceable = validConfig();
  unenforceable.constraints.minAltitudeAsfM = 5.0;

  // WHEN: validated
  // THEN: startup fails rather than silently ignoring a configured bottom clearance, which is the
  // worst of the three outcomes
  EXPECT_FALSE(validate(unenforceable, &errors, &warnings));
  EXPECT_TRUE(mentions(errors, "reports_altitude_asf"));

  // GIVEN: the same clearance on a platform that does report it
  arlcore::autopilot::AutopilotConfig enforceable = unenforceable;
  enforceable.platformCapabilities.reportsAltitudeAsf = true;

  // WHEN: validated
  // THEN: it passes
  EXPECT_TRUE(validate(enforceable, &errors, &warnings));
}

TEST(ConfigValidationTest, TheSimHeadingLoopKeysAreRangeChecked) {
  // GIVEN: the two sim heading-loop settings, which had no validation at all until the servo
  //        model was added and they were missed
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  // WHEN: each is set negative
  arlcore::autopilot::AutopilotConfig negativeGain = validConfig();
  negativeGain.simVehicle.headingGainRpsPerRad = -1.0;
  arlcore::autopilot::AutopilotConfig negativeLag = validConfig();
  negativeLag.simVehicle.headingLagS = -0.5;

  // THEN: both are rejected. A negative gain used to fall silently into the deadbeat branch,
  //       substituting the one plant the curvature feedforward cannot be calibrated against.
  EXPECT_FALSE(validate(negativeGain, &errors, &warnings));
  EXPECT_TRUE(mentions(errors, "heading_gain_rps_per_rad"));
  EXPECT_FALSE(validate(negativeLag, &errors, &warnings));
  EXPECT_TRUE(mentions(errors, "heading_lag_s"));

  // ... and zero is legal for both: it selects the deadbeat limiter and disables the lag.
  arlcore::autopilot::AutopilotConfig zeroed = validConfig();
  zeroed.simVehicle.headingGainRpsPerRad = 0.0;
  zeroed.simVehicle.headingLagS = 0.0;
  EXPECT_TRUE(validate(zeroed, &errors, &warnings));
}

TEST(ConfigValidationTest, ArbitrationMustLeaveSafeModeOnTop) {
  // GIVEN: a safe priority that does not outrank every command priority
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig config = validConfig();
  config.arbitration.safePriority = config.arbitration.remote.vectorPriority;

  // WHEN: validated
  // THEN: rejected. A safe-mode maneuver that loses the driving resource to an operator command
  //       is a safety system that cannot act.
  EXPECT_FALSE(validate(config, &errors, &warnings));
  EXPECT_TRUE(mentions(errors, "safe_priority"));

  // ... while remote not outranking local is only a warning, since that is a policy choice
  arlcore::autopilot::AutopilotConfig policy = validConfig();
  policy.arbitration.remote.vectorPriority = 1;
  policy.arbitration.remote.waypointPriority = 1;
  EXPECT_TRUE(validate(policy, &errors, &warnings));
  EXPECT_FALSE(warnings.empty());
}

TEST(ConfigValidationTest, ClampRangesRejectInvertedMinAndMax) {
  // GIVEN: static clamps whose minimum exceeds their maximum
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig speed = validConfig();
  speed.constraints.minSpeedMps = 3.0;
  speed.constraints.maxSpeedMps = 1.0;
  arlcore::autopilot::AutopilotConfig depth = validConfig();
  depth.constraints.minDepthM = 30.0;
  depth.constraints.maxDepthM = 5.0;

  // WHEN: validated
  // THEN: both are rejected rather than resolved silently at runtime
  EXPECT_FALSE(validate(speed, &errors, &warnings));
  EXPECT_FALSE(validate(depth, &errors, &warnings));
}

TEST(ConfigValidationTest, TheSafeModeStrategyIsRestrictedToWhatTheFactoryImplements) {
  // GIVEN: each strategy name the factory implements, and one it does not
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  // WHEN: each is validated
  // THEN: only the implemented names pass. This is the sibling of the vehicle_control.type rule:
  //       a config naming a strategy nothing can build must not reach startup.
  for (const std::string& strategy : {std::string("srp"), std::string("zero_speed_hold")}) {
    arlcore::autopilot::AutopilotConfig config = validConfig();
    config.safety.safeMode.strategy = strategy;
    validate(config, &errors, &warnings);
    EXPECT_FALSE(mentions(errors, "safe_mode.strategy")) << strategy;
  }
  arlcore::autopilot::AutopilotConfig bad = validConfig();
  bad.safety.safeMode.strategy = "banana";
  EXPECT_FALSE(validate(bad, &errors, &warnings));
  EXPECT_TRUE(mentions(errors, "safe_mode.strategy"));
}

TEST(ConfigValidationTest, AnSrpCsvWithoutAnExplicitOriginIsRejected) {
  // GIVEN: an SRP CSV with no origin, and the same with one
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig noOrigin = validConfig();
  noOrigin.safety.safeMode.srp.csvPath = "srp.csv";

  // WHEN: validated
  // THEN: rejected. A safe-return path whose origin floats with the first GPS fix describes a
  //       different piece of water on every boot.
  EXPECT_FALSE(validate(noOrigin, &errors, &warnings));

  arlcore::autopilot::AutopilotConfig withOrigin = noOrigin;
  withOrigin.safety.safeMode.srp.originLatDeg = 39.0;
  withOrigin.safety.safeMode.srp.originLonDeg = -76.5;
  EXPECT_TRUE(validate(withOrigin, &errors, &warnings));
}

TEST(ConfigValidationTest, RemovedKeysAreErrorsAndUnknownKeysAreOnlyWarnings) {
  // GIVEN: a config carrying a removed key, and one carrying an unrecognised key
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig removed = validConfig();
  removed.removedKeys.push_back("planner.tracker.trim_gain: the self-calibrating trim was removed");
  arlcore::autopilot::AutopilotConfig unknown = validConfig();
  unknown.unknownKeys.push_back("planner.tracker.heading_loop_tau");

  // WHEN: each is validated
  // THEN: a removed key stops the load, because honouring a config that describes a control law
  //       the software no longer implements is worse than refusing it; an unknown key only warns,
  //       because a gap in the key registry must never be able to ground the vehicle.
  EXPECT_FALSE(validate(removed, &errors, &warnings));
  EXPECT_TRUE(mentions(errors, "trim_gain"));
  EXPECT_TRUE(validate(unknown, &errors, &warnings));
  EXPECT_TRUE(mentions(warnings, "heading_loop_tau"));
}

TEST(ConfigValidationTest, WarningsAreAllCollectedAndNoneFailTheLoad) {
  // GIVEN: a config tripping several warn-only rules at once
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig config = validConfig();
  config.planner.turnRadiusMargin = 0.8;
  config.zones.ellipseSegments = 512;
  config.loop.controlPeriodMs = 2000;
  config.console.platformId = config.identity.platformId;

  // WHEN: validated
  // THEN: the load succeeds and every rule reports, rather than short-circuiting at the first one
  EXPECT_TRUE(validate(config, &errors, &warnings));
  EXPECT_TRUE(errors.empty());
  EXPECT_GE(warnings.size(), 3u);
}

TEST(ConfigValidationTest, TrackerAuthorityBoundsAreEnforced) {
  // GIVEN: tracker settings outside their legal ranges
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  arlcore::autopilot::AutopilotConfig zeroFf = validConfig();
  zeroFf.planner.tracker.feedforwardLimitRad = 0.0;
  EXPECT_FALSE(validate(zeroFf, &errors, &warnings));

  arlcore::autopilot::AutopilotConfig negativeTau = validConfig();
  negativeTau.planner.tracker.headingLoopTauS = -0.1;
  EXPECT_FALSE(validate(negativeTau, &errors, &warnings));

  // WHEN: the approach angle reaches or passes pi/2
  arlcore::autopilot::AutopilotConfig wideApproach = validConfig();
  wideApproach.planner.tracker.crossTrackApproachRad = M_PI_2;
  // THEN: rejected: beyond a right angle the correction points the command across the path rather
  //       than back toward it
  EXPECT_FALSE(validate(wideApproach, &errors, &warnings));

  arlcore::autopilot::AutopilotConfig zeroGain = validConfig();
  zeroGain.planner.tracker.crossTrackGainPerM = 0.0;
  EXPECT_FALSE(validate(zeroGain, &errors, &warnings));
}

TEST(ConfigValidationTest, ACorrectionLimitBelowTheApproachPlusIntegralWarns) {
  // GIVEN: a total-correction clamp that sits below the sum of the two terms it clamps
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig config = validConfig();
  config.planner.tracker.crossTrackApproachRad = 0.6;
  config.planner.xte.integratorLimitRad = 0.35;
  config.planner.xte.correctionLimitRad = 0.5;

  // WHEN: validated
  // THEN: it loads but warns. The approach angle would otherwise be silently capped below the
  //       value the operator set, which looks like a tuning failure rather than a config one.
  EXPECT_TRUE(validate(config, &errors, &warnings));
  EXPECT_TRUE(mentions(warnings, "correction_limit_rad"));
}

TEST(ConfigValidationTest, AnUnreadableQosFileWarnsButDoesNotBlockStartup) {
  // GIVEN: a QoS profile path that does not exist
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  arlcore::autopilot::AutopilotConfig config = validConfig();
  config.dds.qosFile = "definitely-not-here.xml";

  // WHEN: validated
  // THEN: it warns loudly but still loads, because DDS has a working default and refusing to boot
  //       over a QoS file would be worse than running with reduced delivery guarantees
  EXPECT_TRUE(validate(config, &errors, &warnings));
  EXPECT_TRUE(mentions(warnings, "qos_file"));
}
