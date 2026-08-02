#include <gtest/gtest.h>

#include <cstdint>

#include "InternalTypes.h"
#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/guidance/PlannerParamsFactory.hpp"

TEST(PlannerParamsFactoryTest, EveryConfiguredPlannerFieldReachesThePlannerParams) {
  // GIVEN: a config with every mapped field set to a distinct non-default value
  arlcore::autopilot::AutopilotConfig config;
  config.planner.leadDistanceM = 61.0;
  config.waypointTolerances.positionM = 3.5;
  config.waypointTolerances.yawRad = 0.21;
  config.waypointTolerances.elevationM = 1.5;
  config.planner.maxMissesPerWaypoint = 5;
  config.planner.elevationCountsAsMiss = false;
  config.planner.maxReplans = 7;
  config.planner.sampleStepM = 3.25;
  config.planner.tracker.headingLoopTauS = 0.83;
  config.planner.tracker.feedforwardLimitRad = 0.55;
  config.planner.tracker.crossTrackApproachRad = 0.44;
  config.planner.tracker.crossTrackGainPerM = 0.22;
  config.planner.xte.kpScale = 1.7;
  config.planner.xte.ki = 0.033;
  config.planner.xte.integratorLimitRad = 0.28;
  config.planner.xte.integratorGateM = 6.5;
  config.planner.xte.correctionLimitRad = 1.05;
  config.zones.safetyMarginM = 8.5;
  config.planner.rrt.seed = 999;
  config.planner.rrt.maxIterations = 1234;
  config.planner.rrt.timeBudgetMs = 33;
  config.planner.rrt.goalBias = 0.17;
  config.planner.rrt.nearK = 11;
  config.planner.rrt.edgeCheckStepM = 5.5;
  config.planner.rrt.finalCheckStepM = 0.75;

  // WHEN: the planner params are derived
  const arlcore::autopilot::PlannerParams p = arlcore::autopilot::derivePlannerParams(config);

  // THEN: every field arrives. This mapping is a long transcription, and a copy-paste slip in it
  //       is invisible at runtime: the planner simply runs on a default nobody chose.
  EXPECT_DOUBLE_EQ(p.leadDistanceM, 61.0);
  EXPECT_DOUBLE_EQ(p.posCaptureM, 3.5);
  EXPECT_DOUBLE_EQ(p.yawCaptureRad, 0.21);
  EXPECT_DOUBLE_EQ(p.elevCaptureM, 1.5);
  EXPECT_EQ(p.maxMissesPerWaypoint, 5);
  EXPECT_FALSE(p.elevationCountsAsMiss);
  EXPECT_EQ(p.maxReplans, 7);
  EXPECT_DOUBLE_EQ(p.sampleStepM, 3.25);
  EXPECT_DOUBLE_EQ(p.tracker.headingLoopTauS, 0.83);
  EXPECT_DOUBLE_EQ(p.tracker.feedforwardLimitRad, 0.55);
  EXPECT_DOUBLE_EQ(p.tracker.crossTrackApproachRad, 0.44);
  EXPECT_DOUBLE_EQ(p.tracker.crossTrackGainPerM, 0.22);
  EXPECT_DOUBLE_EQ(p.tracker.xte.kpScale, 1.7);
  EXPECT_DOUBLE_EQ(p.tracker.xte.ki, 0.033);
  EXPECT_DOUBLE_EQ(p.tracker.xte.integratorLimitRad, 0.28);
  EXPECT_DOUBLE_EQ(p.tracker.xte.integratorGateM, 6.5);
  EXPECT_DOUBLE_EQ(p.tracker.xte.correctionLimitRad, 1.05);
  EXPECT_DOUBLE_EQ(p.zoneMarginM, 8.5);
  EXPECT_EQ(p.rrt.seed, 999u);
  EXPECT_EQ(p.rrt.maxIterations, 1234);
  EXPECT_EQ(p.rrt.timeBudgetMs, 33);
  EXPECT_DOUBLE_EQ(p.rrt.goalBias, 0.17);
  EXPECT_EQ(p.rrt.nearK, 11);
  EXPECT_DOUBLE_EQ(p.rrt.edgeCheckStepM, 5.5);
  EXPECT_DOUBLE_EQ(p.rrt.finalCheckStepM, 0.75);
}

TEST(PlannerParamsFactoryTest, TheTurnRadiusIsTheKinematicMinimumInflatedByTheMargin) {
  // GIVEN: the shipped operating point
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.2618;
  config.planner.turnRadiusMargin = 1.25;

  // WHEN: the params are derived
  const arlcore::autopilot::PlannerParams p = arlcore::autopilot::derivePlannerParams(config);

  // THEN: the radius is margin * speed / turn rate, derived here rather than transcribed. Every
  //       tracking threshold and the whole preview window scale off this number.
  EXPECT_DOUBLE_EQ(p.turnRadiusM, 1.25 * 3.0 / 0.2618);
}

TEST(PlannerParamsFactoryTest, CruisingSpeedIsPreferredOverMaxForwardSpeed) {
  // GIVEN: a platform that can sprint faster than it cruises
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.surface.maxForwardSpeedMps = 6.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.2618;
  config.planner.turnRadiusMargin = 1.0;

  // WHEN: derived without and then with a cruising speed
  const flt64_t withoutCruise = arlcore::autopilot::derivePlannerParams(config).turnRadiusM;
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  const flt64_t withCruise = arlcore::autopilot::derivePlannerParams(config).turnRadiusM;

  // THEN: cruising speed wins when present. Planning turns at max speed on a platform that
  //       cruises slower reserves margin the tracker never needs, and widens every arc.
  EXPECT_DOUBLE_EQ(withoutCruise, 6.0 / 0.2618);
  EXPECT_DOUBLE_EQ(withCruise, 3.0 / 0.2618);
}

TEST(PlannerParamsFactoryTest, AMarginBelowOneStillFloorsAtTheKinematicMinimum) {
  // GIVEN: a margin below 1.0, which validation warns about but accepts
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.2618;
  config.planner.turnRadiusMargin = 0.5;

  // WHEN: the params are derived
  const arlcore::autopilot::PlannerParams p = arlcore::autopilot::derivePlannerParams(config);

  // THEN: the planned radius is the kinematic minimum, not half of it. A warned-but-accepted
  //       config must still not plan a turn the platform physically cannot fly.
  EXPECT_DOUBLE_EQ(p.turnRadiusM, 3.0 / 0.2618);
}

TEST(PlannerParamsFactoryTest, AbsentCapabilitiesFallBackToTheConservativeRadius) {
  // GIVEN: configs missing each of the capabilities the radius derives from
  // WHEN: the params are derived
  // THEN: the radius is the conservative 25 m in every case. AutopilotApp::initialize rejects
  //       these configs, but mission_runner and mission_console call this factory directly and
  //       were never subject to that check, so the fallback is reachable in the tools.
  arlcore::autopilot::AutopilotConfig noSpeed;
  noSpeed.platformCapabilities.surface.maxTurnRateRps = 0.2618;
  EXPECT_DOUBLE_EQ(arlcore::autopilot::derivePlannerParams(noSpeed).turnRadiusM, 25.0);

  arlcore::autopilot::AutopilotConfig noRate;
  noRate.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  EXPECT_DOUBLE_EQ(arlcore::autopilot::derivePlannerParams(noRate).turnRadiusM, 25.0);

  arlcore::autopilot::AutopilotConfig zeroRate;
  zeroRate.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  zeroRate.platformCapabilities.surface.maxTurnRateRps = 0.0;
  EXPECT_DOUBLE_EQ(arlcore::autopilot::derivePlannerParams(zeroRate).turnRadiusM, 25.0);
}

TEST(PlannerParamsFactoryTest, UnderwaterDepthRateIsCarriedOnlyWhenTheRegimeIsEnabled) {
  // GIVEN: an underwater depth-change rate with the regime disabled, then enabled
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.underwater.maxDepthChangeRateMps = 0.4;

  // WHEN: derived either way
  const flt64_t disabled = arlcore::autopilot::derivePlannerParams(config).maxDepthRateMps;
  config.platformCapabilities.underwaterEnabled = true;
  const flt64_t enabled = arlcore::autopilot::derivePlannerParams(config).maxDepthRateMps;

  // THEN: only an enabled regime supplies a depth rate, so a surface platform's spiral budget is
  //       not sized off a limit it never declared it would use
  EXPECT_DOUBLE_EQ(disabled, 0.0);
  EXPECT_DOUBLE_EQ(enabled, 0.4);
}
