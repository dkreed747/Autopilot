#include <gtest/gtest.h>

#include "autopilot/umaa/PlatformReportFactory.hpp"

TEST(PlatformReportFactoryTest, MapsEverySpecsField) {
  // GIVEN: a fully populated platform_specs config
  arlcore::autopilot::PlatformSpecsConfig specs;
  specs.name = "test-vehicle";
  specs.lengthAtWaterlineM = 5.0;
  specs.beamAtWaterlineM = 1.0;
  specs.draftM = 0.5;
  specs.forwardDistanceM = 2.5;
  specs.aftDistanceM = 2.4;
  specs.portDistanceM = 0.6;
  specs.starboardDistanceM = 0.7;
  specs.topDistanceM = 0.4;
  specs.bottomDistanceM = 0.3;
  specs.displacementMetricTon = 1.5;
  specs.weightLightMetricTon = 1.2;
  specs.weightLoadedMetricTon = 1.8;

  // WHEN: the specs report is built
  const auto report = arlcore::autopilot::makePlatformSpecsReport(specs);

  // THEN: every field is carried across
  EXPECT_EQ(report.name(), "test-vehicle");
  EXPECT_DOUBLE_EQ(report.lengthAtWaterline(), 5.0);
  EXPECT_DOUBLE_EQ(report.beamAtWaterline(), 1.0);
  EXPECT_DOUBLE_EQ(report.draft(), 0.5);
  EXPECT_DOUBLE_EQ(report.forwardDistance(), 2.5);
  EXPECT_DOUBLE_EQ(report.aftDistance(), 2.4);
  EXPECT_DOUBLE_EQ(report.portDistance(), 0.6);
  EXPECT_DOUBLE_EQ(report.starboardDistance(), 0.7);
  EXPECT_DOUBLE_EQ(report.topDistance(), 0.4);
  EXPECT_DOUBLE_EQ(report.bottomDistance(), 0.3);
  EXPECT_DOUBLE_EQ(report.displacement(), 1.5);
  EXPECT_DOUBLE_EQ(report.weightLight(), 1.2);
  EXPECT_DOUBLE_EQ(report.weightLoaded(), 1.8);
}

TEST(PlatformReportFactoryTest, MapsCapabilitiesWithUnderwater) {
  // GIVEN: capabilities with underwater enabled and a mix of set/unset
  // optionals
  arlcore::autopilot::PlatformCapabilitiesConfig caps;
  caps.minWaterDepthM = 2.0;
  caps.surface.maxForwardSpeedMps = 6.0;
  caps.surface.maxTurnRateRps = 0.26;
  caps.underwaterEnabled = true;
  caps.underwater.maxForwardSpeedMps = 4.0;
  caps.underwater.maxDepthChangeRateMps = 0.2;

  // WHEN: the capabilities report is built
  const auto report = arlcore::autopilot::makePlatformCapabilitiesReport(caps);

  // THEN: surface and underwater blocks carry set fields and leave unset
  // optionals empty
  EXPECT_DOUBLE_EQ(report.minWaterDepth(), 2.0);
  ASSERT_TRUE(report.surfaceCapabilities().maxForwardSpeed().has_value());
  EXPECT_DOUBLE_EQ(report.surfaceCapabilities().maxForwardSpeed().value(), 6.0);
  ASSERT_TRUE(report.surfaceCapabilities().maxTurnRate().has_value());
  EXPECT_DOUBLE_EQ(report.surfaceCapabilities().maxTurnRate().value(), 0.26);
  EXPECT_FALSE(report.surfaceCapabilities().maxReverseSpeed().has_value());
  ASSERT_TRUE(report.underwaterCapabilities().has_value());
  ASSERT_TRUE(report.underwaterCapabilities().value().maxForwardSpeed().has_value());
  EXPECT_DOUBLE_EQ(report.underwaterCapabilities().value().maxForwardSpeed().value(), 4.0);
  ASSERT_TRUE(report.underwaterCapabilities().value().maxDepthChangeRate().has_value());
  EXPECT_DOUBLE_EQ(report.underwaterCapabilities().value().maxDepthChangeRate().value(), 0.2);
  EXPECT_FALSE(report.underwaterCapabilities().value().cruisingSpeed().has_value());
}

TEST(PlatformReportFactoryTest, OmitsUnderwaterBlockWhenDisabled) {
  // GIVEN: capabilities with underwater disabled but underwater values present
  // in config
  arlcore::autopilot::PlatformCapabilitiesConfig caps;
  caps.underwaterEnabled = false;
  caps.underwater.maxForwardSpeedMps = 4.0;

  // WHEN: the capabilities report is built
  const auto report = arlcore::autopilot::makePlatformCapabilitiesReport(caps);

  // THEN: the underwater optional stays unset
  EXPECT_FALSE(report.underwaterCapabilities().has_value());
}
