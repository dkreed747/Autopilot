#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "InternalTypes.h"
#include "autopilot/core/AutopilotApp.hpp"
#include "autopilot/vehicle/VehicleControlTypes.hpp"

// These cases cover only the checks AutopilotApp::initialize makes BEFORE it acquires a DDS
// participant: the identity UUIDs, the platform capabilities and the vehicle-control type. That
// prefix is deliberately ordered so a bad config fails without touching a domain, and it is the
// part a platform integrator hits first.
//
// TODO(autopilot-#5): the tick-ordering contract in step() (manual edges before mode commands
// before the command providers before idle revert) and the shutdown discipline in run()/stop()
// are still untested. Both need the collaborators injected rather than built inside initialize(),
// which is a move of roughly 25 members whose declaration order is load-bearing for DDS
// destruction. Deferred rather than done during release lockdown; smoke-console and smoke-runner
// cover the same paths end to end in the meantime.

//! \brief A config that clears every precondition, so each test can break exactly one thing.
static arlcore::autopilot::AutopilotConfig startableConfig() {
  arlcore::autopilot::AutopilotConfig config;
  config.identity.platformId = "11111111-1111-1111-1111-111111111111";
  config.identity.vectorSourceId = "22222222-2222-2222-2222-222222222222";
  config.identity.waypointSourceId = "33333333-3333-3333-3333-333333333333";
  config.identity.specsSourceId = "44444444-4444-4444-4444-444444444444";
  config.identity.capabilitiesSourceId = "55555555-5555-5555-5555-555555555555";
  config.identity.navSourceId = "66666666-6666-6666-6666-666666666666";
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  config.platformCapabilities.surface.maxForwardSpeedMps = 6.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.2618;
  config.vehicleControlType = "sim";
  return config;
}

TEST(AutopilotAppTest, InitializeRejectsAMalformedIdentityUuid) {
  // GIVEN: each identity field in turn set to something that is not a UUID
  const char* const kFields[] = {"platform_id",     "vector_source_id",       "waypoint_source_id",
                                 "specs_source_id", "capabilities_source_id", "nav_source_id"};
  for (int32_t field = 0; field < 6; field++) {
    arlcore::autopilot::AutopilotConfig config = startableConfig();
    switch (field) {
      case 0:
        config.identity.platformId = "not-a-uuid";
        break;
      case 1:
        config.identity.vectorSourceId = "not-a-uuid";
        break;
      case 2:
        config.identity.waypointSourceId = "not-a-uuid";
        break;
      case 3:
        config.identity.specsSourceId = "not-a-uuid";
        break;
      case 4:
        config.identity.capabilitiesSourceId = "not-a-uuid";
        break;
      default:
        config.identity.navSourceId = "not-a-uuid";
        break;
    }

    // WHEN: the app initializes
    arlcore::autopilot::AutopilotApp app;

    // THEN: it refuses. A garbage GUID parses to something, so without this check the autopilot
    //       would come up publishing under an identity nobody addresses and every command sent
    //       to the configured ID would silently never match.
    EXPECT_FALSE(app.initialize(config)) << kFields[field];
  }
}

TEST(AutopilotAppTest, InitializeRejectsCapabilitiesThatCannotDeriveATurnRadius) {
  // GIVEN: capabilities missing each of the values the planned turn radius derives from
  arlcore::autopilot::AutopilotConfig noSpeed = startableConfig();
  noSpeed.platformCapabilities.surface.cruisingSpeedMps.reset();
  noSpeed.platformCapabilities.surface.maxForwardSpeedMps.reset();

  arlcore::autopilot::AutopilotConfig noRate = startableConfig();
  noRate.platformCapabilities.surface.maxTurnRateRps.reset();

  arlcore::autopilot::AutopilotConfig zeroRate = startableConfig();
  zeroRate.platformCapabilities.surface.maxTurnRateRps = 0.0;

  // WHEN: each initializes
  // THEN: all three refuse. Without them the planner would fall back to a conservative 25 m
  //       radius on a platform that never asked for it, and every tracking threshold derived
  //       from that radius would describe a vehicle that does not exist.
  arlcore::autopilot::AutopilotApp a;
  arlcore::autopilot::AutopilotApp b;
  arlcore::autopilot::AutopilotApp c;
  EXPECT_FALSE(a.initialize(noSpeed));
  EXPECT_FALSE(b.initialize(noRate));
  EXPECT_FALSE(c.initialize(zeroRate));
}

TEST(AutopilotAppTest, InitializeRejectsAnUnknownVehicleControlType) {
  // GIVEN: a config that is valid apart from naming a strategy nothing implements
  arlcore::autopilot::AutopilotConfig config = startableConfig();
  config.vehicleControlType = "hovercraft";

  // WHEN: the app initializes
  arlcore::autopilot::AutopilotApp app;

  // THEN: it refuses, and does so before acquiring a DDS participant. This used to warn and then
  //       construct the simulator, so an operator wiring up a real hull got a fully reporting
  //       autopilot that was driving a simulation.
  EXPECT_FALSE(app.initialize(config));
}

TEST(AutopilotAppTest, InitializeAcceptsEveryRegisteredVehicleControlTypeUpToTheDdsBoundary) {
  // GIVEN: the registered strategy names
  // WHEN: each is checked against the pre-DDS prefix of initialize
  // THEN: none of them is the reason a startup would fail. This pins that the type check added
  //       ahead of participant creation rejects only unregistered names, so registering a
  //       strategy is genuinely sufficient to get past it.
  for (const std::string& type : arlcore::autopilot::vehicleControlTypes()) {
    arlcore::autopilot::AutopilotConfig config = startableConfig();
    config.vehicleControlType = type;
    EXPECT_TRUE(arlcore::autopilot::isKnownVehicleControlType(config.vehicleControlType)) << type;
  }
  EXPECT_FALSE(arlcore::autopilot::isKnownVehicleControlType("hovercraft"));
}
