#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "InternalTypes.h"
#include "LocalReaderSender.h"
#include "UuidFactory.h"
#include "autopilot/config/ConfigValidation.hpp"
#include "autopilot/vehicle/SimVehicleControl.hpp"
#include "autopilot/vehicle/VehicleControlFactory.hpp"
#include "autopilot/vehicle/VehicleControlTypes.hpp"

using FactoryPoseReportType = UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using FactorySpeedReportType = UMAA::SA::SpeedStatus::SpeedReportType;
using FactoryVelocityReportType = UMAA::SA::VelocityStatus::VelocityReportType;

//! \brief A loopback nav transport plus a config that satisfies every hard precondition, so a
//! failure in these tests is about the factory rather than about the config.
struct FactoryFixture {
  std::shared_ptr<arlcore::io::LocalReaderSender<FactoryPoseReportType>> poseIo =
      std::make_shared<arlcore::io::LocalReaderSender<FactoryPoseReportType>>();
  std::shared_ptr<arlcore::io::LocalReaderSender<FactorySpeedReportType>> speedIo =
      std::make_shared<arlcore::io::LocalReaderSender<FactorySpeedReportType>>();
  std::shared_ptr<arlcore::io::LocalReaderSender<FactoryVelocityReportType>> velocityIo =
      std::make_shared<arlcore::io::LocalReaderSender<FactoryVelocityReportType>>();

  arlcore::autopilot::AutopilotConfig config;

  FactoryFixture() {
    config.platformCapabilities.surface.maxForwardSpeedMps = 6.0;
    config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
    config.platformCapabilities.surface.maxTurnRateRps = 0.2618;
    config.simVehicle.cycleRateHz = 20.0;
    config.simVehicle.initialLatitudeDeg = 39.0;
    config.simVehicle.initialLongitudeDeg = -76.5;
    config.simVehicle.accelMps2 = 1.0;
    config.identity.navSourceId = "11111111-2222-3333-4444-555555555555";
  }

  arlcore::autopilot::NavReportSenders senders() const {
    arlcore::autopilot::NavReportSenders s;
    s.pose = poseIo;
    s.speed = speedIo;
    s.velocity = velocityIo;
    return s;
  }
};

//! \brief The factory returns the interface, but "it built the sim strategy" is part of its
//! contract, and stepping the sim directly keeps these tests off its background thread.
static arlcore::autopilot::SimVehicleControl* asSim(arlcore::autopilot::IVehicleControl* vehicle) {
  return dynamic_cast<arlcore::autopilot::SimVehicleControl*>(vehicle);
}

TEST(VehicleControlFactoryTest, TheSimTypeBuildsAStrategyWiredToTheInjectedTransport) {
  // GIVEN: a sim-typed config and a loopback nav transport
  FactoryFixture f;
  f.config.vehicleControlType = "sim";

  // WHEN: the factory builds the strategy and one cycle is stepped
  std::unique_ptr<arlcore::autopilot::IVehicleControl> vehicle =
      arlcore::autopilot::makeVehicleControl(f.config, f.senders());
  ASSERT_NE(vehicle, nullptr);
  arlcore::autopilot::SimVehicleControl* sim = asSim(vehicle.get());
  ASSERT_NE(sim, nullptr) << "the sim type must build the simulated strategy, not some other one";
  arlcore::autopilot::ControlVector cv;
  cv.headingRad = 0.0;
  cv.speedMps = 1.0;
  EXPECT_TRUE(sim->sendControlVector(cv));
  sim->stepOnce(0.05);

  // THEN: the sim publishes its navigation reports on the senders it was handed, not on a
  //       transport of its own. A factory that dropped them would look fine until nothing on the
  //       domain ever saw a pose.
  FactoryPoseReportType pose;
  FactorySpeedReportType speed;
  FactoryVelocityReportType velocity;
  EXPECT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_EQ(f.speedIo->readLatest(&speed), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_EQ(f.velocityIo->readLatest(&velocity), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_FALSE(vehicle->isManualEngaged());
}

TEST(VehicleControlFactoryTest, TheSimStrategyCarriesTheConfiguredNavSourceIdentity) {
  // GIVEN: a sim-typed config with a known nav source id
  FactoryFixture f;
  f.config.vehicleControlType = "sim";

  // WHEN: the strategy is built and publishes a pose
  std::unique_ptr<arlcore::autopilot::IVehicleControl> vehicle =
      arlcore::autopilot::makeVehicleControl(f.config, f.senders());
  ASSERT_NE(vehicle, nullptr);
  arlcore::autopilot::SimVehicleControl* sim = asSim(vehicle.get());
  ASSERT_NE(sim, nullptr);
  sim->stepOnce(0.05);

  // THEN: the report carries that identity. Consumers filter on it, so a factory that dropped it
  //       would stay invisible until a second nav publisher joined the domain.
  FactoryPoseReportType pose;
  ASSERT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_EQ(pose.source().id(),
            arlcore::UuidFactory::getInstance().parseGuidFromString(f.config.identity.navSourceId).getGuid());
}

TEST(VehicleControlFactoryTest, TheSimStrategyHonoursTheConfiguredTurnRateEnvelope) {
  // GIVEN: a deliberately slow turn-rate envelope
  FactoryFixture f;
  f.config.vehicleControlType = "sim";
  f.config.platformCapabilities.surface.maxTurnRateRps = 0.1;

  // WHEN: a hard turn is commanded and stepped repeatedly
  std::unique_ptr<arlcore::autopilot::IVehicleControl> vehicle =
      arlcore::autopilot::makeVehicleControl(f.config, f.senders());
  ASSERT_NE(vehicle, nullptr);
  arlcore::autopilot::SimVehicleControl* sim = asSim(vehicle.get());
  ASSERT_NE(sim, nullptr);
  arlcore::autopilot::ControlVector cv;
  cv.headingRad = M_PI_2;
  cv.speedMps = 1.0;
  sim->sendControlVector(cv);

  // THEN: the heading never advances faster than the configured envelope, which proves the factory
  //       forwarded platformCapabilities rather than a default-constructed copy
  constexpr flt64_t kStepS = 0.05;
  flt64_t previousYaw = sim->state().headingRad;
  for (int32_t i = 0; i < 20; i++) {
    sim->stepOnce(kStepS);
    const flt64_t yaw = sim->state().headingRad;
    EXPECT_LE(yaw - previousYaw, 0.1 * kStepS + 1e-9) << "step " << i;
    previousYaw = yaw;
  }
  EXPECT_GT(previousYaw, 0.0) << "the vehicle must actually be turning for the bound to mean anything";
}

TEST(VehicleControlFactoryTest, AnUnknownTypeYieldsNoStrategyRatherThanASimulatedOne) {
  // GIVEN: type values that are not registered, including one differing only in case
  FactoryFixture f;

  // WHEN: the factory is asked for each
  // THEN: it returns nullptr every time. Before the factory existed this warned and silently ran
  //       the simulator, so an operator on a real hull saw a fully reporting autopilot driving
  //       nothing at all.
  for (const std::string& type : {std::string("hovercraft"), std::string(""), std::string("SIM")}) {
    f.config.vehicleControlType = type;
    EXPECT_EQ(arlcore::autopilot::makeVehicleControl(f.config, f.senders()), nullptr) << "type='" << type << "'";
  }
}

TEST(VehicleControlFactoryTest, TheSimTypeRefusesToBuildWithoutAllThreeNavSenders) {
  // GIVEN: a sim-typed config missing one of the nav senders in turn
  for (int32_t missing = 0; missing < 3; missing++) {
    FactoryFixture f;
    f.config.vehicleControlType = "sim";
    arlcore::autopilot::NavReportSenders senders = f.senders();
    if (missing == 0) {
      senders.pose = nullptr;
    } else if (missing == 1) {
      senders.speed = nullptr;
    } else {
      senders.velocity = nullptr;
    }

    // WHEN: the factory is asked to build
    // THEN: it refuses. The sim is its own navigation source, so a missing sender is a startup
    //       failure rather than a strategy that crashes on its first publish.
    EXPECT_EQ(arlcore::autopilot::makeVehicleControl(f.config, senders), nullptr) << "missing=" << missing;
  }
}

TEST(VehicleControlFactoryTest, EveryRegisteredTypeIsBuildableAndValid) {
  // GIVEN: the registered type names, which config validation and the factory share
  ASSERT_FALSE(arlcore::autopilot::vehicleControlTypes().empty());

  // WHEN: each is both validated and built
  for (const std::string& type : arlcore::autopilot::vehicleControlTypes()) {
    FactoryFixture f;
    f.config.vehicleControlType = type;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    arlcore::autopilot::validateConfig(f.config, &errors, &warnings);

    // THEN: validation raises no complaint about the type, and the factory produces a strategy.
    //       This is the anti-drift case: a name added to the registry without a factory branch,
    //       or a branch without a registry entry, fails here rather than at startup on a boat.
    for (const std::string& error : errors) {
      EXPECT_EQ(error.find("vehicle_control.type"), std::string::npos) << "type='" << type << "': " << error;
    }
    EXPECT_NE(arlcore::autopilot::makeVehicleControl(f.config, f.senders()), nullptr) << "type='" << type << "'";
  }
}

TEST(VehicleControlFactoryTest, ValidationRejectsATypeTheFactoryCannotBuild) {
  // GIVEN: a config naming an unregistered strategy
  FactoryFixture f;
  f.config.vehicleControlType = "hovercraft";

  // WHEN: the config is validated
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  const bool valid = arlcore::autopilot::validateConfig(f.config, &errors, &warnings);

  // THEN: validation fails and names the key, so startup stops at the config rather than at the
  //       factory. Validation is the first gate; the factory's nullptr is defence in depth.
  EXPECT_FALSE(valid);
  bool named = false;
  for (const std::string& error : errors) {
    named = named || error.find("vehicle_control.type") != std::string::npos;
  }
  EXPECT_TRUE(named);
}
