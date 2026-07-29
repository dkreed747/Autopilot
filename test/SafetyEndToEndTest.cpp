#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/core/AutopilotBrain.hpp"
#include "autopilot/safety/ConstraintSupervisor.hpp"
#include "autopilot/safety/SafeModeStrategyFactory.hpp"
#include "UuidFactory.h"
#include "WaterZoneConditional.h"
#include "InternalTypes.h"

using WaterZoneKindEnumType =
    UMAA::Common::MaritimeEnumeration::WaterZoneKindEnumModule::WaterZoneKindEnumType;
using DateTime = UMAA::Common::Measurement::DateTime;
using GeoPosition2D = UMAA::Common::Measurement::GeoPosition2D;
using GlobalPoseReportType = UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

constexpr flt64_t kLat = 39.0;
constexpr flt64_t kLon = -76.5;
const char* kSrpCsvPath = "safety_e2e_srp.csv";

static const GeographicLib::LocalCartesian& testFrame() {
  static const GeographicLib::LocalCartesian frame(kLat, kLon, 0.0);
  return frame;
}

//! \brief Captures emitted control vectors and integrates simple kinematics from them.
class EndToEndSimVehicle : public arlcore::autopilot::IVehicleControl {
 public:
  bool initialize() override { return true; }
  bool sendControlVector(const arlcore::autopilot::ControlVector& cv) override {
    last = cv;
    return true;
  }
  bool isManualEngaged() const override { return manualEngaged; }

  GlobalPoseReportType pose() const {
    GlobalPoseReportType p;
    flt64_t lat = 0.0;
    flt64_t lon = 0.0;
    flt64_t h = 0.0;
    testFrame().Reverse(xE, yN, 0.0, lat, lon, h);
    p.position().geodeticLatitude(lat);
    p.position().geodeticLongitude(lon);
    p.attitude().yaw().yaw(yawRad);
    p.depth() = 5.0;  // inside the zone's 0..100 m band (the conditional needs a depth)
    return p;
  }

  void integrate(flt64_t dtS) {
    if (!last.has_value()) {
      return;
    }
    const flt64_t err = arlcore::autopilot::wrapPi(last->headingRad - yawRad);
    const flt64_t maxDelta = 0.5 * dtS;
    yawRad = arlcore::autopilot::wrapPi(yawRad + std::clamp(err, -maxDelta, maxDelta));
    xE += last->speedMps * dtS * std::sin(yawRad);
    yN += last->speedMps * dtS * std::cos(yawRad);
  }

  flt64_t xE = 0.0;
  flt64_t yN = 0.0;
  flt64_t yawRad = 0.0;
  std::optional<arlcore::autopilot::ControlVector> last;
  bool manualEngaged = false;
};

//! \brief A keep-in [0,200]x[0,200] water-zone conditional in the test frame.
static std::shared_ptr<arlcore::umaa::conditional::WaterZoneConditional> keepInConditional() {
  const arlcore::NumericGuid conditionalId = arlcore::UuidFactory::getInstance().generateGuid();
  const arlcore::NumericGuid specId = arlcore::UuidFactory::getInstance().generateGuid();
  const DateTime stamp(1, 0);

  UMAA::MM::Conditional::WaterZoneConditionalType spec;
  spec.specializationReferenceID(specId.getGuid());
  spec.specializationReferenceTimestamp(stamp);
  spec.zoneKind(WaterZoneKindEnumType::INSIDE);
  spec.ceiling().ElevationVariantTypeSubtypes().DepthVariantVariant(UMAA::Common::Measurement::DepthVariantType());
  spec.ceiling().ElevationVariantTypeSubtypes().DepthVariantVariant().depth(0.0);
  spec.floor().ElevationVariantTypeSubtypes().DepthVariantVariant(UMAA::Common::Measurement::DepthVariantType());
  spec.floor().ElevationVariantTypeSubtypes().DepthVariantVariant().depth(100.0);

  UMAA::MM::BaseType::PolygonVariantType polygon;
  for (const auto& [e, n] : {std::pair{0.0, 0.0}, {200.0, 0.0}, {200.0, 200.0}, {0.0, 200.0}}) {
    flt64_t lat = 0.0;
    flt64_t lon = 0.0;
    flt64_t h = 0.0;
    testFrame().Reverse(e, n, 0.0, lat, lon, h);
    polygon.referencePoints().push_back(GeoPosition2D(lat, lon));
  }
  UMAA::MM::BaseType::ShapeVariantType shape;
  shape.ShapeVariantTypeSubtypes().PolygonVariantVariant(polygon);
  spec.zone().push_back(shape);

  const arlcore::umaa::conditional::ConditionalType base(conditionalId.getGuid(), "keep-in",
                             specId.getGuid(), stamp,
                             UMAA::MM::Conditional::WaterZoneConditionalTypeTopic);
  return std::make_shared<arlcore::umaa::conditional::WaterZoneConditional>(base, spec);
}

static arlcore::autopilot::AutopilotConfig safetyConfig() {
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  config.platformCapabilities.surface.maxForwardSpeedMps = 6.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.5;
  config.loop.navStalenessTimeoutMs = 60000;  // the harness feeds poses manually
  config.safety.violationConfirmTicks = 2;
  config.safety.clearHoldS = 0.0;
  config.safety.stateReportPeriodMs = 3600000;  // keep state reports out of the way
  config.recovery.completeHoldS = 0.0;
  config.zones.safetyMarginM = 5.0;
  return config;
}

//! \brief Full safety stack minus DDS: vehicle, brain, zone map, supervisor, conditionals.
struct Harness {
  explicit Harness(arlcore::autopilot::AutopilotConfig cfg,
                   std::unique_ptr<arlcore::autopilot::ISafeModeStrategy> strategy)
      : config(std::move(cfg)), zoneMap(config.zones) {
    brain = std::make_unique<arlcore::autopilot::AutopilotBrain>(&nav, &vehicle, config);
    brain->setZoneMap(&zoneMap);
    supervisor = std::make_unique<arlcore::autopilot::ConstraintSupervisor>(config, &nav, &zoneMap,
        nullptr, arlcore::UuidFactory::getInstance().generateGuid());
    brain->setConstraintSource(supervisor.get());
    supervisor->attachSafety(brain.get(), std::move(strategy));

    zone = keepInConditional();
    supervisor->activeSetObserver()->update({zone});
    supervisor->conditionalSetObserver()->update({zone});
  }

  //! \brief One control tick: pose -> conditional -> brain -> supervisor -> integrate.
  void step(flt64_t dtS = 0.25) {
    const GlobalPoseReportType pose = vehicle.pose();
    nav.setPose(pose);
    zone->update(pose);
    brain->onNavUpdate();
    supervisor->update();
    vehicle.integrate(dtS);
  }

  void stepFor(int32_t ticks, flt64_t dtS = 0.25) {
    for (int32_t i = 0; i < ticks; ++i) {
      step(dtS);
    }
  }

  arlcore::autopilot::AutopilotConfig config;
  arlcore::autopilot::NavState nav;
  EndToEndSimVehicle vehicle;
  arlcore::autopilot::ZoneMap zoneMap;
  std::unique_ptr<arlcore::autopilot::AutopilotBrain> brain;
  std::unique_ptr<arlcore::autopilot::ConstraintSupervisor> supervisor;
  std::shared_ptr<arlcore::umaa::conditional::WaterZoneConditional> zone;
};

static void writeSrpCsv() {
  std::ofstream out(kSrpCsvPath);
  out << "east_m,north_m,speed_mps,capture_radius_m\n"
      << "100,80,3.0,8.0\n"
      << "100,40,3.0,8.0\n";
  out.close();
}

static arlcore::autopilot::SrpConfig srpConfig(bool acceptAfter) {
  arlcore::autopilot::SrpConfig srp;
  srp.csvPath = kSrpCsvPath;
  srp.originLatDeg = kLat;
  srp.originLonDeg = kLon;
  srp.acceptCommandsAfterSrp = acceptAfter;
  srp.holdRadiusM = 15.0;
  srp.repositionSpeedMps = 2.0;
  return srp;
}

class SafetyEndToEndTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(kSrpCsvPath); }
};

TEST_F(SafetyEndToEndTest, ZoneExitRecoversAndResumes) {
  // GIVEN: ample grace (recovery must win) and the vehicle outside the keep-in mid-command,
  // facing away (west)
  arlcore::autopilot::AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 30.0;
  Harness h(config, arlcore::autopilot::makeSafeModeStrategy(config.safety, std::nullopt));

  h.vehicle.xE = -30.0;
  h.vehicle.yN = 100.0;
  h.vehicle.yawRad = -M_PI / 2.0;

  // WHEN: enough ticks run to confirm the violation
  h.stepFor(4);

  // THEN: the FSM is recovering and inbound commands are blocked
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::RECOVERING);
  EXPECT_TRUE(h.brain->recovering());
  EXPECT_FALSE(h.supervisor->commandsAllowed());

  // WHEN: recovery steers back toward the keep-in (east) until the vehicle is compliant again
  h.stepFor(400);

  // THEN: the FSM returns to MONITORING with the vehicle back inside
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::MONITORING);
  EXPECT_FALSE(h.brain->recovering());
  EXPECT_TRUE(h.supervisor->commandsAllowed());
  EXPECT_GT(h.vehicle.xE, 0.0);  // back inside
}

TEST_F(SafetyEndToEndTest, ZeroGraceEngagesSafeModeInstantlyAndReleasesOnAllClear) {
  // GIVEN: zero grace with exit-on-all-clear and the vehicle outside the keep-in
  arlcore::autopilot::AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.0;
  config.safety.exitOnAllClear = true;
  Harness h(config, arlcore::autopilot::makeSafeModeStrategy(config.safety, std::nullopt));

  h.vehicle.xE = -30.0;
  h.vehicle.yN = 100.0;

  // WHEN: the violation confirms
  h.stepFor(4);

  // THEN: safe mode engages instantly with a zero-speed hold (not driving anywhere)
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_FALSE(h.supervisor->commandsAllowed());
  EXPECT_EQ(h.brain->mode(), arlcore::autopilot::DriveSource::SAFE);
  ASSERT_TRUE(h.vehicle.last.has_value());
  EXPECT_DOUBLE_EQ(h.vehicle.last->speedMps, 0.0);

  // WHEN: the vehicle is towed back inside
  h.vehicle.xE = 100.0;
  h.vehicle.yN = 100.0;
  h.stepFor(10);

  // THEN: all-clear releases safe mode
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::MONITORING);
  EXPECT_TRUE(h.supervisor->commandsAllowed());
  EXPECT_EQ(h.brain->mode(), arlcore::autopilot::DriveSource::NONE);
}

TEST_F(SafetyEndToEndTest, GraceExpiryEscalatesRecoveryToSafeMode) {
  // GIVEN: a grace period that expires long before recovery can finish, and the vehicle far
  // outside the keep-in
  arlcore::autopilot::AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.05;
  Harness h(config, arlcore::autopilot::makeSafeModeStrategy(config.safety, std::nullopt));

  h.vehicle.xE = -500.0;
  h.vehicle.yN = 100.0;

  // WHEN: the violation confirms
  h.stepFor(4);

  // THEN: recovery engages first
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::RECOVERING);

  // WHEN: the grace period expires mid-recovery
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  h.stepFor(2);

  // THEN: the FSM escalates to safe mode and recovery stands down
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_FALSE(h.brain->recovering());
}

TEST_F(SafetyEndToEndTest, SrpRunsToCompletionAndReleases) {
  // GIVEN: an SRP strategy that owns the release and accepts commands after completion, with
  // the vehicle violating the keep-in
  writeSrpCsv();
  arlcore::autopilot::AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.0;
  config.safety.exitOnAllClear = false;
  config.safety.safeMode.strategy = "srp";
  config.safety.safeMode.srp = srpConfig(/*acceptAfter=*/true);
  std::string error;
  auto srp = arlcore::autopilot::SafeReturnPath::load(config.safety.safeMode.srp, &error);
  ASSERT_TRUE(srp.has_value()) << error;
  Harness h(config, arlcore::autopilot::makeSafeModeStrategy(config.safety, srp));

  h.vehicle.xE = -30.0;
  h.vehicle.yN = 100.0;

  // WHEN: the violation confirms
  h.stepFor(4);

  // THEN: safe mode engages with the SRP route incomplete
  ASSERT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_EQ(h.brain->mode(), arlcore::autopilot::DriveSource::SAFE);
  EXPECT_FALSE(h.brain->safeRouteComplete());

  // WHEN: the SRP flies to completion
  int32_t ticks = 0;
  while (h.supervisor->safetyState() ==
             arlcore::autopilot::ConstraintSupervisor::SafetyState::SAFE_MODE &&
         ticks < 3000) {
    h.step();
    ++ticks;
  }

  // THEN: the strategy releases the autopilot near the final SRP waypoint (100, 40)
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::MONITORING);
  EXPECT_TRUE(h.supervisor->commandsAllowed());
  EXPECT_EQ(h.brain->mode(), arlcore::autopilot::DriveSource::NONE);
  EXPECT_NEAR(h.vehicle.xE, 100.0, 25.0);
  EXPECT_NEAR(h.vehicle.yN, 40.0, 25.0);
}

TEST_F(SafetyEndToEndTest, SrpHoldRepositionsOnDriftOut) {
  // GIVEN: an SRP strategy that holds after completion, engaged by a keep-in violation
  writeSrpCsv();
  arlcore::autopilot::AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.0;
  config.safety.exitOnAllClear = false;
  config.safety.safeMode.strategy = "srp";
  config.safety.safeMode.srp = srpConfig(/*acceptAfter=*/false);
  std::string error;
  auto srp = arlcore::autopilot::SafeReturnPath::load(config.safety.safeMode.srp, &error);
  ASSERT_TRUE(srp.has_value()) << error;
  Harness h(config, arlcore::autopilot::makeSafeModeStrategy(config.safety, srp));

  h.vehicle.xE = -30.0;
  h.vehicle.yN = 100.0;
  h.stepFor(4);
  ASSERT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::SAFE_MODE);

  // WHEN: the SRP runs to its end (with accept_commands_after_srp=false the strategy switches
  // to holding the final waypoint; the completion is consumed within the same supervisor tick,
  // so observe the vehicle position rather than the transient route state)
  const auto driftM = [&h] { return std::hypot(h.vehicle.xE - 100.0, h.vehicle.yN - 40.0); };
  const auto holding = [&h] { return h.vehicle.last.has_value() && h.vehicle.last->speedMps == 0.0; };
  int32_t ticks = 0;
  while (!holding() && ticks < 4000) {
    h.step();
    ++ticks;
  }

  // THEN: the vehicle holds within the radius of the final waypoint without releasing
  ASSERT_TRUE(holding());
  ASSERT_LT(driftM(), 15.0);
  EXPECT_EQ(h.supervisor->safetyState(),
            arlcore::autopilot::ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_FALSE(h.supervisor->commandsAllowed());

  // WHEN: the vehicle drifts out of the hold circle (east)
  h.vehicle.xE = 100.0 + 30.0;
  h.vehicle.yN = 40.0;
  h.stepFor(4);

  // THEN: a reposition route is driving back to the center
  ASSERT_TRUE(h.vehicle.last.has_value());
  EXPECT_GT(h.vehicle.last->speedMps, 0.0);

  // WHEN: the reposition runs until the vehicle holds again
  ticks = 0;
  while (!holding() && ticks < 4000) {
    h.step();
    ++ticks;
  }

  // THEN: back within the hold radius, arrived headed opposite the drift (westbound)
  ASSERT_TRUE(holding());
  EXPECT_LT(driftM(), 15.0);
  EXPECT_NEAR(std::fabs(arlcore::autopilot::wrapPi(h.vehicle.yawRad - (-M_PI / 2.0))), 0.0, 0.6);
}
