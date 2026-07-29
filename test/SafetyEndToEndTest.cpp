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

namespace arlcore::autopilot {

using arlcore::umaa::conditional::ConditionalType;
using arlcore::umaa::conditional::WaterZoneConditional;
using UMAA::Common::MaritimeEnumeration::WaterZoneKindEnumModule::WaterZoneKindEnumType;
using UMAA::Common::Measurement::DateTime;
using UMAA::Common::Measurement::GeoPosition2D;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;

constexpr flt64_t kLat = 39.0;
constexpr flt64_t kLon = -76.5;
const char* kSrpCsvPath = "safety_e2e_srp.csv";

static const GeographicLib::LocalCartesian& testFrame() {
  static const GeographicLib::LocalCartesian frame(kLat, kLon, 0.0);
  return frame;
}

//! \brief Captures emitted control vectors and integrates simple kinematics from them.
class EndToEndSimVehicle : public IVehicleControl {
 public:
  bool initialize() override { return true; }
  bool sendControlVector(const ControlVector& cv) override {
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
    const flt64_t err = wrapPi(last->headingRad - yawRad);
    const flt64_t maxDelta = 0.5 * dtS;
    yawRad = wrapPi(yawRad + std::clamp(err, -maxDelta, maxDelta));
    xE += last->speedMps * dtS * std::sin(yawRad);
    yN += last->speedMps * dtS * std::cos(yawRad);
  }

  flt64_t xE = 0.0;
  flt64_t yN = 0.0;
  flt64_t yawRad = 0.0;
  std::optional<ControlVector> last;
  bool manualEngaged = false;
};

//! \brief A keep-in [0,200]x[0,200] water-zone conditional in the test frame.
static std::shared_ptr<WaterZoneConditional> keepInConditional() {
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

  const ConditionalType base(conditionalId.getGuid(), "keep-in", specId.getGuid(), stamp,
                             UMAA::MM::Conditional::WaterZoneConditionalTypeTopic);
  return std::make_shared<WaterZoneConditional>(base, spec);
}

static AutopilotConfig safetyConfig() {
  AutopilotConfig config;
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
  explicit Harness(AutopilotConfig cfg, std::unique_ptr<ISafeModeStrategy> strategy)
      : config(std::move(cfg)), zoneMap(config.zones) {
    brain = std::make_unique<AutopilotBrain>(&nav, &vehicle, config);
    brain->setZoneMap(&zoneMap);
    supervisor = std::make_unique<ConstraintSupervisor>(config, &nav, &zoneMap, nullptr,
        arlcore::UuidFactory::getInstance().generateGuid());
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

  AutopilotConfig config;
  NavState nav;
  EndToEndSimVehicle vehicle;
  ZoneMap zoneMap;
  std::unique_ptr<AutopilotBrain> brain;
  std::unique_ptr<ConstraintSupervisor> supervisor;
  std::shared_ptr<WaterZoneConditional> zone;
};

static void writeSrpCsv() {
  std::ofstream out(kSrpCsvPath);
  out << "east_m,north_m,speed_mps,capture_radius_m\n"
      << "100,80,3.0,8.0\n"
      << "100,40,3.0,8.0\n";
  out.close();
}

static SrpConfig srpConfig(bool acceptAfter) {
  SrpConfig srp;
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
  AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 30.0;  // plenty of grace: recovery must win
  Harness h(config, makeSafeModeStrategy(config.safety, std::nullopt));

  // The vehicle finds itself outside the keep-in (mid-command).
  h.vehicle.xE = -30.0;
  h.vehicle.yN = 100.0;
  h.vehicle.yawRad = -M_PI / 2.0;  // facing away (west)

  h.stepFor(4);
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::RECOVERING);
  EXPECT_TRUE(h.brain->recovering());
  EXPECT_FALSE(h.supervisor->commandsAllowed());

  // Recovery steers back toward the keep-in (east) and the FSM returns to MONITORING once
  // the vehicle is compliant again.
  h.stepFor(400);
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::MONITORING);
  EXPECT_FALSE(h.brain->recovering());
  EXPECT_TRUE(h.supervisor->commandsAllowed());
  EXPECT_GT(h.vehicle.xE, 0.0);  // back inside
}

TEST_F(SafetyEndToEndTest, ZeroGraceEngagesSafeModeInstantlyAndReleasesOnAllClear) {
  AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.0;
  config.safety.exitOnAllClear = true;
  Harness h(config, makeSafeModeStrategy(config.safety, std::nullopt));

  h.vehicle.xE = -30.0;
  h.vehicle.yN = 100.0;

  h.stepFor(4);
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_FALSE(h.supervisor->commandsAllowed());
  EXPECT_EQ(h.brain->mode(), DriveSource::SAFE);
  // Zero-speed hold: the vehicle must not be driving anywhere.
  ASSERT_TRUE(h.vehicle.last.has_value());
  EXPECT_DOUBLE_EQ(h.vehicle.last->speedMps, 0.0);

  // Tow the vehicle back inside: all-clear releases safe mode.
  h.vehicle.xE = 100.0;
  h.vehicle.yN = 100.0;
  h.stepFor(10);
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::MONITORING);
  EXPECT_TRUE(h.supervisor->commandsAllowed());
  EXPECT_EQ(h.brain->mode(), DriveSource::NONE);
}

TEST_F(SafetyEndToEndTest, GraceExpiryEscalatesRecoveryToSafeMode) {
  AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.05;  // expires long before recovery can finish
  Harness h(config, makeSafeModeStrategy(config.safety, std::nullopt));

  h.vehicle.xE = -500.0;  // far outside: recovery cannot complete in time
  h.vehicle.yN = 100.0;

  h.stepFor(4);
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::RECOVERING);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  h.stepFor(2);
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_FALSE(h.brain->recovering());
}

TEST_F(SafetyEndToEndTest, SrpRunsToCompletionAndReleases) {
  writeSrpCsv();
  AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.0;
  config.safety.exitOnAllClear = false;  // the SRP strategy owns the release
  config.safety.safeMode.strategy = "srp";
  config.safety.safeMode.srp = srpConfig(/*acceptAfter=*/true);
  std::string error;
  auto srp = SafeReturnPath::load(config.safety.safeMode.srp, &error);
  ASSERT_TRUE(srp.has_value()) << error;
  Harness h(config, makeSafeModeStrategy(config.safety, srp));

  h.vehicle.xE = -30.0;  // violate the keep-in
  h.vehicle.yN = 100.0;

  h.stepFor(4);
  ASSERT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_EQ(h.brain->mode(), DriveSource::SAFE);
  EXPECT_FALSE(h.brain->safeRouteComplete());

  // Fly the SRP to completion; the strategy then releases the autopilot.
  int32_t ticks = 0;
  while (h.supervisor->safetyState() == ConstraintSupervisor::SafetyState::SAFE_MODE && ticks < 3000) {
    h.step();
    ++ticks;
  }
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::MONITORING);
  EXPECT_TRUE(h.supervisor->commandsAllowed());
  EXPECT_EQ(h.brain->mode(), DriveSource::NONE);
  // The vehicle arrived near the final SRP waypoint (100, 40).
  EXPECT_NEAR(h.vehicle.xE, 100.0, 25.0);
  EXPECT_NEAR(h.vehicle.yN, 40.0, 25.0);
}

TEST_F(SafetyEndToEndTest, SrpHoldRepositionsOnDriftOut) {
  writeSrpCsv();
  AutopilotConfig config = safetyConfig();
  config.safety.gracePeriodS = 0.0;
  config.safety.exitOnAllClear = false;
  config.safety.safeMode.strategy = "srp";
  config.safety.safeMode.srp = srpConfig(/*acceptAfter=*/false);
  std::string error;
  auto srp = SafeReturnPath::load(config.safety.safeMode.srp, &error);
  ASSERT_TRUE(srp.has_value()) << error;
  Harness h(config, makeSafeModeStrategy(config.safety, srp));

  h.vehicle.xE = -30.0;
  h.vehicle.yN = 100.0;
  h.stepFor(4);
  ASSERT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::SAFE_MODE);

  // Run the SRP to its end; with accept_commands_after_srp=false the strategy switches to
  // holding the final waypoint (the completion is consumed within the same supervisor tick,
  // so observe the vehicle position rather than the transient route state).
  const auto driftM = [&h] { return std::hypot(h.vehicle.xE - 100.0, h.vehicle.yN - 40.0); };
  const auto holding = [&h] { return h.vehicle.last.has_value() && h.vehicle.last->speedMps == 0.0; };
  int32_t ticks = 0;
  while (!holding() && ticks < 4000) {
    h.step();
    ++ticks;
  }
  ASSERT_TRUE(holding());
  ASSERT_LT(driftM(), 15.0);
  EXPECT_EQ(h.supervisor->safetyState(), ConstraintSupervisor::SafetyState::SAFE_MODE);
  EXPECT_FALSE(h.supervisor->commandsAllowed());

  // Drift the vehicle out of the hold circle: the strategy repositions back to the center,
  // arriving headed opposite the drift direction (drifted east -> arrive westbound).
  h.vehicle.xE = 100.0 + 30.0;
  h.vehicle.yN = 40.0;
  h.stepFor(4);
  ASSERT_TRUE(h.vehicle.last.has_value());
  EXPECT_GT(h.vehicle.last->speedMps, 0.0);  // a reposition route is driving

  ticks = 0;
  while (!holding() && ticks < 4000) {
    h.step();
    ++ticks;
  }
  ASSERT_TRUE(holding());
  // Back within the hold radius of the final SRP waypoint, and arrived headed west.
  EXPECT_LT(driftM(), 15.0);
  EXPECT_NEAR(std::fabs(wrapPi(h.vehicle.yawRad - (-M_PI / 2.0))), 0.0, 0.6);
}

}  // namespace arlcore::autopilot
