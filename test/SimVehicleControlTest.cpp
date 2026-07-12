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

#include <cmath>
#include <memory>

#include "AngleMath.h"
#include "LocalReaderSender.h"
#include "SimVehicleControl.h"
#include "UuidFactory.h"

namespace arlcore::autopilot {

namespace {

using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;
using UMAA::SA::VelocityStatus::VelocityReportType;

struct SimFixture {
  std::shared_ptr<arlcore::io::LocalReaderSender<GlobalPoseReportType>> poseIo =
      std::make_shared<arlcore::io::LocalReaderSender<GlobalPoseReportType>>();
  std::shared_ptr<arlcore::io::LocalReaderSender<SpeedReportType>> speedIo =
      std::make_shared<arlcore::io::LocalReaderSender<SpeedReportType>>();
  std::shared_ptr<arlcore::io::LocalReaderSender<VelocityReportType>> velocityIo =
      std::make_shared<arlcore::io::LocalReaderSender<VelocityReportType>>();

  PlatformSpecsConfig specs;
  PlatformCapabilitiesConfig caps;
  SimVehicleConfig sim;

  SimFixture() {
    specs.name = "test-vehicle";
    caps.surface.maxForwardSpeedMps = 6.0;
    caps.surface.maxReverseSpeedMps = 2.0;
    caps.surface.maxTurnRateRps = 0.25;
    sim.cycleRateHz = 20.0;
    sim.initialLatitudeDeg = 39.0;
    sim.initialLongitudeDeg = -76.5;
    sim.initialHeadingRad = 0.0;
    sim.accelMps2 = 1.0;
  }

  std::unique_ptr<SimVehicleControl> make() {
    return std::make_unique<SimVehicleControl>(
        specs, caps, sim, arlcore::UuidFactory::getInstance().generateGuid(),
        poseIo, speedIo, velocityIo);
  }
};

ControlVector makeCv(double headingRad, double speedMps) {
  ControlVector cv;
  cv.headingRad = headingRad;
  cv.speedMps = speedMps;
  return cv;
}

}  // namespace

TEST(SimVehicleControlTest, PublishesAllThreeNavReportsEachStep) {
  SimFixture f;
  auto vehicle = f.make();
  vehicle->stepOnce(0.05);

  GlobalPoseReportType pose;
  SpeedReportType speed;
  VelocityReportType velocity;
  EXPECT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_EQ(f.speedIo->readLatest(&speed), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_EQ(f.velocityIo->readLatest(&velocity), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_NEAR(pose.position().geodeticLatitude(), 39.0, 1e-6);
  EXPECT_NEAR(pose.position().geodeticLongitude(), -76.5, 1e-6);
  ASSERT_TRUE(speed.speedOverGround().has_value());
  EXPECT_NEAR(speed.speedOverGround().value(), 0.0, 1e-9);
}

TEST(SimVehicleControlTest, RespectsTurnRateLimit) {
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(M_PI_2, 0.0));
  vehicle->stepOnce(0.1);
  // 0.25 rad/s * 0.1 s = 0.025 rad per step, far less than the pi/2 error.
  EXPECT_NEAR(vehicle->state().headingRad, 0.025, 1e-9);
  for (int i = 0; i < 100; i++) {
    vehicle->stepOnce(0.1);
  }
  // After 10+ seconds it converges on the commanded heading.
  EXPECT_NEAR(vehicle->state().headingRad, M_PI_2, 1e-6);
}

TEST(SimVehicleControlTest, RespectsAccelerationAndSpeedCap) {
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(0.0, 100.0));  // way over the platform limit
  vehicle->stepOnce(0.5);
  EXPECT_NEAR(vehicle->state().speedMps, 0.5, 1e-9);  // 1 m/s^2 * 0.5 s
  for (int i = 0; i < 40; i++) {
    vehicle->stepOnce(0.5);
  }
  EXPECT_NEAR(vehicle->state().speedMps, 6.0, 1e-9);  // clamped at maxForwardSpeed
}

TEST(SimVehicleControlTest, MovesNorthWhenCommandedNorth) {
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(0.0, 4.0));
  for (int i = 0; i < 200; i++) {
    vehicle->stepOnce(0.1);  // 20 s: ramps to 4 m/s then cruises north
  }
  const auto st = vehicle->state();
  EXPECT_GT(st.latitudeDeg, 39.0);
  EXPECT_NEAR(st.longitudeDeg, -76.5, 1e-6);
  GlobalPoseReportType pose;
  ASSERT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_NEAR(pose.position().geodeticLatitude(), st.latitudeDeg, 1e-9);
}

TEST(SimVehicleControlTest, VelocityReportMatchesHeadingAndSpeed) {
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(M_PI_2, 2.0));  // east
  for (int i = 0; i < 400; i++) {
    vehicle->stepOnce(0.1);
  }
  VelocityReportType velocity;
  ASSERT_EQ(f.velocityIo->readLatest(&velocity), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_NEAR(velocity.velocity().eastSpeed(), 2.0, 1e-6);
  EXPECT_NEAR(velocity.velocity().northSpeed(), 0.0, 1e-6);
}

TEST(SimVehicleControlTest, DrivesDepthAndPublishesAltitudeAboveSeaFloor) {
  SimFixture f;
  f.caps.underwaterEnabled = true;
  f.caps.underwater.maxDepthChangeRateMps = 1.0;
  f.sim.floorDepthM = 50.0;
  auto vehicle = f.make();

  ControlVector cv = makeCv(0.0, 0.0);
  cv.elevationM = 20.0;
  cv.elevationFrame = ElevationFrame::DEPTH;
  vehicle->sendControlVector(cv);
  for (int i = 0; i < 300; i++) {
    vehicle->stepOnce(0.1);
  }
  EXPECT_NEAR(vehicle->state().depthM, 20.0, 1e-6);

  GlobalPoseReportType pose;
  ASSERT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  ASSERT_TRUE(pose.depth().has_value());
  ASSERT_TRUE(pose.altitudeASF().has_value());
  EXPECT_NEAR(pose.depth().value(), 20.0, 1e-6);
  EXPECT_NEAR(pose.altitudeASF().value(), 30.0, 1e-6);  // floor 50 - depth 20

  // Above-sea-floor setpoint converts against the configured floor: 10 m ASF = 40 m depth.
  cv.elevationM = 10.0;
  cv.elevationFrame = ElevationFrame::ALTITUDE_ASF;
  vehicle->sendControlVector(cv);
  for (int i = 0; i < 300; i++) {
    vehicle->stepOnce(0.1);
  }
  EXPECT_NEAR(vehicle->state().depthM, 40.0, 1e-6);

  // Depth commands clamp to the floor.
  cv.elevationM = 500.0;
  cv.elevationFrame = ElevationFrame::DEPTH;
  vehicle->sendControlVector(cv);
  for (int i = 0; i < 300; i++) {
    vehicle->stepOnce(0.1);
  }
  EXPECT_NEAR(vehicle->state().depthM, 50.0, 1e-6);
}

TEST(SimVehicleControlTest, ThreadedRunPublishesAtCycleRate) {
  SimFixture f;
  f.sim.cycleRateHz = 50.0;
  auto vehicle = f.make();
  ASSERT_TRUE(vehicle->initialize());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  vehicle->shutdown();

  // ~15 cycles expected in 300 ms at 50 Hz; allow generous scheduling slop.
  int count = 0;
  GlobalPoseReportType pose;
  while (f.poseIo->read(&pose) == arlcore::io::ReadStatus::SUCCESS) {
    count++;
  }
  EXPECT_GE(count, 5);
  // Shutdown is idempotent and re-initialization works.
  vehicle->shutdown();
  EXPECT_TRUE(vehicle->initialize());
  vehicle->shutdown();
}

}  // namespace arlcore::autopilot
