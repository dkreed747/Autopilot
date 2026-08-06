#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "InternalTypes.h"
#include "LocalReaderSender.h"
#include "UuidFactory.h"
#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/vehicle/SimVehicleControl.hpp"

using GlobalPoseReportType = UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using SpeedReportType = UMAA::SA::SpeedStatus::SpeedReportType;
using VelocityReportType = UMAA::SA::VelocityStatus::VelocityReportType;

struct SimFixture {
  std::shared_ptr<arlcore::io::LocalReaderSender<GlobalPoseReportType>> poseIo =
      std::make_shared<arlcore::io::LocalReaderSender<GlobalPoseReportType>>();
  std::shared_ptr<arlcore::io::LocalReaderSender<SpeedReportType>> speedIo =
      std::make_shared<arlcore::io::LocalReaderSender<SpeedReportType>>();
  std::shared_ptr<arlcore::io::LocalReaderSender<VelocityReportType>> velocityIo =
      std::make_shared<arlcore::io::LocalReaderSender<VelocityReportType>>();

  arlcore::autopilot::PlatformCapabilitiesConfig caps;
  arlcore::autopilot::SimVehicleConfig sim;

  SimFixture() {
    caps.surface.maxForwardSpeedMps = 6.0;
    caps.surface.maxReverseSpeedMps = 2.0;
    caps.surface.maxTurnRateRps = 0.25;
    sim.cycleRateHz = 20.0;
    sim.initialLatitudeDeg = 39.0;
    sim.initialLongitudeDeg = -76.5;
    sim.initialHeadingRad = 0.0;
    sim.accelMps2 = 1.0;
  }

  std::unique_ptr<arlcore::autopilot::SimVehicleControl> make() {
    return std::make_unique<arlcore::autopilot::SimVehicleControl>(
        caps, sim, arlcore::UuidFactory::getInstance().generateGuid(),
        arlcore::UuidFactory::getInstance().generateGuid(), poseIo, speedIo, velocityIo);
  }
};

static arlcore::autopilot::ControlVector makeCv(flt64_t headingRad, flt64_t speedMps) {
  arlcore::autopilot::ControlVector cv;
  cv.headingRad = headingRad;
  cv.speedMps = speedMps;
  return cv;
}

TEST(SimVehicleControlTest, PublishesAllThreeNavReportsEachStep) {
  // GIVEN: a sim vehicle at its configured initial pose
  SimFixture f;
  auto vehicle = f.make();

  // WHEN: a single simulation step runs
  vehicle->stepOnce(0.05);

  // THEN: pose, speed, and velocity reports publish with the initial values
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
  // GIVEN: a sim vehicle commanded to turn from 0 to pi/2, an error far beyond where its
  //        proportional heading loop saturates (0.25 / 0.9 = 0.28 rad)
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(M_PI_2, 0.0));

  // WHEN: the simulation runs long enough to complete the turn
  flt64_t peakRate = 0.0;
  flt64_t previousHeading = vehicle->state().headingRad;
  for (int32_t i = 0; i < 200; i++) {
    vehicle->stepOnce(0.1);
    const flt64_t heading = vehicle->state().headingRad;
    peakRate = std::max(peakRate, std::fabs(arlcore::autopilot::wrapPi(heading - previousHeading)) / 0.1);
    previousHeading = heading;
  }

  // THEN: the turn rate never exceeded the platform limit, and the heading converged
  EXPECT_LE(peakRate, 0.25 + 1e-9);
  EXPECT_GT(peakRate, 0.2);  // the limit was actually reached, so the bound is not vacuous
  EXPECT_NEAR(vehicle->state().headingRad, M_PI_2, 1e-6);
}

TEST(SimVehicleControlTest, HeadingGainOfOneOverStepReproducesAPureRateLimiter) {
  // GIVEN: a sim vehicle whose heading gain is 1/dt with no actuator lag, which is the
  //        deadbeat rate limiter the model used before it simulated a servo
  SimFixture f;
  f.sim.headingGainRpsPerRad = 1.0 / 0.1;
  f.sim.headingLagS = 0.0;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(M_PI_2, 0.0));

  // WHEN: a single 0.1 s step runs
  vehicle->stepOnce(0.1);

  // THEN: heading advances by exactly the rate limit, so the servo model is a strict
  //       generalization of the previous behavior rather than a replacement for it
  EXPECT_NEAR(vehicle->state().headingRad, 0.25 * 0.1, 1e-9);
}

TEST(SimVehicleControlTest, HeadingLoopHoldsTheProportionalSteadyStateOffset) {
  // GIVEN: a lag-free proportional heading loop and a command inside its linear region
  SimFixture f;
  f.sim.headingGainRpsPerRad = 0.9;
  f.sim.headingLagS = 0.0;
  auto vehicle = f.make();
  const flt64_t target = 0.1;  // 0.9 * 0.1 = 0.09 rad/s, well inside the 0.25 envelope
  vehicle->sendControlVector(makeCv(target, 0.0));

  // WHEN: one step runs from zero heading
  vehicle->stepOnce(0.1);

  // THEN: the turn rate is gain * error, the property a pure rate limiter cannot express and
  //       the one the tracker's curvature feedforward is sized against
  EXPECT_NEAR(vehicle->state().headingRad, 0.9 * target * 0.1, 1e-9);
}

TEST(SimVehicleControlTest, NonFiniteHeadingLoopSettingsCannotPoisonThePose) {
  // GIVEN: heading-loop settings config validation rejects but that a directly constructed
  //        SimVehicleConfig can still carry. An infinite gain times a zero heading error is NaN,
  //        and std::clamp propagates NaN straight through to the published pose.
  for (int32_t field = 0; field < 4; field++) {
    SimFixture f;
    if (field == 0) {
      f.sim.headingGainRpsPerRad = std::numeric_limits<flt64_t>::infinity();
    } else if (field == 1) {
      f.sim.headingGainRpsPerRad = std::nan("");
    } else if (field == 2) {
      f.sim.headingLagS = std::numeric_limits<flt64_t>::infinity();
    } else {
      f.sim.headingLagS = std::nan("");
    }
    auto vehicle = f.make();
    // A zero heading error is the case that produces inf * 0 rather than a saturated clamp.
    vehicle->sendControlVector(makeCv(0.0, 1.0));

    // WHEN: several steps run
    for (int32_t i = 0; i < 5; i++) {
      vehicle->stepOnce(0.1);
    }

    // THEN: the reported pose stays finite. A NaN heading here would be published as the platform
    //       position and reach every consumer on the domain.
    EXPECT_TRUE(std::isfinite(vehicle->state().headingRad)) << "field=" << field;
    EXPECT_TRUE(std::isfinite(vehicle->state().latitudeDeg)) << "field=" << field;
    EXPECT_TRUE(std::isfinite(vehicle->state().longitudeDeg)) << "field=" << field;
  }
}

TEST(SimVehicleControlTest, RespectsAccelerationAndSpeedCap) {
  // GIVEN: a sim vehicle commanded far beyond the platform speed limit
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(0.0, 100.0));  // way over the platform limit

  // WHEN: a single 0.5 s step runs
  vehicle->stepOnce(0.5);

  // THEN: speed ramps at the configured acceleration
  EXPECT_NEAR(vehicle->state().speedMps, 0.5, 1e-9);  // 1 m/s^2 * 0.5 s

  // WHEN: the ramp continues for 20 more seconds
  for (int32_t i = 0; i < 40; i++) {
    vehicle->stepOnce(0.5);
  }

  // THEN: speed is clamped at the platform cap
  EXPECT_NEAR(vehicle->state().speedMps, 6.0, 1e-9);  // clamped at maxForwardSpeed
}

TEST(SimVehicleControlTest, MovesNorthWhenCommandedNorth) {
  // GIVEN: a sim vehicle commanded due north at 4 m/s
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(0.0, 4.0));

  // WHEN: 20 seconds of simulation run
  for (int32_t i = 0; i < 200; i++) {
    vehicle->stepOnce(0.1);  // 20 s: ramps to 4 m/s then cruises north
  }

  // THEN: latitude increases, longitude holds, and the pose report matches state
  const auto st = vehicle->state();
  EXPECT_GT(st.latitudeDeg, 39.0);
  EXPECT_NEAR(st.longitudeDeg, -76.5, 1e-6);
  GlobalPoseReportType pose;
  ASSERT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_NEAR(pose.position().geodeticLatitude(), st.latitudeDeg, 1e-9);
}

TEST(SimVehicleControlTest, VelocityReportMatchesHeadingAndSpeed) {
  // GIVEN: a sim vehicle commanded due east at 2 m/s
  SimFixture f;
  auto vehicle = f.make();
  vehicle->sendControlVector(makeCv(M_PI_2, 2.0));  // east

  // WHEN: 40 seconds of simulation run
  for (int32_t i = 0; i < 400; i++) {
    vehicle->stepOnce(0.1);
  }

  // THEN: the velocity report shows 2 m/s east and no northward component
  VelocityReportType velocity;
  ASSERT_EQ(f.velocityIo->readLatest(&velocity), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_NEAR(velocity.velocity().eastSpeed(), 2.0, 1e-6);
  EXPECT_NEAR(velocity.velocity().northSpeed(), 0.0, 1e-6);
}

TEST(SimVehicleControlTest, DrivesDepthAndPublishesAltitudeAboveSeaFloor) {
  // GIVEN: an underwater-capable sim vehicle over a 50 m sea floor
  SimFixture f;
  f.caps.underwaterEnabled = true;
  f.caps.reportsAltitudeAsf = true;
  f.caps.underwater.maxDepthChangeRateMps = 1.0;
  f.sim.floorDepthM = 50.0;
  auto vehicle = f.make();

  // WHEN: a 20 m depth setpoint runs to convergence
  arlcore::autopilot::ControlVector cv = makeCv(0.0, 0.0);
  cv.elevationM = 20.0;
  cv.elevationFrame = arlcore::autopilot::ElevationFrame::DEPTH;
  vehicle->sendControlVector(cv);
  for (int32_t i = 0; i < 300; i++) {
    vehicle->stepOnce(0.1);
  }

  // THEN: depth settles at 20 m and the pose reports depth plus altitude-ASF
  EXPECT_NEAR(vehicle->state().depthM, 20.0, 1e-6);

  GlobalPoseReportType pose;
  ASSERT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  ASSERT_TRUE(pose.depth().has_value());
  ASSERT_TRUE(pose.altitudeASF().has_value());
  EXPECT_NEAR(pose.depth().value(), 20.0, 1e-6);
  EXPECT_NEAR(pose.altitudeASF().value(), 30.0, 1e-6);  // floor 50 - depth 20

  // WHEN: a 10 m above-sea-floor setpoint runs to convergence
  // Above-sea-floor setpoint converts against the configured floor: 10 m ASF = 40 m depth.
  cv.elevationM = 10.0;
  cv.elevationFrame = arlcore::autopilot::ElevationFrame::ALTITUDE_ASF;
  vehicle->sendControlVector(cv);
  for (int32_t i = 0; i < 300; i++) {
    vehicle->stepOnce(0.1);
  }

  // THEN: depth settles at 40 m
  EXPECT_NEAR(vehicle->state().depthM, 40.0, 1e-6);

  // WHEN: a 500 m depth setpoint beyond the floor runs to convergence
  cv.elevationM = 500.0;
  cv.elevationFrame = arlcore::autopilot::ElevationFrame::DEPTH;
  vehicle->sendControlVector(cv);
  for (int32_t i = 0; i < 300; i++) {
    vehicle->stepOnce(0.1);
  }

  // THEN: depth clamps at the 50 m floor
  EXPECT_NEAR(vehicle->state().depthM, 50.0, 1e-6);
}

TEST(SimVehicleControlTest, AltitudeAboveSeaFloorIsWithheldWithoutTheCapability) {
  // GIVEN: an underwater-capable sim vehicle that does not declare an altimeter
  SimFixture f;
  f.caps.underwaterEnabled = true;
  f.caps.reportsAltitudeAsf = false;
  auto vehicle = f.make();

  // WHEN: a cycle publishes the pose
  vehicle->stepOnce(0.1);

  // THEN: depth is reported but altitude above sea floor is absent, so the autopilot has no
  // seafloor reference - the sim can stand in for a platform with no bottom lock
  GlobalPoseReportType pose;
  ASSERT_EQ(f.poseIo->readLatest(&pose), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_TRUE(pose.depth().has_value());
  EXPECT_FALSE(pose.altitudeASF().has_value());
}

TEST(SimVehicleControlTest, ThreadedRunPublishesAtCycleRate) {
  // GIVEN: a sim vehicle configured to cycle at 50 Hz
  SimFixture f;
  f.sim.cycleRateHz = 50.0;
  auto vehicle = f.make();

  // WHEN: the threaded run executes for 300 ms and shuts down
  ASSERT_TRUE(vehicle->initialize());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  vehicle->shutdown();

  // THEN: multiple pose reports were published
  // ~15 cycles expected in 300 ms at 50 Hz; allow generous scheduling slop.
  int32_t count = 0;
  GlobalPoseReportType pose;
  while (f.poseIo->read(&pose) == arlcore::io::ReadStatus::SUCCESS) {
    count++;
  }
  EXPECT_GE(count, 5);

  // WHEN: shutdown repeats and the vehicle re-initializes
  // THEN: shutdown is idempotent and re-initialization succeeds
  vehicle->shutdown();
  EXPECT_TRUE(vehicle->initialize());
  vehicle->shutdown();
}
