#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <thread>
#include <vector>

#include "InternalTypes.h"
#include "autopilot/core/AutopilotBrain.hpp"

//! Captures every control vector the brain emits.
class BrainTestMockVehicle : public arlcore::autopilot::IVehicleControl {
 public:
  BrainTestMockVehicle() {
    ON_CALL(*this, initialize()).WillByDefault(::testing::Return(true));
    ON_CALL(*this, sendControlVector(::testing::_))
        .WillByDefault(::testing::Invoke([this](const arlcore::autopilot::ControlVector& cv) {
          last = cv;
          ++sendCount;
          return true;
        }));
    ON_CALL(*this, isManualEngaged()).WillByDefault(::testing::ReturnPointee(&manualEngaged));
  }

  MOCK_METHOD(bool, initialize, (), (override));
  MOCK_METHOD(bool, sendControlVector, (const arlcore::autopilot::ControlVector& cv), (override));
  MOCK_METHOD(bool, isManualEngaged, (), (const, override));

  std::optional<arlcore::autopilot::ControlVector> last;
  int32_t sendCount = 0;
  bool manualEngaged = false;
};

class BrainTestMockConstraintSource : public arlcore::autopilot::IConstraintSource {
 public:
  BrainTestMockConstraintSource() {
    ON_CALL(*this, snapshot()).WillByDefault(::testing::ReturnPointee(&snapshot_));
    ON_CALL(*this, revision()).WillByDefault(::testing::Invoke([this] { return snapshot_.revision; }));
  }

  MOCK_METHOD(arlcore::autopilot::ConstraintSnapshot, snapshot, (), (const, override));
  MOCK_METHOD(uint64_t, revision, (), (const, override));

  void set(const arlcore::autopilot::ConstraintSnapshot& s) { snapshot_ = s; }

 private:
  arlcore::autopilot::ConstraintSnapshot snapshot_;
};

static arlcore::autopilot::AutopilotConfig testConfig() {
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  config.platformCapabilities.surface.maxForwardSpeedMps = 8.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.5;
  return config;
}

static UMAA::MO::GlobalVectorControl::GlobalVectorCommandType vectorCommand(
    flt64_t headingRad, flt64_t speedMps, std::optional<flt64_t> depthM = std::nullopt) {
  UMAA::MO::GlobalVectorControl::GlobalVectorCommandType cmd;
  UMAA::Common::Orientation::DirectionTrueNorthRequirementVariantType dir;
  dir.direction().direction(headingRad);
  cmd.direction().DirectionRequirementVariantTypeSubtypes().DirectionTrueNorthRequirementVariantVariant(dir);
  UMAA::Common::Speed::GroundSpeedRequirementVariantType speed;
  speed.speed().speed(speedMps);
  cmd.speed().SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant(speed);
  if (depthM.has_value()) {
    UMAA::Common::Measurement::ElevationRequirementVariantType elev;
    elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant(
        UMAA::Common::Measurement::DepthRequirementVariantType());
    elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant().depth().depth(depthM.value());
    cmd.elevation() = elev;
  }
  return cmd;
}

static UMAA::SA::GlobalPoseStatus::GlobalPoseReportType poseAt(flt64_t yawRad) {
  UMAA::SA::GlobalPoseStatus::GlobalPoseReportType pose;
  pose.position().geodeticLatitude(39.0);
  pose.position().geodeticLongitude(-76.5);
  pose.attitude().yaw().yaw(yawRad);
  return pose;
}

class AutopilotBrainSafetyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    brain_ = std::make_unique<arlcore::autopilot::AutopilotBrain>(&nav_, &vehicle_, testConfig());
    nav_.setPose(poseAt(0.0));
  }

  arlcore::autopilot::NavState nav_;
  ::testing::NiceMock<BrainTestMockVehicle> vehicle_;
  std::unique_ptr<arlcore::autopilot::AutopilotBrain> brain_;
  ::testing::NiceMock<BrainTestMockConstraintSource> source_;
};

TEST_F(AutopilotBrainSafetyTest, NoConstraintSourceEmitsUnclamped) {
  // GIVEN: a brain with no constraint source and an installed vector setpoint
  brain_->setVectorSetpoint(vectorCommand(1.0, 6.0));

  // WHEN: a nav update drives the control path
  brain_->onNavUpdate();

  // THEN: the commanded speed reaches the vehicle unclamped
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 6.0);
}

TEST_F(AutopilotBrainSafetyTest, ManualEngagedSuppressesAllActuation) {
  // GIVEN: an installed vector setpoint driving the vehicle
  brain_->setVectorSetpoint(vectorCommand(1.0, 3.0));
  brain_->onNavUpdate();
  ASSERT_TRUE(vehicle_.last.has_value());
  const int32_t sendsBefore = vehicle_.sendCount;

  // WHEN: the platform engages manual control and control paths keep running
  vehicle_.manualEngaged = true;
  brain_->onNavUpdate();
  brain_->clearSetpoint(arlcore::autopilot::DriveSource::VECTOR);  // would normally emit a zero-speed hold
  brain_->activateSafeHold();                                      // even safe mode must not actuate

  // THEN: nothing reached the platform while manual was engaged
  EXPECT_EQ(vehicle_.sendCount, sendsBefore);

  // WHEN: manual is released
  vehicle_.manualEngaged = false;
  brain_->activateSafeHold();

  // THEN: actuation resumes
  EXPECT_GT(vehicle_.sendCount, sendsBefore);
}

TEST_F(AutopilotBrainSafetyTest, StaticPlatformCapClampsWhenSourceInstalled) {
  // GIVEN: an installed constraint source with an empty dynamic snapshot; static caps still apply
  brain_->setConstraintSource(&source_);
  brain_->setVectorSetpoint(vectorCommand(1.0, 12.0));

  // WHEN: the brain emits on a nav update
  brain_->onNavUpdate();

  // THEN: the platform max_forward_speed_mps clamps the command
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 8.0);
}

TEST_F(AutopilotBrainSafetyTest, DynamicConstraintClampsSpeedAndDepth) {
  // GIVEN: a snapshot capping speed at 2 m/s and depth at 20 m
  arlcore::autopilot::ConstraintSnapshot snapshot;
  snapshot.revision = 1;
  snapshot.maxSpeedMps = 2.0;
  snapshot.maxDepthM = 20.0;
  source_.set(snapshot);
  brain_->setConstraintSource(&source_);

  // WHEN: a faster, deeper vector command is emitted
  brain_->setVectorSetpoint(vectorCommand(1.0, 6.0, 30.0));
  brain_->onNavUpdate();

  // THEN: speed and depth are clamped, heading untouched
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 2.0);
  ASSERT_TRUE(vehicle_.last->elevationM.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->elevationM.value(), 20.0);
  EXPECT_DOUBLE_EQ(vehicle_.last->headingRad, 1.0);

  // WHEN: the constraint relaxes to 5 m/s
  snapshot.maxSpeedMps = 5.0;
  snapshot.revision = 2;
  source_.set(snapshot);
  brain_->onNavUpdate();

  // THEN: the very next emit follows the new snapshot
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 5.0);
}

TEST_F(AutopilotBrainSafetyTest, ZeroSpeedHoldNeverRaisedByMinSpeed) {
  // GIVEN: a snapshot with a 1.5 m/s minimum speed and an active vector setpoint
  arlcore::autopilot::ConstraintSnapshot snapshot;
  snapshot.minSpeedMps = 1.5;
  source_.set(snapshot);
  brain_->setConstraintSource(&source_);
  brain_->setVectorSetpoint(vectorCommand(1.0, 3.0));

  // WHEN: clearing the setpoint emits the zero-speed hold
  brain_->clearSetpoint(arlcore::autopilot::DriveSource::VECTOR);

  // THEN: the hold stays at zero speed
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 0.0);
}

TEST_F(AutopilotBrainSafetyTest, StalePoseCommandsZeroSpeedHold) {
  // GIVEN: an active vector command with a 1 ms staleness budget and an aging pose
  arlcore::autopilot::AutopilotConfig config = testConfig();
  config.loop.navStalenessTimeoutMs = 1;
  brain_ = std::make_unique<arlcore::autopilot::AutopilotBrain>(&nav_, &vehicle_, config);
  brain_->setVectorSetpoint(vectorCommand(1.0, 3.0));
  brain_->onNavUpdate();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  vehicle_.last.reset();

  // WHEN: the staleness guard runs with no fresh pose
  brain_->enforceNavStaleness();

  // THEN: a zero-speed hold is commanded
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 0.0);
}

TEST_F(AutopilotBrainSafetyTest, StalePoseGuardSkipsIdleVehicle) {
  // GIVEN: no command installed (mode NONE) and an aging pose
  arlcore::autopilot::AutopilotConfig config = testConfig();
  config.loop.navStalenessTimeoutMs = 1;
  brain_ = std::make_unique<arlcore::autopilot::AutopilotBrain>(&nav_, &vehicle_, config);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  vehicle_.last.reset();

  // WHEN: the staleness guard runs
  brain_->enforceNavStaleness();

  // THEN: nothing is commanded (there is nothing to hold against)
  EXPECT_FALSE(vehicle_.last.has_value());
}
