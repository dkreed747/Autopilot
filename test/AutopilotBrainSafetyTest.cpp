#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "autopilot/core/AutopilotBrain.hpp"

namespace arlcore::autopilot {

//! Captures every control vector the brain emits.
class FakeVehicle : public IVehicleControl {
 public:
  bool initialize() override { return true; }
  bool sendControlVector(const ControlVector& cv) override {
    last = cv;
    ++sendCount;
    return true;
  }
  bool isManualEngaged() const override { return manualEngaged; }

  std::optional<ControlVector> last;
  int sendCount = 0;
  bool manualEngaged = false;
};

class FakeConstraintSource : public IConstraintSource {
 public:
  ConstraintSnapshot snapshot() const override { return snapshot_; }
  uint64_t revision() const override { return snapshot_.revision; }
  void set(const ConstraintSnapshot& s) { snapshot_ = s; }

 private:
  ConstraintSnapshot snapshot_;
};

static AutopilotConfig testConfig() {
  AutopilotConfig config;
  config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
  config.platformCapabilities.surface.maxForwardSpeedMps = 8.0;
  config.platformCapabilities.surface.maxTurnRateRps = 0.5;
  return config;
}

static UMAA::MO::GlobalVectorControl::GlobalVectorCommandType vectorCommand(
    double headingRad, double speedMps, std::optional<double> depthM = std::nullopt) {
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
    elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant().depth()
        .depth(depthM.value());
    cmd.elevation() = elev;
  }
  return cmd;
}

static UMAA::SA::GlobalPoseStatus::GlobalPoseReportType poseAt(double yawRad) {
  UMAA::SA::GlobalPoseStatus::GlobalPoseReportType pose;
  pose.position().geodeticLatitude(39.0);
  pose.position().geodeticLongitude(-76.5);
  pose.attitude().yaw().yaw(yawRad);
  return pose;
}

class AutopilotBrainSafetyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    brain_ = std::make_unique<AutopilotBrain>(&nav_, &vehicle_, testConfig());
    nav_.setPose(poseAt(0.0));
  }

  NavState nav_;
  FakeVehicle vehicle_;
  std::unique_ptr<AutopilotBrain> brain_;
  FakeConstraintSource source_;
};

TEST_F(AutopilotBrainSafetyTest, NoConstraintSourceEmitsUnclamped) {
  brain_->setVectorSetpoint(vectorCommand(1.0, 6.0));
  brain_->onNavUpdate();
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 6.0);
}

TEST_F(AutopilotBrainSafetyTest, ManualEngagedSuppressesAllActuation) {
  // GIVEN: an installed vector setpoint driving the vehicle
  brain_->setVectorSetpoint(vectorCommand(1.0, 3.0));
  brain_->onNavUpdate();
  ASSERT_TRUE(vehicle_.last.has_value());
  const int sendsBefore = vehicle_.sendCount;

  // WHEN: the platform engages manual control and control paths keep running
  vehicle_.manualEngaged = true;
  brain_->onNavUpdate();
  brain_->clearSetpoint(DriveSource::VECTOR);  // would normally emit a zero-speed hold
  brain_->activateSafeHold();                  // even safe mode must not actuate

  // THEN: nothing reached the platform while manual was engaged
  EXPECT_EQ(vehicle_.sendCount, sendsBefore);

  // WHEN: manual is released
  vehicle_.manualEngaged = false;
  brain_->activateSafeHold();

  // THEN: actuation resumes
  EXPECT_GT(vehicle_.sendCount, sendsBefore);
}

TEST_F(AutopilotBrainSafetyTest, StaticPlatformCapClampsWhenSourceInstalled) {
  brain_->setConstraintSource(&source_);  // empty dynamic snapshot; static caps still apply
  brain_->setVectorSetpoint(vectorCommand(1.0, 12.0));
  brain_->onNavUpdate();
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 8.0);  // platform max_forward_speed_mps
}

TEST_F(AutopilotBrainSafetyTest, DynamicConstraintClampsSpeedAndDepth) {
  ConstraintSnapshot snapshot;
  snapshot.revision = 1;
  snapshot.maxSpeedMps = 2.0;
  snapshot.maxDepthM = 20.0;
  source_.set(snapshot);
  brain_->setConstraintSource(&source_);

  brain_->setVectorSetpoint(vectorCommand(1.0, 6.0, 30.0));
  brain_->onNavUpdate();
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 2.0);
  ASSERT_TRUE(vehicle_.last->elevationM.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->elevationM.value(), 20.0);
  EXPECT_DOUBLE_EQ(vehicle_.last->headingRad, 1.0);  // heading untouched

  // Constraint relaxes: the very next emit follows the new snapshot.
  snapshot.maxSpeedMps = 5.0;
  snapshot.revision = 2;
  source_.set(snapshot);
  brain_->onNavUpdate();
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 5.0);
}

TEST_F(AutopilotBrainSafetyTest, ZeroSpeedHoldNeverRaisedByMinSpeed) {
  ConstraintSnapshot snapshot;
  snapshot.minSpeedMps = 1.5;
  source_.set(snapshot);
  brain_->setConstraintSource(&source_);

  brain_->setVectorSetpoint(vectorCommand(1.0, 3.0));
  brain_->clearSetpoint(DriveSource::VECTOR);  // emits the zero-speed hold
  ASSERT_TRUE(vehicle_.last.has_value());
  EXPECT_DOUBLE_EQ(vehicle_.last->speedMps, 0.0);
}

}  // namespace arlcore::autopilot
