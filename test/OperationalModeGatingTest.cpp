#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "autopilot/core/AutopilotBrain.hpp"
#include "LocalReaderSender.h"
#include "autopilot/core/NavState.hpp"
#include "autopilot/modes/OperationalModeManager.hpp"
#include "UmaaUtils.h"
#include "UuidFactory.h"
#include "autopilot/umaa/VectorControlServiceProvider.hpp"
#include "autopilot/umaa/WaypointControlServiceProvider.hpp"
#include "InternalTypes.h"

using GatingStatus = UMAA::Common::MaritimeEnumeration::
    CommandStatusEnumModule::CommandStatusEnumType;
using GatingReason = UMAA::Common::MaritimeEnumeration::
    CommandStatusReasonEnumModule::CommandStatusReasonEnumType;

class MockGatingVehicle : public arlcore::autopilot::IVehicleControl {
 public:
  MOCK_METHOD(bool, initialize, (), (override));
  MOCK_METHOD(bool, sendControlVector,
              (const arlcore::autopilot::ControlVector& cv), (override));
  MOCK_METHOD(bool, isManualEngaged, (), (const, override));
};

static arlcore::NumericGuid gatingPlatformGuid() {
  std::array<uint8_t, 16> bytes{};
  bytes[0] = 0xc0;
  bytes[15] = 0xfe;
  return arlcore::NumericGuid(bytes);
}

static arlcore::NumericGuid gatingForeignGuid() {
  std::array<uint8_t, 16> bytes{};
  bytes[0] = 0xaa;
  return arlcore::NumericGuid(bytes);
}

static UMAA::MO::GlobalVectorControl::GlobalVectorCommandType
gatingVectorCommand(const arlcore::NumericGuid& sessionId,
                    const arlcore::NumericGuid& parentId,
                    flt64_t speedMps = 2.0) {
  UMAA::MO::GlobalVectorControl::GlobalVectorCommandType cmd;
  UMAA::Common::Orientation::DirectionTrueNorthRequirementVariantType dir;
  dir.direction().direction(1.0);
  cmd.direction()
      .DirectionRequirementVariantTypeSubtypes()
      .DirectionTrueNorthRequirementVariantVariant(dir);
  UMAA::Common::Speed::GroundSpeedRequirementVariantType speed;
  speed.speed().speed(speedMps);
  cmd.speed()
      .SpeedRequirementVariantTypeSubtypes()
      .GroundSpeedRequirementVariantVariant(speed);
  cmd.sessionID(sessionId.getGuid());
  cmd.source().id(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  cmd.source().parentID(parentId.getGuid());
  cmd.timeStamp() = arlcore::umaa::getTimestamp();
  return cmd;
}

//! \brief A waypoint command whose large list never completes (unknown listID,
//! size 1), so the session dwells in COMMANDED assembling it.
static UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandType
gatingWaypointCommand(const arlcore::NumericGuid& sessionId,
                      const arlcore::NumericGuid& parentId) {
  UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandType cmd;
  UMAA::Common::LargeListMetadata metadata;
  metadata.listID(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  metadata.size(1);
  cmd.waypointsListMetadata() = metadata;
  cmd.sessionID(sessionId.getGuid());
  cmd.source().id(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  cmd.source().parentID(parentId.getGuid());
  cmd.timeStamp() = arlcore::umaa::getTimestamp();
  return cmd;
}

template <class StatusType>
static std::vector<std::pair<GatingStatus, GatingReason>> drainStatuses(
    const std::shared_ptr<arlcore::io::LocalReaderSender<StatusType>>&
        statusIo) {
  std::vector<std::pair<GatingStatus, GatingReason>> out;
  StatusType status;
  while (true) {
    const arlcore::io::ReadStatus rs = statusIo->read(&status);
    if (rs == arlcore::io::ReadStatus::NO_DATA) {
      break;
    }
    if (rs == arlcore::io::ReadStatus::SUCCESS) {
      out.emplace_back(status.commandStatus(), status.commandStatusReason());
    }
  }
  return out;
}

class OperationalModeGatingTest : public ::testing::Test {
 protected:
  //! \brief Build the manager (booted into STANDBY), brain, and both gated
  //! providers.
  void init(bool allowImplicit, bool failOutOfMode,
            flt64_t idleRevertS = 3600.0) {
    ON_CALL(vehicle_, initialize()).WillByDefault(::testing::Return(true));
    ON_CALL(vehicle_, sendControlVector(::testing::_))
        .WillByDefault(::testing::Return(true));
    ON_CALL(vehicle_, isManualEngaged())
        .WillByDefault(::testing::Return(false));

    arlcore::autopilot::OperationalModeConfig modeConfig;
    modeConfig.allowImplicitModeTransitions = allowImplicit;
    modeConfig.commandsOutOfModeAreFailed = failOutOfMode;
    modeConfig.idleRevertS = idleRevertS;
    manager_ = std::make_unique<arlcore::autopilot::OperationalModeManager>(
        modeConfig, gatingPlatformGuid());
    manager_->beginStep(false);

    arlcore::autopilot::AutopilotConfig config;
    config.platformCapabilities.surface.cruisingSpeedMps = 3.0;
    config.platformCapabilities.surface.maxForwardSpeedMps = 8.0;
    config.platformCapabilities.surface.maxTurnRateRps = 0.5;
    brain_ = std::make_unique<arlcore::autopilot::AutopilotBrain>(
        &nav_, &vehicle_, config);

    vecCmdIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalVectorCommandType>>();
    vecAckIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalVectorCommandAckReportType>>();
    vecStatusIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalVectorCommandStatusType>>();
    vecExeIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalVectorExecutionStatusReportType>>();
    vectorProvider_ = std::make_unique<
        arlcore::autopilot::VectorControlServiceProvider>(
        arlcore::UuidFactory::getInstance().generateGuid(),
        std::make_shared<arlcore::autopilot::VectorControlServiceProviderIo>(
            vecCmdIo_, vecAckIo_, vecStatusIo_, vecExeIo_),
        brain_.get(), 8.0, nullptr, manager_.get());

    wpCmdIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalWaypointCommandType>>();
    wpAckIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalWaypointCommandAckReportType>>();
    wpStatusIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalWaypointCommandStatusType>>();
    wpExeIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalWaypointExecutionStatusReportType>>();
    wpElementIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::GlobalWaypointCommandTypeWaypointsListElement>>();
    waypointProvider_ = std::make_unique<
        arlcore::autopilot::WaypointControlServiceProvider>(
        arlcore::UuidFactory::getInstance().generateGuid(),
        std::make_shared<arlcore::autopilot::WaypointControlServiceProviderIo>(
            wpCmdIo_, wpAckIo_, wpStatusIo_, wpExeIo_, wpElementIo_),
        brain_.get(), 8.0, /*maxListWaitCycles=*/50, nullptr, nullptr,
        manager_.get());
  }

  arlcore::autopilot::NavState nav_;
  ::testing::NiceMock<MockGatingVehicle> vehicle_;
  std::unique_ptr<arlcore::autopilot::OperationalModeManager> manager_;
  std::unique_ptr<arlcore::autopilot::AutopilotBrain> brain_;

  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalVectorCommandType>>
      vecCmdIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalVectorCommandAckReportType>>
      vecAckIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalVectorCommandStatusType>>
      vecStatusIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalVectorExecutionStatusReportType>>
      vecExeIo_;
  std::unique_ptr<arlcore::autopilot::VectorControlServiceProvider>
      vectorProvider_;

  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalWaypointCommandType>>
      wpCmdIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalWaypointCommandAckReportType>>
      wpAckIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalWaypointCommandStatusType>>
      wpStatusIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalWaypointExecutionStatusReportType>>
      wpExeIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::GlobalWaypointCommandTypeWaypointsListElement>>
      wpElementIo_;
  std::unique_ptr<arlcore::autopilot::WaypointControlServiceProvider>
      waypointProvider_;
};

TEST_F(OperationalModeGatingTest,
       ImplicitLocalVectorEntersAutonomousAndExecutes) {
  // GIVEN: STANDBY, implicit transitions on, fail policy
  init(true, true);

  // WHEN: a local-classified vector command arrives
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();

  // THEN: the command runs ISSUED -> COMMANDED -> EXECUTING in one cycle and
  // the mode hopped to AUTONOMOUS with the vector provider holding the driving
  // resource
  const auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 3u);
  EXPECT_EQ(statuses[0].first, GatingStatus::ISSUED);
  EXPECT_EQ(statuses[1].first, GatingStatus::COMMANDED);
  EXPECT_EQ(statuses[2].first, GatingStatus::EXECUTING);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
  EXPECT_EQ(brain_->arbiter().currentHolder(),
            arlcore::autopilot::DriveSource::VECTOR);
}

TEST_F(OperationalModeGatingTest, FailPolicyRejectsLocalCommandInRemote) {
  // GIVEN: explicit REMOTE mode under the fail policy
  init(true, true);
  ASSERT_TRUE(
      manager_->commandMode(arlcore::autopilot::OperationalMode::REMOTE));

  // WHEN: a local-classified vector command arrives
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();

  // THEN: it fails validation from ISSUED and the mode never moves
  const auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[0].first, GatingStatus::ISSUED);
  EXPECT_EQ(statuses[1].first, GatingStatus::FAILED);
  EXPECT_EQ(statuses[1].second, GatingReason::VALIDATION_FAILED);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::REMOTE);
}

TEST_F(OperationalModeGatingTest, HoldPolicyParksThenExplicitReleaseExecutes) {
  // GIVEN: explicit REMOTE mode under the hold policy
  init(true, false);
  ASSERT_TRUE(
      manager_->commandMode(arlcore::autopilot::OperationalMode::REMOTE));

  // WHEN: a local-classified vector command arrives
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();

  // THEN: only the ISSUED status is published and further cycles stay silent
  // (held)
  auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 1u);
  EXPECT_EQ(statuses[0].first, GatingStatus::ISSUED);
  vectorProvider_->cycle();
  vectorProvider_->cycle();
  EXPECT_TRUE(drainStatuses(vecStatusIo_).empty());

  // WHEN: AUTONOMOUS is explicitly commanded
  ASSERT_TRUE(
      manager_->commandMode(arlcore::autopilot::OperationalMode::AUTONOMOUS));
  vectorProvider_->cycle();

  // THEN: the held command advances to EXECUTING in that cycle
  statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[0].first, GatingStatus::COMMANDED);
  EXPECT_EQ(statuses[1].first, GatingStatus::EXECUTING);
}

TEST_F(OperationalModeGatingTest, ExplicitStandbyFlushesHeldLocalCommand) {
  // GIVEN: a local command held at ISSUED in REMOTE under the hold policy
  init(true, false);
  ASSERT_TRUE(
      manager_->commandMode(arlcore::autopilot::OperationalMode::REMOTE));
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();
  ASSERT_EQ(drainStatuses(vecStatusIo_).size(), 1u);  // ISSUED only

  // WHEN: STANDBY is explicitly commanded (an authoritative lockdown)
  ASSERT_TRUE(
      manager_->commandMode(arlcore::autopilot::OperationalMode::STANDBY));
  vectorProvider_->cycle();

  // THEN: the held command is flushed with INTERRUPTED and no implicit re-entry
  // occurs
  const auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 1u);
  EXPECT_EQ(statuses[0].first, GatingStatus::FAILED);
  EXPECT_EQ(statuses[0].second, GatingReason::INTERRUPTED);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::STANDBY);
}

TEST_F(OperationalModeGatingTest, IdleRevertReleasesHeldCommandTwoStep) {
  // GIVEN: implicit REMOTE (from a remote command since canceled) holding a
  // local command
  init(true, false, 0.0);
  const arlcore::NumericGuid remoteSession =
      arlcore::UuidFactory::getInstance().generateGuid();
  const auto remoteCmd =
      gatingVectorCommand(remoteSession, gatingForeignGuid());
  vecCmdIo_->send(remoteCmd);
  vectorProvider_->cycle();
  ASSERT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::REMOTE);
  vecCmdIo_->dispose(remoteCmd);
  vectorProvider_->cycle();
  drainStatuses(vecStatusIo_);

  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();
  const auto heldStatuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(heldStatuses.size(), 1u);
  ASSERT_EQ(heldStatuses[0].first, GatingStatus::ISSUED);
  ASSERT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::REMOTE);

  // WHEN: the remote class goes idle and the implicit mode reverts
  // (idle_revert_s = 0)
  manager_->endStep(/*localCommandActive=*/true, /*remoteCommandActive=*/false);
  ASSERT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::STANDBY);
  vectorProvider_->cycle();

  // THEN: the held local command implicitly re-enters AUTONOMOUS and executes
  // (two-step)
  const auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[0].first, GatingStatus::COMMANDED);
  EXPECT_EQ(statuses[1].first, GatingStatus::EXECUTING);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
}

TEST_F(OperationalModeGatingTest,
       RemoteVectorPreemptsExecutingLocalVectorSameProvider) {
  // GIVEN: a local vector command executing in implicit AUTONOMOUS
  init(true, true);
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();
  drainStatuses(vecStatusIo_);

  // WHEN: a remote vector command arrives at the same provider
  vecCmdIo_->send(gatingVectorCommand(
      arlcore::UuidFactory::getInstance().generateGuid(), gatingForeignGuid()));
  vectorProvider_->cycle();

  // THEN: the local session is CANCELED (same-provider CANCEL_EXISTING), the
  // mode hops to REMOTE, and the remote command executes in the same cycle
  const auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 4u);
  EXPECT_EQ(statuses[0].first, GatingStatus::CANCELED);
  EXPECT_EQ(statuses[1].first, GatingStatus::ISSUED);
  EXPECT_EQ(statuses[2].first, GatingStatus::COMMANDED);
  EXPECT_EQ(statuses[3].first, GatingStatus::EXECUTING);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::REMOTE);
}

TEST_F(OperationalModeGatingTest, RemoteWaypointPreemptsLocalVectorViaArbiter) {
  // GIVEN: a local vector command executing in implicit AUTONOMOUS
  init(true, true);
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();
  drainStatuses(vecStatusIo_);

  // WHEN: a remote waypoint command arrives (its large list still assembling)
  wpCmdIo_->send(gatingWaypointCommand(
      arlcore::UuidFactory::getInstance().generateGuid(), gatingForeignGuid()));
  waypointProvider_->cycle();
  vectorProvider_->cycle();

  // THEN: the waypoint command acquired the resource (never RESOURCE_REJECTED)
  // and the local vector command was interrupted on its next cycle
  const auto wpStatuses = drainStatuses(wpStatusIo_);
  ASSERT_EQ(wpStatuses.size(), 2u);
  EXPECT_EQ(wpStatuses[0].first, GatingStatus::ISSUED);
  EXPECT_EQ(wpStatuses[1].first, GatingStatus::COMMANDED);
  const auto vecStatuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(vecStatuses.size(), 1u);
  EXPECT_EQ(vecStatuses[0].first, GatingStatus::FAILED);
  EXPECT_EQ(vecStatuses[0].second, GatingReason::INTERRUPTED);
  EXPECT_EQ(brain_->arbiter().currentHolder(),
            arlcore::autopilot::DriveSource::WAYPOINT);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::REMOTE);
}

TEST_F(OperationalModeGatingTest,
       ManualEngageInterruptsExecutingAndRejectsNewCommands) {
  // GIVEN: a local vector command executing in implicit AUTONOMOUS
  init(true, true);
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();
  drainStatuses(vecStatusIo_);

  // WHEN: the platform engages manual control
  ON_CALL(vehicle_, isManualEngaged()).WillByDefault(::testing::Return(true));
  manager_->beginStep(true);
  vectorProvider_->cycle();

  // THEN: the executing command is interrupted
  auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 1u);
  EXPECT_EQ(statuses[0].first, GatingStatus::FAILED);
  EXPECT_EQ(statuses[0].second, GatingReason::INTERRUPTED);

  // WHEN: a new command arrives while MANUAL (even under a hold-free fail
  // policy)
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid()));
  vectorProvider_->cycle();

  // THEN: it fails validation immediately
  statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[1].first, GatingStatus::FAILED);
  EXPECT_EQ(statuses[1].second, GatingReason::VALIDATION_FAILED);
}

TEST_F(OperationalModeGatingTest,
       ExplicitStandbyInterruptsWaypointMidListAssembly) {
  // GIVEN: a local waypoint command dwelling in COMMANDED while its list
  // assembles
  init(true, true);
  wpCmdIo_->send(
      gatingWaypointCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                            gatingPlatformGuid()));
  waypointProvider_->cycle();
  waypointProvider_->cycle();
  const auto setupStatuses = drainStatuses(wpStatusIo_);
  ASSERT_EQ(setupStatuses.size(), 2u);
  ASSERT_EQ(setupStatuses[1].first, GatingStatus::COMMANDED);
  ASSERT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);

  // WHEN: STANDBY is explicitly commanded
  ASSERT_TRUE(
      manager_->commandMode(arlcore::autopilot::OperationalMode::STANDBY));
  waypointProvider_->cycle();

  // THEN: the command fails INTERRUPTED from COMMANDED and the resource is
  // released
  const auto statuses = drainStatuses(wpStatusIo_);
  ASSERT_EQ(statuses.size(), 1u);
  EXPECT_EQ(statuses[0].first, GatingStatus::FAILED);
  EXPECT_EQ(statuses[0].second, GatingReason::INTERRUPTED);
  EXPECT_EQ(brain_->arbiter().currentHolder(),
            arlcore::autopilot::DriveSource::NONE);
}

TEST_F(OperationalModeGatingTest, UpdateWhileDisallowedParksAndReleasesResources) {
  // GIVEN: hold policy; a local vector command EXECUTING in implicit AUTONOMOUS
  init(true, false);
  const arlcore::NumericGuid session = arlcore::UuidFactory::getInstance().generateGuid();
  vecCmdIo_->send(gatingVectorCommand(session, gatingPlatformGuid()));
  vectorProvider_->cycle();
  drainStatuses(vecStatusIo_);
  ASSERT_EQ(brain_->arbiter().currentHolder(), arlcore::autopilot::DriveSource::VECTOR);

  // WHEN: REMOTE is explicitly commanded and an update to the same session arrives before
  // the provider's next cycle (the update path re-enters ISSUED ahead of the interrupt)
  ASSERT_TRUE(manager_->commandMode(arlcore::autopilot::OperationalMode::REMOTE));
  vecCmdIo_->send(gatingVectorCommand(session, gatingPlatformGuid()));
  vectorProvider_->cycle();

  // THEN: the session parks held at ISSUED and everything it had acquired is released —
  // the brain must not keep driving a local setpoint in REMOTE mode
  EXPECT_EQ(brain_->arbiter().currentHolder(), arlcore::autopilot::DriveSource::NONE);
  auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_FALSE(statuses.empty());
  EXPECT_EQ(statuses.back().first, GatingStatus::ISSUED);  // the UPDATED re-issue, then held

  // WHEN: AUTONOMOUS is explicitly commanded
  ASSERT_TRUE(manager_->commandMode(arlcore::autopilot::OperationalMode::AUTONOMOUS));
  vectorProvider_->cycle();

  // THEN: the held command resumes and re-acquires the driving resource
  statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[0].first, GatingStatus::COMMANDED);
  EXPECT_EQ(statuses[1].first, GatingStatus::EXECUTING);
  EXPECT_EQ(brain_->arbiter().currentHolder(), arlcore::autopilot::DriveSource::VECTOR);
}

TEST_F(OperationalModeGatingTest, InvalidContentNeverMovesTheMode) {
  // GIVEN: STANDBY with implicit transitions enabled
  init(true, true);

  // WHEN: a local vector command with an over-limit speed arrives (9 > platform
  // max 8)
  vecCmdIo_->send(
      gatingVectorCommand(arlcore::UuidFactory::getInstance().generateGuid(),
                          gatingPlatformGuid(), /*speedMps=*/9.0));
  vectorProvider_->cycle();

  // THEN: it fails validation and the invalid command never fired the implicit
  // transition
  const auto statuses = drainStatuses(vecStatusIo_);
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[1].first, GatingStatus::FAILED);
  EXPECT_EQ(statuses[1].second, GatingReason::VALIDATION_FAILED);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::STANDBY);
}
