#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "LocalReaderSender.h"
#include "autopilot/modes/OperationalModeControlProvider.hpp"
#include "UmaaUtils.h"
#include "UuidFactory.h"

using ModeCmdEnum = UMAA::Common::MaritimeEnumeration::
    OperationalModeControlEnumModule::OperationalModeControlEnumType;
using ModeCmdStatus = UMAA::Common::MaritimeEnumeration::
    CommandStatusEnumModule::CommandStatusEnumType;
using ModeCmdReason = UMAA::Common::MaritimeEnumeration::
    CommandStatusReasonEnumModule::CommandStatusReasonEnumType;

static arlcore::NumericGuid modeTestPlatformGuid() {
  std::array<uint8_t, 16> bytes{};
  bytes[7] = 0x77;
  return arlcore::NumericGuid(bytes);
}

static UMAA::MM::OperationalModeControl::OperationalModeCommandType modeCommand(
    ModeCmdEnum mode, const arlcore::NumericGuid& parentId) {
  UMAA::MM::OperationalModeControl::OperationalModeCommandType cmd;
  cmd.operationalMode() = mode;
  cmd.sessionID(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  cmd.source().id(arlcore::UuidFactory::getInstance().generateGuid().getGuid());
  cmd.source().parentID(parentId.getGuid());
  cmd.timeStamp() = arlcore::umaa::getTimestamp();
  return cmd;
}

class OperationalModeControlProviderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arlcore::autopilot::OperationalModeConfig modeConfig;
    manager_ = std::make_unique<arlcore::autopilot::OperationalModeManager>(
        modeConfig, modeTestPlatformGuid());
    manager_->setModeChangedCallback(
        [this](arlcore::autopilot::OperationalMode mode) {
          reported_.push_back(mode);
        });
    manager_->beginStep(false);

    cmdIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::OperationalModeCommandType>>();
    ackIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::OperationalModeCommandAckReportType>>();
    statusIo_ = std::make_shared<arlcore::io::LocalReaderSender<
        arlcore::autopilot::OperationalModeCommandStatusType>>();
    provider_ = std::make_unique<
        arlcore::autopilot::OperationalModeControlProvider>(
        arlcore::UuidFactory::getInstance().generateGuid(),
        std::make_shared<arlcore::autopilot::OperationalModeControlProviderIo>(
            cmdIo_, ackIo_, statusIo_),
        manager_.get());
  }

  std::vector<std::pair<ModeCmdStatus, ModeCmdReason>> drain() {
    std::vector<std::pair<ModeCmdStatus, ModeCmdReason>> out;
    arlcore::autopilot::OperationalModeCommandStatusType status;
    while (true) {
      const arlcore::io::ReadStatus rs = statusIo_->read(&status);
      if (rs == arlcore::io::ReadStatus::NO_DATA) {
        break;
      }
      if (rs == arlcore::io::ReadStatus::SUCCESS) {
        out.emplace_back(status.commandStatus(), status.commandStatusReason());
      }
    }
    return out;
  }

  std::unique_ptr<arlcore::autopilot::OperationalModeManager> manager_;
  std::vector<arlcore::autopilot::OperationalMode> reported_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::OperationalModeCommandType>>
      cmdIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::OperationalModeCommandAckReportType>>
      ackIo_;
  std::shared_ptr<arlcore::io::LocalReaderSender<
      arlcore::autopilot::OperationalModeCommandStatusType>>
      statusIo_;
  std::unique_ptr<arlcore::autopilot::OperationalModeControlProvider> provider_;
};

TEST_F(OperationalModeControlProviderTest, CommandRunsToCompletedInOneCycle) {
  // GIVEN: the provider in STANDBY
  // WHEN: an AUTONOMOUS mode command arrives
  cmdIo_->send(modeCommand(ModeCmdEnum::AUTONOMOUS, modeTestPlatformGuid()));
  provider_->cycle();

  // THEN: the full ISSUED -> COMMANDED -> EXECUTING -> COMPLETED sequence
  // publishes in one cycle, an ack was sent, and the manager applied the mode
  // (reported once)
  const auto statuses = drain();
  ASSERT_EQ(statuses.size(), 4u);
  EXPECT_EQ(statuses[0].first, ModeCmdStatus::ISSUED);
  EXPECT_EQ(statuses[1].first, ModeCmdStatus::COMMANDED);
  EXPECT_EQ(statuses[2].first, ModeCmdStatus::EXECUTING);
  EXPECT_EQ(statuses[3].first, ModeCmdStatus::COMPLETED);
  EXPECT_EQ(statuses[3].second, ModeCmdReason::SUCCEEDED);
  arlcore::autopilot::OperationalModeCommandAckReportType ack;
  EXPECT_EQ(ackIo_->read(&ack), arlcore::io::ReadStatus::SUCCESS);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
  EXPECT_EQ(reported_.back(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
}

TEST_F(OperationalModeControlProviderTest,
       SameModeCommandCompletesWithoutReport) {
  // GIVEN: the provider already in STANDBY
  const size_t reportsBefore = reported_.size();

  // WHEN: STANDBY is commanded again
  cmdIo_->send(modeCommand(ModeCmdEnum::STANDBY, modeTestPlatformGuid()));
  provider_->cycle();

  // THEN: the command completes trivially and no mode change is reported
  const auto statuses = drain();
  ASSERT_EQ(statuses.size(), 4u);
  EXPECT_EQ(statuses[3].first, ModeCmdStatus::COMPLETED);
  EXPECT_EQ(reported_.size(), reportsBefore);
}

TEST_F(OperationalModeControlProviderTest, CommandFailsValidationInManual) {
  // GIVEN: the platform engaged manual control
  manager_->beginStep(true);
  ASSERT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::MANUAL);

  // WHEN: a mode command arrives
  cmdIo_->send(modeCommand(ModeCmdEnum::REMOTE, modeTestPlatformGuid()));
  provider_->cycle();

  // THEN: it fails validation from ISSUED and the mode stays MANUAL
  const auto statuses = drain();
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[0].first, ModeCmdStatus::ISSUED);
  EXPECT_EQ(statuses[1].first, ModeCmdStatus::FAILED);
  EXPECT_EQ(statuses[1].second, ModeCmdReason::VALIDATION_FAILED);
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::MANUAL);
}

TEST_F(OperationalModeControlProviderTest,
       RemoteClassifiedSourceMayCommandAutonomous) {
  // GIVEN: a commander whose parentID does NOT match the platform (a remote
  // operator)
  std::array<uint8_t, 16> foreign{};
  foreign[0] = 0x99;

  // WHEN: it commands AUTONOMOUS
  cmdIo_->send(
      modeCommand(ModeCmdEnum::AUTONOMOUS, arlcore::NumericGuid(foreign)));
  provider_->cycle();

  // THEN: classification does not apply to mode commands; the transition is
  // honored
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
}

TEST_F(OperationalModeControlProviderTest,
       CancelAfterCompletionDoesNotRevertTheMode) {
  // GIVEN: a completed REMOTE mode command
  const auto cmd = modeCommand(ModeCmdEnum::REMOTE, modeTestPlatformGuid());
  cmdIo_->send(cmd);
  provider_->cycle();
  drain();
  ASSERT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::REMOTE);

  // WHEN: the commander disposes (cancels) the already-terminal session
  cmdIo_->dispose(cmd);
  provider_->cycle();

  // THEN: nothing changes
  EXPECT_TRUE(drain().empty());
  EXPECT_EQ(manager_->mode(), arlcore::autopilot::OperationalMode::REMOTE);
}
