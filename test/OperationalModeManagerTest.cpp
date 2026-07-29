#include <gtest/gtest.h>

#include <UMAA/Common/IdentifierType.hpp>
#include <array>
#include <cstdint>
#include <vector>

#include "NumericGuid.h"
#include "OperationalModeManager.hpp"

static arlcore::autopilot::OperationalModeConfig makeModeConfig(
    bool allowImplicit = true, bool failOutOfMode = true,
    double idleRevertS = 0.0) {
  arlcore::autopilot::OperationalModeConfig config;
  config.allowImplicitModeTransitions = allowImplicit;
  config.commandsOutOfModeAreFailed = failOutOfMode;
  config.idleRevertS = idleRevertS;
  return config;
}

static arlcore::NumericGuid platformGuid() {
  std::array<uint8_t, 16> bytes{};
  bytes[15] = 0xfe;
  return arlcore::NumericGuid(bytes);
}

//! \brief Manager plus a recorder of every mode-changed callback.
struct ManagerHarness {
  explicit ManagerHarness(
      const arlcore::autopilot::OperationalModeConfig& config)
      : manager(config, platformGuid()) {
    manager.setModeChangedCallback(
        [this](arlcore::autopilot::OperationalMode mode) {
          reported.push_back(mode);
        });
  }

  arlcore::autopilot::OperationalModeManager manager;
  std::vector<arlcore::autopilot::OperationalMode> reported;
};

TEST(OperationalModeManagerTest, BootsStandbyWhenManualDisengaged) {
  // GIVEN: a fresh manager whose platform has manual disengaged
  ManagerHarness h(makeModeConfig());

  // WHEN: the first manual poll arrives
  h.manager.beginStep(false);

  // THEN: the mode is STANDBY and the initial mode was announced exactly once
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
  ASSERT_EQ(h.reported.size(), 1u);
  EXPECT_EQ(h.reported.front(), arlcore::autopilot::OperationalMode::STANDBY);
}

TEST(OperationalModeManagerTest, BootsManualWhenEngagedAtFirstPoll) {
  // GIVEN: a fresh manager whose platform boots with manual engaged
  ManagerHarness h(makeModeConfig());

  // WHEN: the first manual poll arrives
  h.manager.beginStep(true);

  // THEN: the mode is MANUAL and was announced
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::MANUAL);
  ASSERT_EQ(h.reported.size(), 1u);
  EXPECT_EQ(h.reported.front(), arlcore::autopilot::OperationalMode::MANUAL);
}

TEST(OperationalModeManagerTest, ManualEngageWinsFromEveryUnmannedState) {
  // GIVEN: managers sitting in STANDBY, REMOTE, and AUTONOMOUS
  for (const auto setup : {arlcore::autopilot::OperationalMode::STANDBY,
                           arlcore::autopilot::OperationalMode::REMOTE,
                           arlcore::autopilot::OperationalMode::AUTONOMOUS}) {
    ManagerHarness h(makeModeConfig());
    h.manager.beginStep(false);
    if (setup != arlcore::autopilot::OperationalMode::STANDBY) {
      ASSERT_TRUE(h.manager.commandMode(setup));
    }
    const uint64_t epochBefore = h.manager.authoritativeEpoch();

    // WHEN: the platform engages manual control
    h.manager.beginStep(true);

    // THEN: the mode is MANUAL and the authoritative epoch was bumped
    EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::MANUAL);
    EXPECT_EQ(h.manager.authoritativeEpoch(), epochBefore + 1u);
    EXPECT_EQ(h.reported.back(), arlcore::autopilot::OperationalMode::MANUAL);
  }
}

TEST(OperationalModeManagerTest, ManualDisengageRevertsToStandby) {
  // GIVEN: a manager in MANUAL
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(true);

  // WHEN: the platform releases manual control
  h.manager.beginStep(false);

  // THEN: the mode is STANDBY
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
  EXPECT_EQ(h.reported.back(), arlcore::autopilot::OperationalMode::STANDBY);
}

TEST(OperationalModeManagerTest, ExplicitCommandsMoveBetweenAllUnmannedModes) {
  // GIVEN: a manager in STANDBY
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(false);

  // WHEN: REMOTE, AUTONOMOUS, and STANDBY are commanded in sequence
  EXPECT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::REMOTE));
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::REMOTE);
  EXPECT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::AUTONOMOUS));
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
  EXPECT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::STANDBY));

  // THEN: every hop was applied and reported in order
  const std::vector<arlcore::autopilot::OperationalMode> expected = {
      arlcore::autopilot::OperationalMode::STANDBY,
      arlcore::autopilot::OperationalMode::REMOTE,
      arlcore::autopilot::OperationalMode::AUTONOMOUS,
      arlcore::autopilot::OperationalMode::STANDBY};
  EXPECT_EQ(h.reported, expected);
}

TEST(OperationalModeManagerTest, ExplicitCommandsFailInManual) {
  // GIVEN: a manager in MANUAL
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(true);

  // WHEN: any unmanned mode is commanded
  // THEN: the command is refused and the mode stays MANUAL
  EXPECT_FALSE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::REMOTE));
  EXPECT_FALSE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::STANDBY));
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::MANUAL);
}

TEST(OperationalModeManagerTest, ManualIsNeverCommandable) {
  // GIVEN: a manager in STANDBY
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(false);

  // WHEN: MANUAL is requested as an explicit command
  // THEN: it is refused (MANUAL is platform-owned)
  EXPECT_FALSE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::MANUAL));
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
}

TEST(OperationalModeManagerTest,
     SameModeCommandUpgradesEntryKindWithoutReport) {
  // GIVEN: AUTONOMOUS entered implicitly by a local command
  ManagerHarness h(makeModeConfig(true, true, 0.0));
  h.manager.beginStep(false);
  ASSERT_EQ(h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL),
            arlcore::autopilot::AdmissionDecision::ADMIT);
  ASSERT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
  const size_t reportsBefore = h.reported.size();

  // WHEN: AUTONOMOUS is re-commanded explicitly and the class then goes idle
  EXPECT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::AUTONOMOUS));
  h.manager.endStep(false, false);

  // THEN: no new report was published and the now-explicit mode never
  // idle-reverts
  EXPECT_EQ(h.reported.size(), reportsBefore);
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
}

TEST(OperationalModeManagerTest,
     StandbyLocalCommandImplicitlyEntersAutonomous) {
  // GIVEN: STANDBY with implicit transitions enabled
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(false);

  // WHEN: a local command requests admission
  const auto decision =
      h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL);

  // THEN: it is admitted and the mode hopped to AUTONOMOUS with a report
  EXPECT_EQ(decision, arlcore::autopilot::AdmissionDecision::ADMIT);
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
  EXPECT_EQ(h.reported.back(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
}

TEST(OperationalModeManagerTest, StandbyRemoteCommandImplicitlyEntersRemote) {
  // GIVEN: STANDBY with implicit transitions enabled
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(false);

  // WHEN: a remote command requests admission
  const auto decision =
      h.manager.requestAdmission(arlcore::autopilot::CommandClass::REMOTE);

  // THEN: it is admitted and the mode hopped to REMOTE
  EXPECT_EQ(decision, arlcore::autopilot::AdmissionDecision::ADMIT);
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::REMOTE);
}

TEST(OperationalModeManagerTest, RemoteCommandPreemptsAutonomousImplicitly) {
  // GIVEN: AUTONOMOUS (explicitly entered) with implicit transitions enabled
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(false);
  ASSERT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::AUTONOMOUS));

  // WHEN: a remote command requests admission
  const auto decision =
      h.manager.requestAdmission(arlcore::autopilot::CommandClass::REMOTE);

  // THEN: operator precedence hops the mode to REMOTE, and the hop is implicit
  // (it later idle-reverts to STANDBY rather than restoring AUTONOMOUS)
  EXPECT_EQ(decision, arlcore::autopilot::AdmissionDecision::ADMIT);
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::REMOTE);
  h.manager.endStep(false, false);
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
}

TEST(OperationalModeManagerTest, LocalCommandNeverImplicitlyLeavesRemote) {
  // GIVEN: REMOTE mode with implicit transitions enabled
  ManagerHarness h(makeModeConfig(true, false, 60.0));
  h.manager.beginStep(false);
  ASSERT_EQ(
      h.manager.requestAdmission(arlcore::autopilot::CommandClass::REMOTE),
      arlcore::autopilot::AdmissionDecision::ADMIT);

  // WHEN: a local command requests admission
  const auto decision =
      h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL);

  // THEN: it is held and the mode stays REMOTE
  EXPECT_EQ(decision, arlcore::autopilot::AdmissionDecision::HOLD);
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::REMOTE);
}

TEST(OperationalModeManagerTest, ImplicitTransitionsDisabledHoldsInStandby) {
  // GIVEN: STANDBY with implicit transitions disabled and the hold policy
  ManagerHarness h(makeModeConfig(false, false));
  h.manager.beginStep(false);

  // WHEN: local and remote commands request admission
  // THEN: both are held, the mode never moves, and validation does not reject
  // them
  EXPECT_EQ(h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL),
            arlcore::autopilot::AdmissionDecision::HOLD);
  EXPECT_EQ(
      h.manager.requestAdmission(arlcore::autopilot::CommandClass::REMOTE),
      arlcore::autopilot::AdmissionDecision::HOLD);
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
  EXPECT_FALSE(
      h.manager.rejectedAtValidation(arlcore::autopilot::CommandClass::LOCAL));
}

TEST(OperationalModeManagerTest, FailPolicyRejectsOutOfModeAtValidation) {
  // GIVEN: STANDBY with implicit transitions disabled and the fail policy
  ManagerHarness h(makeModeConfig(false, true));
  h.manager.beginStep(false);

  // WHEN: validation is consulted for either class
  // THEN: out-of-mode commands are rejected at validation
  EXPECT_TRUE(
      h.manager.rejectedAtValidation(arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_TRUE(
      h.manager.rejectedAtValidation(arlcore::autopilot::CommandClass::REMOTE));
}

TEST(OperationalModeManagerTest, ManualRejectsAtValidationRegardlessOfPolicy) {
  // GIVEN: MANUAL under the hold policy (which would otherwise never reject)
  ManagerHarness h(makeModeConfig(true, false));
  h.manager.beginStep(true);

  // WHEN: validation is consulted
  // THEN: everything is rejected: nothing executes or holds in MANUAL
  EXPECT_TRUE(
      h.manager.rejectedAtValidation(arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_TRUE(
      h.manager.rejectedAtValidation(arlcore::autopilot::CommandClass::REMOTE));
}

TEST(OperationalModeManagerTest, WouldAdmitIsSideEffectFree) {
  // GIVEN: STANDBY with implicit transitions enabled
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(false);
  const size_t reportsBefore = h.reported.size();

  // WHEN: wouldAdmit is polled repeatedly (as isCommandValid does every cycle)
  EXPECT_TRUE(h.manager.wouldAdmit(arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_TRUE(h.manager.wouldAdmit(arlcore::autopilot::CommandClass::REMOTE));

  // THEN: the mode never moved and nothing was reported
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
  EXPECT_EQ(h.reported.size(), reportsBefore);
}

TEST(OperationalModeManagerTest, ClassAllowedMatrix) {
  // GIVEN: a manager walked through every mode
  ManagerHarness h(makeModeConfig());
  h.manager.beginStep(false);

  // THEN: STANDBY allows nothing
  EXPECT_FALSE(h.manager.classAllowed(arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_FALSE(
      h.manager.classAllowed(arlcore::autopilot::CommandClass::REMOTE));

  // THEN: REMOTE allows only remote
  ASSERT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::REMOTE));
  EXPECT_FALSE(h.manager.classAllowed(arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_TRUE(h.manager.classAllowed(arlcore::autopilot::CommandClass::REMOTE));

  // THEN: AUTONOMOUS allows only local
  ASSERT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::AUTONOMOUS));
  EXPECT_TRUE(h.manager.classAllowed(arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_FALSE(
      h.manager.classAllowed(arlcore::autopilot::CommandClass::REMOTE));

  // THEN: MANUAL allows nothing
  h.manager.beginStep(true);
  EXPECT_FALSE(h.manager.classAllowed(arlcore::autopilot::CommandClass::LOCAL));
  EXPECT_FALSE(
      h.manager.classAllowed(arlcore::autopilot::CommandClass::REMOTE));
}

TEST(OperationalModeManagerTest, ImplicitModeRevertsWhenActiveClassIdle) {
  // GIVEN: AUTONOMOUS entered implicitly, idle_revert_s = 0
  ManagerHarness h(makeModeConfig(true, true, 0.0));
  h.manager.beginStep(false);
  ASSERT_EQ(h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL),
            arlcore::autopilot::AdmissionDecision::ADMIT);

  // WHEN: the local class reports idle at the end of a tick
  h.manager.endStep(false, false);

  // THEN: the mode reverted to STANDBY and was reported
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
  EXPECT_EQ(h.reported.back(), arlcore::autopilot::OperationalMode::STANDBY);
}

TEST(OperationalModeManagerTest, ActiveClassActivitySuppressesRevert) {
  // GIVEN: AUTONOMOUS entered implicitly, idle_revert_s = 0
  ManagerHarness h(makeModeConfig(true, true, 0.0));
  h.manager.beginStep(false);
  ASSERT_EQ(h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL),
            arlcore::autopilot::AdmissionDecision::ADMIT);

  // WHEN: a local command is still active at the end of the tick
  h.manager.endStep(true, false);

  // THEN: the mode stays AUTONOMOUS
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::AUTONOMOUS);
}

TEST(OperationalModeManagerTest, WrongClassActivityDoesNotBlockRevert) {
  // GIVEN: AUTONOMOUS entered implicitly, idle_revert_s = 0, with a (held)
  // remote command active
  ManagerHarness h(makeModeConfig(true, true, 0.0));
  h.manager.beginStep(false);
  ASSERT_EQ(h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL),
            arlcore::autopilot::AdmissionDecision::ADMIT);

  // WHEN: only the remote class reports activity
  h.manager.endStep(false, true);

  // THEN: AUTONOMOUS still reverts (only the active class counts)
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::STANDBY);
}

TEST(OperationalModeManagerTest, LargeIdleRevertKeepsModeAcrossIdleTicks) {
  // GIVEN: REMOTE entered implicitly with a long idle_revert_s
  ManagerHarness h(makeModeConfig(true, true, 3600.0));
  h.manager.beginStep(false);
  ASSERT_EQ(
      h.manager.requestAdmission(arlcore::autopilot::CommandClass::REMOTE),
      arlcore::autopilot::AdmissionDecision::ADMIT);

  // WHEN: several idle ticks pass
  h.manager.endStep(false, false);
  h.manager.endStep(false, false);

  // THEN: the debounce holds the mode
  EXPECT_EQ(h.manager.mode(), arlcore::autopilot::OperationalMode::REMOTE);
}

TEST(OperationalModeManagerTest, EpochBumpsOnlyOnAuthoritativeTransitions) {
  // GIVEN: a manager in STANDBY
  ManagerHarness h(makeModeConfig(true, true, 0.0));
  h.manager.beginStep(false);
  const uint64_t epoch0 = h.manager.authoritativeEpoch();

  // WHEN: an implicit transition fires and then idle-reverts
  ASSERT_EQ(h.manager.requestAdmission(arlcore::autopilot::CommandClass::LOCAL),
            arlcore::autopilot::AdmissionDecision::ADMIT);
  h.manager.endStep(false, false);

  // THEN: passive transitions never bumped the epoch
  EXPECT_EQ(h.manager.authoritativeEpoch(), epoch0);

  // WHEN: an explicit command and then a manual engagement occur
  ASSERT_TRUE(
      h.manager.commandMode(arlcore::autopilot::OperationalMode::REMOTE));
  EXPECT_EQ(h.manager.authoritativeEpoch(), epoch0 + 1u);
  h.manager.beginStep(true);

  // THEN: each authoritative transition bumped the epoch once
  EXPECT_EQ(h.manager.authoritativeEpoch(), epoch0 + 2u);
}

TEST(OperationalModeManagerTest, ClassifyMatchesParentIdAgainstPlatformId) {
  // GIVEN: a manager keyed to this platform's id
  ManagerHarness h(makeModeConfig());

  // WHEN: sources with a matching, nil, and foreign parentID are classified
  UMAA::Common::IdentifierType localSource;
  localSource.parentID(platformGuid().getGuid());
  UMAA::Common::IdentifierType nilSource;
  UMAA::Common::IdentifierType foreignSource;
  std::array<uint8_t, 16> foreign{};
  foreign[0] = 0x42;
  foreignSource.parentID(foreign);

  // THEN: only the matching parentID is LOCAL; nil and foreign are REMOTE
  EXPECT_EQ(h.manager.classify(localSource),
            arlcore::autopilot::CommandClass::LOCAL);
  EXPECT_EQ(h.manager.classify(nilSource),
            arlcore::autopilot::CommandClass::REMOTE);
  EXPECT_EQ(h.manager.classify(foreignSource),
            arlcore::autopilot::CommandClass::REMOTE);
}
