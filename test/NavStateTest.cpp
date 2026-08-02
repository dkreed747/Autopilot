#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include "InternalTypes.h"
#include "autopilot/core/NavState.hpp"

static UMAA::SA::VelocityStatus::VelocityReportType velocityReport(flt64_t yawRateRps) {
  UMAA::SA::VelocityStatus::VelocityReportType report;
  report.attitudeRate().yawRate(yawRateRps);
  return report;
}

static UMAA::SA::SpeedStatus::SpeedReportType speedReport(flt64_t speedMps) {
  UMAA::SA::SpeedStatus::SpeedReportType report;
  report.speedOverGround(speedMps);
  return report;
}

TEST(NavStateTest, AgesAreUnsetBeforeTheFirstReportOfEachKind) {
  // GIVEN: a freshly constructed NavState
  arlcore::autopilot::NavState nav;

  // WHEN: the ages are read before anything has arrived
  // THEN: they are absent rather than an age measured from a default-constructed clock point,
  //       which would read as billions of milliseconds and look like a stale-but-present fix
  EXPECT_FALSE(nav.poseAgeMs().has_value());
  EXPECT_FALSE(nav.velocityAgeMs().has_value());
  EXPECT_FALSE(nav.hasPose());
  EXPECT_FALSE(nav.velocity().has_value());
}

TEST(NavStateTest, TheVelocityPayloadAndItsAgeAppearTogether) {
  // GIVEN: a NavState with no velocity yet
  arlcore::autopilot::NavState nav;

  // WHEN: one velocity report is stored
  nav.setVelocity(velocityReport(0.12));

  // THEN: the payload and a plausible age are both visible. The timestamp is written inside the
  //       same lock that guards the payload, so a reader can never see one without the other.
  ASSERT_TRUE(nav.velocity().has_value());
  EXPECT_DOUBLE_EQ(nav.velocity()->attitudeRate().yawRate(), 0.12);
  ASSERT_TRUE(nav.velocityAgeMs().has_value());
  EXPECT_GE(nav.velocityAgeMs().value(), 0);
  EXPECT_LT(nav.velocityAgeMs().value(), 5000);
}

TEST(NavStateTest, GroundSpeedScreensNonFiniteAndAbsentReports) {
  // GIVEN: a NavState with no speed report
  arlcore::autopilot::NavState nav;
  EXPECT_DOUBLE_EQ(nav.groundSpeedMps(), 0.0);

  // WHEN: a finite speed, then a NaN one, are reported
  nav.setSpeed(speedReport(2.5));
  const flt64_t good = nav.groundSpeedMps();
  nav.setSpeed(speedReport(std::nan("")));

  // THEN: the finite value passes through and the NaN reads as zero rather than propagating into
  //       the tracker's turn-rate demand
  EXPECT_DOUBLE_EQ(good, 2.5);
  EXPECT_DOUBLE_EQ(nav.groundSpeedMps(), 0.0);
}

TEST(NavStateTest, ConcurrentWritersNeverExposeAnAgeWithoutItsPayload) {
  // GIVEN: a NavState and a writer thread hammering setVelocity
  //
  // This is a smoke test, not a proof: the absence of a data race is established by the
  // timestamp being assigned inside the mutex that guards the payload, not by any number of
  // iterations passing here. There is no sanitizer preset (CMakePresets.json is pinned to the
  // four documented presets), so a TSan run is a manual activity.
  arlcore::autopilot::NavState nav;
  std::atomic<bool> stop{false};
  std::thread writer([&nav, &stop]() {
    for (int32_t i = 0; !stop.load() && i < 200000; i++) {
      nav.setVelocity(velocityReport(0.001 * static_cast<flt64_t>(i % 100)));
    }
  });

  // WHEN: a reader interleaves payload and age reads
  for (int32_t i = 0; i < 20000; i++) {
    const std::optional<int64_t> ageMs = nav.velocityAgeMs();
    if (ageMs.has_value()) {
      // THEN: an observable age always implies an observable payload, and the age is sane
      ASSERT_TRUE(nav.velocity().has_value());
      ASSERT_GE(ageMs.value(), 0);
      ASSERT_LT(ageMs.value(), 60000);
    }
  }
  stop.store(true);
  writer.join();
}
