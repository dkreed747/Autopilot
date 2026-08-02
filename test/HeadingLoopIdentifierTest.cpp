#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "InternalTypes.h"
#include "autopilot/guidance/HeadingLoopIdentifier.hpp"

constexpr flt64_t kProbeDtS = 0.02;
constexpr flt64_t kProbeDwellS = 8.0;
constexpr flt64_t kOmegaMaxRps = 0.2618;

//! \brief Test-local plant: a proportional heading loop saturated at a turn-rate envelope,
//! with independently settable gains per turn direction. A mock cannot express the motion
//! the identifier is meant to read back out of a step response.
static std::vector<arlcore::autopilot::HeadingProbeSample> simulateProbe(flt64_t gainPort, flt64_t gainStbd,
                                                                         flt64_t omegaMaxRps,
                                                                         const std::vector<flt64_t>& stepsRad,
                                                                         flt64_t dwellS = kProbeDwellS) {
  std::vector<arlcore::autopilot::HeadingProbeSample> samples;
  flt64_t heading = 0.0;
  flt64_t commanded = 0.0;
  flt64_t elapsed = 0.0;
  for (std::size_t i = 0; i < stepsRad.size(); i++) {
    commanded += stepsRad[i];
    for (flt64_t t = 0.0; t < dwellS; t += kProbeDtS) {
      const flt64_t err = commanded - heading;
      const flt64_t gain = err >= 0.0 ? gainStbd : gainPort;
      const flt64_t omega = std::clamp(gain * err, -omegaMaxRps, omegaMaxRps);
      arlcore::autopilot::HeadingProbeSample sample;
      sample.elapsedS = elapsed;
      sample.headingErrRad = err;
      sample.yawRateRps = omega;
      sample.speedMps = 3.0;
      sample.stepIndex = static_cast<int32_t>(i);
      sample.stepRad = stepsRad[i];
      samples.push_back(sample);
      heading += omega * kProbeDtS;
      elapsed += kProbeDtS;
    }
  }
  return samples;
}

//! \brief The schedule the probe ships with: alternating, summing to zero, spanning the
//! unsaturated and saturated regions.
static std::vector<flt64_t> defaultSchedule() {
  const flt64_t deg = M_PI / 180.0;
  return {5.0 * deg,   -5.0 * deg, 10.0 * deg,  -10.0 * deg, 20.0 * deg,
          -20.0 * deg, 40.0 * deg, -40.0 * deg, 90.0 * deg,  -90.0 * deg};
}

TEST(HeadingLoopIdentifierTest, RecoversGainAndRateLimitFromAProportionalLoop) {
  // GIVEN: a proportional heading loop with a known gain and turn-rate envelope
  const flt64_t trueGain = 0.9;
  const std::vector<arlcore::autopilot::HeadingProbeSample> samples =
      simulateProbe(trueGain, trueGain, kOmegaMaxRps, defaultSchedule());

  // WHEN: the loop is identified from the step responses
  const arlcore::autopilot::HeadingLoopModel model = arlcore::autopilot::identifyHeadingLoop(samples);

  // THEN: the gain, the envelope and the saturation error are recovered within 5%
  EXPECT_EQ(model.verdict, "OK");
  EXPECT_NEAR(model.gainRpsPerRad, trueGain, 0.05 * trueGain);
  EXPECT_NEAR(model.omegaMaxRps, kOmegaMaxRps, 0.05 * kOmegaMaxRps);
  EXPECT_NEAR(model.satErrRad, kOmegaMaxRps / trueGain, 0.05 * kOmegaMaxRps / trueGain);
  EXPECT_GT(model.gainR2, 0.99);
  EXPECT_GE(model.omegaMaxWindows, 3);
}

TEST(HeadingLoopIdentifierTest, ReportsTheTimeConstantTheTrackerConsumes) {
  // GIVEN: the same proportional loop, whose lag is by construction 1 / gain
  const flt64_t trueGain = 0.9;
  const std::vector<arlcore::autopilot::HeadingProbeSample> samples =
      simulateProbe(trueGain, trueGain, kOmegaMaxRps, defaultSchedule());

  // WHEN: the loop is identified
  const arlcore::autopilot::HeadingLoopModel model = arlcore::autopilot::identifyHeadingLoop(samples);

  // THEN: the reported feedforward time constant is 1 / gain, and the independently measured
  //       closed-loop decay confirms the loop really is first order
  EXPECT_NEAR(model.headingLoopTauS, 1.0 / trueGain, 0.05 / trueGain);
  EXPECT_NEAR(model.decayTauS, 1.0 / trueGain, 0.10 / trueGain);
  EXPECT_NEAR(model.modelConsistency, 1.0, 0.15);
}

TEST(HeadingLoopIdentifierTest, ReportsSaturationDominatedForARateLimiterPlant) {
  // GIVEN: a deadbeat rate limiter, i.e. an effective gain of 1/dt, which leaves almost no
  //        unsaturated region to regress
  const std::vector<arlcore::autopilot::HeadingProbeSample> samples =
      simulateProbe(1.0 / kProbeDtS, 1.0 / kProbeDtS, kOmegaMaxRps, defaultSchedule());

  // WHEN: the loop is identified
  const arlcore::autopilot::HeadingLoopModel model = arlcore::autopilot::identifyHeadingLoop(samples);

  // THEN: it says so rather than fitting noise, while still recovering the envelope and the
  //       knee, which are the only trustworthy readings for such a plant
  EXPECT_EQ(model.verdict, "SATURATION_DOMINATED");
  EXPECT_NEAR(model.omegaMaxRps, kOmegaMaxRps, 0.05 * kOmegaMaxRps);
  EXPECT_GT(model.satErrKneeRad, 0.0);
  EXPECT_LT(model.satErrKneeRad, 0.05);
  EXPECT_TRUE(std::isfinite(model.gainRpsPerRad));
}

TEST(HeadingLoopIdentifierTest, ReportsUnreliableEnvelopeWithoutEnoughLargeSteps) {
  // GIVEN: a schedule of small steps only, so the loop never saturates
  const flt64_t deg = M_PI / 180.0;
  const std::vector<arlcore::autopilot::HeadingProbeSample> samples =
      simulateProbe(0.9, 0.9, kOmegaMaxRps, {5.0 * deg, -5.0 * deg, 10.0 * deg, -10.0 * deg});

  // WHEN: the loop is identified
  const arlcore::autopilot::HeadingLoopModel model = arlcore::autopilot::identifyHeadingLoop(samples);

  // THEN: the envelope is flagged as unmeasured instead of being reported as zero
  EXPECT_EQ(model.verdict, "UNRELIABLE_OMEGA_MAX");
  EXPECT_LT(model.omegaMaxWindows, 3);
}

TEST(HeadingLoopIdentifierTest, DetectsTurnDirectionAsymmetry) {
  // GIVEN: a hull that turns to starboard twice as readily as to port
  const std::vector<arlcore::autopilot::HeadingProbeSample> samples =
      simulateProbe(0.6, 1.2, kOmegaMaxRps, defaultSchedule());

  // WHEN: the loop is identified
  const arlcore::autopilot::HeadingLoopModel model = arlcore::autopilot::identifyHeadingLoop(samples);

  // THEN: both gains are recovered separately and the asymmetry is reported
  EXPECT_NEAR(model.gainPortRpsPerRad, 0.6, 0.08);
  EXPECT_NEAR(model.gainStbdRpsPerRad, 1.2, 0.15);
  EXPECT_GT(model.asymmetryFrac, 0.4);
}

TEST(HeadingLoopIdentifierTest, EmptyAndNonFiniteInputAreRejectedWithoutCrashing) {
  // GIVEN: no samples at all, and a set whose values are all non-finite
  std::vector<arlcore::autopilot::HeadingProbeSample> garbage;
  for (int32_t i = 0; i < 50; i++) {
    arlcore::autopilot::HeadingProbeSample sample;
    sample.elapsedS = std::nan("");
    sample.headingErrRad = std::numeric_limits<flt64_t>::infinity();
    sample.yawRateRps = std::nan("");
    sample.stepIndex = i;
    garbage.push_back(sample);
  }

  // WHEN: each is identified
  const arlcore::autopilot::HeadingLoopModel empty = arlcore::autopilot::identifyHeadingLoop({});
  const arlcore::autopilot::HeadingLoopModel bad = arlcore::autopilot::identifyHeadingLoop(garbage);

  // THEN: both report no data, with every field finite and no division by zero
  EXPECT_EQ(empty.verdict, "NO_DATA");
  EXPECT_EQ(bad.verdict, "NO_DATA");
  EXPECT_TRUE(std::isfinite(bad.gainRpsPerRad));
  EXPECT_TRUE(std::isfinite(bad.omegaMaxRps));
  EXPECT_TRUE(std::isfinite(bad.headingLoopTauS));
}
