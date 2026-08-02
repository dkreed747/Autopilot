#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "InternalTypes.h"
#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/guidance/PathTracker.hpp"

// The shipped operating point: 3 m/s cruise, a 15 deg/s turn-rate envelope, and a planned arc
// radius of 1.25 * v / omega_max. Derived, never transcribed.
constexpr flt64_t kVMps = 3.0;
constexpr flt64_t kOmegaMaxRps = 0.2618;
constexpr flt64_t kRhoM = 1.25 * kVMps / kOmegaMaxRps;
constexpr flt64_t kKappaStbd = 1.0 / kRhoM;  // azimuth frame: positive curvature turns starboard
constexpr flt64_t kTauS = 1.11;              // steady-state 1/K of the inner loop
constexpr flt64_t kDtS = 0.05;

//! \brief Params at the shipped operating point.
static arlcore::autopilot::PathTracker::Params trackerParams() {
  arlcore::autopilot::PathTracker::Params p;
  p.headingLoopTauS = kTauS;
  p.crossTrackApproachRad = 0.6;
  p.crossTrackGainPerM = 0.15;
  return p;
}

static arlcore::autopilot::PathTracker::Inputs trackerInputs(flt64_t pathAz, flt64_t kappa, flt64_t xteM) {
  arlcore::autopilot::PathTracker::Inputs in;
  in.pathAzimuthRad = pathAz;
  in.pathCurvatureAzRadPerM = kappa;
  in.crossTrackErrorM = xteM;
  in.groundSpeedMps = kVMps;
  in.dtS = kDtS;
  return in;
}

TEST(PathTrackerTest, ZeroCurvatureAndZeroErrorCommandsThePathAzimuthExactly) {
  // GIVEN: a tracker on a straight leg, exactly on the path
  arlcore::autopilot::PathTracker tracker(trackerParams());

  // WHEN: the law runs for a range of path azimuths
  // THEN: the command is the tangent bit-for-bit, so no residual bias survives the refactor
  for (flt64_t pathAz : {0.0, 0.7, -2.9, 1.5707963267948966}) {
    const arlcore::autopilot::PathTracker::Output out = tracker.update(trackerInputs(pathAz, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(out.headingRad, arlcore::autopilot::wrapPi(pathAz)) << "pathAz=" << pathAz;
    EXPECT_DOUBLE_EQ(out.feedforwardRad, 0.0);
    EXPECT_DOUBLE_EQ(out.crossTrackRad, 0.0);
  }
}

TEST(PathTrackerTest, HeadingIsExactlyTangentPlusFeedforwardMinusCrossTrack) {
  // GIVEN: the shipped operating point
  // WHEN: the law runs across a grid of tangent, curvature, error and speed
  // THEN: the command is exactly the superposition, in that arrangement. The whole law is now
  //       this arrangement, so it is asserted over a grid rather than at a single point.
  for (flt64_t pathAz : {0.0, 0.4, -2.9, 3.0}) {
    for (flt64_t kappa : {0.0, kKappaStbd, -kKappaStbd, 4.0 * kKappaStbd}) {
      for (flt64_t xteM : {0.0, 3.0, -12.0, 400.0}) {
        for (flt64_t speed : {0.0, 1.0, kVMps, 9.0}) {
          arlcore::autopilot::PathTracker::Inputs in = trackerInputs(pathAz, kappa, xteM);
          in.groundSpeedMps = speed;
          arlcore::autopilot::PathTracker tracker(trackerParams());
          const arlcore::autopilot::PathTracker::Output out = tracker.update(in);
          EXPECT_NEAR(out.headingRad, arlcore::autopilot::wrapPi(pathAz + out.feedforwardRad - out.crossTrackRad),
                      1e-12)
              << "az=" << pathAz << " kappa=" << kappa << " xte=" << xteM << " v=" << speed;
          EXPECT_NEAR(out.desiredYawRateRps, std::max(speed, 0.0) * kappa, 1e-12);
        }
      }
    }
  }
}

TEST(PathTrackerTest, FeedforwardLeadsIntoTheTurnForBothDirections) {
  // GIVEN: a starboard arc and the mirrored port arc
  arlcore::autopilot::PathTracker tracker(trackerParams());
  const flt64_t expected = kVMps * kKappaStbd * kTauS;

  // WHEN: each is tracked from exactly on the path
  const arlcore::autopilot::PathTracker::Output stbd = tracker.update(trackerInputs(0.0, kKappaStbd, 0.0));
  const arlcore::autopilot::PathTracker::Output port = tracker.update(trackerInputs(0.0, -kKappaStbd, 0.0));

  // THEN: the command is biased into the turn by omega_desired * tau, with the sign of the turn
  EXPECT_NEAR(stbd.headingRad, expected, 1e-12);
  EXPECT_NEAR(port.headingRad, -expected, 1e-12);
  EXPECT_NEAR(stbd.desiredYawRateRps, kVMps * kKappaStbd, 1e-12);
}

TEST(PathTrackerTest, FeedforwardScalesWithSpeedCurvatureAndTheTimeConstant) {
  // GIVEN: the same arc at doubled speed, doubled curvature, and a halved time constant
  arlcore::autopilot::PathTracker::Params params = trackerParams();
  arlcore::autopilot::PathTracker tracker(params);
  const flt64_t base = kVMps * kKappaStbd * kTauS;

  // WHEN: each factor is varied independently
  arlcore::autopilot::PathTracker::Inputs faster = trackerInputs(0.0, kKappaStbd, 0.0);
  faster.groundSpeedMps = 2.0 * kVMps;
  const flt64_t fasterFf = tracker.update(faster).feedforwardRad;
  const flt64_t sharperFf = tracker.update(trackerInputs(0.0, 2.0 * kKappaStbd, 0.0)).feedforwardRad;
  params.headingLoopTauS = 0.5 * kTauS;
  arlcore::autopilot::PathTracker quicker(params);
  const flt64_t quickerFf = quicker.update(trackerInputs(0.0, kKappaStbd, 0.0)).feedforwardRad;

  // THEN: the feedforward is linear in speed and curvature and in the loop's time constant
  EXPECT_NEAR(fasterFf, 2.0 * base, 1e-12);
  EXPECT_NEAR(sharperFf, 2.0 * base, 1e-12);
  EXPECT_NEAR(quickerFf, 0.5 * base, 1e-12);
}

TEST(PathTrackerTest, FeedforwardClampsSymmetrically) {
  // GIVEN: a limit well below what the configured time constant would ask for
  arlcore::autopilot::PathTracker::Params params = trackerParams();
  params.feedforwardLimitRad = 0.05;
  arlcore::autopilot::PathTracker tracker(params);

  // WHEN: a turn is demanded in each direction
  // THEN: the bias saturates at the limit with the correct sign, and a value inside it passes
  EXPECT_DOUBLE_EQ(tracker.update(trackerInputs(0.0, kKappaStbd, 0.0)).feedforwardRad, 0.05);
  EXPECT_DOUBLE_EQ(tracker.update(trackerInputs(0.0, -kKappaStbd, 0.0)).feedforwardRad, -0.05);
  params.feedforwardLimitRad = 10.0;
  arlcore::autopilot::PathTracker loose(params);
  EXPECT_NEAR(loose.update(trackerInputs(0.0, kKappaStbd, 0.0)).feedforwardRad, kVMps * kKappaStbd * kTauS, 1e-12);
}

TEST(PathTrackerTest, StoppedVehicleProducesNoFeedforward) {
  // GIVEN: a vehicle at rest, and one reporting a negative speed
  arlcore::autopilot::PathTracker tracker(trackerParams());

  // WHEN: an arc is tracked
  for (flt64_t speed : {0.0, -1.0}) {
    arlcore::autopilot::PathTracker::Inputs in = trackerInputs(0.3, kKappaStbd, 0.0);
    in.groundSpeedMps = speed;
    const arlcore::autopilot::PathTracker::Output out = tracker.update(in);
    // THEN: nothing is commanded beyond the tangent; a stopped vehicle needs no arc bias
    EXPECT_DOUBLE_EQ(out.feedforwardRad, 0.0) << "speed=" << speed;
    EXPECT_DOUBLE_EQ(out.desiredYawRateRps, 0.0) << "speed=" << speed;
  }
}

TEST(PathTrackerTest, CrossTrackCorrectionIsStrictlyBoundedByTheApproachLimit) {
  // GIVEN: a tracker whose cross-track term is capped at 0.6 rad
  arlcore::autopilot::PathTracker tracker(trackerParams());

  // WHEN: the error grows without bound
  for (flt64_t err : {1.0, 10.0, 100.0, 1.0e3, 1.0e6, 1.0e9}) {
    const arlcore::autopilot::PathTracker::Output out = tracker.update(trackerInputs(0.0, 0.0, err));
    // THEN: the correction never reaches the limit, so it can never consume the feedforward's
    //       share of the turn authority
    EXPECT_LT(out.crossTrackRad, 0.6) << "err=" << err;
    EXPECT_GT(out.crossTrackRad, 0.0) << "err=" << err;
  }
  EXPECT_NEAR(tracker.update(trackerInputs(0.0, 0.0, 1.0e9)).crossTrackRad, 0.6, 1e-6);
}

TEST(PathTrackerTest, CrossTrackCorrectionIsOddAndStrictlyMonotone) {
  // GIVEN: a tracker with the bounded cross-track law
  arlcore::autopilot::PathTracker tracker(trackerParams());

  // WHEN: the error is swept across zero
  flt64_t previous = -1.0;
  for (flt64_t err = -200.0; err <= 200.0; err += 0.5) {
    const flt64_t correction = tracker.update(trackerInputs(0.0, 0.0, err)).crossTrackRad;
    const flt64_t mirrored = tracker.update(trackerInputs(0.0, 0.0, -err)).crossTrackRad;
    // THEN: it is strictly increasing and exactly odd, and starboard error turns to port
    if (err > -200.0) {
      ASSERT_GT(correction, previous) << "err=" << err;
    }
    ASSERT_NEAR(mirrored, -correction, 1e-12) << "err=" << err;
    previous = correction;
  }
  EXPECT_LT(tracker.update(trackerInputs(0.0, 0.0, 4.0)).headingRad, 0.0);
}

TEST(PathTrackerTest, CrossTrackSlopeAtTheOriginIsSetByTheGain) {
  // GIVEN: trackers differing only in cross-track gain
  for (flt64_t gain : {0.05, 0.15, 0.5}) {
    arlcore::autopilot::PathTracker::Params params = trackerParams();
    params.crossTrackGainPerM = gain;
    arlcore::autopilot::PathTracker tracker(params);

    // WHEN: the slope at zero error is measured by central difference
    const flt64_t h = 1.0e-4;
    const flt64_t high = tracker.update(trackerInputs(0.0, 0.0, h)).crossTrackRad;
    const flt64_t low = tracker.update(trackerInputs(0.0, 0.0, -h)).crossTrackRad;

    // THEN: it is approachLimit * (2/pi) * gain, so the gain keeps its tuning meaning
    EXPECT_NEAR((high - low) / (2.0 * h), 0.6 * (2.0 / M_PI) * gain, 1e-6) << "gain=" << gain;
  }
}

TEST(PathTrackerTest, NonFinitePathAzimuthIsPropagatedForRejection) {
  // GIVEN: a tracker handed a non-finite tangent
  arlcore::autopilot::PathTracker tracker(trackerParams());

  // WHEN: the law runs
  for (flt64_t bad : {std::nan(""), std::numeric_limits<flt64_t>::infinity()}) {
    const arlcore::autopilot::PathTracker::Output out = tracker.update(trackerInputs(bad, kKappaStbd, 1.0));
    // THEN: the command is non-finite rather than a plausible heading, so the caller's screen
    //       rejects the cycle instead of the vehicle steering due north
    EXPECT_FALSE(std::isfinite(out.headingRad));
    EXPECT_DOUBLE_EQ(out.feedforwardRad, 0.0);
    EXPECT_DOUBLE_EQ(out.crossTrackRad, 0.0);
  }
}

TEST(PathTrackerTest, NonFiniteKinematicsFallBackToTheTangent) {
  // GIVEN: a valid tangent but a corrupt speed, curvature or cross-track error
  arlcore::autopilot::PathTracker tracker(trackerParams());
  const flt64_t pathAz = 0.6;

  // WHEN: each field in turn is non-finite
  for (int32_t field = 0; field < 3; field++) {
    arlcore::autopilot::PathTracker::Inputs in = trackerInputs(pathAz, kKappaStbd, 1.0);
    if (field == 0) {
      in.groundSpeedMps = std::nan("");
    } else if (field == 1) {
      in.pathCurvatureAzRadPerM = std::numeric_limits<flt64_t>::infinity();
    } else {
      in.crossTrackErrorM = std::nan("");
    }
    const arlcore::autopilot::PathTracker::Output out = tracker.update(in);

    // THEN: the affected term is suppressed and the command stays finite
    EXPECT_TRUE(std::isfinite(out.headingRad)) << "field=" << field;
    if (field < 2) {
      EXPECT_DOUBLE_EQ(out.feedforwardRad, 0.0) << "field=" << field;
    } else {
      EXPECT_DOUBLE_EQ(out.crossTrackRad, 0.0) << "field=" << field;
    }
  }
}

TEST(PathTrackerTest, TrackerIsStatelessExceptTheCrossTrackIntegral) {
  // GIVEN: two trackers at the shipped defaults, where ki is 0
  arlcore::autopilot::PathTracker fresh(trackerParams());
  arlcore::autopilot::PathTracker worn(trackerParams());

  // WHEN: one of them has already flown ten thousand cycles of varied geometry
  for (int32_t i = 0; i < 10000; i++) {
    const flt64_t phase = 0.001 * static_cast<flt64_t>(i);
    worn.update(trackerInputs(std::sin(phase), kKappaStbd * std::cos(phase), 6.0 * std::sin(3.0 * phase)));
  }

  // THEN: it commands bit-identically to the fresh one. This is the case that proves no learning
  //       state survived the trim's removal: any retained adaptation would show up here.
  const arlcore::autopilot::PathTracker::Inputs probe = trackerInputs(0.4, kKappaStbd, 2.0);
  const arlcore::autopilot::PathTracker::Output a = fresh.update(probe);
  const arlcore::autopilot::PathTracker::Output b = worn.update(probe);
  EXPECT_EQ(a.headingRad, b.headingRad);
  EXPECT_EQ(a.feedforwardRad, b.feedforwardRad);
  EXPECT_EQ(a.crossTrackRad, b.crossTrackRad);
}

TEST(PathTrackerTest, TheCrossTrackIntegralIsTheOnlyStateAndResetLegClearsIt) {
  // GIVEN: a tracker with the integral enabled
  arlcore::autopilot::PathTracker::Params params = trackerParams();
  params.xte.ki = 0.1;
  arlcore::autopilot::PathTracker tracker(params);
  const arlcore::autopilot::PathTracker::Inputs probe = trackerInputs(0.0, 0.0, 2.0);
  const flt64_t first = tracker.update(probe).crossTrackRad;

  // WHEN: the same geometry is held long enough for the integral to accumulate
  for (int32_t i = 0; i < 20; i++) {
    tracker.update(probe);
  }

  // THEN: the command has moved, and resetLeg returns it to the first-cycle value
  EXPECT_GT(tracker.xteIntegratorRad(), 0.0);
  tracker.resetLeg();
  EXPECT_EQ(tracker.xteIntegratorRad(), 0.0);
  EXPECT_DOUBLE_EQ(tracker.update(probe).crossTrackRad, first);
}

TEST(PathTrackerTest, CrossTrackIntegralIsFrozenOnNonPositiveOrNonFiniteDt) {
  // GIVEN: a tracker with the integral enabled
  arlcore::autopilot::PathTracker::Params params = trackerParams();
  params.xte.ki = 0.1;

  // WHEN: dt is zero, negative, NaN or infinite
  for (flt64_t dt : {0.0, -0.5, std::nan(""), std::numeric_limits<flt64_t>::infinity()}) {
    arlcore::autopilot::PathTracker tracker(params);
    arlcore::autopilot::PathTracker::Inputs in = trackerInputs(0.0, 0.0, 2.0);
    in.dtS = dt;
    tracker.update(in);
    // THEN: nothing integrates. A non-finite dt must read as no elapsed time rather than poison
    //       the integral, and the command stays finite.
    EXPECT_EQ(tracker.xteIntegratorRad(), 0.0) << "dt=" << dt;
    EXPECT_TRUE(std::isfinite(tracker.update(in).headingRad)) << "dt=" << dt;
  }
}

TEST(PathTrackerTest, NonFiniteCrossTrackErrorLeavesTheIntegratorUntouched) {
  // GIVEN: a tracker with an integral already wound up on real error
  arlcore::autopilot::PathTracker::Params params = trackerParams();
  params.xte.ki = 0.1;
  arlcore::autopilot::PathTracker tracker(params);
  for (int32_t i = 0; i < 10; i++) {
    tracker.update(trackerInputs(0.0, 0.0, 2.0));
  }
  const flt64_t wound = tracker.xteIntegratorRad();
  ASSERT_GT(wound, 0.0);

  // WHEN: a cycle arrives with a corrupt cross-track error
  const arlcore::autopilot::PathTracker::Output out = tracker.update(trackerInputs(0.0, 0.0, std::nan("")));

  // THEN: the whole cross-track term is skipped and the accumulated integral is preserved rather
  //       than either poisoned or silently discarded
  EXPECT_DOUBLE_EQ(out.crossTrackRad, 0.0);
  EXPECT_EQ(tracker.xteIntegratorRad(), wound);
}

TEST(PathTrackerTest, ANegativeFeedforwardLimitCannotInvertTheClamp) {
  // GIVEN: a negative feedforward limit, which config validation rejects but which Params, being
  //        a public aggregate, cannot stop a caller from setting
  arlcore::autopilot::PathTracker::Params params = trackerParams();
  params.feedforwardLimitRad = -0.5;
  arlcore::autopilot::PathTracker tracker(params);

  // WHEN: an arc is tracked in each direction
  // THEN: the term is disabled rather than inverted, and std::clamp is never called with hi < lo
  EXPECT_DOUBLE_EQ(tracker.update(trackerInputs(0.0, kKappaStbd, 0.0)).feedforwardRad, 0.0);
  EXPECT_DOUBLE_EQ(tracker.update(trackerInputs(0.0, -kKappaStbd, 0.0)).feedforwardRad, 0.0);
}

TEST(PathTrackerTest, ANegativeTimeConstantCannotInvertTheFeedforward) {
  // GIVEN: a negative configured time constant
  arlcore::autopilot::PathTracker::Params params = trackerParams();
  params.headingLoopTauS = -1.0;
  arlcore::autopilot::PathTracker tracker(params);

  // WHEN: an arc is tracked
  // THEN: the bias is zero, not a bias into the outside of the turn
  EXPECT_DOUBLE_EQ(tracker.update(trackerInputs(0.0, kKappaStbd, 0.0)).feedforwardRad, 0.0);
}
