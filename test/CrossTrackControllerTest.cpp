#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "autopilot/guidance/CrossTrackController.hpp"

// The shipped proportional shape, so the integral tests below drive integrate() through the same
// term the tracker feeds it rather than a stand-in.
static flt64_t shippedP(flt64_t errM) {
  return arlcore::autopilot::CrossTrackController::approachCorrection(errM, 0.6, 0.15);
}

TEST(CrossTrackControllerTest, ApproachCorrectionSaturatesAtTheApproachLimit) {
  // GIVEN: the approach law at a 0.6 rad limit
  // WHEN: the error is swept far past anything the gain resolves
  // THEN: the correction is strictly bounded by the approach limit and monotonically approaches it
  flt64_t previous = 0.0;
  for (flt64_t err = 0.0; err <= 5000.0; err += 13.7) {
    const flt64_t out = arlcore::autopilot::CrossTrackController::approachCorrection(err, 0.6, 0.15);
    EXPECT_LT(out, 0.6) << "err=" << err;
    EXPECT_GE(out, previous) << "err=" << err;
    previous = out;
  }
  EXPECT_NEAR(previous, 0.6, 1e-3);
}

TEST(CrossTrackControllerTest, ApproachCorrectionIsOddWithTheDocumentedOriginSlope) {
  // GIVEN: the approach law, whose documented slope at zero error is approach * (2/pi) * gain
  const flt64_t approachRad = 0.6;
  const flt64_t gainPerM = 0.15;

  // WHEN: the response either side of zero is compared, and the central slope measured
  // THEN: it is odd, and the slope matches the closed form
  for (flt64_t err = 0.1; err <= 50.0; err += 3.3) {
    EXPECT_DOUBLE_EQ(arlcore::autopilot::CrossTrackController::approachCorrection(-err, approachRad, gainPerM),
                     -arlcore::autopilot::CrossTrackController::approachCorrection(err, approachRad, gainPerM));
  }
  const flt64_t h = 1e-6;
  const flt64_t slope = (arlcore::autopilot::CrossTrackController::approachCorrection(h, approachRad, gainPerM) -
                         arlcore::autopilot::CrossTrackController::approachCorrection(-h, approachRad, gainPerM)) /
                        (2.0 * h);
  EXPECT_NEAR(slope, approachRad * (2.0 / M_PI) * gainPerM, 1e-9);
}

TEST(CrossTrackControllerTest, ApproachCorrectionIsHalfTheLimitAtTheInverseGain) {
  // GIVEN: the approach law, where atan(1) = pi/4 puts the response at exactly half the limit
  // WHEN: the error equals 1 / gain
  // THEN: the correction is half the approach angle, which is how the gain is chosen when tuning
  const flt64_t out = arlcore::autopilot::CrossTrackController::approachCorrection(1.0 / 0.15, 0.6, 0.15);
  EXPECT_NEAR(out, 0.3, 1e-12);
}

TEST(CrossTrackControllerTest, IntegrateIsAClampedPassThroughWhenKiIsZero) {
  // GIVEN: a controller at the shipped defaults, where ki is 0
  arlcore::autopilot::CrossTrackController ctl;

  // WHEN: proportional terms across and beyond the correction clamp are passed through
  // THEN: the output is exactly the clamped input and no integral ever accumulates
  for (flt64_t err = -500.0; err <= 500.0; err += 7.3) {
    const flt64_t p = shippedP(err);
    EXPECT_EQ(ctl.integrate(p, err, 0.5), std::clamp(p, -1.2, 1.2)) << "err=" << err;
  }
  EXPECT_EQ(ctl.integratorRad(), 0.0);
}

TEST(CrossTrackControllerTest, IntegratorAccumulatesAndClamps) {
  // GIVEN: a PI controller with ki = 0.1 and the default 0.35 rad integrator clamp
  arlcore::autopilot::CrossTrackController::Params params;
  params.ki = 0.1;
  arlcore::autopilot::CrossTrackController ctl(params);

  // WHEN: a steady 2 m error inside the gate is integrated for one 0.5 s tick
  ctl.integrate(shippedP(2.0), 2.0, 0.5);
  // THEN: the integrator holds ki * err * dt
  EXPECT_NEAR(ctl.integratorRad(), 0.1 * 2.0 * 0.5, 1e-12);

  // WHEN: the error persists long enough to wind far beyond the clamp
  for (int32_t i = 0; i < 100; ++i) {
    ctl.integrate(shippedP(4.9), 4.9, 1.0);
  }
  // THEN: the integrator saturates at the configured limit
  EXPECT_DOUBLE_EQ(ctl.integratorRad(), params.integratorLimitRad);
}

TEST(CrossTrackControllerTest, GateBlocksIntegrationDuringCapture) {
  // GIVEN: a PI controller with the default 5 m integrator gate
  arlcore::autopilot::CrossTrackController::Params params;
  params.ki = 0.1;
  arlcore::autopilot::CrossTrackController ctl(params);

  // WHEN: a gross 40 m error (initial capture) is applied repeatedly
  for (int32_t i = 0; i < 50; ++i) {
    ctl.integrate(shippedP(40.0), 40.0, 0.5);
  }
  // THEN: the integrator never moves
  EXPECT_EQ(ctl.integratorRad(), 0.0);
}

TEST(CrossTrackControllerTest, AntiWindupFreezesWhenSaturatedDeeper) {
  // GIVEN: a PI controller whose total correction saturates at 0.3 rad
  arlcore::autopilot::CrossTrackController::Params params;
  params.ki = 0.1;
  params.correctionLimitRad = 0.3;
  arlcore::autopilot::CrossTrackController ctl(params);

  // WHEN: a steady in-gate error drives the summed output into saturation
  for (int32_t i = 0; i < 200; ++i) {
    ctl.integrate(shippedP(4.0), 4.0, 0.5);
  }
  const flt64_t frozen = ctl.integratorRad();
  ctl.integrate(shippedP(4.0), 4.0, 0.5);
  // THEN: further same-sign error does not wind the integrator any deeper
  EXPECT_EQ(ctl.integratorRad(), frozen);
  EXPECT_LT(frozen, params.integratorLimitRad);

  // WHEN: the error flips sign (vehicle crossed the path)
  ctl.integrate(shippedP(-4.0), -4.0, 0.5);
  // THEN: back-off is allowed immediately
  EXPECT_LT(ctl.integratorRad(), frozen);
}

TEST(CrossTrackControllerTest, DtIsClampedAndResetZeroes) {
  // GIVEN: a PI controller
  arlcore::autopilot::CrossTrackController::Params params;
  params.ki = 0.1;
  arlcore::autopilot::CrossTrackController ctl(params);

  // WHEN: a negative dt is supplied (clock went backwards / first tick)
  ctl.integrate(shippedP(2.0), 2.0, -3.0);
  // THEN: nothing integrates
  EXPECT_EQ(ctl.integratorRad(), 0.0);

  // WHEN: a huge dt is supplied (pose gap)
  ctl.integrate(shippedP(2.0), 2.0, 30.0);
  // THEN: at most one second of error is integrated
  EXPECT_NEAR(ctl.integratorRad(), 0.1 * 2.0 * 1.0, 1e-12);

  // WHEN: the controller is reset (leg boundary)
  ctl.reset();
  // THEN: the integrator is zeroed
  EXPECT_EQ(ctl.integratorRad(), 0.0);
}
