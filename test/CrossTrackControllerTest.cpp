#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "autopilot/guidance/CrossTrackController.hpp"

TEST(CrossTrackControllerTest, KiZeroMatchesLegacyShapeExactly) {
  // GIVEN: a controller at defaults (ki = 0, kp_scale = 1, limit 1.2)
  arlcore::autopilot::CrossTrackController ctl;

  // WHEN: corrections are computed across the error/radius envelope
  // THEN: every output is bit-identical to the legacy clamp(atan2(err, max(R,1)), +/-1.2) law
  for (flt64_t err = -500.0; err <= 500.0; err += 7.3) {
    for (flt64_t r : {0.5, 1.0, 14.3, 20.0, 25.0}) {
      const flt64_t legacy = std::clamp(std::atan2(err, std::max(r, 1.0)), -1.2, 1.2);
      EXPECT_EQ(ctl.correction(err, r, 0.5), legacy) << "err=" << err << " r=" << r;
    }
  }
  EXPECT_EQ(ctl.integratorRad(), 0.0);
}

TEST(CrossTrackControllerTest, IntegratorAccumulatesAndClamps) {
  // GIVEN: a PI controller with ki = 0.1 and the default 0.35 rad integrator clamp
  arlcore::autopilot::CrossTrackController::Params params;
  params.ki = 0.1;
  arlcore::autopilot::CrossTrackController ctl(params);

  // WHEN: a steady 2 m error inside the gate is integrated for one 0.5 s tick
  ctl.correction(2.0, 20.0, 0.5);
  // THEN: the integrator holds ki * err * dt
  EXPECT_NEAR(ctl.integratorRad(), 0.1 * 2.0 * 0.5, 1e-12);

  // WHEN: the error persists long enough to wind far beyond the clamp
  for (int32_t i = 0; i < 100; ++i) {
    ctl.correction(4.9, 20.0, 1.0);
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
    ctl.correction(40.0, 20.0, 0.5);
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
    ctl.correction(4.0, 20.0, 0.5);
  }
  const flt64_t frozen = ctl.integratorRad();
  ctl.correction(4.0, 20.0, 0.5);
  // THEN: further same-sign error does not wind the integrator any deeper
  EXPECT_EQ(ctl.integratorRad(), frozen);
  EXPECT_LT(frozen, params.integratorLimitRad);

  // WHEN: the error flips sign (vehicle crossed the path)
  ctl.correction(-4.0, 20.0, 0.5);
  // THEN: back-off is allowed immediately
  EXPECT_LT(ctl.integratorRad(), frozen);
}

TEST(CrossTrackControllerTest, DtIsClampedAndResetZeroes) {
  // GIVEN: a PI controller
  arlcore::autopilot::CrossTrackController::Params params;
  params.ki = 0.1;
  arlcore::autopilot::CrossTrackController ctl(params);

  // WHEN: a negative dt is supplied (clock went backwards / first tick)
  ctl.correction(2.0, 20.0, -3.0);
  // THEN: nothing integrates
  EXPECT_EQ(ctl.integratorRad(), 0.0);

  // WHEN: a huge dt is supplied (pose gap)
  ctl.correction(2.0, 20.0, 30.0);
  // THEN: at most one second of error is integrated
  EXPECT_NEAR(ctl.integratorRad(), 0.1 * 2.0 * 1.0, 1e-12);

  // WHEN: the controller is reset (leg boundary)
  ctl.reset();
  // THEN: the integrator is zeroed
  EXPECT_EQ(ctl.integratorRad(), 0.0);
}

TEST(CrossTrackControllerTest, KpScaleSteepensTheResponse) {
  // GIVEN: two P-only controllers, one with kp_scale = 2
  arlcore::autopilot::CrossTrackController::Params sharp;
  sharp.kpScale = 2.0;
  arlcore::autopilot::CrossTrackController base;
  arlcore::autopilot::CrossTrackController steep(sharp);

  // WHEN: the same small error is corrected
  // THEN: the scaled controller commands a larger correction of the same sign
  EXPECT_GT(steep.correction(2.0, 20.0, 0.5), base.correction(2.0, 20.0, 0.5));
  EXPECT_LT(steep.correction(-2.0, 20.0, 0.5), base.correction(-2.0, 20.0, 0.5));
}
