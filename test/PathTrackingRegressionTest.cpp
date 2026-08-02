#include <gtest/gtest.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "InternalTypes.h"
#include "ServoSimVehicle.hpp"
#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/guidance/AngleMath.hpp"
#include "autopilot/guidance/DubinsPathPlanner.hpp"
#include "autopilot/guidance/PlannerParamsFactory.hpp"

using RegressionWaypointType = UMAA::MO::GlobalWaypointControl::GlobalWaypointType;

// The shipped operating point. R_plan = 1.25 * v / omega_max = 14.32 m and the kinematic
// minimum is v / omega_max = 11.46 m, so the planner reserves 2.86 m of turn-radius margin for
// disturbance rejection. The defect this file fences was that the tracker spent all of it.
constexpr flt64_t kCruiseMps = 3.0;
constexpr flt64_t kOmegaMax = 0.2618;
constexpr flt64_t kMargin = 1.25 * kCruiseMps / kOmegaMax - kCruiseMps / kOmegaMax;
constexpr flt64_t kPlannedOmega = kOmegaMax / 1.25;  // v / R_plan
constexpr flt64_t kStepS = 0.05;                     // the shipped 20 Hz control period

static arlcore::autopilot::AutopilotConfig shippedConfig() {
  arlcore::autopilot::AutopilotConfig config;
  config.platformCapabilities.surface.cruisingSpeedMps = kCruiseMps;
  config.platformCapabilities.surface.maxForwardSpeedMps = 6.0;
  config.platformCapabilities.surface.maxTurnRateRps = kOmegaMax;
  return config;
}

static RegressionWaypointType regressionWaypoint(flt64_t xE, flt64_t yN, flt64_t arrivalYawRad) {
  GeographicLib::LocalCartesian frame(39.0, -76.5, 0.0);
  flt64_t lat = 0.0;
  flt64_t lon = 0.0;
  flt64_t h = 0.0;
  frame.Reverse(xE, yN, 0.0, lat, lon, h);
  RegressionWaypointType wp;
  wp.position().value().geodeticLatitude(lat);
  wp.position().value().geodeticLongitude(lon);
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant(
      UMAA::Common::Speed::RequiredSpeedVariantType());
  wp.speed()
      .VariableSpeedVariantTypeSubtypes()
      .RequiredSpeedVariantVariant()
      .speed()
      .SpeedRequirementVariantTypeSubtypes()
      .GroundSpeedRequirementVariantVariant(UMAA::Common::Speed::GroundSpeedRequirementVariantType());
  wp.speed()
      .VariableSpeedVariantTypeSubtypes()
      .RequiredSpeedVariantVariant()
      .speed()
      .SpeedRequirementVariantTypeSubtypes()
      .GroundSpeedRequirementVariantVariant()
      .speed()
      .speed(kCruiseMps);
  UMAA::Common::Orientation::Orientation3DNEDRequirement attitude;
  attitude.yawZ().yaw().yaw(arrivalYawRad);
  wp.attitude() = attitude;
  return wp;
}

//! \brief The lawnmower-20m geometry from docs/mission-results, whose 20 m lane spacing is
//! narrower than the 28.6 m planned turn diameter and therefore forces genuine bulb turns.
static std::vector<RegressionWaypointType> lawnmower20mRoute() {
  // Arrival attitudes alternate north/south exactly as the recorded mission specifies; they are
  // what forces the bulb turns rather than letting the planner pick a convenient fly-through.
  return {regressionWaypoint(0.0, 100.0, 0.0),   regressionWaypoint(0.0, 300.0, 0.0),
          regressionWaypoint(20.0, 300.0, M_PI), regressionWaypoint(20.0, 100.0, M_PI),
          regressionWaypoint(40.0, 100.0, 0.0),  regressionWaypoint(40.0, 300.0, 0.0),
          regressionWaypoint(60.0, 300.0, M_PI), regressionWaypoint(60.0, 100.0, M_PI)};
}

//! \brief Per-sample record of a flown mission, in the terms the acceptance gates are stated in.
struct TrackingRun {
  std::vector<flt64_t> arcRadialOffsetM;  // negative = inside the planned arc
  std::vector<flt64_t> arcOmegaRps;
  std::vector<flt64_t> straightXteM;
  bool completed = false;
  bool failed = false;
};

//! \brief Fly the route under planner guidance, classifying each sample by how hard the vehicle
//! is actually turning.
//!
//! "On an arc" is a LOWER bound on turn rate only. An upper bound would filter out exactly the
//! saturated samples this file exists to catch, and the test would then pass on the broken law.
//! The radial offset is sign(turn direction) * cross-track error, which equals the signed
//! distance from the planned arc, so cutting the corner is negative and overshooting positive.
static TrackingRun flyRoute(ServoSimVehicle* vehicle, flt64_t headingLoopTauS) {
  arlcore::autopilot::AutopilotConfig config = shippedConfig();
  config.planner.tracker.headingLoopTauS = headingLoopTauS;
  arlcore::autopilot::PlannerParams params = arlcore::autopilot::derivePlannerParams(config);
  params.posCaptureM = 2.5;

  arlcore::autopilot::DubinsPathPlanner planner;
  planner.plan(lawnmower20mRoute(), vehicle->pose(), params);

  TrackingRun run;
  // Acquiring the next leg after a capture briefly demands whatever turn authority exists, which
  // is a different regime from holding an arc. Skip the first second of each arc so the sample
  // population is steady tracking rather than entry transients.
  constexpr int32_t kArcWarmupTicks = 20;
  int32_t arcRunTicks = 0;
  for (int32_t i = 0; i < 40000 && !planner.routeComplete() && !planner.failed(); i++) {
    const arlcore::autopilot::ControlVector cv = planner.update(vehicle->pose(), vehicle->speedMps, kStepS);
    const std::optional<flt64_t> xte = planner.progress().crossTrackErrorM;
    if (vehicle->speedMps > 0.5 && xte.has_value()) {
      const flt64_t omega = vehicle->yawRateRps;
      if (std::fabs(omega) >= 0.5 * kPlannedOmega) {
        arcRunTicks++;
        if (arcRunTicks > kArcWarmupTicks) {
          run.arcRadialOffsetM.push_back(-std::copysign(1.0, omega) * xte.value());
          run.arcOmegaRps.push_back(omega);
        }
      } else {
        arcRunTicks = 0;
        if (std::fabs(omega) <= 0.1 * kPlannedOmega) {
          run.straightXteM.push_back(xte.value());
        }
      }
    }
    vehicle->step(cv, kStepS);
  }
  run.completed = planner.routeComplete();
  run.failed = planner.failed();
  return run;
}

static flt64_t meanOf(const std::vector<flt64_t>& values) {
  flt64_t total = 0.0;
  for (const flt64_t v : values) {
    total += v;
  }
  return values.empty() ? 0.0 : total / static_cast<flt64_t>(values.size());
}

static flt64_t percentileOf(std::vector<flt64_t> values, flt64_t q) {
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t index = static_cast<std::size_t>(q * static_cast<flt64_t>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

static flt64_t meanAbsOf(const std::vector<flt64_t>& values) {
  std::vector<flt64_t> magnitudes;
  magnitudes.reserve(values.size());
  for (const flt64_t v : values) {
    magnitudes.push_back(std::fabs(v));
  }
  return meanOf(magnitudes);
}

TEST(ServoSimVehicleTest, HeadingGainOfOneOverStepReproducesTheDeadbeatLoop) {
  // GIVEN: a servo vehicle whose gain is 1/dt with no lag, and a deadbeat reference integrated
  //        locally with the law the other test vehicles use
  ServoSimVehicle vehicle;
  vehicle.kTrueRpsPerRad = 1.0 / kStepS;
  flt64_t refYaw = 0.0;
  arlcore::autopilot::ControlVector cv;
  cv.headingRad = 1.2;
  cv.speedMps = kCruiseMps;

  // WHEN: 500 identical commands are applied to both
  for (int32_t i = 0; i < 500; i++) {
    vehicle.step(cv, kStepS);
    const flt64_t maxDelta = vehicle.maxTurnRateRps * kStepS;
    refYaw = arlcore::autopilot::wrapPi(
        refYaw + std::clamp(arlcore::autopilot::wrapPi(cv.headingRad - refYaw), -maxDelta, maxDelta));
  }

  // THEN: the servo model reproduces the deadbeat loop exactly, so it generalizes rather than
  //       replaces the plant the existing planner tests were tuned against
  EXPECT_NEAR(vehicle.yawRad, refYaw, 1e-9);
}

TEST(ServoSimVehicleTest, HoldsTheProportionalSteadyStateOffsetAndRespectsTheLimit) {
  // GIVEN: a servo vehicle with a modest gain
  ServoSimVehicle vehicle;
  vehicle.kTrueRpsPerRad = 5.0;
  arlcore::autopilot::ControlVector cv;
  cv.speedMps = kCruiseMps;

  // WHEN: a command inside the linear region is held
  cv.headingRad = 0.01;  // 5 * 0.01 = 0.05 rad/s, well inside the envelope
  vehicle.step(cv, kStepS);
  const flt64_t linearRate = vehicle.yawRateRps;
  // ... and then one far outside it
  cv.headingRad = 3.0;
  vehicle.step(cv, kStepS);

  // THEN: the rate is gain * error in the linear region and clamps at the envelope beyond it
  EXPECT_NEAR(linearRate, 5.0 * 0.01, 1e-12);
  EXPECT_LE(std::fabs(vehicle.yawRateRps), vehicle.maxTurnRateRps + 1e-12);
  EXPECT_TRUE(vehicle.rateSaturated);
}

TEST(PathTrackingRegressionTest, LawnmowerArcsHoldThePlannedRadiusAndPreserveTurnMargin) {
  // GIVEN: the lawnmower-20m route and a servo vehicle whose loop matches the configured
  //        time constant
  // matching the shipped simulated vehicle, including its actuator lag: the acceptance
  // thresholds below were measured against that configuration
  ServoSimVehicle vehicle;
  vehicle.kTrueRpsPerRad = 0.9;
  vehicle.maxTurnRateRps = kOmegaMax;
  vehicle.lagTauS = 0.5;
  vehicle.yN = 0.0;  // the sim start, 100 m south of the first waypoint

  // WHEN: the mission is flown to completion
  const TrackingRun run = flyRoute(&vehicle, 1.0 / 0.9);

  // THEN: the route completes and the arc population is large enough to be meaningful
  ASSERT_TRUE(run.completed);
  EXPECT_FALSE(run.failed);
  ASSERT_GT(run.arcRadialOffsetM.size(), 300u);

  // THEN: the vehicle rides the planned arc instead of cutting inside it. The phase-lead law
  //       measured a mean of -1.52 m with a p10 of -2.86 m against 2.86 m of available margin,
  //       and the self-calibrating trim that replaced it measured -0.36 m. These bounds were
  //       re-measured after the trim was deleted: pure feedforward at a correctly measured time
  //       constant holds -0.04 m, so they are set roughly 4x looser than what actually holds.
  const flt64_t meanRadial = meanOf(run.arcRadialOffsetM);
  const flt64_t p10Radial = percentileOf(run.arcRadialOffsetM, 0.10);
  EXPECT_LT(std::fabs(meanRadial), 0.15) << "mean radial offset " << meanRadial;
  EXPECT_GT(p10Radial, -0.40) << "p10 radial offset " << p10Radial;

  // THEN: the turn-radius margin survives the turn, rather than being spent holding it
  const flt64_t deepest = *std::min_element(run.arcRadialOffsetM.begin(), run.arcRadialOffsetM.end());
  EXPECT_GT(deepest, -0.25 * kMargin) << "deepest radial offset " << deepest;

  // THEN: turn authority is left over. The phase-lead law pinned the rate at 100% of the
  //       envelope through every arc while the plan only ever demanded 80% of it.
  std::vector<flt64_t> magnitudes;
  for (const flt64_t omega : run.arcOmegaRps) {
    magnitudes.push_back(std::fabs(omega));
  }
  // Stated as a high percentile plus a saturated fraction, not as a peak: acquiring the next
  // leg after a capture is a different regime from holding an arc, and a peak over a thousand
  // samples is dominated by those transients rather than by steady tracking.
  const flt64_t medianOmega = percentileOf(magnitudes, 0.5);
  const flt64_t p95Omega = percentileOf(magnitudes, 0.95);
  std::size_t saturated = 0;
  for (const flt64_t omega : magnitudes) {
    saturated += omega >= 0.95 * kOmegaMax ? 1u : 0u;
  }
  const flt64_t saturatedFrac = static_cast<flt64_t>(saturated) / static_cast<flt64_t>(magnitudes.size());
  EXPECT_NEAR(medianOmega, kPlannedOmega, 0.10 * kPlannedOmega) << "median arc |omega| " << medianOmega;
  EXPECT_LE(p95Omega, 0.95 * kOmegaMax) << "p95 arc |omega| " << p95Omega;
  // Slightly looser than the 2% campaign gate: this harness has no speed ramp and steps the
  // planner on a fixed clock rather than on pose arrival, so it sees a few more transient
  // samples than a full DDS-in-the-loop run, which measures 0.4% on this route.
  EXPECT_LT(saturatedFrac, 0.03) << "fraction of arc samples at the envelope " << saturatedFrac;

  // THEN: straight-line tracking, which was already good, has not regressed
  EXPECT_LT(meanAbsOf(run.straightXteM), 0.05);
}

TEST(PathTrackingRegressionTest, ActuatorLagDoesNotDestabiliseTheArcs) {
  // GIVEN: a vehicle with six control periods of actuator lag on its turn rate
  ServoSimVehicle vehicle;
  vehicle.kTrueRpsPerRad = 0.9;
  vehicle.maxTurnRateRps = kOmegaMax;
  vehicle.lagTauS = 0.3;
  vehicle.yN = 0.0;  // the sim start, 100 m south of the first waypoint

  // WHEN: the mission is flown
  const TrackingRun run = flyRoute(&vehicle, 1.0 / 0.9);

  // THEN: it completes and the arcs stay bounded rather than oscillating. Re-measured after the
  //       trim removal: 0.16 m mean and -0.49 m deepest, so these are ~2x looser than reality.
  ASSERT_TRUE(run.completed);
  ASSERT_GT(run.arcRadialOffsetM.size(), 300u);
  EXPECT_LT(meanAbsOf(run.arcRadialOffsetM), 0.35);
  const flt64_t deepest = *std::min_element(run.arcRadialOffsetM.begin(), run.arcRadialOffsetM.end());
  EXPECT_GT(deepest, -0.5 * kMargin) << "deepest radial offset " << deepest;
}

TEST(PathTrackingRegressionTest, MisconfiguredTimeConstantDegradesTrackingWithoutFailingTheRoute) {
  // GIVEN: a vehicle whose inner loop is meaningfully less responsive than configured, so the
  //        feedforward under-biases every arc. Nothing corrects this in flight any more, which is
  //        exactly why heading_loop_tau_s must be measured with tools/heading_probe.
  ServoSimVehicle vehicle;
  vehicle.kTrueRpsPerRad = 0.65;  // true 1/K is 1.54 s against a configured 1.11 s
  vehicle.maxTurnRateRps = kOmegaMax;
  vehicle.yN = 0.0;

  // WHEN: the mission is flown with the wrong time constant
  const TrackingRun run = flyRoute(&vehicle, 1.0 / 0.9);

  // THEN: the route still completes and the error stays bounded: a mis-measured time constant
  //       degrades tracking, it does not break it
  ASSERT_TRUE(run.completed);
  EXPECT_FALSE(run.failed);
  ASSERT_GT(run.arcRadialOffsetM.size(), 300u);
  EXPECT_LT(meanAbsOf(run.arcRadialOffsetM), 1.20) << "mean |radial offset| " << meanAbsOf(run.arcRadialOffsetM);

  // THEN: and it fails in the diagnosable direction. Too small a time constant under-biases, so
  //       the vehicle rides OUTSIDE the planned arc; too large a one cuts inside. That sign is
  //       what docs/tuning.md tells an integrator to read the correction direction from, so it is
  //       asserted rather than left as folklore.
  EXPECT_GT(meanOf(run.arcRadialOffsetM), 0.0) << "mean radial offset " << meanOf(run.arcRadialOffsetM);
}

TEST(PathTrackingRegressionTest, TwoIdenticalRunsProduceIdenticalTracks) {
  // GIVEN: two identically configured vehicles on the same route
  ServoSimVehicle first;
  first.kTrueRpsPerRad = 0.9;
  first.maxTurnRateRps = kOmegaMax;
  first.lagTauS = 0.5;
  first.yN = 0.0;
  ServoSimVehicle second = first;

  // WHEN: both fly the mission
  const TrackingRun a = flyRoute(&first, 1.0 / 0.9);
  const TrackingRun b = flyRoute(&second, 1.0 / 0.9);

  // THEN: the tracks are bit-identical. The trim was the only source of cross-run adaptation, so
  //       determinism is now an invariant of the law and worth fencing at mission level.
  ASSERT_EQ(a.arcRadialOffsetM.size(), b.arcRadialOffsetM.size());
  for (std::size_t i = 0; i < a.arcRadialOffsetM.size(); i++) {
    ASSERT_EQ(a.arcRadialOffsetM[i], b.arcRadialOffsetM[i]) << "sample " << i;
  }
  EXPECT_EQ(a.straightXteM.size(), b.straightXteM.size());
}
