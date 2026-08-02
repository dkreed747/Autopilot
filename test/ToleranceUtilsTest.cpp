#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "InternalTypes.h"
#include "autopilot/guidance/ToleranceUtils.hpp"

// The generated UMAA variant paths run well past 100 characters, so the CLAUDE.md carve-out for
// file-scope aliases of generated types applies here.
using DirectionReq = UMAA::Common::Orientation::DirectionRequirementVariantType;
using SpeedReq = UMAA::Common::Speed::SpeedRequirementVariantType;
using VariableSpeedReq = UMAA::Common::Speed::VariableSpeedVariantType;
using ElevationReq = UMAA::Common::Measurement::ElevationRequirementVariantType;
using Orientation3D = UMAA::Common::Orientation::Orientation3DNEDRequirement;

static DirectionReq trueNorthDirection(flt64_t headingRad, bool withTolerance, flt64_t ccwRad = 0.0,
                                       flt64_t cwRad = 0.0) {
  DirectionReq dir;
  dir.DirectionRequirementVariantTypeSubtypes().DirectionTrueNorthRequirementVariantVariant(
      UMAA::Common::Orientation::DirectionTrueNorthRequirementVariantType());
  auto& req = dir.DirectionRequirementVariantTypeSubtypes().DirectionTrueNorthRequirementVariantVariant().direction();
  req.direction(headingRad);
  if (withTolerance) {
    UMAA::Common::Orientation::DirectionToleranceType tol;
    tol.lowerlimit(ccwRad);
    tol.upperlimit(cwRad);
    req.directionTolerance() = tol;
  }
  return dir;
}

static SpeedReq groundSpeed(flt64_t speedMps, bool withTolerance, flt64_t lower = 0.0, flt64_t upper = 0.0) {
  SpeedReq speed;
  speed.SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant(
      UMAA::Common::Speed::GroundSpeedRequirementVariantType());
  auto& req = speed.SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant().speed();
  req.speed(speedMps);
  if (withTolerance) {
    UMAA::Common::Speed::GroundSpeedTolerance tol;
    tol.lowerlimit(lower);
    tol.upperlimit(upper);
    req.speedTolerance() = tol;
  }
  return speed;
}

static ElevationReq depthElevation(flt64_t depthM, bool withTolerance, flt64_t lower = 0.0, flt64_t upper = 0.0) {
  ElevationReq elev;
  elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant(
      UMAA::Common::Measurement::DepthRequirementVariantType());
  auto& req = elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant().depth();
  req.depth(depthM);
  if (withTolerance) {
    UMAA::Common::Measurement::DepthToleranceType tol;
    tol.lowerLimit(lower);
    tol.upperlimit(upper);
    req.depthTolerance() = tol;
  }
  return elev;
}

TEST(ToleranceUtilsTest, TrueNorthDirectionCarriesTheHeadingAndBothToleranceSides) {
  // GIVEN: a true-north direction with an asymmetric tolerance
  const DirectionReq dir = trueNorthDirection(1.2, true, 0.05, 0.30);

  // WHEN: it is extracted
  const std::optional<arlcore::autopilot::DirectionValue> out = arlcore::autopilot::tolerance::extractDirection(dir);

  // THEN: lowerlimit lands on the counterclockwise side and upperlimit on the clockwise side, as
  //       magnitudes. Transposing these two is the single most likely slip in this file and it
  //       would silently shift every achieved-flag window to the wrong side of the setpoint.
  ASSERT_TRUE(out.has_value());
  EXPECT_DOUBLE_EQ(out->headingRad, 1.2);
  ASSERT_TRUE(out->ccwToleranceRad.has_value());
  ASSERT_TRUE(out->cwToleranceRad.has_value());
  EXPECT_DOUBLE_EQ(out->ccwToleranceRad.value(), 0.05);
  EXPECT_DOUBLE_EQ(out->cwToleranceRad.value(), 0.30);
}

TEST(ToleranceUtilsTest, ANegativeDirectionToleranceIsTakenAsAMagnitude) {
  // GIVEN: a direction whose tolerance limits are expressed as signed deviations
  const DirectionReq dir = trueNorthDirection(0.0, true, -0.10, 0.20);

  // WHEN: it is extracted
  const std::optional<arlcore::autopilot::DirectionValue> out = arlcore::autopilot::tolerance::extractDirection(dir);

  // THEN: both sides come back positive, so a sign convention on the wire cannot invert the window
  ASSERT_TRUE(out.has_value());
  EXPECT_DOUBLE_EQ(out->ccwToleranceRad.value(), 0.10);
  EXPECT_DOUBLE_EQ(out->cwToleranceRad.value(), 0.20);
}

TEST(ToleranceUtilsTest, AnUnsupportedDirectionVariantIsRejectedRatherThanDefaulted) {
  // GIVEN: a default-constructed direction requirement, whose variant the autopilot cannot steer
  const DirectionReq dir;

  // WHEN: it is extracted
  // THEN: nullopt, not a 0.0 heading. A silent default here points the vehicle due north.
  EXPECT_FALSE(arlcore::autopilot::tolerance::extractDirection(dir).has_value());
}

TEST(ToleranceUtilsTest, GroundSpeedToleranceIsAnAbsoluteRangeNotAnOffset) {
  // GIVEN: a 3 m/s ground speed with limits of 2.5 and 3.5
  const SpeedReq speed = groundSpeed(3.0, true, 2.5, 3.5);

  // WHEN: it is extracted
  const std::optional<arlcore::autopilot::SpeedValue> out = arlcore::autopilot::tolerance::extractSpeed(speed);

  // THEN: the range is the absolute limits, not deviations added to the setpoint. Reading them as
  //       offsets would put the window at [5.5, 6.5] and no run would ever satisfy it.
  ASSERT_TRUE(out.has_value());
  EXPECT_DOUBLE_EQ(out->speedMps, 3.0);
  ASSERT_TRUE(out->allowable.has_value());
  EXPECT_DOUBLE_EQ(out->allowable->lower, 2.5);
  EXPECT_DOUBLE_EQ(out->allowable->upper, 3.5);
}

TEST(ToleranceUtilsTest, OnlyTheRequiredVariableSpeedVariantIsSupported) {
  // GIVEN: a waypoint speed carrying the required variant, and a default-constructed one
  VariableSpeedReq required;
  required.VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant(
      UMAA::Common::Speed::RequiredSpeedVariantType());
  required.VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant().speed() = groundSpeed(2.0, false);
  const VariableSpeedReq unsupported;

  // WHEN: each is extracted
  // THEN: only the required variant yields a speed. RECOMMENDED and TIMEWITHSPEED are refused
  //       rather than flown at zero.
  const std::optional<arlcore::autopilot::SpeedValue> ok = arlcore::autopilot::tolerance::extractSpeed(required);
  ASSERT_TRUE(ok.has_value());
  EXPECT_DOUBLE_EQ(ok->speedMps, 2.0);
  EXPECT_FALSE(arlcore::autopilot::tolerance::extractSpeed(unsupported).has_value());
}

TEST(ToleranceUtilsTest, DepthToleranceReadsLowerLimitWithACapitalL) {
  // GIVEN: a depth requirement with a tolerance. The depth IDL spells the field lowerLimit while
  //        the speed and direction IDLs spell it lowerlimit, forty lines apart in the same file.
  const ElevationReq elev = depthElevation(12.0, true, 10.0, 14.0);

  // WHEN: it is extracted
  const std::optional<arlcore::autopilot::ElevationValue> out = arlcore::autopilot::tolerance::extractElevation(elev);

  // THEN: the lower bound arrives. A "consistency" edit to the other spelling either fails to
  //       compile or silently reads a different field, and this is the case that catches it.
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->frame, arlcore::autopilot::ElevationFrame::DEPTH);
  EXPECT_DOUBLE_EQ(out->valueM, 12.0);
  ASSERT_TRUE(out->allowable.has_value());
  EXPECT_DOUBLE_EQ(out->allowable->lower, 10.0);
  EXPECT_DOUBLE_EQ(out->allowable->upper, 14.0);
}

TEST(ToleranceUtilsTest, AltitudeAboveSeaFloorCarriesItsOwnFrameAndTolerance) {
  // GIVEN: an above-sea-floor elevation requirement with a tolerance
  ElevationReq elev;
  elev.ElevationRequirementVariantTypeSubtypes().AltitudeASFRequirementVariantVariant(
      UMAA::Common::Measurement::AltitudeASFRequirementVariantType());
  auto& req = elev.ElevationRequirementVariantTypeSubtypes().AltitudeASFRequirementVariantVariant().altitude();
  req.altitude(8.0);
  UMAA::Common::Measurement::AltitudeASFToleranceType tol;
  tol.lowerLimit(7.0);
  tol.upperlimit(9.0);
  req.altitudeTolerance() = tol;

  // WHEN: it is extracted
  const std::optional<arlcore::autopilot::ElevationValue> out = arlcore::autopilot::tolerance::extractElevation(elev);

  // THEN: it keeps the ASF frame rather than collapsing onto depth, which is what makes it
  //       clampable in its own frame downstream
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->frame, arlcore::autopilot::ElevationFrame::ALTITUDE_ASF);
  EXPECT_DOUBLE_EQ(out->valueM, 8.0);
  ASSERT_TRUE(out->allowable.has_value());
  EXPECT_DOUBLE_EQ(out->allowable->lower, 7.0);
  EXPECT_DOUBLE_EQ(out->allowable->upper, 9.0);
}

TEST(ToleranceUtilsTest, DirectionAchievedUsesTheAsymmetricWindowAndFallsBackWhenAbsent) {
  // GIVEN: a heading of 0 with 0.1 rad allowed counterclockwise and 0.3 clockwise
  const std::optional<arlcore::autopilot::DirectionValue> dir =
      arlcore::autopilot::tolerance::extractDirection(trueNorthDirection(0.0, true, 0.10, 0.30));
  ASSERT_TRUE(dir.has_value());

  // WHEN: actual headings either side of the setpoint are tested
  // THEN: the window is asymmetric in the documented sense, and negative error is counterclockwise
  EXPECT_TRUE(arlcore::autopilot::tolerance::directionAchieved(dir.value(), -0.09, 0.05));
  EXPECT_FALSE(arlcore::autopilot::tolerance::directionAchieved(dir.value(), -0.11, 0.05));
  EXPECT_TRUE(arlcore::autopilot::tolerance::directionAchieved(dir.value(), 0.29, 0.05));
  EXPECT_FALSE(arlcore::autopilot::tolerance::directionAchieved(dir.value(), 0.31, 0.05));

  // WHEN: the command carries no tolerance
  const std::optional<arlcore::autopilot::DirectionValue> bare =
      arlcore::autopilot::tolerance::extractDirection(trueNorthDirection(0.0, false));
  ASSERT_TRUE(bare.has_value());
  // THEN: the configured default applies symmetrically
  EXPECT_TRUE(arlcore::autopilot::tolerance::directionAchieved(bare.value(), 0.04, 0.05));
  EXPECT_TRUE(arlcore::autopilot::tolerance::directionAchieved(bare.value(), -0.04, 0.05));
  EXPECT_FALSE(arlcore::autopilot::tolerance::directionAchieved(bare.value(), 0.06, 0.05));
}

TEST(ToleranceUtilsTest, DirectionAchievedComparesAcrossTheWrap) {
  // GIVEN: a heading requirement just below +pi
  const std::optional<arlcore::autopilot::DirectionValue> dir =
      arlcore::autopilot::tolerance::extractDirection(trueNorthDirection(3.10, false));
  ASSERT_TRUE(dir.has_value());

  // WHEN: the actual heading has wrapped to just above -pi
  // THEN: the two are recognised as 0.08 rad apart rather than 6.2 rad apart
  EXPECT_TRUE(arlcore::autopilot::tolerance::directionAchieved(dir.value(), -3.10, 0.10));
}

TEST(ToleranceUtilsTest, SpeedAndElevationAchievedUseTheAbsoluteRangeWhenGiven) {
  // GIVEN: a speed and an elevation each carrying an absolute allowable range
  const std::optional<arlcore::autopilot::SpeedValue> speed =
      arlcore::autopilot::tolerance::extractSpeed(groundSpeed(3.0, true, 2.5, 3.5));
  const std::optional<arlcore::autopilot::ElevationValue> elev =
      arlcore::autopilot::tolerance::extractElevation(depthElevation(12.0, true, 10.0, 14.0));
  ASSERT_TRUE(speed.has_value());
  ASSERT_TRUE(elev.has_value());

  // WHEN: values inside, on, and outside the range are tested
  // THEN: the boundaries are inclusive and the default tolerance is ignored while a range exists
  EXPECT_TRUE(arlcore::autopilot::tolerance::speedAchieved(speed.value(), 2.5, 0.01));
  EXPECT_TRUE(arlcore::autopilot::tolerance::speedAchieved(speed.value(), 3.5, 0.01));
  EXPECT_FALSE(arlcore::autopilot::tolerance::speedAchieved(speed.value(), 2.4, 5.0));
  EXPECT_TRUE(arlcore::autopilot::tolerance::elevationAchieved(elev.value(), 10.0, 0.01));
  EXPECT_TRUE(arlcore::autopilot::tolerance::elevationAchieved(elev.value(), 14.0, 0.01));
  EXPECT_FALSE(arlcore::autopilot::tolerance::elevationAchieved(elev.value(), 14.1, 5.0));

  // WHEN: no range is present
  const std::optional<arlcore::autopilot::SpeedValue> bare =
      arlcore::autopilot::tolerance::extractSpeed(groundSpeed(3.0, false));
  ASSERT_TRUE(bare.has_value());
  // THEN: the configured default half-width applies around the setpoint
  EXPECT_TRUE(arlcore::autopilot::tolerance::speedAchieved(bare.value(), 3.2, 0.25));
  EXPECT_FALSE(arlcore::autopilot::tolerance::speedAchieved(bare.value(), 3.3, 0.25));
}

TEST(ToleranceUtilsTest, AttitudeAchievedTreatsTheYawWindowAsClockwiseAcrossTheWrap) {
  // GIVEN: an arrival-yaw window from 3.0 rad round to -3.0 rad, i.e. straddling +/-pi
  Orientation3D attitude;
  attitude.yawZ().yaw().yaw(M_PI);
  UMAA::Common::Orientation::YawZNEDTolerance tol;
  tol.lowerlimit().yaw(3.0);
  tol.upperlimit().yaw(-3.0);
  attitude.yawZ().yawTolerance() = tol;
  const arlcore::autopilot::AttitudeValue out = arlcore::autopilot::tolerance::extractYaw(attitude);
  ASSERT_TRUE(out.allowable.has_value());

  // WHEN: yaws inside and outside the wrapped interval are tested
  // THEN: pi is inside the window and 0 is outside. This is the only wrap-aware comparison in the
  //       file, and treating the interval as a plain [lower, upper] would invert it.
  EXPECT_TRUE(arlcore::autopilot::tolerance::attitudeAchieved(out, M_PI, 0.01));
  EXPECT_TRUE(arlcore::autopilot::tolerance::attitudeAchieved(out, -3.05, 0.01));
  EXPECT_FALSE(arlcore::autopilot::tolerance::attitudeAchieved(out, 0.0, 0.01));
}

TEST(ToleranceUtilsTest, AttitudeAchievedFallsBackToASymmetricDefault) {
  // GIVEN: an arrival yaw with no tolerance
  Orientation3D attitude;
  attitude.yawZ().yaw().yaw(1.0);
  const arlcore::autopilot::AttitudeValue out = arlcore::autopilot::tolerance::extractYaw(attitude);

  // WHEN: yaws either side are tested against the configured default
  // THEN: the default is symmetric and wrap-aware
  EXPECT_FALSE(out.allowable.has_value());
  EXPECT_TRUE(arlcore::autopilot::tolerance::attitudeAchieved(out, 1.15, 0.1745));
  EXPECT_FALSE(arlcore::autopilot::tolerance::attitudeAchieved(out, 1.25, 0.1745));
}

TEST(ToleranceUtilsTest, TrackToleranceIsTheCrossTrackDistance) {
  // GIVEN: a track tolerance of 4 m
  UMAA::Common::Distance::DistanceRequirementType track;
  track.distance(4.0);

  // WHEN: it is extracted
  // THEN: it comes back as the allowed cross-track distance from the planned line
  const std::optional<flt64_t> out = arlcore::autopilot::tolerance::extractTrackToleranceM(track);
  ASSERT_TRUE(out.has_value());
  EXPECT_DOUBLE_EQ(out.value(), 4.0);
}
