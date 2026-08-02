#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "InternalTypes.h"
#include "autopilot/umaa/ElevationAdmission.hpp"

static arlcore::autopilot::ElevationValue elevationAt(flt64_t valueM, arlcore::autopilot::ElevationFrame frame) {
  arlcore::autopilot::ElevationValue el;
  el.valueM = valueM;
  el.frame = frame;
  return el;
}

static arlcore::autopilot::ElevationValue depthAt(flt64_t valueM) {
  return elevationAt(valueM, arlcore::autopilot::ElevationFrame::DEPTH);
}

static arlcore::autopilot::ElevationValue asfAt(flt64_t valueM) {
  return elevationAt(valueM, arlcore::autopilot::ElevationFrame::ALTITUDE_ASF);
}

//! \brief Limits for a depth-constrained platform that does report an altitude above the sea floor.
static arlcore::autopilot::ElevationAdmissionLimits asfCapableLimits() {
  arlcore::autopilot::ElevationAdmissionLimits limits;
  limits.maxDepthM = 50.0;
  limits.minAltitudeAsfM = 5.0;
  limits.platformReportsAsf = true;
  return limits;
}

TEST(ElevationAdmissionTest, NonFiniteElevationIsRejected) {
  // GIVEN: a NaN elevation, which orders false against every bound below
  std::string reason;

  // WHEN: admitting it
  // THEN: it is refused by the finiteness screen rather than passing every comparison
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(depthAt(std::numeric_limits<flt64_t>::quiet_NaN()),
                                                       asfCapableLimits(), std::nullopt, &reason));
  EXPECT_NE(reason.find("finite"), std::string::npos);
}

TEST(ElevationAdmissionTest, DepthBeyondTheCeilingIsRejected) {
  // GIVEN: a 50 m depth ceiling
  std::string reason;

  // WHEN: admitting depths either side of it
  // THEN: only the one past the ceiling is refused, and the reason names the key
  EXPECT_TRUE(arlcore::autopilot::elevationAdmissible(depthAt(40.0), asfCapableLimits(), std::nullopt, &reason));
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(depthAt(60.0), asfCapableLimits(), std::nullopt, &reason));
  EXPECT_NE(reason.find("constraints.max_depth_m"), std::string::npos);
}

TEST(ElevationAdmissionTest, AsfUnderTheBottomClearanceIsRejectedWithoutAnyReference) {
  // GIVEN: a 5 m bottom clearance and no seafloor reference
  std::string reason;

  // WHEN: admitting altitudes either side of it
  // THEN: the clearance is checkable in its own frame, so no reference is needed
  EXPECT_TRUE(arlcore::autopilot::elevationAdmissible(asfAt(10.0), asfCapableLimits(), std::nullopt, &reason));
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(asfAt(3.0), asfCapableLimits(), std::nullopt, &reason));
  EXPECT_NE(reason.find("constraints.min_altitude_asf_m"), std::string::npos);
}

TEST(ElevationAdmissionTest, AsfBelowTheSeaFloorIsRejected) {
  // GIVEN: limits with no bottom clearance configured
  arlcore::autopilot::ElevationAdmissionLimits limits;
  limits.platformReportsAsf = true;
  std::string reason;

  // WHEN: admitting a negative altitude above the sea floor
  // THEN: it is refused as under the seabed, outside the UMAA DistanceASF range
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(asfAt(-1.0), limits, std::nullopt, &reason));
  EXPECT_NE(reason.find("below the sea floor"), std::string::npos);
}

TEST(ElevationAdmissionTest, AsfIsRejectedOnAPlatformThatCannotReportIt) {
  // GIVEN: a platform that does not publish an altitude above the sea floor
  arlcore::autopilot::ElevationAdmissionLimits limits;
  limits.platformReportsAsf = false;
  std::string reason;

  // WHEN: admitting an otherwise unobjectionable ASF elevation
  // THEN: it is refused - nothing could track it or bound it - and DEPTH still works
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(asfAt(20.0), limits, std::nullopt, &reason));
  EXPECT_NE(reason.find("reports_altitude_asf"), std::string::npos);
  EXPECT_TRUE(arlcore::autopilot::elevationAdmissible(depthAt(20.0), limits, std::nullopt, &reason));
}

TEST(ElevationAdmissionTest, TheSeafloorReferenceConvertsAcrossFrames) {
  // GIVEN: a 50 m depth ceiling, a 5 m bottom clearance, and a sea floor 60 m down
  const std::optional<flt64_t> floorM = 60.0;
  std::string reason;

  // WHEN: admitting an ASF elevation whose implied depth is past the ceiling
  // THEN: it is refused up front rather than admitted and held away by the output clamp
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(asfAt(5.0), asfCapableLimits(), floorM, &reason));
  EXPECT_NE(reason.find("constraints.max_depth_m"), std::string::npos);

  // WHEN: admitting an ASF elevation whose implied depth clears it
  // THEN: it is accepted
  EXPECT_TRUE(arlcore::autopilot::elevationAdmissible(asfAt(20.0), asfCapableLimits(), floorM, &reason));

  // WHEN: the bottom comes up to 52 m and a DEPTH elevation inside the ceiling leaves less than
  // the bottom clearance
  // THEN: the conversion runs the other way too, catching what no depth bound would
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(depthAt(48.0), asfCapableLimits(), 52.0, &reason));
  EXPECT_NE(reason.find("constraints.min_altitude_asf_m"), std::string::npos);
  EXPECT_TRUE(arlcore::autopilot::elevationAdmissible(depthAt(45.0), asfCapableLimits(), 52.0, &reason));
}

TEST(ElevationAdmissionTest, WithoutAReferenceTheCrossFrameChecksAreSkipped) {
  // GIVEN: the same limits and no seafloor reference (a route, or a platform with no lock)
  std::string reason;

  // WHEN: admitting the elevations the reference would have refused
  // THEN: both are admitted; the per-tick output clamp bounds them instead
  EXPECT_TRUE(arlcore::autopilot::elevationAdmissible(asfAt(5.0), asfCapableLimits(), std::nullopt, &reason));
  EXPECT_TRUE(arlcore::autopilot::elevationAdmissible(depthAt(48.0), asfCapableLimits(), std::nullopt, &reason));
}

TEST(ElevationAdmissionTest, UnconvertibleFramesAreRejectedOnlyWhenALimitExists) {
  // GIVEN: an MSL elevation, which has no depth equivalent without a geoid reference
  std::string reason;

  // WHEN: admitting it against configured limits
  // THEN: it is refused, because neither admission nor the clamp could check it
  EXPECT_FALSE(arlcore::autopilot::elevationAdmissible(
      elevationAt(-20.0, arlcore::autopilot::ElevationFrame::ALTITUDE_MSL), asfCapableLimits(), 60.0, &reason));
  EXPECT_NE(reason.find("msl"), std::string::npos);

  // WHEN: no elevation limit is configured at all
  // THEN: there is nothing to check it against, so it is admitted as before
  EXPECT_TRUE(
      arlcore::autopilot::elevationAdmissible(elevationAt(-20.0, arlcore::autopilot::ElevationFrame::ALTITUDE_MSL),
                                              arlcore::autopilot::ElevationAdmissionLimits{}, std::nullopt, &reason));
}
