//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
//
// Applied Research Laboratory
// Pennsylvania State University
// P.O. Box 30
// State College, PA 16804-0030
//
// DISTRIBUTION STATEMENT A. Approved for public release.
// Distribution is unlimited.
// This software was developed by the Department of the Navy,
// NAVSEA Unmanned and Small Combatants. It is provided under the terms of
// use found in the LICENSE file at the source code root directory.
//
//---------------------------------------------------------------------------

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_TOLERANCEUTILS_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_TOLERANCEUTILS_H_

#include <optional>

#include <UMAA/Common/Distance/DistanceRequirementType.hpp>
#include <UMAA/Common/Measurement/ElevationRequirementVariantType.hpp>
#include <UMAA/Common/Orientation/DirectionRequirementVariantType.hpp>
#include <UMAA/Common/Orientation/Orientation3DNEDRequirement.hpp>
#include <UMAA/Common/Speed/SpeedRequirementVariantType.hpp>
#include <UMAA/Common/Speed/VariableSpeedVariantType.hpp>

#include "ControlVector.h"

namespace arlcore::autopilot {

//! \brief An absolute allowable range [lower, upper] for a scalar quantity (UMAA speed,
//! depth, and altitude tolerances specify "limits of allowable values", not offsets).
struct ValueRange {
  double lower = 0.0;
  double upper = 0.0;
};

//! \brief An absolute allowable angular interval, clockwise from lower to upper (UMAA yaw
//! tolerances specify absolute bounds).
struct AngleRange {
  double lowerRad = 0.0;
  double upperRad = 0.0;
};

//! \brief A heading requirement (radians, true north). Per the UMAA DirectionToleranceType
//! IDL, the tolerance limits are deviations from the setpoint: lowerlimit counterclockwise
//! and upperlimit clockwise (magnitudes).
struct DirectionValue {
  double headingRad = 0.0;
  std::optional<double> ccwToleranceRad;
  std::optional<double> cwToleranceRad;
};

//! \brief A speed requirement: setpoint (m/s) with an optional absolute allowable range.
struct SpeedValue {
  double speedMps = 0.0;
  std::optional<ValueRange> allowable;
};

//! \brief An elevation/depth requirement: setpoint, frame, and optional allowable range.
struct ElevationValue {
  double valueM = 0.0;
  ElevationFrame frame = ElevationFrame::DEPTH;
  std::optional<ValueRange> allowable;
};

//! \brief An arrival-yaw requirement: setpoint (radians, NED) and optional absolute bounds.
struct AttitudeValue {
  double yawRad = 0.0;
  std::optional<AngleRange> allowable;
};

//! \brief Helpers to pull plain scalar values + tolerances out of UMAA requirement-variant
//! unions, and to evaluate achievement against them. Centralizing this keeps the
//! union-discriminator handling and the per-type tolerance semantics in one place.
namespace tolerance {

//! \brief Extract the commanded heading + tolerance from a direction requirement. Supports
//! true-north / magnetic-north reference frames; returns nullopt for unsupported variants.
std::optional<DirectionValue> extractDirection(
    const UMAA::Common::Orientation::DirectionRequirementVariantType& dir);

//! \brief Extract ground/water speed + tolerance from a speed requirement.
std::optional<SpeedValue> extractSpeed(
    const UMAA::Common::Speed::SpeedRequirementVariantType& speed);

//! \brief Extract speed from a waypoint's variable-speed requirement (required/recommended).
std::optional<SpeedValue> extractSpeed(
    const UMAA::Common::Speed::VariableSpeedVariantType& speed);

//! \brief Extract elevation/depth value + frame + tolerance from an elevation requirement.
std::optional<ElevationValue> extractElevation(
    const UMAA::Common::Measurement::ElevationRequirementVariantType& elevation);

//! \brief Extract the arrival yaw + tolerance from a 3D NED orientation requirement.
AttitudeValue extractYaw(const UMAA::Common::Orientation::Orientation3DNEDRequirement& attitude);

//! \brief Extract the cross-track distance tolerance (meters) from a track tolerance, if set.
std::optional<double> extractTrackToleranceM(
    const UMAA::Common::Distance::DistanceRequirementType& trackTolerance);

//! \brief Whether an actual heading satisfies the direction requirement (falls back to a
//! symmetric half-width of defaultTolRad when the command carries no tolerance).
bool directionAchieved(const DirectionValue& dir, double actualRad, double defaultTolRad);

//! \brief Whether an actual speed satisfies the speed requirement.
bool speedAchieved(const SpeedValue& speed, double actualMps, double defaultTolMps);

//! \brief Whether an actual elevation satisfies the elevation requirement.
bool elevationAchieved(const ElevationValue& elevation, double actualM, double defaultTolM);

//! \brief Whether an actual yaw satisfies the arrival-attitude requirement.
bool attitudeAchieved(const AttitudeValue& attitude, double actualYawRad, double defaultTolRad);

}  // namespace tolerance
}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_TOLERANCEUTILS_H_
