#ifndef AUTOPILOT_GUIDANCE_TOLERANCEUTILS_HPP_
#define AUTOPILOT_GUIDANCE_TOLERANCEUTILS_HPP_

#include <optional>

#include <UMAA/Common/Distance/DistanceRequirementType.hpp>
#include <UMAA/Common/Measurement/ElevationRequirementVariantType.hpp>
#include <UMAA/Common/Orientation/DirectionRequirementVariantType.hpp>
#include <UMAA/Common/Orientation/Orientation3DNEDRequirement.hpp>
#include <UMAA/Common/Speed/SpeedRequirementVariantType.hpp>
#include <UMAA/Common/Speed/VariableSpeedVariantType.hpp>

#include "autopilot/guidance/ControlVector.hpp"
#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief An absolute allowable range [lower, upper] for a scalar quantity (UMAA speed,
//! depth, and altitude tolerances specify "limits of allowable values", not offsets).
struct ValueRange {
  flt64_t lower = 0.0;
  flt64_t upper = 0.0;
};

//! \brief An absolute allowable angular interval, clockwise from lower to upper (UMAA yaw
//! tolerances specify absolute bounds).
struct AngleRange {
  flt64_t lowerRad = 0.0;
  flt64_t upperRad = 0.0;
};

//! \brief A heading requirement (radians, true north). Per the UMAA DirectionToleranceType
//! IDL, the tolerance limits are deviations from the setpoint: lowerlimit counterclockwise
//! and upperlimit clockwise (magnitudes).
struct DirectionValue {
  flt64_t headingRad = 0.0;
  std::optional<flt64_t> ccwToleranceRad;
  std::optional<flt64_t> cwToleranceRad;
};

//! \brief A speed requirement: setpoint (m/s) with an optional absolute allowable range.
struct SpeedValue {
  flt64_t speedMps = 0.0;
  std::optional<ValueRange> allowable;
};

//! \brief An elevation/depth requirement: setpoint, frame, and optional allowable range.
struct ElevationValue {
  flt64_t valueM = 0.0;
  ElevationFrame frame = ElevationFrame::DEPTH;
  std::optional<ValueRange> allowable;
};

//! \brief An arrival-yaw requirement: setpoint (radians, NED) and optional absolute bounds.
struct AttitudeValue {
  flt64_t yawRad = 0.0;
  std::optional<AngleRange> allowable;
};

}  // namespace arlcore::autopilot

//! \brief Helpers to pull plain scalar values + tolerances out of UMAA requirement-variant
//! unions, and to evaluate achievement against them.
namespace arlcore::autopilot::tolerance {

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
std::optional<flt64_t> extractTrackToleranceM(
    const UMAA::Common::Distance::DistanceRequirementType& trackTolerance);

//! \brief Whether an actual heading satisfies the direction requirement (falls back to a
//! symmetric half-width of defaultTolRad when the command carries no tolerance).
bool directionAchieved(const DirectionValue& dir, flt64_t actualRad, flt64_t defaultTolRad);

//! \brief Whether an actual speed satisfies the speed requirement.
bool speedAchieved(const SpeedValue& speed, flt64_t actualMps, flt64_t defaultTolMps);

//! \brief Whether an actual elevation satisfies the elevation requirement.
bool elevationAchieved(const ElevationValue& elevation, flt64_t actualM, flt64_t defaultTolM);

//! \brief Whether an actual yaw satisfies the arrival-attitude requirement.
bool attitudeAchieved(const AttitudeValue& attitude, flt64_t actualYawRad, flt64_t defaultTolRad);

}  // namespace arlcore::autopilot::tolerance
#endif  // AUTOPILOT_GUIDANCE_TOLERANCEUTILS_HPP_
