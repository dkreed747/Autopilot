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

#include "ToleranceUtils.h"

#include <cmath>

#include "AngleMath.h"
#include "Logger.h"

namespace arlcore::autopilot::tolerance {

using UMAA::Common::Orientation::DirectionRequirementVariantTypeEnum;
using UMAA::Common::Speed::SpeedRequirementVariantTypeEnum;
using UMAA::Common::Speed::VariableSpeedVariantTypeEnum;
using UMAA::Common::Measurement::ElevationRequirementVariantTypeEnum;

std::optional<DirectionValue> extractDirection(
    const UMAA::Common::Orientation::DirectionRequirementVariantType& dir) {
  const auto& sub = dir.DirectionRequirementVariantTypeSubtypes();
  DirectionValue out;
  switch (sub._d()) {
    case DirectionRequirementVariantTypeEnum::DIRECTIONTRUENORTHREQUIREMENTVARIANT_D: {
      const auto& req = sub.DirectionTrueNorthRequirementVariantVariant().direction();
      out.headingRad = req.direction();
      if (req.directionTolerance().has_value()) {
        // Per the IDL: lowerlimit is the allowed deviation counterclockwise of the setpoint,
        // upperlimit clockwise (magnitudes).
        out.ccwToleranceRad = std::fabs(req.directionTolerance().value().lowerlimit());
        out.cwToleranceRad = std::fabs(req.directionTolerance().value().upperlimit());
      }
      return out;
    }
    case DirectionRequirementVariantTypeEnum::DIRECTIONMAGNETICNORTHREQUIREMENTVARIANT_D: {
      // Passed through as a heading; magnetic-vs-true offset is the platform's concern.
      const auto& req = sub.DirectionMagneticNorthRequirementVariantVariant().direction();
      out.headingRad = req.direction();
      if (req.directionTolerance().has_value()) {
        out.ccwToleranceRad = std::fabs(req.directionTolerance().value().lowerlimit());
        out.cwToleranceRad = std::fabs(req.directionTolerance().value().upperlimit());
      }
      return out;
    }
    default:
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Unsupported direction variant for autopilot heading control")
      return std::nullopt;
  }
}

std::optional<SpeedValue> extractSpeed(
    const UMAA::Common::Speed::SpeedRequirementVariantType& speed) {
  const auto& sub = speed.SpeedRequirementVariantTypeSubtypes();
  SpeedValue out;
  switch (sub._d()) {
    case SpeedRequirementVariantTypeEnum::GROUNDSPEEDREQUIREMENTVARIANT_D: {
      const auto& req = sub.GroundSpeedRequirementVariantVariant().speed();
      out.speedMps = req.speed();
      if (req.speedTolerance().has_value()) {
        // Absolute limits of allowable values per the IDL.
        out.allowable = ValueRange{req.speedTolerance().value().lowerlimit(),
                                   req.speedTolerance().value().upperlimit()};
      }
      return out;
    }
    case SpeedRequirementVariantTypeEnum::WATERSPEEDREQUIREMENTVARIANT_D: {
      const auto& req = sub.WaterSpeedRequirementVariantVariant().speed();
      out.speedMps = req.speed();
      if (req.speedTolerance().has_value()) {
        out.allowable = ValueRange{req.speedTolerance().value().lowerlimit(),
                                   req.speedTolerance().value().upperlimit()};
      }
      return out;
    }
    default:
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Unsupported speed variant for autopilot speed control")
      return std::nullopt;
  }
}

std::optional<SpeedValue> extractSpeed(
    const UMAA::Common::Speed::VariableSpeedVariantType& speed) {
  const auto& sub = speed.VariableSpeedVariantTypeSubtypes();
  switch (sub._d()) {
    case VariableSpeedVariantTypeEnum::REQUIREDSPEEDVARIANT_D:
      // RequiredSpeedVariantType wraps a SpeedRequirementVariantType.
      return extractSpeed(sub.RequiredSpeedVariantVariant().speed());
    default:
      // RECOMMENDED (SpeedVariantType) and TIMEWITHSPEED variants are not supported for
      // waypoint speed control.
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Unsupported variable speed variant for waypoint speed")
      return std::nullopt;
  }
}

std::optional<ElevationValue> extractElevation(
    const UMAA::Common::Measurement::ElevationRequirementVariantType& elevation) {
  const auto& sub = elevation.ElevationRequirementVariantTypeSubtypes();
  ElevationValue out;
  switch (sub._d()) {
    case ElevationRequirementVariantTypeEnum::DEPTHREQUIREMENTVARIANT_D: {
      const auto& req = sub.DepthRequirementVariantVariant().depth();
      out.valueM = req.depth();
      out.frame = ElevationFrame::DEPTH;
      if (req.depthTolerance().has_value()) {
        out.allowable = ValueRange{req.depthTolerance().value().lowerLimit(),
                                   req.depthTolerance().value().upperlimit()};
      }
      return out;
    }
    case ElevationRequirementVariantTypeEnum::ALTITUDEMSLREQUIREMENTVARIANT_D: {
      const auto& req = sub.AltitudeMSLRequirementVariantVariant().altitude();
      out.valueM = req.altitude();
      out.frame = ElevationFrame::ALTITUDE_MSL;
      if (req.altitudeTolerance().has_value()) {
        out.allowable = ValueRange{req.altitudeTolerance().value().lowerLimit(),
                                   req.altitudeTolerance().value().upperlimit()};
      }
      return out;
    }
    case ElevationRequirementVariantTypeEnum::ALTITUDEAGLREQUIREMENTVARIANT_D: {
      const auto& req = sub.AltitudeAGLRequirementVariantVariant().altitude();
      out.valueM = req.altitude();
      out.frame = ElevationFrame::ALTITUDE_AGL;
      if (req.altitudeTolerance().has_value()) {
        out.allowable = ValueRange{req.altitudeTolerance().value().lowerLimit(),
                                   req.altitudeTolerance().value().upperlimit()};
      }
      return out;
    }
    case ElevationRequirementVariantTypeEnum::ALTITUDEASFREQUIREMENTVARIANT_D: {
      const auto& req = sub.AltitudeASFRequirementVariantVariant().altitude();
      out.valueM = req.altitude();
      out.frame = ElevationFrame::ALTITUDE_ASF;
      if (req.altitudeTolerance().has_value()) {
        out.allowable = ValueRange{req.altitudeTolerance().value().lowerLimit(),
                                   req.altitudeTolerance().value().upperlimit()};
      }
      return out;
    }
    case ElevationRequirementVariantTypeEnum::ALTITUDEGEODETICREQUIREMENTVARIANT_D: {
      const auto& req = sub.AltitudeGeodeticRequirementVariantVariant().altitude();
      out.valueM = req.altitude();
      out.frame = ElevationFrame::ALTITUDE_GEODETIC;
      if (req.altitudeTolerance().has_value()) {
        out.allowable = ValueRange{req.altitudeTolerance().value().lowerLimit(),
                                   req.altitudeTolerance().value().upperlimit()};
      }
      return out;
    }
    default:
      UMAA_LOG_WARN(util::SYSTEM_LOGGER, "Unsupported elevation variant for autopilot elevation control")
      return std::nullopt;
  }
}

AttitudeValue extractYaw(const UMAA::Common::Orientation::Orientation3DNEDRequirement& attitude) {
  AttitudeValue out;
  out.yawRad = attitude.yawZ().yaw().yaw();
  if (attitude.yawZ().yawTolerance().has_value()) {
    // Absolute yaw bounds per the IDL ("defines the lower/upper bound").
    out.allowable = AngleRange{attitude.yawZ().yawTolerance().value().lowerlimit().yaw(),
                               attitude.yawZ().yawTolerance().value().upperlimit().yaw()};
  }
  return out;
}

std::optional<double> extractTrackToleranceM(
    const UMAA::Common::Distance::DistanceRequirementType& trackTolerance) {
  // The track tolerance's distance field is the allowed cross-track distance from the line.
  return trackTolerance.distance();
}

bool directionAchieved(const DirectionValue& dir, double actualRad, double defaultTolRad) {
  const double err = wrapPi(actualRad - dir.headingRad);
  if (dir.ccwToleranceRad.has_value() || dir.cwToleranceRad.has_value()) {
    // err < 0 is counterclockwise of the setpoint, err > 0 clockwise.
    const double ccw = dir.ccwToleranceRad.value_or(0.0);
    const double cw = dir.cwToleranceRad.value_or(0.0);
    return err >= -ccw && err <= cw;
  }
  return std::fabs(err) <= defaultTolRad;
}

bool speedAchieved(const SpeedValue& speed, double actualMps, double defaultTolMps) {
  if (speed.allowable.has_value()) {
    return actualMps >= speed.allowable->lower && actualMps <= speed.allowable->upper;
  }
  return std::fabs(actualMps - speed.speedMps) <= defaultTolMps;
}

bool elevationAchieved(const ElevationValue& elevation, double actualM, double defaultTolM) {
  if (elevation.allowable.has_value()) {
    return actualM >= elevation.allowable->lower && actualM <= elevation.allowable->upper;
  }
  return std::fabs(actualM - elevation.valueM) <= defaultTolM;
}

bool attitudeAchieved(const AttitudeValue& attitude, double actualYawRad, double defaultTolRad) {
  if (attitude.allowable.has_value()) {
    // Angular interval [lower, upper] traversed clockwise; membership via offsets from lower.
    const double span = attitude.allowable->upperRad - attitude.allowable->lowerRad;
    const double spanNorm = (span >= 0.0) ? span : span + 2.0 * M_PI;
    double rel = std::fmod(actualYawRad - attitude.allowable->lowerRad, 2.0 * M_PI);
    if (rel < 0.0) {
      rel += 2.0 * M_PI;
    }
    return rel <= spanNorm;
  }
  return std::fabs(wrapPi(actualYawRad - attitude.yawRad)) <= defaultTolRad;
}

}  // namespace arlcore::autopilot::tolerance
