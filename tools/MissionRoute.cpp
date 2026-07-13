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

#include "MissionRoute.h"

#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "UuidFactory.h"

namespace arlcore::autopilot::tools {

using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using CommandStatusEnumType =
    UMAA::Common::MaritimeEnumeration::CommandStatusEnumModule::CommandStatusEnumType;
using CommandStatusReasonEnumType =
    UMAA::Common::MaritimeEnumeration::CommandStatusReasonEnumModule::CommandStatusReasonEnumType;

std::vector<MissionWaypoint> loadMissionCsv(const std::string& path,
                                            const GeographicLib::LocalCartesian& frame) {
  std::vector<MissionWaypoint> route;
  std::ifstream in(path);
  if (!in) {
    return route;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || std::isalpha(static_cast<unsigned char>(line[0]))) {
      continue;  // header/comment
    }
    std::stringstream ss(line);
    std::string field;
    std::vector<std::string> fields;
    while (std::getline(ss, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() < 4) {
      continue;
    }
    MissionWaypoint wp;
    double h = 0.0;
    frame.Reverse(std::stod(fields[0]), std::stod(fields[1]), 0.0, wp.latDeg, wp.lonDeg, h);
    wp.speedMps = std::stod(fields[2]);
    wp.captureRadiusM = std::stod(fields[3]);
    if (fields.size() >= 5 && !fields[4].empty()) {
      wp.arrivalYawRad = std::stod(fields[4]);
    }
    if (fields.size() >= 7 && !fields[5].empty() && !fields[6].empty()) {
      wp.elevValueM = std::stod(fields[5]);
      wp.elevFrame = fields[6];
    }
    route.push_back(wp);
  }
  return route;
}

GlobalWaypointType makeWaypoint(const MissionWaypoint& mw) {
  GlobalWaypointType wp;
  wp.position().value().geodeticLatitude(mw.latDeg);
  wp.position().value().geodeticLongitude(mw.lonDeg);
  UMAA::Common::Position::GeoPosition2DTolerance tol;
  tol.limit(mw.captureRadiusM);
  wp.position().tolerance() = tol;
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant(
      UMAA::Common::Speed::RequiredSpeedVariantType());
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant().speed()
      .SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant(
          UMAA::Common::Speed::GroundSpeedRequirementVariantType());
  wp.speed().VariableSpeedVariantTypeSubtypes().RequiredSpeedVariantVariant().speed()
      .SpeedRequirementVariantTypeSubtypes().GroundSpeedRequirementVariantVariant().speed()
      .speed(mw.speedMps);
  if (mw.arrivalYawRad.has_value()) {
    UMAA::Common::Orientation::Orientation3DNEDRequirement att;
    att.yawZ().yaw().yaw(mw.arrivalYawRad.value());
    wp.attitude() = att;
  }
  if (mw.elevValueM.has_value()) {
    UMAA::Common::Measurement::ElevationRequirementVariantType elev;
    if (mw.elevFrame == "asf") {
      elev.ElevationRequirementVariantTypeSubtypes().AltitudeASFRequirementVariantVariant(
          UMAA::Common::Measurement::AltitudeASFRequirementVariantType());
      elev.ElevationRequirementVariantTypeSubtypes().AltitudeASFRequirementVariantVariant()
          .altitude().altitude(mw.elevValueM.value());
    } else {
      elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant(
          UMAA::Common::Measurement::DepthRequirementVariantType());
      elev.ElevationRequirementVariantTypeSubtypes().DepthRequirementVariantVariant().depth()
          .depth(mw.elevValueM.value());
    }
    wp.elevation() = elev;
  }
  wp.waypointID() = arlcore::UuidFactory::getInstance().generateGuid().getGuid();
  return wp;
}

std::string statusName(CommandStatusEnumType s) {
  switch (s) {
    case CommandStatusEnumType::ISSUED: return "ISSUED";
    case CommandStatusEnumType::COMMANDED: return "COMMANDED";
    case CommandStatusEnumType::EXECUTING: return "EXECUTING";
    case CommandStatusEnumType::COMPLETED: return "COMPLETED";
    case CommandStatusEnumType::CANCELED: return "CANCELED";
    case CommandStatusEnumType::FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

std::string statusReasonName(CommandStatusReasonEnumType r) {
  switch (r) {
    case CommandStatusReasonEnumType::CANCELED: return "CANCELED";
    case CommandStatusReasonEnumType::INTERRUPTED: return "INTERRUPTED";
    case CommandStatusReasonEnumType::OBJECTIVE_FAILED: return "OBJECTIVE_FAILED";
    case CommandStatusReasonEnumType::RESOURCE_FAILED: return "RESOURCE_FAILED";
    case CommandStatusReasonEnumType::RESOURCE_REJECTED: return "RESOURCE_REJECTED";
    case CommandStatusReasonEnumType::SERVICE_FAILED: return "SERVICE_FAILED";
    case CommandStatusReasonEnumType::SUCCEEDED: return "SUCCEEDED";
    case CommandStatusReasonEnumType::TIMEOUT: return "TIMEOUT";
    case CommandStatusReasonEnumType::UPDATED: return "UPDATED";
    case CommandStatusReasonEnumType::VALIDATION_FAILED: return "VALIDATION_FAILED";
    default: return "UNKNOWN";
  }
}

}  // namespace arlcore::autopilot::tools
