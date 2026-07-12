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

//! \brief End-to-end waypoint mission driver for the autopilot. Acts as the UMAA consumer
//! side (via WaypointMissionClient): publishes a GlobalWaypointCommandType (destination =
//! the autopilot's waypoint provider) plus its large-list route, then records the vehicle's
//! Global Pose track and the command status until the mission completes. Outputs:
//!   <out>/track.csv        elapsed_s, lat_deg, lon_deg, yaw_rad, speed_mps, depth_m, alt_asf_m
//!   <out>/waypoints.csv    index, lat_deg, lon_deg, capture_radius_m, arrival_yaw_rad,
//!                          elev_value_m, elev_frame
//!   <out>/planned_path.csv the ideal planned Dubins route (lat_deg, lon_deg samples)
//!   <out>/status.log       command status transitions
//! Usage: mission_runner [autopilot.yaml] [output-dir] [mission.csv]
//!
//! The optional mission CSV defines the route in the local tangent plane at the sim start,
//! one waypoint per line:
//!   east_m,north_m,speed_mps,capture_radius_m[,arrival_yaw_rad][,elev_value_m,elev_frame]
//! (header line ignored; leave arrival_yaw_rad empty for no attitude requirement; elev_frame
//! is `depth` or `asf`). Without a mission file a built-in closed loop is flown.

#include <chrono>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>

#include "AutopilotConfig.h"
#include "CycloneQosProviderWrapper.h"
#include "CycloneReader.h"
#include "CycloneUtilities.h"
#include "DubinsPathPlanner.h"
#include "MissionRoute.h"
#include "PlannerParamsFactory.h"
#include "UuidFactory.h"
#include "WaypointMissionClient.h"
#include "YamlConfigLoader.h"

namespace {

using arlcore::autopilot::tools::MissionWaypoint;
using arlcore::autopilot::tools::WaypointMissionClient;
using arlcore::io::CycloneReader;
using arlcore::io::ReadStatus;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;

}  // namespace

int main(int argc, char** argv) {
  const std::string configPath = (argc > 1) ? argv[1] : "autopilot.yaml";
  const std::string outDir = (argc > 2) ? argv[2] : "mission-out";
  const std::string missionPath = (argc > 3) ? argv[3] : "";

  arlcore::autopilot::AutopilotConfig config;
  if (!arlcore::autopilot::YamlConfigLoader::load(configPath, &config)) {
    std::cerr << "Failed to load config " << configPath << std::endl;
    return 1;
  }
  std::filesystem::create_directories(outDir);

  auto participant = arlcore::io::getDomainParticipant(config.dds.domainId);
  arlcore::io::CycloneQosProviderWrapper qosProvider(config.dds.qosFile, config.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();

  // The mission: from the CSV when given, otherwise a built-in closed loop with turns in
  // both directions. CSV coordinates are relative to the sim start.
  GeographicLib::LocalCartesian frame(config.simVehicle.initialLatitudeDeg,
                                      config.simVehicle.initialLongitudeDeg, 0.0);
  std::vector<MissionWaypoint> route;
  if (!missionPath.empty()) {
    route = arlcore::autopilot::tools::loadMissionCsv(missionPath, frame);
    if (route.empty()) {
      std::cerr << "Failed to load mission from " << missionPath << std::endl;
      return 1;
    }
    std::cout << "Loaded mission from " << missionPath << " (" << route.size()
              << " waypoints)" << std::endl;
  } else {
    for (const auto& [e, n] : std::vector<std::pair<double, double>>{
             {0.0, 350.0}, {250.0, 600.0}, {500.0, 350.0}, {250.0, 100.0}, {-50.0, 350.0}}) {
      MissionWaypoint wp;
      double h = 0.0;
      frame.Reverse(e, n, 0.0, wp.latDeg, wp.lonDeg, h);
      wp.speedMps = 3.0;
      wp.captureRadiusM = 12.0;
      route.push_back(wp);
    }
  }
  std::vector<GlobalWaypointType> waypoints;
  for (const MissionWaypoint& wp : route) {
    waypoints.push_back(arlcore::autopilot::tools::makeWaypoint(wp));
  }

  {
    std::ofstream wpCsv(outDir + "/waypoints.csv");
    wpCsv.precision(10);
    wpCsv << "index,lat_deg,lon_deg,capture_radius_m,arrival_yaw_rad,elev_value_m,elev_frame\n";
    for (std::size_t i = 0; i < waypoints.size(); i++) {
      wpCsv << i << "," << route[i].latDeg << "," << route[i].lonDeg << ","
            << route[i].captureRadiusM << ",";
      if (route[i].arrivalYawRad.has_value()) {
        wpCsv << route[i].arrivalYawRad.value();
      }
      wpCsv << ",";
      if (route[i].elevValueM.has_value()) {
        wpCsv << route[i].elevValueM.value() << "," << route[i].elevFrame;
      } else {
        wpCsv << ",";
      }
      wpCsv << "\n";
    }
  }

  // Export the ideal planned Dubins route for plotting: plan the same route with the same
  // platform-derived parameters from the sim start pose and sample it.
  {
    GlobalPoseReportType startPose;
    startPose.position().geodeticLatitude(config.simVehicle.initialLatitudeDeg);
    startPose.position().geodeticLongitude(config.simVehicle.initialLongitudeDeg);
    startPose.attitude().yaw().yaw(config.simVehicle.initialHeadingRad);
    arlcore::autopilot::DubinsPathPlanner previewPlanner;
    previewPlanner.plan(waypoints, startPose, arlcore::autopilot::derivePlannerParams(config));
    std::ofstream plannedCsv(outDir + "/planned_path.csv");
    plannedCsv.precision(10);
    plannedCsv << "lat_deg,lon_deg\n";
    for (const auto& [lat, lon] : previewPlanner.previewRoute(2.0)) {
      plannedCsv << lat << "," << lon << "\n";
    }
  }

  // Nav readers for the track recording; the mission client owns the command-side IO.
  auto poseReader = std::make_shared<CycloneReader<GlobalPoseReportType>>(
      participant, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, rqos);
  auto speedReader = std::make_shared<CycloneReader<SpeedReportType>>(
      participant, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, rqos);

  WaypointMissionClient client(
      participant, wqos, rqos,
      arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.waypointSourceId));

  // Give discovery a moment so the transient-local route/command reach the autopilot together.
  std::this_thread::sleep_for(std::chrono::seconds(2));
  const arlcore::NumericGuid sessionId = client.start(waypoints);
  if (!client.active()) {
    std::cerr << "Failed to publish waypoint command" << std::endl;
    return 1;
  }
  std::cout << "Mission command published (session " << sessionId << ", "
            << waypoints.size() << " waypoints)" << std::endl;

  std::ofstream track(outDir + "/track.csv");
  track.precision(10);
  track << "elapsed_s,lat_deg,lon_deg,yaw_rad,speed_mps,depth_m,alt_asf_m\n";
  std::ofstream statusLog(outDir + "/status.log");

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::minutes(20);
  double lastSpeed = 0.0;
  std::string finalStatus = "TIMEOUT";

  while (client.active() && std::chrono::steady_clock::now() < deadline) {
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    SpeedReportType speed;
    if (speedReader->readLatest(&speed) == ReadStatus::SUCCESS && speed.speedOverGround().has_value()) {
      lastSpeed = speed.speedOverGround().value();
    }
    GlobalPoseReportType pose;
    if (poseReader->readLatest(&pose) == ReadStatus::SUCCESS) {
      track << elapsed << "," << pose.position().geodeticLatitude() << ","
            << pose.position().geodeticLongitude() << "," << pose.attitude().yaw().yaw() << ","
            << lastSpeed << ",";
      if (pose.depth().has_value()) {
        track << pose.depth().value();
      }
      track << ",";
      if (pose.altitudeASF().has_value()) {
        track << pose.altitudeASF().value();
      }
      track << "\n";
    }

    for (const auto& update : client.pollStatus()) {
      std::cout << "[" << elapsed << "s] command status: " << update.status
                << " (" << update.logMessage << ")" << std::endl;
      statusLog << elapsed << " " << update.status << " " << update.logMessage << "\n";
      if (update.terminal) {
        finalStatus = update.status;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  track.flush();
  statusLog << "final: " << finalStatus << "\n";
  std::cout << "Mission finished with status " << finalStatus << std::endl;
  return finalStatus == "COMPLETED" ? 0 : 2;
}
