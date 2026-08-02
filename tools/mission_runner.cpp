//! \brief End-to-end waypoint mission driver: publishes a UMAA Global Waypoint mission and
//! records the track, command status, and planned path to files (see tools/README.md).

#include <GeographicLib/LocalCartesian.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointExecutionStatusReportType.hpp>
#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>
#include <chrono>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "CycloneQosProviderWrapper.h"
#include "CycloneReader.h"
#include "CycloneUtilities.h"
#include "InternalTypes.h"
#include "UuidFactory.h"
#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/config/ConfigValidation.hpp"
#include "autopilot/config/YamlConfigLoader.hpp"
#include "autopilot/guidance/DubinsPathPlanner.hpp"
#include "autopilot/guidance/MissionRoute.hpp"
#include "autopilot/guidance/PlannerParamsFactory.hpp"
#include "clients/WaypointMissionClient.hpp"

using arlcore::autopilot::MissionWaypoint;
using arlcore::autopilot::tools::WaypointMissionClient;
using arlcore::io::CycloneReader;
using arlcore::io::ReadStatus;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;
using UMAA::SA::VelocityStatus::VelocityReportType;

//! \brief Seconds since the Unix epoch, so a recording can be aligned with vehicle-side logs.
static flt64_t wallClockEpochS() {
  return std::chrono::duration<flt64_t>(std::chrono::system_clock::now().time_since_epoch()).count();
}

//! \brief Write an epoch timestamp at full precision: ten integer digits already exhaust the
//! stream's default, which would quantize the stamp to whole seconds.
static void writeEpochS(std::ofstream* out, flt64_t epochS) {
  const std::streamsize prior = out->precision(17);
  *out << epochS;
  out->precision(prior);
}

//! \brief Seconds since `stamp`, or empty when the source has not reported yet.
static void writeAgeS(std::ofstream* out, bool has, const std::chrono::steady_clock::time_point& stamp) {
  if (has) {
    *out << std::chrono::duration<flt64_t>(std::chrono::steady_clock::now() - stamp).count();
  }
}

//! \brief Emit an optional capability as a meta.csv value, blank when the platform omits it.
static void writeMetaOptional(std::ofstream* out, const std::string& key, const std::optional<flt64_t>& value) {
  *out << key << ",";
  if (value.has_value()) {
    *out << value.value();
  }
  *out << "\n";
}

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

  if (!arlcore::autopilot::isValidUuid(config.identity.waypointSourceId)) {
    std::cerr << "mission_runner requires a valid UUID for identity.waypoint_source_id (got '"

              << config.identity.waypointSourceId << "')" << std::endl;

    return 1;
  }

  auto participant = arlcore::io::getDomainParticipant(config.dds.domainId);
  arlcore::io::CycloneQosProviderWrapper qosProvider(config.dds.qosFile, config.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();

  // Mission from the CSV when given (coordinates relative to the sim start), otherwise a
  // built-in closed loop with turns in both directions.
  GeographicLib::LocalCartesian frame(config.simVehicle.initialLatitudeDeg, config.simVehicle.initialLongitudeDeg, 0.0);
  std::vector<MissionWaypoint> route;
  if (!missionPath.empty()) {
    route = arlcore::autopilot::loadMissionCsv(missionPath, frame);
    if (route.empty()) {
      std::cerr << "Failed to load mission from " << missionPath << std::endl;
      return 1;
    }
    std::cout << "Loaded mission from " << missionPath << " (" << route.size() << " waypoints)" << std::endl;
  } else {
    for (const auto& [e, n] : std::vector<std::pair<flt64_t, flt64_t>>{
             {0.0, 350.0}, {250.0, 600.0}, {500.0, 350.0}, {250.0, 100.0}, {-50.0, 350.0}}) {
      MissionWaypoint wp;
      flt64_t h = 0.0;
      frame.Reverse(e, n, 0.0, wp.latDeg, wp.lonDeg, h);
      wp.speedMps = 3.0;
      wp.captureRadiusM = 12.0;
      route.push_back(wp);
    }
  }
  std::vector<GlobalWaypointType> waypoints;
  for (const MissionWaypoint& wp : route) {
    waypoints.push_back(arlcore::autopilot::makeWaypoint(wp));
  }

  {
    std::ofstream wpCsv(outDir + "/waypoints.csv");
    wpCsv.precision(10);
    wpCsv << "index,lat_deg,lon_deg,capture_radius_m,arrival_yaw_rad,elev_value_m,elev_frame\n";
    for (std::size_t i = 0; i < waypoints.size(); i++) {
      wpCsv << i << "," << route[i].latDeg << "," << route[i].lonDeg << "," << route[i].captureRadiusM << ",";
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

  const arlcore::autopilot::PlannerParams plannerParams = arlcore::autopilot::derivePlannerParams(config);

  // Export the ideal planned Dubins route for plotting: plan the same route with the same
  // platform-derived parameters from the sim start pose and sample it. Curvature comes from
  // the planned geometry itself so the analysis never has to differentiate the polyline.
  {
    GlobalPoseReportType startPose;
    startPose.position().geodeticLatitude(config.simVehicle.initialLatitudeDeg);
    startPose.position().geodeticLongitude(config.simVehicle.initialLongitudeDeg);
    startPose.attitude().yaw().yaw(config.simVehicle.initialHeadingRad);
    arlcore::autopilot::DubinsPathPlanner previewPlanner;
    previewPlanner.plan(waypoints, startPose, plannerParams);
    std::ofstream plannedCsv(outDir + "/planned_path.csv");
    plannedCsv.precision(10);
    plannedCsv << "lat_deg,lon_deg,s_m,kappa_1pm,az_rad,leg_index\n";
    for (const arlcore::autopilot::PreviewSample& s : previewPlanner.previewRouteDetailed(plannerParams.sampleStepM)) {
      plannedCsv << s.latDeg << "," << s.lonDeg << "," << s.arcLengthM << "," << s.curvatureMathRadPerM << ","
                 << s.azimuthRad << "," << s.legIndex << "\n";
    }
  }

  // Record which configuration produced this run: without it no threshold can be applied to
  // the recording after the fact.
  {
    const arlcore::autopilot::CapabilityLimits& surf = config.platformCapabilities.surface;
    std::ofstream meta(outDir + "/meta.csv");
    meta.precision(10);
    meta << "key,value\n";
    meta << "config_path," << configPath << "\n";
    meta << "mission_csv," << missionPath << "\n";
    meta << "wall_utc_start_s,";
    writeEpochS(&meta, wallClockEpochS());
    meta << "\n";
    meta << "turn_radius_m," << plannerParams.turnRadiusM << "\n";
    writeMetaOptional(&meta, "max_turn_rate_rps", surf.maxTurnRateRps);
    writeMetaOptional(&meta, "cruising_speed_mps", surf.cruisingSpeedMps);
    writeMetaOptional(&meta, "max_forward_speed_mps", surf.maxForwardSpeedMps);
    // The kinematic minimum radius the margin is measured against.
    const std::optional<flt64_t> repSpeed =
        surf.cruisingSpeedMps.has_value() ? surf.cruisingSpeedMps : surf.maxForwardSpeedMps;
    std::optional<flt64_t> minRadius;
    if (repSpeed.has_value() && surf.maxTurnRateRps.has_value() && surf.maxTurnRateRps.value() > 0.0) {
      minRadius = repSpeed.value() / surf.maxTurnRateRps.value();
    }
    writeMetaOptional(&meta, "min_turn_radius_m", minRadius);
    meta << "turn_radius_margin," << config.planner.turnRadiusMargin << "\n";
    meta << "sample_step_m," << plannerParams.sampleStepM << "\n";
    meta << "lead_distance_m," << plannerParams.leadDistanceM << "\n";
    meta << "pos_capture_m," << plannerParams.posCaptureM << "\n";
    const arlcore::autopilot::PathTracker::Params& tr = plannerParams.tracker;
    meta << "heading_loop_tau_s," << tr.headingLoopTauS << "\n";
    meta << "feedforward_limit_rad," << tr.feedforwardLimitRad << "\n";
    meta << "cross_track_approach_rad," << tr.crossTrackApproachRad << "\n";
    meta << "cross_track_gain_per_m," << tr.crossTrackGainPerM << "\n";
    meta << "xte_kp_scale," << tr.xte.kpScale << "\n";
    meta << "xte_ki," << tr.xte.ki << "\n";
    meta << "xte_integrator_limit_rad," << tr.xte.integratorLimitRad << "\n";
    meta << "xte_integrator_gate_m," << tr.xte.integratorGateM << "\n";
    meta << "xte_correction_limit_rad," << tr.xte.correctionLimitRad << "\n";
    meta << "sim_heading_gain_rps_per_rad," << config.simVehicle.headingGainRpsPerRad << "\n";
    meta << "sim_heading_lag_s," << config.simVehicle.headingLagS << "\n";
    meta << "control_period_ms," << config.loop.controlPeriodMs << "\n";
    meta << "sim_cycle_rate_hz," << config.simVehicle.cycleRateHz << "\n";
    meta << "current_east_mps," << config.simVehicle.currentEastMps << "\n";
    meta << "current_north_mps," << config.simVehicle.currentNorthMps << "\n";
    meta << "initial_latitude_deg," << config.simVehicle.initialLatitudeDeg << "\n";
    meta << "initial_longitude_deg," << config.simVehicle.initialLongitudeDeg << "\n";
  }

  // Nav readers for the track recording; the mission client owns the command-side IO.
  auto poseReader = std::make_shared<CycloneReader<GlobalPoseReportType>>(
      participant, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, rqos);
  auto speedReader =
      std::make_shared<CycloneReader<SpeedReportType>>(participant, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, rqos);
  auto velocityReader = std::make_shared<CycloneReader<VelocityReportType>>(
      participant, UMAA::SA::VelocityStatus::VelocityReportTypeTopic, rqos);
  auto execReader = std::make_shared<CycloneReader<GlobalWaypointExecutionStatusReportType>>(
      participant, UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportTypeTopic, rqos);

  // Attribute each execution-status report back to its waypoint index, so the analysis can
  // reset its projection pointer at the same leg boundaries the planner uses.
  std::map<arlcore::NumericGuid, int32_t> waypointIndexById;
  for (std::size_t i = 0; i < waypoints.size(); i++) {
    waypointIndexById[arlcore::NumericGuid(waypoints[i].waypointID())] = static_cast<int32_t>(i);
  }

  // The runner acts as this platform's onboard autonomy: its commands classify LOCAL and
  // (with implicit transitions enabled) drive the autopilot into AUTONOMOUS.
  WaypointMissionClient client(
      participant, wqos, rqos,
      arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.waypointSourceId),
      arlcore::autopilot::tools::makeLocalAutonomyIdentity(config.identity));

  // Give discovery a moment so the transient-local route/command reach the autopilot together.
  std::this_thread::sleep_for(std::chrono::seconds(2));
  const arlcore::NumericGuid sessionId = client.start(waypoints);
  if (!client.active()) {
    std::cerr << "Failed to publish waypoint command" << std::endl;
    return 1;
  }
  std::cout << "Mission command published (session " << sessionId << ", " << waypoints.size() << " waypoints)"
            << std::endl;

  std::ofstream track(outDir + "/track.csv");
  track.precision(10);
  // Append-only: the first seven columns stay byte-identical so archived recordings and any
  // ad-hoc tooling over them keep parsing. Everything downstream reads by header name.
  track << "elapsed_s,lat_deg,lon_deg,yaw_rad,speed_mps,depth_m,alt_asf_m"
        << ",wall_utc_s,yaw_rate_rps,vel_north_mps,vel_east_mps,vel_down_mps,vel_age_s"
        << ",waypoint_index,cross_track_error_m,distance_to_waypoint_m,distance_remaining_m"
        << ",cumulative_distance_m,waypoints_remaining,exec_age_s\n";
  std::ofstream statusLog(outDir + "/status.log");

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::minutes(20);
  flt64_t lastSpeed = 0.0;
  std::optional<VelocityReportType> lastVel;
  std::chrono::steady_clock::time_point lastVelAt;
  std::optional<GlobalWaypointExecutionStatusReportType> lastExec;
  std::chrono::steady_clock::time_point lastExecAt;
  std::string finalStatus = "TIMEOUT";

  while (client.active() && std::chrono::steady_clock::now() < deadline) {
    const flt64_t elapsed = std::chrono::duration<flt64_t>(std::chrono::steady_clock::now() - start).count();

    SpeedReportType speed;
    if (speedReader->readLatest(&speed) == ReadStatus::SUCCESS && speed.speedOverGround().has_value()) {
      lastSpeed = speed.speedOverGround().value();
    }
    VelocityReportType velocity;
    if (velocityReader->readLatest(&velocity) == ReadStatus::SUCCESS) {
      lastVel = velocity;
      lastVelAt = std::chrono::steady_clock::now();
    }
    // Drain rather than readLatest: the topic is bus-wide, so another commander's report must
    // not be mistaken for this session's.
    GlobalWaypointExecutionStatusReportType exec;
    while (execReader->read(&exec) == ReadStatus::SUCCESS) {
      if (arlcore::NumericGuid(exec.sessionID()) == sessionId) {
        lastExec = exec;
        lastExecAt = std::chrono::steady_clock::now();
      }
    }
    GlobalPoseReportType pose;
    if (poseReader->readLatest(&pose) == ReadStatus::SUCCESS) {
      track << elapsed << "," << pose.position().geodeticLatitude() << "," << pose.position().geodeticLongitude() << ","
            << pose.attitude().yaw().yaw() << "," << lastSpeed << ",";
      if (pose.depth().has_value()) {
        track << pose.depth().value();
      }
      track << ",";
      if (pose.altitudeASF().has_value()) {
        track << pose.altitudeASF().value();
      }
      track << ",";
      writeEpochS(&track, wallClockEpochS());
      track << ",";
      if (lastVel.has_value()) {
        track << lastVel->attitudeRate().yawRate() << "," << lastVel->velocity().northSpeed() << ","
              << lastVel->velocity().eastSpeed() << "," << lastVel->velocity().downSpeed() << ",";
      } else {
        track << ",,,,";
      }
      writeAgeS(&track, lastVel.has_value(), lastVelAt);
      track << ",";
      if (lastExec.has_value()) {
        const auto idx = waypointIndexById.find(arlcore::NumericGuid(lastExec->waypointID()));
        if (idx != waypointIndexById.end()) {
          track << idx->second;
        }
        track << ",";
        if (lastExec->crossTrackError().has_value()) {
          track << lastExec->crossTrackError().value();
        }
        track << "," << lastExec->distanceToWaypoint() << "," << lastExec->distanceRemaining() << ","
              << lastExec->cumulativeDistance() << "," << lastExec->waypointsRemaining() << ",";
      } else {
        track << ",,,,,,";
      }
      writeAgeS(&track, lastExec.has_value(), lastExecAt);
      track << "\n";
    }

    for (const auto& update : client.pollStatus()) {
      std::cout << "[" << elapsed << "s] command status: " << update.status << " (" << update.logMessage << ")"
                << std::endl;
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
