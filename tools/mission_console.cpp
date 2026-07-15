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

//! \brief Live mission-control console for the autopilot: bridges the UMAA DDS bus to a web
//! GUI. Subscribes to the three SA navigation reports plus the Global Waypoint command /
//! ack / status / execution-status topics, and serves:
//!   GET  /                    the single-page GUI (from --web <dir>)
//!   GET  /api/state           full state snapshot (telemetry + mission) as JSON
//!   GET  /api/stream          the same snapshot streamed as server-sent events (~5 Hz)
//!   POST /api/mission         start a mission: {"waypoints":[{lat_deg, lon_deg, speed_mps,
//!                             capture_radius_m[, arrival_yaw_rad][, elev_value_m,
//!                             elev_frame]}]} -> {"session": id}
//!   POST /api/mission/cancel  cancel the active mission (disposes the command instance)
//!   POST /api/preview         plan the ideal Dubins route for a candidate mission from the
//!                             vehicle's current pose -> {"path":[[lat,lon],...]}
//! Usage: mission_console [autopilot.yaml] [port] [webroot]
//!
//! The command side reuses WaypointMissionClient (the matured mission_runner core), so the
//! console is a full UMAA Global Waypoint control consumer: command + large-list route out,
//! ack + status + execution status back.

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointExecutionStatusReportType.hpp>
#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>

#include "AutopilotConfig.h"
#include "ConstraintsClient.h"
#include "CycloneQosProviderWrapper.h"
#include "CycloneReader.h"
#include "CycloneUtilities.h"
#include "DubinsPathPlanner.h"
#include "MissionRoute.h"
#include "NumericGuid.h"
#include "PlannerParamsFactory.h"
#include "UuidFactory.h"
#include "WaypointMissionClient.h"
#include "YamlConfigLoader.h"

// httplib drags in <netdb.h>, whose NO_DATA/NO_ADDRESS macros collide with the SDK's
// ReadStatus enumerators — keep it (and anything after it) below the project headers.
#include <httplib.h>          // NOLINT(build/include_order) vendored third-party/httplib
#include <nlohmann/json.hpp>  // NOLINT(build/include_order) vendored third-party/nlohmann

namespace {

using arlcore::autopilot::AutopilotConfig;
using arlcore::autopilot::MissionWaypoint;
using arlcore::autopilot::tools::ConstraintEvent;
using arlcore::autopilot::tools::ConstraintRecord;
using arlcore::autopilot::tools::ConstraintsClient;
using arlcore::autopilot::tools::WaypointMissionClient;
using arlcore::io::CycloneReader;
using arlcore::io::ReadStatus;
using nlohmann::json;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;
using UMAA::SA::VelocityStatus::VelocityReportType;

std::string guidToString(const arlcore::NumericGuid& guid) {
  std::ostringstream oss;
  oss << guid;
  return oss.str();
}

//! \brief Everything the GUI needs, guarded by one mutex and serialized per request.
class ConsoleState {
 public:
  void updatePose(const GlobalPoseReportType& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = pose;
    lastPoseAt_ = std::chrono::steady_clock::now();
  }
  void updateSpeed(const SpeedReportType& speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_ = speed;
  }
  void updateVelocity(const VelocityReportType& vel) {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = vel;
  }
  void updateExecStatus(const GlobalWaypointExecutionStatusReportType& exec) {
    std::lock_guard<std::mutex> lock(mutex_);
    execStatus_ = exec;
    lastExecAt_ = std::chrono::steady_clock::now();
  }
  void pushCommandStatus(const std::string& status, const std::string& reason,
                         const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    statusHistory_.push_back({status, reason, message});
    while (statusHistory_.size() > 25) {
      statusHistory_.pop_front();
    }
  }
  void setMission(const std::string& sessionId, const json& waypoints, const json& preview) {
    std::lock_guard<std::mutex> lock(mutex_);
    sessionId_ = sessionId;
    missionWaypoints_ = waypoints;
    previewPath_ = preview;
    statusHistory_.clear();
    execStatus_.reset();
  }
  void setAck(bool ack) {
    std::lock_guard<std::mutex> lock(mutex_);
    ackReceived_ = ack;
  }
  void setActive(bool active) {
    std::lock_guard<std::mutex> lock(mutex_);
    missionActive_ = active;
  }
  void setConstraints(json constraints) {
    std::lock_guard<std::mutex> lock(mutex_);
    constraints_ = std::move(constraints);
  }

  std::optional<GlobalPoseReportType> latestPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
  }

  json snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    json j;
    j["telemetry"] = json::object();
    if (pose_.has_value()) {
      const auto& p = pose_.value();
      json t;
      t["lat_deg"] = p.position().geodeticLatitude();
      t["lon_deg"] = p.position().geodeticLongitude();
      t["yaw_rad"] = p.attitude().yaw().yaw();
      if (p.depth().has_value()) t["depth_m"] = p.depth().value();
      if (p.altitudeASF().has_value()) t["alt_asf_m"] = p.altitudeASF().value();
      if (p.altitude().has_value()) t["alt_msl_m"] = p.altitude().value();
      const double ageS = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - lastPoseAt_).count();
      t["age_s"] = ageS;
      if (speed_.has_value() && speed_->speedOverGround().has_value()) {
        t["sog_mps"] = speed_->speedOverGround().value();
      }
      if (velocity_.has_value()) {
        t["vel_north_mps"] = velocity_->velocity().northSpeed();
        t["vel_east_mps"] = velocity_->velocity().eastSpeed();
        t["vel_down_mps"] = velocity_->velocity().downSpeed();
      }
      j["telemetry"] = t;
    }

    json m;
    m["active"] = missionActive_;
    m["ack_received"] = ackReceived_;
    if (!sessionId_.empty()) m["session"] = sessionId_;
    m["waypoints"] = missionWaypoints_.is_null() ? json::array() : missionWaypoints_;
    m["planned_path"] = previewPath_.is_null() ? json::array() : previewPath_;
    m["status_history"] = json::array();
    for (const auto& s : statusHistory_) {
      m["status_history"].push_back({{"status", s.status}, {"reason", s.reason},
                                     {"message", s.message}});
    }
    if (execStatus_.has_value()) {
      const auto& e = execStatus_.value();
      json ex;
      ex["waypoint_id"] = guidToString(arlcore::NumericGuid(e.waypointID()));
      ex["waypoints_remaining"] = e.waypointsRemaining();
      ex["distance_to_waypoint_m"] = e.distanceToWaypoint();
      ex["distance_remaining_m"] = e.distanceRemaining();
      ex["cumulative_distance_m"] = e.cumulativeDistance();
      if (e.crossTrackError().has_value()) ex["cross_track_error_m"] = e.crossTrackError().value();
      ex["position_achieved"] = e.positionAchieved();
      if (e.attitudeAchieved().has_value()) ex["attitude_achieved"] = e.attitudeAchieved().value();
      ex["elevation_achieved"] = e.elevationAchieved();
      ex["speed_achieved"] = e.speedAchieved();
      ex["track_line_achieved"] = e.trackLineAchieved();
      const double ageS = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - lastExecAt_).count();
      ex["age_s"] = ageS;
      j["exec_status"] = ex;
    }
    j["mission"] = m;
    j["constraints"] = constraints_.is_null() ? json{{"enabled", false}} : constraints_;
    return j;
  }

 private:
  struct StatusEntry {
    std::string status;
    std::string reason;
    std::string message;
  };

  mutable std::mutex mutex_;
  std::optional<GlobalPoseReportType> pose_;
  std::optional<SpeedReportType> speed_;
  std::optional<VelocityReportType> velocity_;
  std::optional<GlobalWaypointExecutionStatusReportType> execStatus_;
  std::chrono::steady_clock::time_point lastPoseAt_;
  std::chrono::steady_clock::time_point lastExecAt_;
  std::deque<StatusEntry> statusHistory_;
  std::string sessionId_;
  json missionWaypoints_;
  json previewPath_;
  json constraints_;
  bool missionActive_ = false;
  bool ackReceived_ = false;
};

//! \brief Parse the GUI's mission JSON into route waypoints. Throws json exceptions on
//! malformed input (reported as a 400 by the handler).
std::vector<MissionWaypoint> parseMission(const json& body) {
  std::vector<MissionWaypoint> route;
  for (const auto& w : body.at("waypoints")) {
    MissionWaypoint wp;
    wp.latDeg = w.at("lat_deg").get<double>();
    wp.lonDeg = w.at("lon_deg").get<double>();
    wp.speedMps = w.value("speed_mps", 3.0);
    wp.captureRadiusM = w.value("capture_radius_m", 2.5);
    if (w.contains("arrival_yaw_rad") && !w["arrival_yaw_rad"].is_null()) {
      wp.arrivalYawRad = w["arrival_yaw_rad"].get<double>();
    }
    if (w.contains("elev_value_m") && !w["elev_value_m"].is_null()) {
      wp.elevValueM = w["elev_value_m"].get<double>();
      wp.elevFrame = w.value("elev_frame", "depth");
    }
    route.push_back(wp);
  }
  return route;
}

//! \brief The GUI's own record of a waypoint (echoed back in state so every client sees the
//! active mission, not just the one that created it).
json waypointsToJson(const std::vector<MissionWaypoint>& route) {
  json arr = json::array();
  for (const auto& wp : route) {
    json w;
    w["lat_deg"] = wp.latDeg;
    w["lon_deg"] = wp.lonDeg;
    w["speed_mps"] = wp.speedMps;
    w["capture_radius_m"] = wp.captureRadiusM;
    if (wp.arrivalYawRad.has_value()) w["arrival_yaw_rad"] = wp.arrivalYawRad.value();
    if (wp.elevValueM.has_value()) {
      w["elev_value_m"] = wp.elevValueM.value();
      w["elev_frame"] = wp.elevFrame;
    }
    arr.push_back(w);
  }
  return arr;
}

//! \brief Plan the ideal Dubins route for a candidate mission from the given start pose.
json previewPath(const std::vector<MissionWaypoint>& route, const GlobalPoseReportType& start,
                 const AutopilotConfig& config) {
  std::vector<GlobalWaypointType> waypoints;
  for (const auto& wp : route) {
    waypoints.push_back(arlcore::autopilot::makeWaypoint(wp));
  }
  arlcore::autopilot::DubinsPathPlanner planner;
  planner.plan(waypoints, start, arlcore::autopilot::derivePlannerParams(config));
  json path = json::array();
  for (const auto& [lat, lon] : planner.previewRoute(2.0)) {
    path.push_back({lat, lon});
  }
  return path;
}

//! \brief The GUI-facing constraints block: the autopilot's constraint list, the applied
//! active set (from the standing ack), and the latest command activity.
json constraintsJson(const ConstraintsClient& client) {
  json j;
  j["enabled"] = true;
  j["active_known"] = client.activeKnown();
  j["active_ids"] = json::array();
  for (const auto& id : client.activeIds()) {
    j["active_ids"].push_back(id);
  }
  j["items"] = json::array();
  for (const ConstraintRecord& record : client.constraints()) {
    json item;
    item["id"] = record.id;
    item["name"] = record.name;
    item["type"] = record.type;
    item["active"] = record.active;
    if (record.state.has_value()) item["state"] = record.state.value();
    if (!record.polygon.empty()) {
      item["polygon"] = json::array();
      for (const auto& [lat, lon] : record.polygon) {
        item["polygon"].push_back({lat, lon});
      }
    }
    if (record.ceilingM.has_value()) {
      item["ceiling_m"] = record.ceilingM.value();
      item["ceiling_frame"] = record.ceilingFrame;
    }
    if (record.floorM.has_value()) {
      item["floor_m"] = record.floorM.value();
      item["floor_frame"] = record.floorFrame;
    }
    if (record.value.has_value()) item["value"] = record.value.value();
    if (!record.op.empty()) item["op"] = record.op;
    j["items"].push_back(item);
  }
  if (client.lastEvent().has_value()) {
    const ConstraintEvent& e = client.lastEvent().value();
    j["last_event"] = {{"service", e.service}, {"status", e.status}, {"reason", e.reason},
                       {"message", e.message}};
  }
  return j;
}

//! \brief Validate the GUI's constraint JSON; throws std::runtime_error on bad input.
void validateConstraintBody(const json& body) {
  const std::string type = body.at("type").get<std::string>();
  if (type == "keep_in" || type == "keep_out") {
    const auto& polygon = body.at("polygon");
    if (!polygon.is_array() || polygon.size() < 3 || polygon.size() > 128) {
      throw std::runtime_error("zone polygon needs 3..128 vertices");
    }
    for (const auto& v : polygon) {
      if (!v.is_array() || v.size() != 2 || !std::isfinite(v[0].get<double>()) ||
          !std::isfinite(v[1].get<double>())) {
        throw std::runtime_error("zone polygon vertices must be [lat, lon] pairs");
      }
    }
    const double ceiling = body.value("ceiling_m", 0.0);
    const double floor = body.value("floor_m", 100.0);
    const std::string ceilingFrame = body.value("ceiling_frame", "depth");
    const std::string floorFrame = body.value("floor_frame", "depth");
    for (const std::string& frame : {ceilingFrame, floorFrame}) {
      if (frame != "depth" && frame != "asf") {
        throw std::runtime_error("zone elevation frames must be 'depth' or 'asf'");
      }
    }
    // Ordering (ceiling shallower than floor) is only checkable within one frame: shallower
    // means a smaller depth but a larger altitude above the sea floor. Mixed frames (e.g.
    // ceiling at depth 0, floor above the sea floor) are always accepted.
    if (ceilingFrame == floorFrame) {
      const bool ordered = ceilingFrame == "depth" ? ceiling < floor : ceiling > floor;
      if (!ordered) {
        throw std::runtime_error(ceilingFrame == "depth"
            ? "zone ceiling_m (shallower) must be less than floor_m (deeper)"
            : "zone ceiling_m (shallower) must be a larger above-floor altitude than floor_m");
      }
    }
  } else if (type == "speed" || type == "depth") {
    const double value = body.at("value").get<double>();
    if (!std::isfinite(value) || value < 0.0) {
      throw std::runtime_error(type + " value must be a non-negative number");
    }
    const std::string op = body.value("op", "lte");
    if (op != "lte" && op != "gte") {
      throw std::runtime_error("op must be 'lte' or 'gte'");
    }
  } else {
    throw std::runtime_error("unknown constraint type '" + type + "'");
  }
}

GlobalPoseReportType fallbackStartPose(const AutopilotConfig& config) {
  GlobalPoseReportType pose;
  pose.position().geodeticLatitude(config.simVehicle.initialLatitudeDeg);
  pose.position().geodeticLongitude(config.simVehicle.initialLongitudeDeg);
  pose.attitude().yaw().yaw(config.simVehicle.initialHeadingRad);
  return pose;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string configPath = (argc > 1) ? argv[1] : "autopilot.yaml";
  const int port = (argc > 2) ? std::stoi(argv[2]) : 8080;
  const std::string webRoot = (argc > 3) ? argv[3] : "web";

  AutopilotConfig config;
  if (!arlcore::autopilot::YamlConfigLoader::load(configPath, &config)) {
    std::cerr << "Failed to load config " << configPath << std::endl;
    return 1;
  }

  auto participant = arlcore::io::getDomainParticipant(config.dds.domainId);
  arlcore::io::CycloneQosProviderWrapper qosProvider(config.dds.qosFile, config.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();

  auto poseReader = std::make_shared<CycloneReader<GlobalPoseReportType>>(
      participant, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, rqos);
  auto speedReader = std::make_shared<CycloneReader<SpeedReportType>>(
      participant, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, rqos);
  auto velocityReader = std::make_shared<CycloneReader<VelocityReportType>>(
      participant, UMAA::SA::VelocityStatus::VelocityReportTypeTopic, rqos);
  auto execReader = std::make_shared<CycloneReader<GlobalWaypointExecutionStatusReportType>>(
      participant,
      UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportTypeTopic, rqos);

  WaypointMissionClient client(
      participant, wqos, rqos,
      arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.waypointSourceId));

  // Constraint services client (only when the autopilot's constraint source is configured).
  std::optional<ConstraintsClient> constraintsClient;
  if (!config.identity.constraintsSourceId.empty()) {
    const auto largeSetRqos = qosProvider.datareader_qos(config.dds.largeCollectionsQosProfile);
    constraintsClient.emplace(
        participant, wqos, rqos, largeSetRqos,
        arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.constraintsSourceId));
  }

  ConsoleState state;
  std::mutex clientMutex;  // guards `client` between the poller and the POST handlers
  std::atomic<bool> running{true};

  // DDS poller: drain the report topics into the state at 20 Hz.
  std::thread poller([&]() {
    while (running) {
      GlobalPoseReportType pose;
      if (poseReader->readLatest(&pose) == ReadStatus::SUCCESS) {
        state.updatePose(pose);
      }
      SpeedReportType speed;
      if (speedReader->readLatest(&speed) == ReadStatus::SUCCESS) {
        state.updateSpeed(speed);
      }
      VelocityReportType vel;
      if (velocityReader->readLatest(&vel) == ReadStatus::SUCCESS) {
        state.updateVelocity(vel);
      }
      GlobalWaypointExecutionStatusReportType exec;
      if (execReader->readLatest(&exec) == ReadStatus::SUCCESS) {
        state.updateExecStatus(exec);
      }
      {
        std::lock_guard<std::mutex> lock(clientMutex);
        for (const auto& update : client.pollStatus()) {
          state.pushCommandStatus(update.status, update.reason, update.logMessage);
        }
        state.setAck(client.pollAck());
        state.setActive(client.active());
        if (constraintsClient.has_value()) {
          constraintsClient->poll();
          state.setConstraints(constraintsJson(constraintsClient.value()));
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });

  httplib::Server server;
  if (!server.set_mount_point("/", webRoot)) {
    std::cerr << "Web root '" << webRoot << "' not found (serving API only)" << std::endl;
  }

  server.Get("/api/state", [&](const httplib::Request&, httplib::Response& res) {
    res.set_content(state.snapshot().dump(), "application/json");
  });

  server.Get("/api/stream", [&](const httplib::Request&, httplib::Response& res) {
    res.set_chunked_content_provider("text/event-stream",
        [&state, &running](size_t /*offset*/, httplib::DataSink& sink) {
          if (!running) {
            return false;
          }
          const std::string frame = "data: " + state.snapshot().dump() + "\n\n";
          if (!sink.write(frame.data(), frame.size())) {
            return false;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
          return true;
        });
  });

  server.Post("/api/mission", [&](const httplib::Request& req, httplib::Response& res) {
    try {
      const json body = json::parse(req.body);
      const std::vector<MissionWaypoint> route = parseMission(body);
      if (route.empty()) {
        res.status = 400;
        res.set_content(R"({"error":"mission has no waypoints"})", "application/json");
        return;
      }
      std::lock_guard<std::mutex> lock(clientMutex);
      if (client.active()) {
        res.status = 409;
        res.set_content(R"({"error":"a mission is already active"})", "application/json");
        return;
      }
      std::vector<GlobalWaypointType> waypoints;
      for (const auto& wp : route) {
        waypoints.push_back(arlcore::autopilot::makeWaypoint(wp));
      }
      const GlobalPoseReportType start =
          state.latestPose().value_or(fallbackStartPose(config));
      const json preview = previewPath(route, start, config);
      const arlcore::NumericGuid session = client.start(waypoints);
      state.setMission(guidToString(session), waypointsToJson(route), preview);
      state.setActive(client.active());
      res.set_content(json{{"session", guidToString(session)}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Post("/api/mission/cancel", [&](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(clientMutex);
    if (!client.active()) {
      res.status = 409;
      res.set_content(R"({"error":"no active mission"})", "application/json");
      return;
    }
    const bool ok = client.cancel();
    res.set_content(json{{"canceled", ok}}.dump(), "application/json");
  });

  server.Post("/api/preview", [&](const httplib::Request& req, httplib::Response& res) {
    try {
      const json body = json::parse(req.body);
      const std::vector<MissionWaypoint> route = parseMission(body);
      const GlobalPoseReportType start =
          state.latestPose().value_or(fallbackStartPose(config));
      res.set_content(json{{"path", previewPath(route, start, config)}}.dump(),
                      "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Post("/api/constraints", [&](const httplib::Request& req, httplib::Response& res) {
    if (!constraintsClient.has_value()) {
      res.status = 503;
      res.set_content(R"({"error":"constraints are not configured"})", "application/json");
      return;
    }
    try {
      const json body = json::parse(req.body);
      validateConstraintBody(body);
      const std::string type = body.at("type").get<std::string>();
      const std::string id = body.value("id", "");
      const std::string name = body.value("name", type);
      std::lock_guard<std::mutex> lock(clientMutex);
      std::string newId;
      if (type == "keep_in" || type == "keep_out") {
        std::vector<std::array<double, 2>> polygon;
        for (const auto& v : body.at("polygon")) {
          polygon.push_back({v[0].get<double>(), v[1].get<double>()});
        }
        newId = constraintsClient->upsertZone(id, name, type == "keep_in", polygon,
                                              body.value("ceiling_m", 0.0),
                                              body.value("ceiling_frame", "depth"),
                                              body.value("floor_m", 100.0),
                                              body.value("floor_frame", "depth"));
      } else if (type == "speed") {
        newId = constraintsClient->upsertSpeed(id, name, body.value("op", "lte"),
                                               body.at("value").get<double>());
      } else {
        newId = constraintsClient->upsertDepth(id, name, body.value("op", "lte"),
                                               body.at("value").get<double>());
      }
      res.set_content(json{{"id", newId}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Delete(R"(/api/constraints/([0-9a-fA-F-]+))",
                [&](const httplib::Request& req, httplib::Response& res) {
    if (!constraintsClient.has_value()) {
      res.status = 503;
      res.set_content(R"({"error":"constraints are not configured"})", "application/json");
      return;
    }
    try {
      std::lock_guard<std::mutex> lock(clientMutex);
      const bool ok = constraintsClient->removeConstraint(req.matches[1]);
      res.set_content(json{{"deleted", ok}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Post("/api/constraints/active", [&](const httplib::Request& req, httplib::Response& res) {
    if (!constraintsClient.has_value()) {
      res.status = 503;
      res.set_content(R"({"error":"constraints are not configured"})", "application/json");
      return;
    }
    try {
      const json body = json::parse(req.body);
      std::vector<std::string> ids;
      for (const auto& id : body.at("ids")) {
        ids.push_back(id.get<std::string>());
      }
      std::lock_guard<std::mutex> lock(clientMutex);
      const bool ok = constraintsClient->setActive(ids);
      res.set_content(json{{"commanded", ok}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  std::cout << "Mission console listening on http://0.0.0.0:" << port
            << " (web root: " << webRoot << ", DDS domain " << config.dds.domainId << ")"
            << std::endl;
  server.listen("0.0.0.0", port);

  running = false;
  poller.join();
  return 0;
}
