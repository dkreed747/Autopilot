//! \brief Live mission-control web console bridging the UMAA DDS bus to a browser GUI; the
//! REST API is documented in tools/README.md.

#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointExecutionStatusReportType.hpp>
#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>
#include <UMAA/SA/SpeedStatus/SpeedReportType.hpp>
#include <UMAA/SA/VelocityStatus/VelocityReportType.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "CycloneQosProviderWrapper.h"
#include "CycloneReader.h"
#include "CycloneUtilities.h"
#include "NumericGuid.h"
#include "UmaaUtils.h"
#include "UuidFactory.h"
#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/config/ConfigValidation.hpp"
#include "autopilot/config/YamlConfigLoader.hpp"
#include "autopilot/guidance/DubinsPathPlanner.hpp"
#include "autopilot/guidance/ElevationUtils.hpp"
#include "autopilot/guidance/MissionRoute.hpp"
#include "autopilot/guidance/PlannerParamsFactory.hpp"
#include "autopilot/guidance/ToleranceUtils.hpp"
#include "clients/ClientIdentity.hpp"
#include "clients/ConstraintsClient.hpp"
#include "clients/OperationalModeClient.hpp"
#include "clients/VectorCommandClient.hpp"
#include "clients/WaypointMissionClient.hpp"
#include "monitors/VectorActivityMonitor.hpp"
#include "monitors/WaypointActivityMonitor.hpp"

// clang-format off
// httplib drags in <netdb.h>, whose NO_DATA/NO_ADDRESS macros collide with the SDK's
// ReadStatus enumerators — keep it (and anything after it) below the project headers.
#include <httplib.h>          // NOLINT(build/include_order) vendored third-party/httplib
#include <nlohmann/json.hpp>  // NOLINT(build/include_order) vendored third-party/nlohmann
#include "InternalTypes.h"
// clang-format on

using arlcore::autopilot::AutopilotConfig;
using arlcore::autopilot::MissionWaypoint;
using arlcore::autopilot::tools::ClientIdentity;
using arlcore::autopilot::tools::ConstraintEvent;
using arlcore::autopilot::tools::ConstraintRecord;
using arlcore::autopilot::tools::ConstraintsClient;
using arlcore::autopilot::tools::ObservedMission;
using arlcore::autopilot::tools::ObservedVector;
using arlcore::autopilot::tools::OperationalModeClient;
using arlcore::autopilot::tools::VectorActivityMonitor;
using arlcore::autopilot::tools::VectorCommandClient;
using arlcore::autopilot::tools::VectorSetpoint;
using arlcore::autopilot::tools::WaypointActivityMonitor;
using arlcore::autopilot::tools::WaypointMissionClient;
using arlcore::io::CycloneReader;
using arlcore::io::ReadStatus;
using nlohmann::json;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType;
using UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
using UMAA::SA::GlobalPoseStatus::GlobalPoseReportType;
using UMAA::SA::SpeedStatus::SpeedReportType;
using UMAA::SA::VelocityStatus::VelocityReportType;

static std::string guidToString(const arlcore::NumericGuid& guid) {
  std::ostringstream oss;
  oss << guid;
  return oss.str();
}

//! \brief Everything the GUI needs, guarded by one mutex and serialized per request.
class ConsoleState {
 public:
  void updatePose(const GlobalPoseReportType& pose) {
    std::scoped_lock lock(mutex_);
    pose_ = pose;
    lastPoseAt_ = std::chrono::steady_clock::now();
  }
  void updateSpeed(const SpeedReportType& speed) {
    std::scoped_lock lock(mutex_);
    speed_ = speed;
  }
  void updateVelocity(const VelocityReportType& vel) {
    std::scoped_lock lock(mutex_);
    velocity_ = vel;
  }
  void updateExecStatus(const GlobalWaypointExecutionStatusReportType& exec) {
    std::scoped_lock lock(mutex_);
    execStatus_ = exec;
    lastExecAt_ = std::chrono::steady_clock::now();
  }
  void pushCommandStatus(const std::string& status, const std::string& reason, const std::string& message) {
    std::scoped_lock lock(mutex_);
    statusHistory_.push_back({status, reason, message});
    while (statusHistory_.size() > 25) {
      statusHistory_.pop_front();
    }
  }
  void setMission(const std::string& sessionId, const json& waypoints, const json& preview) {
    std::scoped_lock lock(mutex_);
    sessionId_ = sessionId;
    missionWaypoints_ = waypoints;
    previewPath_ = preview;
    statusHistory_.clear();
    execStatus_.reset();
  }
  void setAck(bool ack) {
    std::scoped_lock lock(mutex_);
    ackReceived_ = ack;
  }
  void setActive(bool active) {
    std::scoped_lock lock(mutex_);
    missionActive_ = active;
  }
  void setConstraints(json constraints) {
    std::scoped_lock lock(mutex_);
    constraints_ = std::move(constraints);
  }
  void setPlatform(json platform) {
    std::scoped_lock lock(mutex_);
    platform_ = std::move(platform);
  }
  void setOperationalMode(json opMode) {
    std::scoped_lock lock(mutex_);
    operationalMode_ = std::move(opMode);
  }
  void setVector(json vector) {
    std::scoped_lock lock(mutex_);
    vector_ = std::move(vector);
  }
  void setTraffic(json traffic) {
    std::scoped_lock lock(mutex_);
    traffic_ = std::move(traffic);
  }
  void pushVectorStatus(const std::string& status, const std::string& reason, const std::string& message) {
    std::scoped_lock lock(mutex_);
    vectorStatusHistory_.push_back({status, reason, message});
    while (vectorStatusHistory_.size() > 25) {
      vectorStatusHistory_.pop_front();
    }
  }
  void clearVectorStatusHistory() {
    std::scoped_lock lock(mutex_);
    vectorStatusHistory_.clear();
  }

  std::optional<GlobalPoseReportType> latestPose() const {
    std::scoped_lock lock(mutex_);
    return pose_;
  }

  json snapshot() const {
    std::scoped_lock lock(mutex_);
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
      const flt64_t ageS = std::chrono::duration<flt64_t>(std::chrono::steady_clock::now() - lastPoseAt_).count();
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
      m["status_history"].push_back({{"status", s.status}, {"reason", s.reason}, {"message", s.message}});
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
      const flt64_t ageS = std::chrono::duration<flt64_t>(std::chrono::steady_clock::now() - lastExecAt_).count();
      ex["age_s"] = ageS;
      j["exec_status"] = ex;
    }
    j["mission"] = m;
    j["constraints"] = constraints_.is_null() ? json{{"enabled", false}} : constraints_;
    if (!platform_.is_null()) {
      j["platform"] = platform_;
    }
    if (!operationalMode_.is_null()) {
      j["operational_mode"] = operationalMode_;
    }
    if (!vector_.is_null()) {
      json v = vector_;
      v["status_history"] = json::array();
      for (const auto& s : vectorStatusHistory_) {
        v["status_history"].push_back({{"status", s.status}, {"reason", s.reason}, {"message", s.message}});
      }
      j["vector"] = v;
    }
    if (!traffic_.is_null()) {
      j["traffic"] = traffic_;
    }
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
  std::deque<StatusEntry> vectorStatusHistory_;
  std::string sessionId_;
  json missionWaypoints_;
  json previewPath_;
  json constraints_;
  json platform_;
  json operationalMode_;
  json vector_;
  json traffic_;
  bool missionActive_ = false;
  bool ackReceived_ = false;
};

//! \brief Parse the GUI's mission JSON into route waypoints. Throws json exceptions on
//! malformed input (reported as a 400 by the handler).
static std::vector<MissionWaypoint> parseMission(const json& body) {
  std::vector<MissionWaypoint> route;
  for (const auto& w : body.at("waypoints")) {
    MissionWaypoint wp;
    wp.latDeg = w.at("lat_deg").get<flt64_t>();
    wp.lonDeg = w.at("lon_deg").get<flt64_t>();
    wp.speedMps = w.value("speed_mps", 3.0);
    wp.captureRadiusM = w.value("capture_radius_m", 2.5);
    if (w.contains("arrival_yaw_rad") && !w["arrival_yaw_rad"].is_null()) {
      wp.arrivalYawRad = w["arrival_yaw_rad"].get<flt64_t>();
    }
    if (w.contains("elev_value_m") && !w["elev_value_m"].is_null()) {
      wp.elevValueM = w["elev_value_m"].get<flt64_t>();
      wp.elevFrame = w.value("elev_frame", "depth");
    }
    route.push_back(wp);
  }
  return route;
}

//! \brief The GUI's own record of a waypoint (echoed back in state so every client sees the
//! active mission, not just the one that created it).
static json waypointsToJson(const std::vector<MissionWaypoint>& route) {
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
static json previewPath(const std::vector<MissionWaypoint>& route, const GlobalPoseReportType& start,
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
static json constraintsJson(const ConstraintsClient& client) {
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
    j["last_event"] = {{"service", e.service}, {"status", e.status}, {"reason", e.reason}, {"message", e.message}};
  }
  return j;
}

//! \brief Validate the GUI's constraint JSON; throws std::runtime_error on bad input.
static void validateConstraintBody(const json& body) {
  const std::string type = body.at("type").get<std::string>();
  if (type == "keep_in" || type == "keep_out") {
    const auto& polygon = body.at("polygon");
    if (!polygon.is_array() || polygon.size() < 3 || polygon.size() > 128) {
      throw std::runtime_error("zone polygon needs 3..128 vertices");
    }
    for (const auto& v : polygon) {
      if (!v.is_array() || v.size() != 2 || !std::isfinite(v[0].get<flt64_t>()) ||
          !std::isfinite(v[1].get<flt64_t>())) {
        throw std::runtime_error("zone polygon vertices must be [lat, lon] pairs");
      }
    }
    const flt64_t ceiling = body.value("ceiling_m", 0.0);
    const flt64_t floor = body.value("floor_m", 100.0);
    const std::string ceilingFrame = body.value("ceiling_frame", "depth");
    const std::string floorFrame = body.value("floor_frame", "depth");
    for (const std::string& frame : {ceilingFrame, floorFrame}) {
      if (frame != "depth" && frame != "asf") {
        throw std::runtime_error("zone elevation frames must be 'depth' or 'asf'");
      }
    }
    // Ceiling/floor ordering is only checkable within one frame (shallower = smaller depth
    // but larger ASF altitude), so mixed frames are always accepted.
    if (ceilingFrame == floorFrame) {
      const bool ordered = ceilingFrame == "depth" ? ceiling < floor : ceiling > floor;
      if (!ordered) {
        throw std::runtime_error(ceilingFrame == "depth"
                                     ? "zone ceiling_m (shallower) must be less than floor_m (deeper)"
                                     : "zone ceiling_m (shallower) must be a larger above-floor altitude than floor_m");
      }
    }
  } else if (type == "speed" || type == "depth") {
    const flt64_t value = body.at("value").get<flt64_t>();
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

static GlobalPoseReportType fallbackStartPose(const AutopilotConfig& config) {
  GlobalPoseReportType pose;
  pose.position().geodeticLatitude(config.simVehicle.initialLatitudeDeg);
  pose.position().geodeticLongitude(config.simVehicle.initialLongitudeDeg);
  pose.attitude().yaw().yaw(config.simVehicle.initialHeadingRad);
  return pose;
}

constexpr flt64_t kRadToDeg = 180.0 / M_PI;
constexpr flt64_t kDegToRad = M_PI / 180.0;
constexpr flt64_t kRcDeadmanS = 2.0;         // rolling DDS endTime while RC is engaged
constexpr flt64_t kRcServerWatchdogS = 1.0;  // browser-silence threshold before auto-stop

//! \brief Classify a bus command's origin: the console itself, this platform's onboard
//! autonomy, or any other (remote) commander. A nil autopilot platform id can never match.
static std::string classifySource(const arlcore::NumericGuid& sourceId, const arlcore::NumericGuid& sourceParentId,
                                  const ClientIdentity& console, const arlcore::NumericGuid& autopilotPlatformId) {
  if (sourceId == console.sourceId) {
    return "own";
  }
  if (autopilotPlatformId != arlcore::NumericGuid() && sourceParentId == autopilotPlatformId) {
    return "local";
  }
  return "remote";
}

//! \brief The GUI-facing operational-mode block.
static json operationalModeJson(const OperationalModeClient& client) {
  json j;
  if (client.reportedMode().has_value()) {
    j["mode"] = client.reportedMode().value();
    j["report_age_s"] = client.reportAgeS().value_or(0.0);
  }
  if (client.pendingMode().has_value()) {
    j["pending_mode"] = client.pendingMode().value();
  }
  j["ack_received"] = client.ackReceived();
  if (client.lastStatus().has_value()) {
    const auto& s = client.lastStatus().value();
    j["last_status"] = {{"status", s.status}, {"reason", s.reason}, {"message", s.message}};
  }
  return j;
}

//! \brief The GUI-facing block for the console's own vector session (history appended by
//! ConsoleState::snapshot).
static json vectorJson(VectorCommandClient& client, bool rcEngaged) {
  json v;
  v["active"] = client.active();
  v["rc_active"] = rcEngaged;
  v["ack_received"] = client.ackReceived();
  if (client.sessionId().has_value()) {
    v["session"] = guidToString(client.sessionId().value());
    if (client.lastSetpoint().has_value()) {
      const VectorSetpoint& sp = client.lastSetpoint().value();
      json s;
      s["heading_deg"] = sp.headingRad * kRadToDeg;
      s["speed_mps"] = sp.speedMps;
      if (sp.elevValueM.has_value()) {
        s["elev_value_m"] = sp.elevValueM.value();
        s["elev_frame"] = sp.elevFrame;
      }
      if (sp.timeoutS.has_value()) {
        s["timeout_s"] = sp.timeoutS.value();
      }
      v["setpoint"] = s;
    }
  }
  const auto exec = client.pollExec();
  if (exec.has_value()) {
    json ex;
    ex["direction_achieved"] = exec->directionAchieved();
    ex["speed_achieved"] = exec->speedAchieved();
    ex["elevation_achieved"] = exec->elevationAchieved();
    ex["age_s"] = client.execAgeS().value_or(0.0);
    v["exec"] = ex;
  }
  return v;
}

//! \brief Seconds until a UMAA end time passes (negative = already past).
static flt64_t secondsUntil(const UMAA::Common::Measurement::DateTime& endTime) {
  const UMAA::Common::Measurement::DateTime now = arlcore::umaa::getTimestamp();
  return static_cast<flt64_t>(endTime.seconds() - now.seconds()) +
         (static_cast<flt64_t>(endTime.nanoseconds()) - static_cast<flt64_t>(now.nanoseconds())) * 1e-9;
}

//! \brief Every waypoint mission and vector command observed on the bus, classified per
//! commander so the GUI can color them.
static json trafficJson(const WaypointActivityMonitor& wpMonitor, const VectorActivityMonitor& vecMonitor,
                        const ClientIdentity& console, const arlcore::NumericGuid& autopilotPlatformId) {
  const auto now = std::chrono::steady_clock::now();
  json traffic;
  traffic["missions"] = json::array();
  for (const auto& [session, mission] : wpMonitor.missions()) {
    json m;
    m["session"] = guidToString(session);
    m["source_id"] = guidToString(mission.sourceId);
    m["classification"] = classifySource(mission.sourceId, mission.sourceParentId, console, autopilotPlatformId);
    if (!mission.lastStatus.empty()) {
      m["status"] = mission.lastStatus;
    }
    m["active"] = !mission.terminal;
    m["list_complete"] = mission.listComplete;
    m["waypoints"] = json::array();
    for (const auto& wp : mission.waypoints) {
      json w;
      w["lat_deg"] = wp.position().value().geodeticLatitude();
      w["lon_deg"] = wp.position().value().geodeticLongitude();
      const auto speed = arlcore::autopilot::tolerance::extractSpeed(wp.speed());
      if (speed.has_value()) {
        w["speed_mps"] = speed->speedMps;
      }
      if (wp.elevation().has_value()) {
        const auto elev = arlcore::autopilot::tolerance::extractElevation(wp.elevation().value());
        if (elev.has_value()) {
          w["elev_value_m"] = elev->valueM;
          w["elev_frame"] = arlcore::autopilot::elevation::frameName(elev->frame);
        }
      }
      m["waypoints"].push_back(w);
    }
    if (mission.execStatus.has_value()) {
      m["waypoints_remaining"] = mission.execStatus->waypointsRemaining();
    }
    m["age_s"] = std::chrono::duration<flt64_t>(now - mission.lastSeen).count();
    traffic["missions"].push_back(m);
  }
  traffic["vectors"] = json::array();
  for (const auto& [session, vec] : vecMonitor.vectors()) {
    json v;
    v["session"] = guidToString(session);
    v["source_id"] = guidToString(vec.sourceId);
    v["classification"] = classifySource(vec.sourceId, vec.sourceParentId, console, autopilotPlatformId);
    if (vec.headingRad.has_value()) {
      v["heading_deg"] = vec.headingRad.value() * kRadToDeg;
    }
    if (vec.speedMps.has_value()) {
      v["speed_mps"] = vec.speedMps.value();
    }
    if (vec.elevValueM.has_value()) {
      v["elev_value_m"] = vec.elevValueM.value();
      v["elev_frame"] = vec.elevFrame;
    }
    if (vec.endTime.has_value()) {
      v["ends_in_s"] = secondsUntil(vec.endTime.value());
    }
    if (!vec.lastStatus.empty()) {
      v["status"] = vec.lastStatus;
    }
    v["active"] = !vec.terminal;
    if (vec.execStatus.has_value()) {
      v["direction_achieved"] = vec.execStatus->directionAchieved();
      v["speed_achieved"] = vec.execStatus->speedAchieved();
      v["elevation_achieved"] = vec.execStatus->elevationAchieved();
    }
    v["age_s"] = std::chrono::duration<flt64_t>(now - vec.lastSeen).count();
    traffic["vectors"].push_back(v);
  }
  return traffic;
}

//! \brief The static platform-limits block (from config, computed once).
static json platformJson(const AutopilotConfig& config) {
  json p;
  p["max_speed_mps"] = config.platformCapabilities.surface.maxForwardSpeedMps.value_or(0.0);
  if (config.platformCapabilities.underwaterEnabled &&
      config.platformCapabilities.underwater.maxForwardSpeedMps.has_value()) {
    p["max_speed_underwater_mps"] = config.platformCapabilities.underwater.maxForwardSpeedMps.value();
  }
  p["floor_depth_m"] = config.simVehicle.floorDepthM;
  return p;
}

//! \brief The speed ceiling for console-issued vector commands: the surface limit, tightened
//! by the underwater limit when a submerged elevation is commanded.
static flt64_t vectorSpeedLimit(const AutopilotConfig& config, bool submerged) {
  flt64_t limit = config.platformCapabilities.surface.maxForwardSpeedMps.value_or(0.0);
  if (submerged && config.platformCapabilities.underwaterEnabled &&
      config.platformCapabilities.underwater.maxForwardSpeedMps.has_value()) {
    limit = std::min(limit, config.platformCapabilities.underwater.maxForwardSpeedMps.value());
  }
  return limit;
}

//! \brief Parse + validate the GUI's vector JSON; throws std::runtime_error on bad input.
static VectorSetpoint parseVectorBody(const json& body, const AutopilotConfig& config) {
  VectorSetpoint sp;
  const flt64_t headingDeg = body.at("heading_deg").get<flt64_t>();
  const flt64_t speedMps = body.at("speed_mps").get<flt64_t>();
  if (!std::isfinite(headingDeg) || !std::isfinite(speedMps)) {
    throw std::runtime_error("heading_deg and speed_mps must be finite numbers");
  }
  sp.headingRad = headingDeg * kDegToRad;
  if (body.contains("elev_value_m") && !body["elev_value_m"].is_null()) {
    const flt64_t elev = body["elev_value_m"].get<flt64_t>();
    if (!std::isfinite(elev) || elev < 0.0) {
      throw std::runtime_error("elev_value_m must be a non-negative number");
    }
    sp.elevValueM = elev;
    sp.elevFrame = body.value("elev_frame", "depth");
    if (sp.elevFrame != "depth" && sp.elevFrame != "asf") {
      throw std::runtime_error("elev_frame must be 'depth' or 'asf'");
    }
  }
  if (body.contains("timeout_s") && !body["timeout_s"].is_null()) {
    const flt64_t timeout = body["timeout_s"].get<flt64_t>();
    if (!std::isfinite(timeout) || timeout <= 0.0) {
      throw std::runtime_error("timeout_s must be a positive number");
    }
    sp.timeoutS = timeout;
  }
  const flt64_t limit = vectorSpeedLimit(config, sp.elevValueM.has_value());
  if (speedMps < 0.0) {
    throw std::runtime_error("speed_mps must be non-negative");
  }
  // Never trust the browser: clamp to the platform limit rather than round-tripping a
  // VALIDATION_FAILED from the autopilot.
  sp.speedMps = (limit > 0.0) ? std::min(speedMps, limit) : speedMps;
  return sp;
}

int main(int argc, char** argv) {
  const std::string configPath = (argc > 1) ? argv[1] : "autopilot.yaml";
  int32_t port = 8080;
  if (argc > 2) {
    try {
      port = std::stoi(argv[2]);
    } catch (const std::exception&) {
      port = -1;
    }
    if (port < 1 || port > 65535) {
      std::cerr << "Usage: mission_console [config.yaml] [port] [webroot] -- invalid port '" << argv[2] << "'"
                << std::endl;
      return 2;
    }
  }
  const std::string webRoot = (argc > 3) ? argv[3] : "web";

  AutopilotConfig config;
  if (!arlcore::autopilot::YamlConfigLoader::load(configPath, &config)) {
    std::cerr << "Failed to load config " << configPath << std::endl;
    return 1;
  }

  // The console addresses its commands to these ids; an empty or malformed id would become
  // an indeterminate GUID and every command would be silently discarded by the autopilot.
  for (const auto& [key, value] :
       {std::pair<const char*, const std::string&>{"identity.waypoint_source_id", config.identity.waypointSourceId},
        std::pair<const char*, const std::string&>{"identity.vector_source_id", config.identity.vectorSourceId}}) {
    if (!arlcore::autopilot::isValidUuid(value)) {
      std::cerr << "mission_console requires a valid UUID for " << key << " (got '" << value << "')" << std::endl;
      return 1;
    }
  }

  auto participant = arlcore::io::getDomainParticipant(config.dds.domainId);
  arlcore::io::CycloneQosProviderWrapper qosProvider(config.dds.qosFile, config.dds.domainQosProfile);
  const auto rqos = qosProvider.datareader_qos();
  const auto wqos = qosProvider.datawriter_qos();

  auto poseReader = std::make_shared<CycloneReader<GlobalPoseReportType>>(
      participant, UMAA::SA::GlobalPoseStatus::GlobalPoseReportTypeTopic, rqos);
  auto speedReader =
      std::make_shared<CycloneReader<SpeedReportType>>(participant, UMAA::SA::SpeedStatus::SpeedReportTypeTopic, rqos);
  auto velocityReader = std::make_shared<CycloneReader<VelocityReportType>>(
      participant, UMAA::SA::VelocityStatus::VelocityReportTypeTopic, rqos);
  // Console identity: platform_id equal to the autopilot's classifies as local autonomy,
  // anything else as a REMOTE operator.
  const ClientIdentity consoleIdentity = arlcore::autopilot::tools::makeClientIdentity(config.console);
  const arlcore::NumericGuid autopilotPlatformId =
      config.identity.platformId.empty()
          ? arlcore::NumericGuid()
          : arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.platformId);

  WaypointMissionClient client(
      participant, wqos, rqos,
      arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.waypointSourceId), consoleIdentity);

  std::optional<ConstraintsClient> constraintsClient;
  if (!config.identity.constraintsSourceId.empty()) {
    const auto largeSetRqos = qosProvider.datareader_qos(config.dds.largeCollectionsQosProfile);
    constraintsClient.emplace(
        participant, wqos, rqos, largeSetRqos,
        arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.constraintsSourceId), consoleIdentity);
  }

  std::optional<OperationalModeClient> modeClient;
  if (!config.identity.operationalModeControlSourceId.empty()) {
    modeClient.emplace(
        participant, wqos, rqos,
        arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.operationalModeControlSourceId),
        consoleIdentity);
  }

  VectorCommandClient vectorClient(
      participant, wqos, rqos, arlcore::UuidFactory::getInstance().parseGuidFromString(config.identity.vectorSourceId),
      consoleIdentity);

  // Bus-wide observers: every waypoint mission and vector command, whoever commanded it.
  WaypointActivityMonitor wpMonitor(participant, rqos,
                                    qosProvider.datareader_qos(config.dds.largeCollectionsQosProfile));
  VectorActivityMonitor vecMonitor(participant, rqos);

  ConsoleState state;
  state.setPlatform(platformJson(config));
  std::mutex clientMutex;  // guards the DDS clients between the poller and the POST handlers
  std::atomic<bool> running{true};

  // Remote-control bookkeeping (guarded by clientMutex): while engaged the browser heartbeats
  // /api/rc; the poller's watchdog stops the vehicle if the heartbeats die.
  bool rcEngaged = false;
  std::chrono::steady_clock::time_point rcLastBeat{};

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
      {
        std::scoped_lock lock(clientMutex);
        for (const auto& update : client.pollStatus()) {
          state.pushCommandStatus(update.status, update.reason, update.logMessage);
        }
        state.setAck(client.pollAck());
        state.setActive(client.active());
        if (constraintsClient.has_value()) {
          constraintsClient->poll();
          state.setConstraints(constraintsJson(constraintsClient.value()));
        }
        wpMonitor.poll();
        vecMonitor.poll();
        // The console's own mission execution status comes session-routed from the monitor
        // (the old unfiltered reader happily displayed other commanders' progress).
        if (client.sessionId().has_value()) {
          const auto exec = wpMonitor.execFor(client.sessionId().value());
          if (exec.has_value()) {
            state.updateExecStatus(exec.value());
          }
        }
        if (modeClient.has_value()) {
          modeClient->poll();
          state.setOperationalMode(operationalModeJson(modeClient.value()));
        }
        for (const auto& update : vectorClient.pollStatus()) {
          state.pushVectorStatus(update.status, update.reason, update.logMessage);
        }
        vectorClient.pollAck();
        // Server-side RC deadman: browser heartbeats stopped, so stop the vehicle (the DDS
        // endTime still covers a dead console process).
        if (rcEngaged && std::chrono::duration<flt64_t>(std::chrono::steady_clock::now() - rcLastBeat).count() >
                             kRcServerWatchdogS) {
          if (vectorClient.active() && vectorClient.lastSetpoint().has_value()) {
            VectorSetpoint stop = vectorClient.lastSetpoint().value();
            stop.speedMps = 0.0;
            stop.timeoutS = kRcDeadmanS;
            vectorClient.update(stop);
            vectorClient.cancel();
          }
          rcEngaged = false;
        }
        state.setVector(vectorJson(vectorClient, rcEngaged));
        state.setTraffic(trafficJson(wpMonitor, vecMonitor, consoleIdentity, autopilotPlatformId));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });

  httplib::Server server;
  // Each SSE client pins a worker thread, so a large pool plus a hard stream cap keeps
  // workers free for /api/rc heartbeats — starving them would trip the deadman mid-drive.
  server.new_task_queue = [] {
    return new httplib::ThreadPool(16);
  };  // NOLINT: httplib takes ownership of the raw pointer
  constexpr int32_t kMaxSseClients = 8;
  auto sseClients = std::make_shared<std::atomic<int32_t>>(0);
  if (!server.set_mount_point("/", webRoot)) {
    std::cerr << "Web root '" << webRoot << "' not found (serving API only)" << std::endl;
  }

  server.Get("/api/state", [&](const httplib::Request&, httplib::Response& res) {
    res.set_content(state.snapshot().dump(), "application/json");
  });

  server.Get("/api/stream", [&, sseClients](const httplib::Request&, httplib::Response& res) {
    if (sseClients->fetch_add(1) >= kMaxSseClients) {
      sseClients->fetch_sub(1);
      res.status = 503;
      res.set_content(R"({"error":"too many stream clients"})", "application/json");
      return;
    }
    res.set_chunked_content_provider(
        "text/event-stream",
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
        },
        [sseClients](bool /*success*/) { sseClients->fetch_sub(1); });
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
      {
        std::scoped_lock lock(clientMutex);
        if (client.active()) {
          res.status = 409;
          res.set_content(R"({"error":"a mission is already active"})", "application/json");
          return;
        }
      }
      // Planning the preview is the expensive part: do it unlocked (like /api/preview) so the
      // 20 Hz poller and RC heartbeats never wait on route planning.
      std::vector<GlobalWaypointType> waypoints;
      for (const auto& wp : route) {
        waypoints.push_back(arlcore::autopilot::makeWaypoint(wp));
      }
      const GlobalPoseReportType start = state.latestPose().value_or(fallbackStartPose(config));
      const json preview = previewPath(route, start, config);
      std::scoped_lock lock(clientMutex);
      if (client.active()) {
        res.status = 409;
        res.set_content(R"({"error":"a mission is already active"})", "application/json");
        return;
      }
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
    std::scoped_lock lock(clientMutex);
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
      const GlobalPoseReportType start = state.latestPose().value_or(fallbackStartPose(config));
      res.set_content(json{{"path", previewPath(route, start, config)}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Post("/api/mode", [&](const httplib::Request& req, httplib::Response& res) {
    if (!modeClient.has_value()) {
      res.status = 503;
      res.set_content(R"({"error":"operational mode control is not configured"})", "application/json");
      return;
    }
    try {
      const json body = json::parse(req.body);
      const std::string mode = body.at("mode").get<std::string>();
      std::scoped_lock lock(clientMutex);
      const auto session = modeClient->command(mode);
      if (!session.has_value()) {
        res.status = 400;
        res.set_content(R"({"error":"mode must be STANDBY, REMOTE, or AUTONOMOUS"})", "application/json");
        return;
      }
      res.set_content(json{{"session", guidToString(session.value())}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Post("/api/vector", [&](const httplib::Request& req, httplib::Response& res) {
    try {
      const json body = json::parse(req.body);
      const VectorSetpoint sp = parseVectorBody(body, config);
      std::scoped_lock lock(clientMutex);
      if (rcEngaged) {
        res.status = 409;
        res.set_content(R"({"error":"remote control is engaged"})", "application/json");
        return;
      }
      // A new session each time (the provider replaces any in-flight vector); mode gating
      // may still reject/hold it, reported via the status chip rather than an HTTP error.
      const arlcore::NumericGuid session = vectorClient.start(sp);
      state.clearVectorStatusHistory();
      res.set_content(json{{"session", guidToString(session)}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Post("/api/vector/cancel", [&](const httplib::Request&, httplib::Response& res) {
    std::scoped_lock lock(clientMutex);
    if (!vectorClient.active()) {
      res.status = 409;
      res.set_content(R"({"error":"no active vector command"})", "application/json");
      return;
    }
    const bool ok = vectorClient.cancel();
    res.set_content(json{{"canceled", ok}}.dump(), "application/json");
  });

  // Remote control: a sticky session with a server-owned deadman, deliberately separate from
  // one-shot /api/vector (layers: rolling DDS endTime, poller watchdog, key-release stop).
  server.Post("/api/rc", [&](const httplib::Request& req, httplib::Response& res) {
    try {
      const json body = json::parse(req.body);
      const bool activate = body.at("active").get<bool>();
      std::scoped_lock lock(clientMutex);
      if (!activate) {
        if (rcEngaged && vectorClient.active() && vectorClient.lastSetpoint().has_value()) {
          VectorSetpoint stop = vectorClient.lastSetpoint().value();
          stop.speedMps = 0.0;
          stop.timeoutS = kRcDeadmanS;
          vectorClient.update(stop);
          vectorClient.cancel();
        }
        rcEngaged = false;
        res.set_content(R"({"stopped":true})", "application/json");
        return;
      }
      VectorSetpoint sp = parseVectorBody(body, config);
      sp.timeoutS = kRcDeadmanS;  // the server owns the RC deadman, never the browser
      if (!rcEngaged) {
        if (vectorClient.active()) {
          vectorClient.cancel();  // the panel's session must not fight the RC session
        }
        vectorClient.start(sp);
        state.clearVectorStatusHistory();
        rcEngaged = true;
      } else if (!vectorClient.update(sp)) {
        // The session died underneath us (endTime expiry, mode rejection): start a fresh one.
        vectorClient.start(sp);
      }
      rcLastBeat = std::chrono::steady_clock::now();
      json out;
      if (vectorClient.sessionId().has_value()) {
        out["session"] = guidToString(vectorClient.sessionId().value());
      }
      out["deadman_s"] = kRcDeadmanS;
      res.set_content(out.dump(), "application/json");
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
      std::scoped_lock lock(clientMutex);
      std::string newId;
      if (type == "keep_in" || type == "keep_out") {
        std::vector<std::array<flt64_t, 2>> polygon;
        for (const auto& v : body.at("polygon")) {
          polygon.push_back({v[0].get<flt64_t>(), v[1].get<flt64_t>()});
        }
        newId = constraintsClient->upsertZone(id, name, type == "keep_in", polygon, body.value("ceiling_m", 0.0),
                                              body.value("ceiling_frame", "depth"), body.value("floor_m", 100.0),
                                              body.value("floor_frame", "depth"));
      } else if (type == "speed") {
        newId = constraintsClient->upsertSpeed(id, name, body.value("op", "lte"), body.at("value").get<flt64_t>());
      } else {
        newId = constraintsClient->upsertDepth(id, name, body.value("op", "lte"), body.at("value").get<flt64_t>());
      }
      res.set_content(json{{"id", newId}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  server.Delete(R"(/api/constraints/([0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}))",
                [&](const httplib::Request& req, httplib::Response& res) {
                  if (!constraintsClient.has_value()) {
                    res.status = 503;
                    res.set_content(R"({"error":"constraints are not configured"})", "application/json");
                    return;
                  }
                  try {
                    std::scoped_lock lock(clientMutex);
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
      std::scoped_lock lock(clientMutex);
      const bool ok = constraintsClient->setActive(ids);
      res.set_content(json{{"commanded", ok}}.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  std::cout << "Mission console listening on http://0.0.0.0:" << port << " (web root: " << webRoot << ", DDS domain "
            << config.dds.domainId << ")" << std::endl;
  server.listen("0.0.0.0", port);

  running = false;
  poller.join();
  return 0;
}
