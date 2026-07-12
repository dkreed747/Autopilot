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

#ifndef APPS_AUTOPILOT_TOOLS_WAYPOINTMISSIONCLIENT_H_
#define APPS_AUTOPILOT_TOOLS_WAYPOINTMISSIONCLIENT_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <dds/dds.hpp>

#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandAckReportType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandStatusType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointType.hpp>

#include "CycloneReader.h"
#include "CycloneSender.h"
#include "LargeListWriter.h"
#include "NumericGuid.h"

namespace arlcore::autopilot::tools {

//! \brief One command-status transition observed for the active session.
struct MissionStatusUpdate {
  std::string status;   // ISSUED / COMMANDED / EXECUTING / COMPLETED / CANCELED / FAILED
  std::string reason;
  std::string logMessage;
  bool terminal = false;
};

//! \brief The UMAA consumer side of the Global Waypoint control service: publishes a
//! waypoint route (as a large list plus the command referencing it), tracks the command
//! acknowledgement and status for the session, and cancels by disposing the command
//! instance. One client drives at most one session at a time; starting a new mission
//! retires the previous list.
class WaypointMissionClient {
 public:
  using GlobalWaypointType = UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
  using CommandType = UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandType;
  using CommandStatusType = UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandStatusType;
  using CommandAckType = UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandAckReportType;
  using ListElement = UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandTypeWaypointsListElement;

  //! \brief `destinationId` is the waypoint provider's source ID (identity.waypoint_source_id).
  WaypointMissionClient(const dds::domain::DomainParticipant& participant,
                        const dds::pub::qos::DataWriterQos& wqos,
                        const dds::sub::qos::DataReaderQos& rqos,
                        const arlcore::NumericGuid& destinationId);

  //! \brief Publish the route and command. Returns the new session ID.
  arlcore::NumericGuid start(const std::vector<GlobalWaypointType>& waypoints);

  //! \brief Cancel the active session by disposing the command instance.
  bool cancel();

  //! \brief Drain new command-status samples for the active session.
  std::vector<MissionStatusUpdate> pollStatus();

  //! \brief True once the session's command acknowledgement has been received.
  bool pollAck();

  //! \brief True while a session is started and not yet terminal.
  bool active() const { return sessionId_.has_value() && !terminal_; }

  const std::optional<arlcore::NumericGuid>& sessionId() const { return sessionId_; }
  const std::vector<GlobalWaypointType>& waypoints() const { return waypoints_; }
  bool ackReceived() const { return ackReceived_; }
  const std::string& lastStatus() const { return lastStatus_; }
  const std::string& lastReason() const { return lastReason_; }

 private:
  std::shared_ptr<arlcore::io::CycloneSender<CommandType>> cmdSender_;
  std::shared_ptr<arlcore::io::CycloneSender<ListElement>> elementSender_;
  std::shared_ptr<arlcore::io::CycloneReader<CommandStatusType>> statusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<CommandAckType>> ackReader_;

  arlcore::NumericGuid sourceId_;
  arlcore::NumericGuid destinationId_;
  std::optional<arlcore::umaa::LargeListWriter<GlobalWaypointType, ListElement>> listWriter_;
  CommandType cmd_;
  std::optional<arlcore::NumericGuid> sessionId_;
  std::vector<GlobalWaypointType> waypoints_;
  bool terminal_ = true;
  bool ackReceived_ = false;
  std::string lastStatus_;
  std::string lastReason_;
};

}  // namespace arlcore::autopilot::tools

#endif  // APPS_AUTOPILOT_TOOLS_WAYPOINTMISSIONCLIENT_H_
