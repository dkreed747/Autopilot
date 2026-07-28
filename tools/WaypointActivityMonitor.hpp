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

#ifndef APPS_AUTOPILOT_TOOLS_WAYPOINTACTIVITYMONITOR_HPP_
#define APPS_AUTOPILOT_TOOLS_WAYPOINTACTIVITYMONITOR_HPP_

#include <UMAA/Common/LargeListMetadata.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandStatusType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointCommandType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointExecutionStatusReportType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointType.hpp>
#include <chrono>
#include <dds/dds.hpp>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "CycloneReader.h"
#include "LargeListReader.h"
#include "NumericGuid.h"

namespace arlcore::autopilot::tools {

//! \brief One waypoint mission observed on the bus, reconstructed from the
//! (unfiltered) command topic, its large list, and the status/execution-status
//! topics.
struct ObservedMission {
  arlcore::NumericGuid sessionId;
  arlcore::NumericGuid sourceId;
  arlcore::NumericGuid sourceParentId;
  std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType> waypoints;
  bool listComplete = false;
  std::string lastStatus;
  std::string lastReason;
  bool terminal = false;
  std::optional<
      UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType>
      execStatus;
  std::chrono::steady_clock::time_point lastSeen;
  std::chrono::steady_clock::time_point execSeen;
};

//! \brief Read-only bus-wide observer of Global Waypoint missions, whoever
//! commanded them: the console renders every route with a per-commander
//! classification. Canceled missions arrive as disposed command instances;
//! terminal missions are evicted after a grace period and the tracked set is
//! capped so the GUI snapshot cannot grow without bound.
class WaypointActivityMonitor {
 public:
  using CommandType =
      UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandType;
  using StatusType =
      UMAA::MO::GlobalWaypointControl::GlobalWaypointCommandStatusType;
  using ExecType =
      UMAA::MO::GlobalWaypointControl::GlobalWaypointExecutionStatusReportType;
  using GlobalWaypointType =
      UMAA::MO::GlobalWaypointControl::GlobalWaypointType;
  using ListElement = UMAA::MO::GlobalWaypointControl::
      GlobalWaypointCommandTypeWaypointsListElement;

  WaypointActivityMonitor(const dds::domain::DomainParticipant& participant,
                          const dds::sub::qos::DataReaderQos& rqos,
                          const dds::sub::qos::DataReaderQos& largeListRqos);

  //! \brief Drain all readers and reconcile the observed-mission table (once
  //! per poller tick).
  void poll();

  const std::map<arlcore::NumericGuid, ObservedMission>& missions() const {
    return missions_;
  }

  //! \brief The latest execution status seen for a session, if any.
  std::optional<ExecType> execFor(const arlcore::NumericGuid& sessionId) const;

 private:
  //! \brief Drop a session's list from the reader unless another tracked session still
  //! references the same listID (its elements could never be re-derived after removal).
  void releaseListIfUnshared(const arlcore::NumericGuid& session,
                             const UMAA::Common::LargeListMetadata& metadata);

  std::shared_ptr<arlcore::io::CycloneReader<CommandType>> cmdReader_;
  std::shared_ptr<arlcore::io::CycloneReader<StatusType>> statusReader_;
  std::shared_ptr<arlcore::io::CycloneReader<ExecType>> execReader_;
  arlcore::umaa::LargeListReader<GlobalWaypointType, ListElement> listReader_;

  std::map<arlcore::NumericGuid, ObservedMission> missions_;
  std::map<arlcore::NumericGuid, UMAA::Common::LargeListMetadata>
      metadataBySession_;
};

}  // namespace arlcore::autopilot::tools
#endif  // APPS_AUTOPILOT_TOOLS_WAYPOINTACTIVITYMONITOR_HPP_
