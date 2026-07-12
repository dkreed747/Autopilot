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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_IAUTOPILOT_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_IAUTOPILOT_H_

#include <vector>

#include <UMAA/MO/GlobalVectorControl/GlobalVectorCommandType.hpp>
#include <UMAA/MO/GlobalWaypointControl/GlobalWaypointType.hpp>

#include "DriveSource.h"
#include "DrivingResourceArbiter.h"
#include "ProgressTypes.h"

namespace arlcore::autopilot {

//! \brief The autopilot "brain": the shared driving controller (formerly "vehicle control").
//! It holds the active driving mode and setpoint, recomputes a control vector from the latest
//! navigation data on every nav tick, and pushes it to the vehicle-control strategy. The two
//! command providers drive it via the setpoint methods and read progress back for UMAA status.
class IAutopilot {
 public:
  virtual ~IAutopilot() = default;

  //! \brief Install a vector setpoint (pass-through driving). The provider must have already
  //! acquired the driving resource from the arbiter.
  virtual void setVectorSetpoint(const UMAA::MO::GlobalVectorControl::GlobalVectorCommandType& cmd) = 0;

  //! \brief Install a waypoint route setpoint. The brain plans a Dubins path from the current
  //! pose. Returns false if no navigation fix is available to plan from.
  virtual bool setWaypointSetpoint(
      const std::vector<UMAA::MO::GlobalWaypointControl::GlobalWaypointType>& waypoints) = 0;

  //! \brief Clear the setpoint owned by `src` (no-op if it is not the active mode).
  virtual void clearSetpoint(DriveSource src) = 0;

  //! \brief Recompute and push a control vector from the latest navigation data. Triggered by
  //! the global pose observer on every new nav packet.
  virtual void onNavUpdate() = 0;

  //! \brief Achieved-flag snapshot for the active vector command.
  virtual VectorProgress vectorProgress() const = 0;

  //! \brief Progress snapshot for the active waypoint route.
  virtual WaypointProgress waypointProgress() const = 0;

  //! \brief The shared driving-resource arbiter consulted by both providers.
  virtual DrivingResourceArbiter& arbiter() = 0;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_IAUTOPILOT_H_
