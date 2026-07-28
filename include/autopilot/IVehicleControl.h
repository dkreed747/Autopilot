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

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_IVEHICLECONTROL_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_IVEHICLECONTROL_H_

#include "ControlVector.h"
#include "OperationalModeTypes.hpp"

namespace arlcore::autopilot {

//! \brief Hardware-abstraction strategy. The autopilot brain drives the vehicle purely
//! through this interface, so the same autopilot runs on different robots by swapping the
//! concrete strategy (Strategy pattern). The platform also owns MANUAL control through this
//! interface: it can always take it and alone decides when it is released.
class IVehicleControl {
 public:
  virtual ~IVehicleControl() = default;

  //! \brief Initialize the underlying hardware/sim link.
  //! \return true on success
  virtual bool initialize() = 0;

  //! \brief Stop the underlying hardware/sim link (joins any internal threads). Safe to call
  //! multiple times; strategies also call it from their destructors.
  virtual void shutdown() {}

  //! \brief Send a vector-like control setpoint (heading, speed, elevation/depth) to the platform.
  //! \param cv The control vector to actuate
  //! \return true if the setpoint was accepted by the platform link
  virtual bool sendControlVector(const ControlVector& cv) = 0;

  //! \brief Whether the platform's manual (onboard/safety-driver) control is engaged. Polled
  //! every control tick: while true the autopilot must not actuate and reports MANUAL; a
  //! strategy that returns true from the first poll boots the autopilot into MANUAL.
  virtual bool isManualEngaged() const = 0;

  //! \brief Optional hook: the reported operational mode changed (no-op by default), so a
  //! hardware strategy can e.g. arm actuators when leaving STANDBY.
  virtual void onOperationalModeChanged(OperationalMode /*mode*/) {}
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_IVEHICLECONTROL_H_
