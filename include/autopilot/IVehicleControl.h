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

#include <UMAA/EO/UVPlatformSpecs/UVPlatformSpecsReportType.hpp>
#include <UMAA/EO/UVPlatformSpecs/UVPlatformCapabilitiesReportType.hpp>

#include "ControlVector.h"

namespace arlcore::autopilot {

//! \brief Hardware-abstraction strategy. The autopilot brain drives the vehicle purely
//! through this interface, so the same autopilot runs on different robots by swapping the
//! concrete strategy (Strategy pattern). It also exposes the platform's UMAA specs and
//! capabilities so the autopilot can validate commands and plan against real limits.
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

  //! \brief The physical platform specifications for this vehicle.
  virtual UMAA::EO::UVPlatformSpecs::UVPlatformSpecsReportType getPlatformSpecs() const = 0;

  //! \brief The performance capabilities (speed/turn-rate limits, etc.) for this vehicle.
  virtual UMAA::EO::UVPlatformSpecs::UVPlatformCapabilitiesReportType getPlatformCapabilities() const = 0;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_IVEHICLECONTROL_H_
