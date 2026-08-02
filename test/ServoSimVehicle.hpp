#ifndef AUTOPILOT_TEST_SERVOSIMVEHICLE_HPP_
#define AUTOPILOT_TEST_SERVOSIMVEHICLE_HPP_

#include <GeographicLib/LocalCartesian.hpp>
#include <UMAA/SA/GlobalPoseStatus/GlobalPoseReportType.hpp>

#include "InternalTypes.h"
#include "autopilot/guidance/ControlVector.hpp"

//! \brief Test-local kinematic vehicle whose inner heading loop is a configurable proportional
//! servo saturated at the platform turn rate, with an optional first-order actuator lag.
//!
//! A mock cannot express vehicle motion, and the other hand-rolled test vehicles all slew at
//! exactly the rate limit whenever there is any heading error at all. That deadbeat behavior has
//! no proportional gain to identify and no steady-state heading offset, so it cannot exercise a
//! curvature feedforward or the trim that calibrates one. Setting kTrueRpsPerRad to 1/dt
//! reproduces the deadbeat loop exactly, which makes this a strict generalization.
class ServoSimVehicle {
 public:
  flt64_t kTrueRpsPerRad = 20.0;
  flt64_t maxTurnRateRps = 0.2618;
  flt64_t lagTauS = 0.0;
  flt64_t driftEastMps = 0.0;  // uniform current the controller cannot see
  flt64_t driftNorthMps = 0.0;

  flt64_t xE = 0.0;
  flt64_t yN = 0.0;
  flt64_t yawRad = 0.0;
  flt64_t speedMps = 0.0;
  flt64_t yawRateRps = 0.0;  // azimuth frame, as the platform reports it
  bool rateSaturated = false;

  //! \brief The current state as a UMAA pose report, reverse-projected into geodetic.
  UMAA::SA::GlobalPoseStatus::GlobalPoseReportType pose() const;

  void step(const arlcore::autopilot::ControlVector& cv, flt64_t dtS);

 private:
  GeographicLib::LocalCartesian frame_{39.0, -76.5, 0.0};
};

#endif  // AUTOPILOT_TEST_SERVOSIMVEHICLE_HPP_
