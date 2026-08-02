#ifndef AUTOPILOT_GUIDANCE_PATHTRACKER_HPP_
#define AUTOPILOT_GUIDANCE_PATHTRACKER_HPP_

#include "InternalTypes.h"
#include "autopilot/guidance/CrossTrackController.hpp"

namespace arlcore::autopilot {

//! \brief Turns planned-path geometry into a heading command.
//!
//! The command is the path tangent plus a curvature feedforward plus a bounded cross-track
//! correction. The feedforward is the load-bearing term: the platform accepts a heading, not a
//! turn rate, so holding a planned arc requires biasing the commanded heading ahead of the
//! tangent by exactly the heading error the inner loop needs to produce that arc's turn rate.
//! That bias is `omega_desired * headingLoopTauS`.
//!
//! headingLoopTauS is the inner loop's steady-state `1/K`, measured from a heading step response
//! with tools/heading_probe. It is NOT the lag-inclusive closed-loop constant: a first-order
//! actuator lag has settled by the time an arc reaches steady state, so it does not change the
//! standing heading error the arc needs. Measure it rather than guessing, because nothing in
//! flight corrects it; a value that is too small overshoots outside every arc and one that is too
//! large cuts inside every arc.
//!
//! The law is stateless apart from the cross-track integral, so the same geometry always produces
//! the same command.
//!
//! Everything is in the azimuth frame: true north, clockwise positive, starboard-positive
//! cross-track error. The caller converts math-frame path curvature once when it fills Inputs,
//! because mixing the two conventions inside the law is where a sign error hides.
class PathTracker {
 public:
  struct Params {
    flt64_t headingLoopTauS = 1.11;       // steady-state 1/K of the inner heading loop
    flt64_t feedforwardLimitRad = 0.7;    // clamp on the curvature feedforward
    flt64_t crossTrackApproachRad = 0.6;  // the angle the cross-track term saturates at
    flt64_t crossTrackGainPerM = 0.15;    // cross-track gain, scaled by xte.kpScale
    CrossTrackController::Params xte;     // integral term and its anti-windup
  };

  //! \brief One cycle of planned geometry and measured motion.
  struct Inputs {
    flt64_t pathAzimuthRad = 0.0;          // path tangent at the projection point
    flt64_t pathCurvatureAzRadPerM = 0.0;  // azimuth frame: positive turns to starboard
    flt64_t crossTrackErrorM = 0.0;        // starboard-positive offset from the planned path
    flt64_t groundSpeedMps = 0.0;
    flt64_t dtS = 0.0;  // seconds since the previous cycle; 0 freezes the cross-track integral
  };

  //! \brief The command, plus the term breakdown for diagnostics and tests.
  struct Output {
    flt64_t headingRad = 0.0;
    flt64_t desiredYawRateRps = 0.0;
    flt64_t feedforwardRad = 0.0;
    flt64_t crossTrackRad = 0.0;
  };

  PathTracker() = default;
  explicit PathTracker(const Params& params) : params_(params) { xteCtl_.configure(params.xte); }

  void configure(const Params& params) {
    params_ = params;
    xteCtl_.configure(params.xte);
  }

  //! \brief One cycle of the tracking law.
  Output update(const Inputs& in);

  //! \brief Leg boundary, replan or new route: clears the cross-track integral, which is the only
  //! state the law carries.
  void resetLeg() { xteCtl_.reset(); }

  //! \brief The cross-track integrator state, for diagnostics.
  flt64_t xteIntegratorRad() const { return xteCtl_.integratorRad(); }

 private:
  Params params_;
  CrossTrackController xteCtl_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_PATHTRACKER_HPP_
