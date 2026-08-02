#ifndef AUTOPILOT_GUIDANCE_CROSSTRACKCONTROLLER_HPP_
#define AUTOPILOT_GUIDANCE_CROSSTRACKCONTROLLER_HPP_

#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief PI correction on cross-track error. The proportional shape is supplied by the caller
//! (see approachCorrection); the integral accumulates the steady crab-angle bias (radians) a
//! lateral current or trim leaves behind, which pure P cannot null. ki = 0 makes integrate() a
//! clamped pass-through of the proportional term.
class CrossTrackController {
 public:
  struct Params {
    flt64_t kpScale = 1.0;
    flt64_t ki = 0.0;                   // rad of bias per meter-second of integrated error
    flt64_t integratorLimitRad = 0.35;  // ~20 deg: bounds the worst-case crab bias
    flt64_t integratorGateM = 5.0;      // integrate only while |err| is inside the gate
    flt64_t correctionLimitRad = 1.2;   // total correction clamp
  };

  CrossTrackController() = default;
  explicit CrossTrackController(const Params& params) : params_(params) {}

  void configure(const Params& params) { params_ = params; }

  //! \brief The PI machinery for a caller-supplied proportional term: applies the integral, its
  //! anti-windup, and the total-correction clamp. dtS is the elapsed time since the previous
  //! update (clamped to [0, 1] s; pass 0 on the first call, which freezes the integral).
  flt64_t integrate(flt64_t pRad, flt64_t errM, flt64_t dtS);

  //! \brief Bounded approach-angle P law: approachLimitRad * (2/pi) * atan(gainPerM * errM).
  //! It saturates at approachLimitRad however large the error, so a gross excursion cannot
  //! consume the turn authority the curvature feedforward needs. Slope at the origin is
  //! approachLimitRad * (2/pi) * gainPerM.
  static flt64_t approachCorrection(flt64_t errM, flt64_t approachLimitRad, flt64_t gainPerM);

  void reset() { integralRad_ = 0.0; }
  flt64_t integratorRad() const { return integralRad_; }

 private:
  Params params_;
  flt64_t integralRad_ = 0.0;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_CROSSTRACKCONTROLLER_HPP_
