#ifndef AUTOPILOT_GUIDANCE_CROSSTRACKCONTROLLER_HPP_
#define AUTOPILOT_GUIDANCE_CROSSTRACKCONTROLLER_HPP_

#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief PI correction on cross-track error. P is the legacy atan(kp*err / ref) shape; the
//! integral accumulates the steady crab-angle bias (radians) a lateral current or trim leaves
//! behind, which pure P cannot null. ki = 0 reproduces the legacy law bit-for-bit.
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

  //! \brief The correction to subtract from the path azimuth. errM is starboard-positive
  //! cross-track error, refM the convergence length scale (turn radius), dtS the elapsed time
  //! since the previous update (clamped to [0, 1] s; pass 0 on the first call).
  flt64_t correction(flt64_t errM, flt64_t refM, flt64_t dtS);

  //! \brief The stateless legacy P law (shared shape with the vector-mode wall standoff).
  static flt64_t pCorrection(flt64_t errM, flt64_t refM, flt64_t limitRad);

  void reset() { integralRad_ = 0.0; }
  flt64_t integratorRad() const { return integralRad_; }

 private:
  Params params_;
  flt64_t integralRad_ = 0.0;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_CROSSTRACKCONTROLLER_HPP_
