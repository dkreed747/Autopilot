#ifndef AUTOPILOT_GUIDANCE_HEADINGLOOPIDENTIFIER_HPP_
#define AUTOPILOT_GUIDANCE_HEADINGLOOPIDENTIFIER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief One sample of a commanded-heading step response. `headingErrRad` is the wrapped
//! commanded-minus-actual heading and `yawRateRps` the measured turn rate, both in the
//! azimuth frame, so a positive error should produce a positive rate.
struct HeadingProbeSample {
  flt64_t elapsedS = 0.0;
  flt64_t headingErrRad = 0.0;
  flt64_t yawRateRps = 0.0;
  flt64_t speedMps = 0.0;
  int32_t stepIndex = -1;
  flt64_t stepRad = 0.0;  // the commanded delta that opened this step
};

//! \brief The identified inner heading loop: a proportional gain saturated at a turn-rate
//! envelope, plus the lag the closed loop actually exhibits.
struct HeadingLoopModel {
  flt64_t gainRpsPerRad = 0.0;  // K, from the unsaturated region
  flt64_t gainR2 = 0.0;
  int32_t gainSamples = 0;
  flt64_t gainPortRpsPerRad = 0.0;
  flt64_t gainStbdRpsPerRad = 0.0;
  flt64_t asymmetryFrac = 0.0;  // |K_port - K_stbd| / mean

  flt64_t omegaMaxRps = 0.0;
  flt64_t omegaMaxCv = 0.0;  // spread across the saturated windows; high means unreliable
  int32_t omegaMaxWindows = 0;

  flt64_t satErrRad = 0.0;      // omega_max / K: the error at which the loop saturates
  flt64_t satErrKneeRad = 0.0;  // the same quantity read off the step decays

  flt64_t headingLoopTauS = 0.0;   // 1/K, which is what the tracker's feedforward consumes
  flt64_t decayTauS = 0.0;         // closed-loop heading decay time constant
  flt64_t modelConsistency = 0.0;  // decayTauS * K; 1.0 when the loop really is first order

  std::string verdict = "NO_DATA";
};

//! \brief Identify the heading loop from probe samples. Never throws and never divides by
//! zero: a data set with too little unsaturated motion to fit reports
//! verdict = SATURATION_DOMINATED with the knee estimate filled in, and one with too few
//! saturated windows to trust reports UNRELIABLE_OMEGA_MAX.
HeadingLoopModel identifyHeadingLoop(const std::vector<HeadingProbeSample>& samples);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_HEADINGLOOPIDENTIFIER_HPP_
