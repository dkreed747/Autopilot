#ifndef AUTOPILOT_UMAA_ELEVATIONADMISSION_HPP_
#define AUTOPILOT_UMAA_ELEVATIONADMISSION_HPP_

#include <optional>
#include <string>

#include "InternalTypes.h"
#include "autopilot/guidance/ToleranceUtils.hpp"

namespace arlcore::autopilot {

//! \brief The static elevation limits a command is admitted against, plus whether the platform
//! can report an altitude above the sea floor at all.
struct ElevationAdmissionLimits {
  std::optional<flt64_t> maxDepthM;
  std::optional<flt64_t> minAltitudeAsfM;
  bool platformReportsAsf = false;
};

//! \brief Whether a commanded elevation may be admitted. Rejecting here rather than at the output
//! clamp is the point: a setpoint the clamp would hold away forever leaves the command executing
//! but never achieved, which fails it on the tolerance delay or spends the route's miss budget.
//!
//! Frame-native checks always apply. `floorDepthM` enables the cross-frame ones and should only be
//! passed for a setpoint flown at the current position - the floor under the vehicle now says
//! nothing about the floor at a distant waypoint. `reason` receives a human-readable rejection.
bool elevationAdmissible(const ElevationValue& el, const ElevationAdmissionLimits& limits,
                         std::optional<flt64_t> floorDepthM, std::string* reason);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_ELEVATIONADMISSION_HPP_
