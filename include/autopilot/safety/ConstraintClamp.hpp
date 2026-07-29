#ifndef AUTOPILOT_SAFETY_CONSTRAINTCLAMP_HPP_
#define AUTOPILOT_SAFETY_CONSTRAINTCLAMP_HPP_

#include <optional>

#include "InternalTypes.h"
#include "autopilot/guidance/ControlVector.hpp"
#include "autopilot/safety/ConstraintTypes.hpp"

namespace arlcore::autopilot {

//! \brief Static speed/elevation limits merged once at startup from the autopilot's own
//! constraint settings and the platform capabilities (the non-dynamic side of the clamp).
struct ClampLimits {
  std::optional<flt64_t> minSpeedMps;
  std::optional<flt64_t> maxSpeedMps;
  std::optional<flt64_t> minDepthM;  // shallowest commanded depth allowed
  std::optional<flt64_t> maxDepthM;  // deepest commanded depth allowed
};

//! \brief Result of clamping one control vector.
struct ClampResult {
  ControlVector cv;
  bool speedClamped = false;
  bool elevationClamped = false;
  //! A min bound exceeded its max bound after merging (contradictory constraints); the max
  //! bound won and the supervisor should treat the conflicted axis as violated.
  bool conflict = false;
};

//! \brief Clamp a control vector to the most restrictive of the dynamic active constraints and
//! the static limits; pure function, applied at the single control-output funnel.
//! Min speed never raises a commanded stop, elevation clamps apply only in the DEPTH frame,
//! and a min bound above its max clamps to the max and flags `conflict`.
ClampResult applyConstraintClamps(const ControlVector& cv, const ConstraintSnapshot& snapshot,
                                  const ClampLimits& staticLimits);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_CONSTRAINTCLAMP_HPP_
