#ifndef AUTOPILOT_SAFETY_CONSTRAINTCLAMP_HPP_
#define AUTOPILOT_SAFETY_CONSTRAINTCLAMP_HPP_

#include <optional>

#include "autopilot/safety/ConstraintTypes.hpp"
#include "autopilot/guidance/ControlVector.hpp"
#include "InternalTypes.h"

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
//! the static limits. Pure function, applied at the single control-output funnel.
//!
//! Rules: max bounds clamp down, min bounds clamp up; the min speed bound only raises commanded
//! speeds that are already nonzero (a commanded stop/hold is never sped up); elevation clamps
//! apply in the DEPTH frame only (other frames pass through untouched); a min bound above a max
//! bound clamps to the max and flags `conflict`.
ClampResult applyConstraintClamps(const ControlVector& cv, const ConstraintSnapshot& snapshot,
                                  const ClampLimits& staticLimits);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_CONSTRAINTCLAMP_HPP_
