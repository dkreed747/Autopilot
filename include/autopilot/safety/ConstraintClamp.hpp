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
  //! Closest to the sea floor a setpoint may be commanded. Static only: UMAA has no scalar
  //! altitude conditional, so no dynamic bound merges into this axis (an ASF-framed water-zone
  //! bound is enforced by the zone machinery instead).
  std::optional<flt64_t> minAltitudeAsfM;
};

//! \brief Result of clamping one control vector.
struct ClampResult {
  ControlVector cv;
  bool speedClamped = false;
  bool elevationClamped = false;
  //! A depth-frame bound applied to an ALTITUDE_ASF setpoint but no seafloor reference was
  //! available to convert it, so the elevation demand was dropped (nullopt = hold current)
  //! rather than forwarded unbounded.
  bool elevationUnbounded = false;
  //! The bounds on an axis cannot all be satisfied: a min above its max after merging, or a
  //! water column too thin to honor both the depth and the bottom-clearance limits. Speed
  //! resolves to its max bound; elevation resolves to the deep-side bound, clamped into the
  //! water column.
  bool conflict = false;
};

//! \brief Clamp a control vector to the most restrictive of the dynamic active constraints and
//! the static limits; pure function of its arguments, applied at the single control-output funnel.
//! `floorDepthM` is the live seafloor reference (depth + altitudeASF) or nullopt when the platform
//! is not reporting one.
//!
//! Min speed never raises a commanded stop. Elevation is bounded in the frame it was commanded in:
//! a DEPTH setpoint by the depth limits and, once the floor is known, by the bottom clearance; an
//! ALTITUDE_ASF setpoint by the bottom clearance directly and, once the floor is known, by the
//! converted depth limits and the surface. An ALTITUDE_ASF setpoint that a configured depth bound
//! cannot be converted for is dropped, not passed through. ALTITUDE_MSL/AGL/GEODETIC have no
//! evaluable equivalent and pass through untouched (admission refuses them when limits exist).
ClampResult applyConstraintClamps(const ControlVector& cv, const ConstraintSnapshot& snapshot,
                                  const ClampLimits& staticLimits, std::optional<flt64_t> floorDepthM);

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_CONSTRAINTCLAMP_HPP_
