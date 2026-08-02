#include "autopilot/umaa/ElevationAdmission.hpp"

#include <cmath>
#include <sstream>

#include "autopilot/guidance/ElevationUtils.hpp"

namespace arlcore::autopilot {

static void setReason(std::string* reason, const std::string& text) {
  if (reason != nullptr) {
    *reason = text;
  }
}

//! \brief Default stream formatting, so 50.0 reads as "50" rather than std::to_string's "50.000000".
static std::string numberText(flt64_t value) {
  std::ostringstream text;
  text << value;
  return text.str();
}

bool elevationAdmissible(const ElevationValue& el, const ElevationAdmissionLimits& limits,
                         std::optional<flt64_t> floorDepthM, std::string* reason) {
  // Screened before every comparison below, which a NaN would pass: NaN orders false against
  // everything, so `value > limit` does not reject it.
  if (!std::isfinite(el.valueM)) {
    setReason(reason, "elevation is not finite");
    return false;
  }

  if (el.frame == ElevationFrame::DEPTH) {
    if (limits.maxDepthM.has_value() && el.valueM > limits.maxDepthM.value()) {
      setReason(reason, "depth " + numberText(el.valueM) + " m exceeds the configured constraints.max_depth_m of " +
                            numberText(limits.maxDepthM.value()) + " m");
      return false;
    }
    if (limits.minAltitudeAsfM.has_value() && floorDepthM.has_value()) {
      const flt64_t impliedAsfM = elevation::flipDepthAsf(el.valueM, floorDepthM.value());
      if (impliedAsfM < limits.minAltitudeAsfM.value()) {
        setReason(reason, "depth " + numberText(el.valueM) + " m leaves only " + numberText(impliedAsfM) +
                              " m above the sea floor (" + numberText(floorDepthM.value()) +
                              " m down), under the configured constraints.min_altitude_asf_m of " +
                              numberText(limits.minAltitudeAsfM.value()) + " m");
        return false;
      }
    }
    return true;
  }

  if (el.frame == ElevationFrame::ALTITUDE_ASF) {
    if (!limits.platformReportsAsf) {
      setReason(reason,
                "elevation is in the ALTITUDE_ASF frame but the platform does not report an "
                "altitude above the sea floor (platform_capabilities.underwater."
                "reports_altitude_asf is not set)");
      return false;
    }
    if (el.valueM < 0.0) {
      setReason(reason, "altitude above sea floor " + numberText(el.valueM) + " m is below the sea floor");
      return false;
    }
    if (limits.minAltitudeAsfM.has_value() && el.valueM < limits.minAltitudeAsfM.value()) {
      setReason(reason, "altitude above sea floor " + numberText(el.valueM) +
                            " m is under the configured constraints.min_altitude_asf_m of " +
                            numberText(limits.minAltitudeAsfM.value()) + " m");
      return false;
    }
    if (limits.maxDepthM.has_value() && floorDepthM.has_value()) {
      const flt64_t impliedDepthM = elevation::flipDepthAsf(el.valueM, floorDepthM.value());
      if (impliedDepthM > limits.maxDepthM.value()) {
        setReason(reason, "altitude above sea floor " + numberText(el.valueM) + " m is a depth of " +
                              numberText(impliedDepthM) + " m over a floor " + numberText(floorDepthM.value()) +
                              " m down, exceeding the configured constraints.max_depth_m of " +
                              numberText(limits.maxDepthM.value()) + " m");
        return false;
      }
    }
    return true;
  }

  // MSL/AGL/geodetic cannot be compared against a depth or altitude limit without a geoid or
  // ground reference the autopilot does not have, and ConstraintClamp cannot clamp them either.
  // Refusing is the only safe answer: such a setpoint would otherwise pass both the admission
  // check and the clamp untouched, and a vehicle that converts it to a depth would then drive
  // straight past the configured limit.
  // TODO(autopilot-#4): accept the MSL/AGL/geodetic frames once a geoid reference is available to
  // convert them.
  if (limits.maxDepthM.has_value() || limits.minAltitudeAsfM.has_value()) {
    setReason(reason, std::string("elevation frame ") + elevation::frameName(el.frame) +
                          " cannot be checked against the configured constraints.max_depth_m / "
                          "constraints.min_altitude_asf_m");
    return false;
  }
  return true;
}

}  // namespace arlcore::autopilot
