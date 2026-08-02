#ifndef AUTOPILOT_GUIDANCE_ISEAFLOORREFERENCE_HPP_
#define AUTOPILOT_GUIDANCE_ISEAFLOORREFERENCE_HPP_

#include <optional>

#include "InternalTypes.h"

namespace arlcore::autopilot {

//! \brief Read-side seam for the live seafloor reference, the one fact that makes a DEPTH bound
//! and an ALTITUDE_ASF setpoint comparable. Injected into the command providers so admission can
//! convert without reaching into the brain or the navigation state.
class ISeafloorReference {
 public:
  virtual ~ISeafloorReference() = default;

  //! \brief Depth of the sea floor under the vehicle, or nullopt when the platform is not
  //! reporting a usable altitude above the sea floor right now (no altimeter, lost bottom lock).
  virtual std::optional<flt64_t> floorDepthM() const = 0;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_GUIDANCE_ISEAFLOORREFERENCE_HPP_
