#ifndef AUTOPILOT_SAFETY_ICONSTRAINTSOURCE_HPP_
#define AUTOPILOT_SAFETY_ICONSTRAINTSOURCE_HPP_

#include <cstdint>

#include "autopilot/safety/ConstraintTypes.hpp"

namespace arlcore::autopilot {

//! \brief Read-side interface the supervisor exposes to the brain/planner.
class IConstraintSource {
 public:
  virtual ~IConstraintSource() = default;

  //! \brief The current constraint snapshot (copied; safe to hold across ticks).
  virtual ConstraintSnapshot snapshot() const = 0;

  //! \brief The revision of the current snapshot, for cheap change detection.
  virtual uint64_t revision() const = 0;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_SAFETY_ICONSTRAINTSOURCE_HPP_
