#ifndef AUTOPILOT_MODES_DRIVINGRESOURCEARBITER_HPP_
#define AUTOPILOT_MODES_DRIVINGRESOURCEARBITER_HPP_

#include <cstdint>
#include <mutex>
#include <set>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/modes/DriveSource.hpp"
#include "autopilot/modes/OperationalModeTypes.hpp"

namespace arlcore::autopilot {

//! \brief Arbitrates the single driving resource between the vector and waypoint command
//! providers (separate CommandProviderBase instances cannot deconflict across command types);
//! higher per-class priority wins. The holder's priority is recorded at grant time so later
//! requests compare correctly even in the mode-transition window where the preempted holder
//! has not yet cycled.
class DrivingResourceArbiter {
 public:
  explicit DrivingResourceArbiter(const ArbitrationConfig& config);

  //! \brief Non-mutating check of whether `who` could acquire the resource right now.
  bool canDrive(DriveSource who, CommandClass cls = CommandClass::LOCAL) const;

  //! \brief Attempt to acquire the resource for `who` commanding as `cls`. Returns true if
  //! granted (preempting any strictly-lower-priority holder), false if denied.
  bool acquire(DriveSource who, CommandClass cls = CommandClass::LOCAL);

  //! \brief Release the resource if `who` currently holds it; also clears its revoked flag.
  void release(DriveSource who);

  //! \brief The current resource holder (NONE if free).
  DriveSource currentHolder() const;

  //! \brief Whether `who` currently owns the resource.
  bool ownsResource(DriveSource who) const;

  //! \brief Whether `who` was preempted by a higher-priority acquire since it last held.
  bool wasRevoked(DriveSource who) const;

 private:
  int32_t priorityOf(DriveSource who, CommandClass cls) const;

  mutable std::mutex mtx_;
  ArbitrationConfig config_;
  DriveSource holder_ = DriveSource::NONE;
  int32_t holderPriority_ = -1;
  std::set<DriveSource> revoked_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_MODES_DRIVINGRESOURCEARBITER_HPP_
