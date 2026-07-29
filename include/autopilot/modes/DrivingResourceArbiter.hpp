#ifndef AUTOPILOT_MODES_DRIVINGRESOURCEARBITER_HPP_
#define AUTOPILOT_MODES_DRIVINGRESOURCEARBITER_HPP_

#include <mutex>
#include <set>

#include "autopilot/config/AutopilotConfig.hpp"
#include "autopilot/modes/DriveSource.hpp"
#include "autopilot/modes/OperationalModeTypes.hpp"

namespace arlcore::autopilot {

//! \brief Arbitrates the single driving resource between the vector and waypoint command
//! providers. Because the two providers are separate CommandProviderBase instances (different
//! command types), the per-provider IncomingCommandBehavior cannot deconflict across them, so
//! this shared arbiter is required.
//!
//! Higher priority wins. Priorities are per command class (local autonomy vs remote operator)
//! so any remote command can preempt any local one. The holder's priority is recorded at
//! grant time and later requests compare against it, which stays correct in the operational
//! mode transition window where a remote command acquires before the preempted local holder
//! has cycled (a class-swapped table would misprice the stale holder there).
//!
//! Acquiring with a higher priority preempts the current lower-priority holder by marking it
//! "revoked"; the preempted provider learns of this by polling wasRevoked() in its
//! isCommandFailed() hook and then fails the command with INTERRUPTED. A lower-priority
//! acquire while a higher-priority holder owns the resource is denied; that provider then
//! fails its command with RESOURCE_REJECTED.
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
  int priorityOf(DriveSource who, CommandClass cls) const;

  mutable std::mutex mtx_;
  ArbitrationConfig config_;
  DriveSource holder_ = DriveSource::NONE;
  int holderPriority_ = -1;
  std::set<DriveSource> revoked_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_MODES_DRIVINGRESOURCEARBITER_HPP_
