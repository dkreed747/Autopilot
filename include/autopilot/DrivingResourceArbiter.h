//---------------------------------------------------------------------------
// Copyright 2025 Pennsylvania State University
//
// Applied Research Laboratory
// Pennsylvania State University
// P.O. Box 30
// State College, PA 16804-0030
//
// DISTRIBUTION STATEMENT A. Approved for public release.
// Distribution is unlimited.
// This software was developed by the Department of the Navy,
// NAVSEA Unmanned and Small Combatants. It is provided under the terms of
// use found in the LICENSE file at the source code root directory.
//
//---------------------------------------------------------------------------

#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVINGRESOURCEARBITER_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVINGRESOURCEARBITER_H_

#include <mutex>
#include <set>

#include "DriveSource.h"

namespace arlcore::autopilot {

//! \brief Arbitrates the single driving resource between the vector and waypoint command
//! providers. Because the two providers are separate CommandProviderBase instances (different
//! command types), the per-provider IncomingCommandBehavior cannot deconflict across them, so
//! this shared arbiter is required.
//!
//! Higher priority wins. Acquiring with a higher priority preempts the current lower-priority
//! holder by marking it "revoked"; the preempted provider learns of this by polling
//! wasRevoked() in its isCommandFailed() hook and then fails the command with INTERRUPTED.
//! A lower-priority acquire while a higher-priority holder owns the resource is denied; that
//! provider then fails its command with RESOURCE_REJECTED.
class DrivingResourceArbiter {
 public:
  DrivingResourceArbiter(int vectorPriority, int waypointPriority);

  //! \brief Non-mutating check of whether `who` could acquire the resource right now.
  bool canDrive(DriveSource who) const;

  //! \brief Attempt to acquire the resource for `who`. Returns true if granted (preempting any
  //! strictly-lower-priority holder), false if denied.
  bool acquire(DriveSource who);

  //! \brief Release the resource if `who` currently holds it; also clears its revoked flag.
  void release(DriveSource who);

  //! \brief The current resource holder (NONE if free).
  DriveSource currentHolder() const;

  //! \brief Whether `who` currently owns the resource.
  bool ownsResource(DriveSource who) const;

  //! \brief Whether `who` was preempted by a higher-priority acquire since it last held.
  bool wasRevoked(DriveSource who) const;

 private:
  int priorityOf(DriveSource who) const;

  mutable std::mutex mtx_;
  int vectorPriority_;
  int waypointPriority_;
  DriveSource holder_ = DriveSource::NONE;
  std::set<DriveSource> revoked_;
};

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_DRIVINGRESOURCEARBITER_H_
