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

#include "DrivingResourceArbiter.h"

#include "Logger.h"

namespace arlcore::autopilot {

DrivingResourceArbiter::DrivingResourceArbiter(int vectorPriority, int waypointPriority) :
    vectorPriority_(vectorPriority), waypointPriority_(waypointPriority) {}

int DrivingResourceArbiter::priorityOf(DriveSource who) const {
  switch (who) {
    case DriveSource::VECTOR:
      return vectorPriority_;
    case DriveSource::WAYPOINT:
      return waypointPriority_;
    default:
      return -1;
  }
}

bool DrivingResourceArbiter::canDrive(DriveSource who) const {
  std::lock_guard<std::mutex> lock(mtx_);
  if (holder_ == DriveSource::NONE || holder_ == who) {
    return true;
  }
  return priorityOf(who) > priorityOf(holder_);
}

bool DrivingResourceArbiter::acquire(DriveSource who) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (holder_ == who) {
    revoked_.erase(who);
    return true;
  }
  if (holder_ == DriveSource::NONE) {
    holder_ = who;
    revoked_.erase(who);
    return true;
  }
  if (priorityOf(who) > priorityOf(holder_)) {
    // Preempt the lower-priority holder.
    revoked_.insert(holder_);
    UMAA_LOG_INFO(util::SYSTEM_LOGGER, "Driving resource preempted by higher-priority source")
    holder_ = who;
    revoked_.erase(who);
    return true;
  }
  // Equal or lower priority: denied.
  return false;
}

void DrivingResourceArbiter::release(DriveSource who) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (holder_ == who) {
    holder_ = DriveSource::NONE;
  }
  revoked_.erase(who);
}

DriveSource DrivingResourceArbiter::currentHolder() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return holder_;
}

bool DrivingResourceArbiter::ownsResource(DriveSource who) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return holder_ == who;
}

bool DrivingResourceArbiter::wasRevoked(DriveSource who) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return revoked_.count(who) > 0;
}

}  // namespace arlcore::autopilot
